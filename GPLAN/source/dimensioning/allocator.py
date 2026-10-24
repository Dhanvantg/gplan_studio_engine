"""Area-budget allocator: decide every room's size envelope up front.

Change B of the door_connectivity sizing plan. On the shipped min-dim path
room size is a residue: the solver minimises onto the request minimums and
the gapless fill pushes the plot's leftover area into whichever room can
reach a notch. This module is the missing decision. Given a room list, a
plot and the NBC rulebook it returns, per room, the envelope the room SHOULD
occupy, such that the allocated areas sum to the plot area (when the program
fits). Generation then sends the allocated targets as the solver minimums -
the one lever the solver actually honours - and the minimal-area solution
already fills the plot, leaving the gap fill nothing to redistribute.

Pure function, no solver, no I/O; unit-tested against the worked example in
documentation/plans/DOOR_CONNECTIVITY_SIZING_AND_POSTPROCESS_PLAN.md 4.6.

Clear floor versus rectangle
----------------------------
NBC minimums are CLEAR-FLOOR figures. Walls are drawn centred on the room
rectangle edges (interior 0.4 ft in the designer), so a rectangle loses half
a wall on every side: a 10 x 11.5 rectangle delivers ~9.6 x 11.1 = 107 sqft
of clear floor against a 115 sqft NBC Living Room minimum. The allocator
therefore works in RECTANGLE space throughout: NBC clear bounds are
converted via ``rect_bounds`` (walls added on top, never subtracted), and
because the rectangles tile the plot exactly under the centreline
convention, ``sum(rect areas) == plot area`` holds by construction.

Known residual, deliberate: boundary rooms lose 0.375 ft (half an exterior
wall) rather than 0.2 ft on their outer sides, which the flat allowance
under-counts by up to 0.175 ft per axis. Which rooms are exterior is not
known before layout; ``wall_allowance_ft`` is a request option so the
residual can be tuned once measured.
"""

import math

from GPLAN.source.postprocessing import nbc_rules

_INF = float("inf")

# Two interior half-walls, 0.2 + 0.2 ft (designer interior wall 0.4 ft).
WALL_ALLOWANCE_FT = 0.4

# Product decision (2026-07-30): surplus goes to the social/private core
# first, service rooms stay at their minimums unless everything above is
# capped. Membership is by canonical rulebook name; unknown types are tier 3.
TIER_1 = ("Living Room", "Master Bedroom", "Bedroom", "Kitchen")
TIER_2 = ("Dining", "Study")

# Each tier may absorb this fraction of the surplus that reaches it
# (1.0 = all of it, the confirmed default). Anything a tier declines or
# cannot hold flows to the next; a final fill pass keeps the plot full
# whenever the program has capacity at all.
DEFAULT_TIER_SHARE = (1.0, 1.0, 1.0)

# Adjacent rooms must overlap >= 2 grid units on the perpendicular axis for
# black edges; a span below this kills the topology with no message beyond
# "Returned false in [longest_path]".
MIN_SPAN_FT = 2.0


def _tier_of(canonical):
    if canonical in TIER_1:
        return 1
    if canonical in TIER_2:
        return 2
    return 3


def rect_bounds(rule, allowance=WALL_ALLOWANCE_FT):
    """Clear NBC bounds -> rectangle bounds, walls added on top.

    Returns ((lo_w, lo_h, lo_area), (hi_w, hi_h, hi_area)) in rectangle feet.
    The area floor beats the shape floor (a 3.5 x 5 toilet is 17.5 sqft
    against an 18 sqft minimum, so the clear height backfills to 18/3.5);
    the area cap beats the shape cap (22 x 24 is 528 sqft against a 420
    Living Room cap, so the clear height caps at 420/22).
    """
    a = float(allowance)
    cw, ch = float(rule["min_width"]), float(rule["min_height"])
    min_area = float(rule["min_area"])
    if cw * ch < min_area:
        ch = min_area / cw
    lo_w, lo_h = cw + a, ch + a
    hi_cw, hi_ch = float(rule["max_width"]), float(rule["max_height"])
    max_area = float(rule["max_area"])
    if max_area < _INF and hi_cw * hi_ch > max_area:
        hi_ch = max_area / hi_cw
    hi_w, hi_h = hi_cw + a, hi_ch + a
    return (lo_w, lo_h, lo_w * lo_h), (hi_w, hi_h, hi_w * hi_h)


def _distribute(offered, members, alloc, lo_area, hi_area):
    """Share `offered` sqft among `members` proportionally to their minimum
    rectangle area, each capped at its own ceiling; repeat until the offer
    is exhausted or every member is capped. Returns the undistributed rest.
    Proportional-to-minimum keeps the program's own hierarchy intact: a
    Living Room and a Toilet must not grow by the same absolute amount.
    """
    remaining = float(offered)
    active = [i for i in members if hi_area[i] - alloc[i] > 1e-9]
    while remaining > 1e-9 and active:
        weight = sum(lo_area[i] for i in active)
        if weight <= 0:
            break
        taken = 0.0
        still = []
        for i in active:
            share = remaining * lo_area[i] / weight
            take = min(share, hi_area[i] - alloc[i])
            alloc[i] += take
            taken += take
            if hi_area[i] - alloc[i] > 1e-9:
                still.append(i)
        remaining -= taken
        if taken <= 1e-12:
            break
        active = still
    return max(remaining, 0.0)


def _shape(area, lo_w, lo_h, hi_w, hi_h, ar_lo, ar_hi, plot_ar):
    """Turn an allocated area into a target (w, h) inside the room's
    rectangle bounds and aspect band, following the plot's own aspect.
    A starting point for the tiling, not a constraint; rounded to 0.5 ft
    for readability (dimensions need not be integers on this path).
    """
    target_ar = min(max(plot_ar, ar_lo), ar_hi)
    w = min(max(math.sqrt(max(area, 1e-9) * target_ar), lo_w), hi_w)
    h = min(max(area / w, lo_h), hi_h)
    w = min(max(area / h, lo_w), hi_w)
    w = max(round(w * 2) / 2.0, MIN_SPAN_FT)
    h = max(round(h * 2) / 2.0, MIN_SPAN_FT)
    return w, h


def _override(value):
    if value is None or value == "none":
        return None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if 0 < v < 99999 else None


def allocate(rooms, plot_width, plot_height,
             wall_allowance_ft=None, tier_share=None):
    """The allocator. `rooms` = [{"name", optional per-room clear-feet
    overrides min_width/min_height/max_width/max_height}, ...].

    Returns the allocation dict (see documentation/allocate_api.md).
    Raises ValueError with every problem joined by "; " - a user maximum
    below the NBC minimum is an error, not a silent clamp.
    """
    problems = []
    try:
        plot_w = float(plot_width)
        plot_h = float(plot_height)
    except (TypeError, ValueError):
        plot_w = plot_h = 0.0
    if not (plot_w > 0 and plot_h > 0):
        problems.append("plot_width and plot_height must be positive numbers")
    if not isinstance(rooms, list) or not rooms:
        problems.append("rooms must be a non-empty list")
        rooms = []
    allowance = WALL_ALLOWANCE_FT if wall_allowance_ft is None \
        else float(wall_allowance_ft)
    if not (0 <= allowance < 5):
        problems.append("wall_allowance_ft must be in [0, 5)")
    shares = DEFAULT_TIER_SHARE if tier_share is None else tuple(tier_share)
    if len(shares) != 3 or any(not (0 <= float(s) <= 1) for s in shares):
        problems.append("tier_share must be three fractions in [0, 1]")
        shares = DEFAULT_TIER_SHARE

    names = []
    canonicals = []
    tiers = []
    lo_w = []
    lo_h = []
    lo_area = []
    hi_w = []
    hi_h = []
    hi_area = []
    ar_lo = []
    ar_hi = []
    known = []
    priorities = []
    explicit_priorities = any(isinstance(r, dict) and "priority" in r for r in rooms)
    for idx, room in enumerate(rooms):
        if not isinstance(room, dict):
            problems.append("room %d must be an object" % idx)
            continue
        name = str(room.get("name") or "").strip()
        priority = room.get("priority", 1)
        if isinstance(priority, bool) or not isinstance(priority, (int, float)) or not math.isfinite(priority) or priority < 0:
            problems.append("%s: priority must be a finite nonnegative number" % (name or idx))
            priority = 1
        priorities.append(priority)
        if not name:
            problems.append("room %d has no name" % idx)
            name = "Room %d" % idx
        canonical = nbc_rules.canonical_name(name)
        rule = nbc_rules.rule_or_fallback(name)
        is_known = nbc_rules.rule_for(name) is not None
        (lw, lh, la), (hw, hh, ha) = rect_bounds(rule, allowance)

        umw = _override(room.get("min_width"))
        umh = _override(room.get("min_height"))
        uxw = _override(room.get("max_width"))
        uxh = _override(room.get("max_height"))
        if uxw is not None and uxw < rule["min_width"]:
            problems.append("%s: max_width %g is below the %g ft minimum"
                            % (name, uxw, rule["min_width"]))
        if uxh is not None and uxh < rule["min_height"]:
            problems.append("%s: max_height %g is below the %g ft minimum"
                            % (name, uxh, rule["min_height"]))
        if umw is not None and uxw is not None and uxw < umw:
            problems.append("%s: max_width %g is below its own minimum %g"
                            % (name, uxw, umw))
        if umh is not None and uxh is not None and uxh < umh:
            problems.append("%s: max_height %g is below its own minimum %g"
                            % (name, uxh, umh))
        if umw is not None:
            lw = max(lw, umw + allowance)
        if umh is not None:
            lh = max(lh, umh + allowance)
        if uxw is not None:
            hw = min(hw, uxw + allowance)
        if uxh is not None:
            hh = min(hh, uxh + allowance)
        hw = max(hw, lw)
        hh = max(hh, lh)
        la = lw * lh
        ha = max(hw * hh, la)
        for key in ("min_area", "max_area"):
            if key in room and (isinstance(room[key], bool) or not isinstance(room[key], (int, float)) or not math.isfinite(room[key]) or room[key] <= 0):
                problems.append("%s: %s must be a finite positive number" % (name, key))
        requested_min = _override(room.get("min_area"))
        requested_max = _override(room.get("max_area"))
        if requested_min is not None:
            la = max(la, requested_min)
        if requested_max is not None:
            ha = min(ha, requested_max)
        if ha < la:
            problems.append("%s: max_area %g is below the required minimum %g" % (name, ha, la))

        band_hi = float(rule["max_aspect"])
        names.append(name)
        canonicals.append(canonical)
        tiers.append(_tier_of(canonical))
        lo_w.append(lw)
        lo_h.append(lh)
        lo_area.append(la)
        hi_w.append(hw)
        hi_h.append(hh)
        hi_area.append(ha)
        ar_lo.append(1.0 / band_hi)
        ar_hi.append(band_hi)
        known.append(is_known)

    if problems:
        raise ValueError("; ".join(problems))

    plot_area = plot_w * plot_h
    plot_ar = plot_w / plot_h
    floor_total = sum(lo_area)
    cap_total = sum(hi_area)
    surplus = plot_area - floor_total
    fits = surplus >= 0

    alloc = list(lo_area)
    remainder = 0.0
    if fits:
        reaching = surplus
        if explicit_priorities:
            # Higher numbers receive surplus first. Within one rank retain
            # the room-type tiers and floor-proportional water filling.
            for priority in sorted(set(priorities), reverse=True):
                for tier in (1, 2, 3):
                    members = [i for i in range(len(alloc)) if priorities[i] == priority and tiers[i] == tier]
                    reaching = _distribute(reaching, members, alloc, lo_area, hi_area)
        for tier_index, share in enumerate((float(s) for s in shares),
                                           start=1):
            if reaching <= 1e-9:
                break
            members = [i for i in range(len(alloc))
                       if tiers[i] == tier_index]
            if not members:
                continue
            offered = reaching * share
            left = _distribute(offered, members, alloc, lo_area, hi_area)
            reaching -= (offered - left)
        # Keep the plot full whenever the program has capacity: whatever the
        # tier shares declined goes to ANY room with headroom, floors-
        # proportional, before it is declared unallocatable.
        if reaching > 1e-9:
            reaching = _distribute(reaching, list(range(len(alloc))),
                                   alloc, lo_area, hi_area)
        remainder = reaching

    targets = [
        _shape(alloc[i], lo_w[i], lo_h[i], hi_w[i], hi_h[i],
               ar_lo[i], ar_hi[i], plot_ar)
        for i in range(len(alloc))
    ]
    for i, room in enumerate(rooms):
        if explicit_priorities or "min_area" in room or "max_area" in room:
            # Half-foot display rounding must not defeat an explicit area cap
            # or floor. Preserve the allocated occupied area exactly.
            area = alloc[i]
            lower = max(lo_w[i], area / hi_h[i], math.sqrt(area * ar_lo[i]))
            upper = min(hi_w[i], area / lo_h[i], math.sqrt(area * ar_hi[i]))
            if lower > upper + 1e-6:
                raise ValueError("%s: area and dimension/aspect bounds cannot fit together" % names[i])
            tw = min(max(targets[i][0], lower), upper)
            targets[i] = (tw, area / tw)

    total_alloc = sum(alloc)
    if total_alloc < plot_area - 1e-6:
        # The program cannot legally fill this plot: build at the allocated
        # (capped) total, in the plot's own aspect, and let the client
        # centre it (change C). The unit keeps the plot.
        bw = min(plot_w, math.sqrt(total_alloc * plot_ar))
        bh = min(plot_h, total_alloc / bw)
        bw = total_alloc / bh
        target_bbox = {"width": round(bw, 2), "height": round(bh, 2)}
    elif fits:
        target_bbox = {"width": round(plot_w, 2), "height": round(plot_h, 2)}
    else:
        sw = math.sqrt(floor_total * plot_ar)
        sh = floor_total / sw
        target_bbox = {"width": round(sw, 2), "height": round(sh, 2)}

    result = {
        "fits": fits,
        "plot": {"width": plot_w, "height": plot_h,
                 "area": round(plot_area, 2)},
        "wall_allowance_ft": allowance,
        "tier_share": [float(s) for s in shares],
        "program_min_area": round(floor_total, 2),
        "program_max_area": round(cap_total, 2),
        "target_bbox": target_bbox,
        "rooms": [],
    }
    if fits:
        result["surplus"] = round(surplus, 2)
        result["remainder"] = round(remainder, 2)
    else:
        sw = math.sqrt(floor_total * plot_ar)
        sh = floor_total / sw
        result["shortfall_sqft"] = round(floor_total - plot_area, 2)
        result["smallest_plot"] = {"width": round(sw, 1),
                                   "height": round(sh, 1)}
        drivers = sorted(range(len(alloc)), key=lambda i: -lo_area[i])[:3]
        result["drivers"] = [
            {"room": names[i], "min_area": round(lo_area[i], 1)}
            for i in drivers
        ]

    for i in range(len(alloc)):
        tw, th = targets[i]
        result["rooms"].append({
            "name": names[i],
            "canonical": canonicals[i],
            "known": known[i],
            "tier": tiers[i],
            "priority": priorities[i],
            # everything below is RECTANGLE feet / sqft (walls included)
            "min_width": round(lo_w[i], 2),
            "min_height": round(lo_h[i], 2),
            "max_width": round(hi_w[i], 2),
            "max_height": round(hi_h[i], 2),
            "min_area": round(lo_area[i], 2),
            "max_area": round(hi_area[i], 2),
            "ar_lo": round(ar_lo[i], 4),
            "ar_hi": round(ar_hi[i], 4),
            "allocated_area": round(alloc[i], 2),
            "target_width": tw,
            "target_height": th,
            "at_cap": hi_area[i] - alloc[i] <= 1e-6,
            "clear_area": round(max(tw - allowance, 0)
                                * max(th - allowance, 0), 2),
        })
    return result
