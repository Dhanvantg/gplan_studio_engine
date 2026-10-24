"""Hard requested bounds on occupied polygons, independent of the solver.

Strict callers may receive fewer plans. A failed repair never buys a plan by
releasing a room ceiling or a required connection.
"""

import copy
import math

from shapely.geometry import Polygon, box
from shapely.ops import unary_union

EPS = 1e-4


def unique_violations(failures, limit=100):
    seen, result = set(), []
    for failure in failures:
        key = repr(sorted(failure.items()))
        if key not in seen:
            seen.add(key)
            result.append(failure)
        if len(result) >= limit:
            break
    return result


def _number(value, default, open_sentinel=True):
    if value is None or value == "none":
        return default
    value = float(value)
    return value if math.isfinite(value) and value > 0 and not (open_sentinel and value == 99999) else default


def bounds_for(node):
    width, height = node.get("width") or {}, node.get("height") or {}
    ratio = node.get("ratio") or {}
    return {
        "min_width": _number(width.get("min"), 0),
        "max_width": _number(width.get("max"), math.inf),
        "min_height": _number(height.get("min"), 0),
        "max_height": _number(height.get("max"), math.inf),
        "min_area": _number(node.get("min_area"), 0, False),
        "max_area": _number(node.get("max_area"), math.inf, False),
        # Legacy min_ratio=3 is a sentinel, not a meaningful w/h floor.
        "max_aspect": _number(ratio.get("max"), math.inf),
    }


def measures(poly):
    x0, y0, x1, y1 = poly.bounds
    w, h = x1 - x0, y1 - y0
    return {"width": w, "height": h, "area": poly.area,
            "aspect": max(w, h) / max(min(w, h), EPS)}


def issues(poly, node):
    b, m = bounds_for(node), measures(poly)
    result = []
    for key, metric in (("min_width", "width"), ("max_width", "width"),
                        ("min_height", "height"), ("max_height", "height"),
                        ("min_area", "area"), ("max_area", "area"),
                        ("max_aspect", "aspect")):
        value, limit = m[metric], b[key]
        if (value < limit - EPS if key.startswith("min_") else value > limit + EPS):
            result.append({"room_id": node.get("id"), "label": node.get("label"),
                           "bound": key, "requested": limit, "actual": round(value, 5)})
    return result


def _connections(polys, nodes, edges, door_overlap):
    index = {n.get("id", i): i for i, n in enumerate(nodes)}
    errors = []
    for edge in edges or []:
        if isinstance(edge, dict):
            a, b, color = edge.get("source"), edge.get("target"), edge.get("color", "black")
        else:
            a, b = edge[:2]
            color = edge[2] if len(edge) > 2 else "black"
        if color != "black" or a not in index or b not in index:
            continue
        shared = polys[index[a]].boundary.intersection(polys[index[b]].boundary)
        # Disjoint wall fragments cannot hold one door.
        from shapely.ops import linemerge
        if shared.geom_type in ("MultiLineString", "GeometryCollection"):
            lines = [g for g in shared.geoms if g.geom_type == "LineString"]
            shared = linemerge(lines) if lines else shared
        length = max((g.length for g in shared.geoms), default=0) if hasattr(shared, "geoms") else shared.length
        if length < door_overlap - EPS:
            errors.append({"room_ids": [a, b], "bound": "door_overlap",
                           "requested": door_overlap, "actual": round(length, 5)})
    return errors


def validate(polygons, nodes, edges=None, door_overlap=2.8):
    if len(polygons) != len(nodes):
        return [{"bound": "room_count", "requested": len(nodes), "actual": len(polygons)}]
    polys = [p if isinstance(p, Polygon) else Polygon(p) for p in polygons]
    if any(p.is_empty or not p.is_valid or p.area <= EPS or p.interiors for p in polys):
        return [{"bound": "valid_connected_room_polygon"}]
    result = [issue for p, n in zip(polys, nodes) for issue in issues(p, n)]
    for i, p in enumerate(polys):
        for j in range(i):
            if p.intersection(polys[j]).area > EPS:
                result.append({"room_ids": [nodes[j].get("id"), nodes[i].get("id")], "bound": "overlap"})
    union = unary_union(polys)
    if any(p.interiors for p in (union.geoms if hasattr(union, "geoms") else [union]) if isinstance(p, Polygon)):
        result.append({"bound": "interior_hole"})
    return result + _connections(polys, nodes, edges, door_overlap)


def repair(polygons, nodes, edges=None, door_overlap=2.8):
    """Try bounded exterior clips, measuring L rooms by occupied area.

    Each candidate must keep floors, all required wall lengths and the plan's
    topology. No valid candidate means a precise failure, never an uncapped retry.
    """
    work = [Polygon(p) for p in polygons]
    initial = validate(work, nodes, edges, door_overlap)
    if not initial:
        return [list(p.exterior.coords)[:-1] for p in work], []
    for _ in range(12):
        changed = False
        for i, (p, n) in enumerate(zip(work, nodes)):
            before = issues(p, n)
            if not before:
                continue
            x0, y0, x1, y1 = p.bounds
            w, h = x1 - x0, y1 - y0
            b = bounds_for(n)
            lengths = [(min(w, b["max_width"]), h), (w, min(h, b["max_height"]))]
            if math.isfinite(b["max_area"]) and p.area > b["max_area"]:
                # Clip positions are found by occupied area, not bbox area.
                for axis in (0, 1):
                    for reverse in (False, True):
                        low, high = 0., (w if axis == 0 else h)
                        for _ in range(35):
                            mid = (low + high) / 2
                            cut = box(x1-mid if reverse and axis == 0 else x0,
                                      y1-mid if reverse and axis == 1 else y0,
                                      x1 if axis != 0 or reverse else x0+mid,
                                      y1 if axis != 1 or reverse else y0+mid)
                            if p.intersection(cut).area <= b["max_area"]:
                                low = mid
                            else:
                                high = mid
                        lengths.append((low, h) if axis == 0 else (w, low))
            if math.isfinite(b["max_aspect"]):
                lengths.append((min(w, h*b["max_aspect"]), min(h, w*b["max_aspect"])))
            candidates = []
            for cw, ch in lengths:
                for right in (False, True):
                    for top in (False, True):
                        candidate = p.intersection(box(x1-cw if right else x0, y1-ch if top else y0,
                                                       x1 if right else x0+cw, y1 if top else y0+ch))
                        if not isinstance(candidate, Polygon) or candidate.area <= EPS or candidate.equals(p):
                            continue
                        trial = list(work)
                        trial[i] = candidate
                        failures = validate(trial, nodes, edges, door_overlap)
                        # Other rooms may still need repair; geometry and floors
                        # never relax while we work through those offenders.
                        if any(f["bound"].startswith("min_") or f["bound"] in {"door_overlap", "overlap", "interior_hole", "valid_connected_room_polygon"} for f in failures):
                            continue
                        if len(issues(candidate, n)) < len(before):
                            candidates.append(candidate)
            if candidates:
                work[i] = max(candidates, key=lambda q: q.area)
                changed = True
        if not changed:
            break
        remaining = validate(work, nodes, edges, door_overlap)
        if not remaining:
            return [list(p.exterior.coords)[:-1] for p in work], []
    return None, validate(work, nodes, edges, door_overlap)


def constrain_plan(plan, nodes, edges, strict=False):
    """Return a detached plan and its final per-room violations."""
    # Graph plans retain the whole topology. Only the geometry attributes are
    # replaced below; copying that topology for every best-effort plan is wasteful.
    candidate = copy.copy(plan) if strict else plan
    failures = validate(candidate.final_traversal, nodes, edges if strict else None)
    if strict and failures:
        polygons, failures = repair(candidate.final_traversal, nodes, edges)
        if polygons is None:
            return None, failures
        candidate.final_traversal = polygons
        polys = [Polygon(p) for p in polygons]
        candidate.area = [p.area for p in polys]
        candidate.room_width = [p.bounds[2]-p.bounds[0] for p in polys]
        candidate.room_height = [p.bounds[3]-p.bounds[1] for p in polys]
        candidate.name_coords = [[p.representative_point().x, p.representative_point().y] for p in polys]
    return candidate, failures
