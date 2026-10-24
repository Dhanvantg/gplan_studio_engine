"""API

Generates floorplamns for given graph data as input.
Current support only for rectangular floorplans.
A running example is available.

"""
from GPLAN.source.inputgraph import InputGraph
import GPLAN.pythongui.gui as gui
import copy
import math
import time
import uuid

from GPLAN.handlers import *
from GPLAN.pythongui.GuiParameters import GuiParameters, DimParameters
import builtins

# Import boundary utilities from Space_Optimization folder
import sys
import os
space_opt_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Space_Optimization')
if space_opt_path not in sys.path:
    sys.path.insert(0, space_opt_path)
from boundary_utils import boundary_to_regions as convert_boundary_to_regions


def boundary_to_regions(boundary, fixed_rooms=None):
    """
    Convert a polygonal boundary to a list of rectangular regions.
    
    Uses the same algorithm as the UI's CAD tool (decompose_into_rectangles).
    Creates a grid-based decomposition with cell merging for optimal region count.
    
    Args:
        boundary: List of [x, y] coordinate pairs defining the polygon vertices
        fixed_rooms: Optional list of fixed room polygons to exclude during decomposition
        
    Returns:
        List of region dictionaries with x, y, width, height
        
    Example:
        boundary = [[0, 0], [10, 0], [10, 5], [5, 5], [5, 10], [0, 10]]
        regions = boundary_to_regions(boundary)
        # Returns L-shape as merged rectangles
    """
    if not boundary or len(boundary) < 3:
        raise ValueError("Boundary must have at least 3 points")
    
    # Use the shared utility function (same as UI)
    return convert_boundary_to_regions(boundary, fixed_rooms=fixed_rooms)


class Asset:
    def __init__(self, _id, properties, asset_type):
        self._id = _id
        self.properties = properties
        self.type = asset_type

    def to_dict(self):
        return {
            "_id": self._id,
            "properties": self.properties,
            "type": self.type
        }


class Wall:
    def __init__(self, _id, x1, y1, x2, y2, assets=None):
        self._id = _id
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.assets = []
        if assets is not None:
            self.assets = [asset.to_dict() for asset in assets]

    def to_dict(self):
        return {
            "_id": self._id,
            "x1": self.x1,
            "y1": self.y1,
            "x2": self.x2,
            "y2": self.y2,
            "assets": self.assets
        }


class Room:
    def __init__(self, _id, name, color, walls=None, assets=None, circular_coordinates=None,name_coord=None,area = None,width = None,height = None):
        self._id = _id
        self.name = name
        self.name_coord = name_coord
        self.area = area
        self.width = width
        self.height = height
        self.assets = []
        if assets is not None:
            self.assets = [asset.to_dict() for asset in assets]
        self.color = color
        if walls is not None:
            self.walls = [wall.to_dict() for wall in walls]
        self.circular_coordinates = circular_coordinates

    def to_dict(self):
        return {
            "_id": self._id,
            "name": self.name,
            "label_coord": self.name_coord,
            "area" : self.area,
            "width" : self.width,
            "height" : self.height,
            "assets": self.assets,
            "color": self.color,
            "walls": self.walls,
            "circular_coordinates": self.circular_coordinates
        }


FLOORPLAN_LIMIT = 30

# Cardinal directions map to boundary-path indices in the 4-completion
# (news.add_news order): N -> paths[0], E -> paths[1], S -> paths[2], W -> paths[3].
CARDINAL_DIR_INDEX = {"N": 0, "E": 1, "S": 2, "W": 3,
                      "NORTH": 0, "EAST": 1, "SOUTH": 2, "WEST": 3}

# A fixed room may be unanchored (exact size only), pinned to one side, or
# pinned to a corner.  Housing defaults its mandatory stair to SW.
# Anchors are direct bbox contacts, stronger than the ordinary cardinal
# "unblocked strip" preference.
FIXED_ANCHOR_DIRECTIONS = {
    "N": (0,), "NE": (0, 1), "E": (1,), "SE": (1, 2),
    "S": (2,), "SW": (2, 3), "W": (3,), "NW": (0, 3),
}
FIXED_ANCHOR_ALIASES = {
    "NORTH": "N", "NORTHEAST": "NE", "EAST": "E",
    "SOUTHEAST": "SE", "SOUTH": "S", "SOUTHWEST": "SW",
    "WEST": "W", "NORTHWEST": "NW",
}


def _fixed_axis_value(node, axis):
    """Exact positive axis value from a fixed node's min == max band."""
    band = node.get(axis)
    if not isinstance(band, dict):
        raise ValueError("fixed room %r needs a %s {min,max} band"
                         % (node.get("label", node.get("id")), axis))
    try:
        lo = float(band.get("min"))
        hi = float(band.get("max"))
    except (TypeError, ValueError):
        raise ValueError("fixed room %r needs numeric %s min/max"
                         % (node.get("label", node.get("id")), axis))
    if lo <= 0 or hi <= 0 or abs(lo - hi) > 1e-6:
        raise ValueError("fixed room %r needs equal positive %s min/max"
                         % (node.get("label", node.get("id")), axis))
    return lo


def normalize_fixed_rooms(nodes_list, dim_inputs, min_dim_enabled):
    """Normalize ``is_fixed`` door-connectivity nodes and harden dimensions.

    Wire contract (backward compatible; ordinary nodes are untouched)::

        {"id": 7, "label": "Staircase", "is_fixed": true,
         "fixed_anchor": "SW",
         "width": {"min": 7, "max": 7},
         "height": {"min": 10, "max": 10}}

    The production view builds ``dim_inputs`` before the engine sees the
    request and older deployments may open every maximum to 99999.  Rewriting
    the four aligned columns here makes the fixed-node contract authoritative
    at the engine boundary.  Area is equality-constrained too so gap filling
    and NBC post-processing see the same lock.
    """
    fixed = []
    for index, node in enumerate(nodes_list or []):
        if not isinstance(node, dict) or not node.get("is_fixed"):
            continue
        if not min_dim_enabled:
            raise ValueError("fixed rooms require minDimEnabled")
        width = _fixed_axis_value(node, "width")
        height = _fixed_axis_value(node, "height")
        raw_anchor = node.get("fixed_anchor")
        anchor = None
        directions = ()
        if raw_anchor is not None and str(raw_anchor).strip():
            key = str(raw_anchor).upper().replace("-", "").replace("_", "").replace(" ", "")
            anchor = FIXED_ANCHOR_ALIASES.get(key, key)
            if anchor not in FIXED_ANCHOR_DIRECTIONS:
                raise ValueError("fixed room %r has unsupported anchor %r"
                                 % (node.get("label", node.get("id")), raw_anchor))
            directions = FIXED_ANCHOR_DIRECTIONS[anchor]
        fixed.append({
            "room": index,
            "width": width,
            "height": height,
            "anchor": anchor,
            "directions": directions,
        })

        # N/W normalize to the engine origin. S/E need the real plot extent as
        # well; touching a smaller generated bbox is not enough to align a
        # physical core drawn at the target's south/east boundary.
        if 1 in directions:
            try:
                plot_width = float(dim_inputs.get("plot_width") or 0)
            except (TypeError, ValueError):
                plot_width = 0
            if plot_width > 0:
                fixed[-1]["right"] = plot_width
        if 2 in directions:
            try:
                plot_height = float(dim_inputs.get("plot_height") or 0)
            except (TypeError, ValueError):
                plot_height = 0
            if plot_height > 0:
                fixed[-1]["bottom"] = plot_height

    if not fixed:
        return []

    # Copy columns before mutation: get_floorplans historically receives a
    # caller-owned dict and some tests reuse it across requests.
    count = len(nodes_list)
    for key, default in (("min_width", 0.0), ("max_width", 99999.0),
                         ("min_height", 0.0), ("max_height", 99999.0),
                         ("min_area", 0.0), ("max_area", 0.0)):
        values = list(dim_inputs.get(key) or [])
        if len(values) < count:
            values.extend([default] * (count - len(values)))
        dim_inputs[key] = values
    for spec in fixed:
        i = spec["room"]
        dim_inputs["min_width"][i] = spec["width"]
        dim_inputs["max_width"][i] = spec["width"]
        dim_inputs["min_height"][i] = spec["height"]
        dim_inputs["max_height"][i] = spec["height"]
        area = spec["width"] * spec["height"]
        dim_inputs["min_area"][i] = area
        dim_inputs["max_area"][i] = area
    return fixed


def fixed_cardinal_pairs(fixed_rooms):
    return [(spec["room"], direction)
            for spec in fixed_rooms for direction in spec.get("directions", ())]


def merge_cardinal_pairs(*groups):
    merged = []
    for group in groups:
        for pair in group or []:
            if pair not in merged:
                merged.append(pair)
    return merged


def normalize_cardinal_constraints(cardinal_constraints, nodecnt):
    """Normalizes user cardinal constraints into engine (node, dir_idx) pairs.

    Accepts entries either as {"room": <node id>, "direction": "N"|"E"|"S"|"W"}
    or as [node, "N"] pairs; the spelled-out names are accepted too. Invalid
    rooms/directions are dropped.

    The direction is matched WHOLE, not by first letter. This used to key on
    ``str(direction).upper()[:1]``, which silently turned any typo starting with
    one of those letters into a real pin: "sideways" pinned a room SOUTH. Dropping
    the entry instead means the caller sees "no pins applied" (and the Django
    endpoint rejects the payload outright), rather than a plan confidently facing
    a direction nobody asked for.
    """
    pairs = []
    for entry in cardinal_constraints or []:
        if isinstance(entry, dict):
            room = entry.get("room")
            direction = entry.get("direction")
        else:
            try:
                room, direction = entry[0], entry[1]
            except (TypeError, IndexError):
                continue
        if not isinstance(room, int) or room < 0 or room >= nodecnt:
            continue
        dir_idx = CARDINAL_DIR_INDEX.get(str(direction).strip().upper())
        if dir_idx is None:
            continue
        pair = (room, dir_idx)
        if pair not in pairs:
            pairs.append(pair)
    return pairs


def _ring_order_satisfies(order, pins):
    """True when the cyclic node order admits a 4-arc split (N,E,S,W, up to
    rotation and reflection) with every pinned node inside its direction's arc
    and every two-direction (corner) pin exactly at its two arcs' interface.

    This mirrors what filter_boundaries_by_cardinal can accept downstream:
    boundary paths are contiguous arcs of the outer cycle in N,E,S,W order
    (any rotation, either orientation), sharing only corner nodes.
    """
    if not pins:
        return True
    for seq in (order, list(reversed(order))):
        dirsets = [pins[n] for n in seq if n in pins]
        cnt = len(dirsets)
        for r in range(cnt):
            base = None
            prog = 0
            ok = True
            for ds in (dirsets[r:] + dirsets[:r]):
                if len(ds) == 1:
                    d = next(iter(ds))
                    if base is None:
                        base = d
                        continue
                    delta = (d - base) % 4
                    if delta < prog:
                        ok = False
                        break
                    prog = delta
                else:
                    d1, d2 = tuple(ds)
                    if (d2 - d1) % 4 != 1:
                        d1, d2 = d2, d1
                    if base is None:
                        base = d1
                        prog = 1
                        continue
                    delta1 = (d1 - base) % 4
                    delta2 = (d2 - base) % 4
                    if delta1 < prog or delta2 != delta1 + 1:
                        ok = False
                        break
                    prog = delta2
            if ok:
                return True
    return False


def _ring_orders_exhaustive(nodecnt, adjset, edges_list, pins,
                            max_orders=48, max_steps=60000):
    """Every cyclic room order (node 0 first, one orientation per cycle) whose
    requested edges are non-crossing chords and whose pins admit an N,E,S,W
    arc split. Backtracking with incremental crossing pruning; bounded by
    `max_steps` so a dense brief degrades to the DFS family below instead of
    stalling. Small briefs (8-12 rooms) are exhausted in milliseconds.

    The DFS-preorder family alone missed both valid orders of the shipped
    2BHK brief (12 edges), so a fixed stair ran with no ring at all and its
    boundary neighbours fell to the biconnectivity augmentation, which put a
    bedroom in the stair's 10 ft row on every topology.
    """
    if nodecnt < 3 or nodecnt > 12:
        return []
    edges = [(int(e[0]), int(e[1])) for e in edges_list if int(e[0]) != int(e[1])]
    found = []
    order = [0]
    pos = {0: 0}
    steps = [0]

    def chords():
        out = []
        for a, b in edges:
            if a in pos and b in pos:
                lo, hi = (pos[a], pos[b]) if pos[a] < pos[b] else (pos[b], pos[a])
                if hi - lo > 1 and not (lo == 0 and hi == nodecnt - 1):
                    out.append((lo, hi))
        return out

    def new_chord_crosses(li):
        all_chords = chords()
        for lo, hi in all_chords:
            if hi != li:
                continue
            for a, b in all_chords:
                if (a, b) == (lo, hi):
                    continue
                if len({a, b, lo, hi}) == 4 and (a < lo < b < hi or lo < a < hi < b):
                    return True
        return False

    def rec():
        if len(found) >= max_orders or steps[0] > max_steps:
            return
        steps[0] += 1
        if len(order) == nodecnt:
            if _ring_order_satisfies(order, pins):
                rev = [0] + order[1:][::-1]
                if tuple(order) <= tuple(rev):
                    found.append(list(order))
            return
        for v in range(1, nodecnt):
            if v in pos:
                continue
            order.append(v)
            pos[v] = len(order) - 1
            if not new_chord_crosses(pos[v]):
                rec()
            order.pop()
            del pos[v]

    rec()
    return found


def _ring_neighbour_score(order, adjset, nodes_list, fixed_rooms):
    """How well the ring neighbours of each fixed room can share its row or
    column. A fixed room's two ring neighbours become its south/west (or
    east/north) wall companions; at least one must be able to take the fixed
    room's exact height (same row) or width (same column) or no rectangular
    tiling can close around it, and a neighbour the brief already joins to
    the fixed room (its door host) keeps that door on the boundary."""
    if not fixed_rooms or not nodes_list:
        return 0
    total = len(order)
    pos = dict((v, i) for i, v in enumerate(order))

    def band(index, axis):
        node = nodes_list[index] if 0 <= index < len(nodes_list) else None
        spec = node.get(axis) if isinstance(node, dict) else None
        if not isinstance(spec, dict):
            return None
        try:
            return float(spec.get("min")), float(spec.get("max"))
        except (TypeError, ValueError):
            return None

    score = 0
    for spec in fixed_rooms:
        room = spec["room"]
        if room not in pos:
            continue
        neighbours = (order[pos[room] - 1], order[(pos[room] + 1) % total])
        row_ok = col_ok = hosts = 0
        for v in neighbours:
            hb = band(v, "height")
            wb = band(v, "width")
            if hb is not None and hb[0] - 1e-6 <= spec["height"] <= hb[1] + 1e-6:
                row_ok += 1
            if wb is not None and wb[0] - 1e-6 <= spec["width"] <= wb[1] + 1e-6:
                col_ok += 1
            if v in adjset.get(room, ()):
                hosts += 1
        score += 3 * min(1, row_ok + col_ok) + row_ok + col_ok + hosts
    return score


def build_cardinal_ring(nodecnt, edges_list, cardinal_pairs, nodes_list=None,
                        fixed_rooms=None):
    """Planar outer-ring augmentation making every cardinal pin boundary-capable.

    A generated floorplan's boundary is the outer face of the planar embedding
    chosen during augmentation. For sparse inputs (the designer's normal case)
    that embedding otherwise comes from nx.planar_layout, which knows nothing
    about the pins - a pinned room can end up interior in EVERY enumerated
    boundary, starving the cardinal filter and forcing the relaxation ladder
    to "ignored" no matter what.

    Builds a Hamiltonian cycle through all rooms - components and subtrees
    ordered so pinned rooms sit contiguously per direction in N,E,S,W cyclic
    order - plus circle coordinates realizing it as a planar straight-line
    drawing (tree edges become non-crossing chords when the cycle follows DFS
    preorder). With the cycle added the graph is biconnected, so the engine
    skips its own augmentation and keeps this embedding: the outer boundary IS
    the ring and the boundary filter can always aim it.

    Returns (ring_edges, coordinates) or None when no valid ring exists
    (opposite-direction pins, ordering conflict, or crossing chords).
    """
    if nodecnt < 3:
        return None
    pins = {}
    for node, dir_idx in cardinal_pairs:
        pins.setdefault(node, set()).add(dir_idx)
    for dirs in pins.values():
        if len(dirs) > 2:
            return None
        if len(dirs) == 2:
            d1, d2 = tuple(dirs)
            if (d2 - d1) % 4 == 2:  # opposite sides can never share a corner
                return None
    adjset = {i: set() for i in range(nodecnt)}
    for edge in edges_list:
        a, b = edge[0], edge[1]
        if a != b:
            adjset[a].add(b)
            adjset[b].add(a)

    # Components, each ordered by DFS preorder (subtrees stay contiguous, so
    # tree edges never cross the circle chords). Children are visited in
    # ascending order of the smallest pin direction in their subtree, which
    # groups same-direction pins together in N,E,S,W order.
    def preorder_from(root, comp_nodes):
        children = {n: [] for n in comp_nodes}
        parent = {root: None}
        stack = [root]
        seen_local = {root}
        dfs_order = []
        while stack:
            cur = stack.pop()
            dfs_order.append(cur)
            for nbr in sorted(adjset[cur]):
                if nbr in comp_nodes and nbr not in seen_local:
                    seen_local.add(nbr)
                    parent[nbr] = cur
                    children[cur].append(nbr)
                    stack.append(nbr)
        subkey = {n: min(pins[n]) if n in pins else 4 for n in comp_nodes}
        for n in reversed(dfs_order):
            for ch in children[n]:
                subkey[n] = min(subkey[n], subkey[ch])
        out = []
        stack = [root]
        while stack:
            cur = stack.pop()
            out.append(cur)
            for ch in sorted(children[cur], key=lambda c: (subkey[c], c),
                             reverse=True):
                stack.append(ch)
        return out

    seen = set()
    comps = []
    for start in range(nodecnt):
        if start in seen:
            continue
        comp = set()
        stack = [start]
        seen.add(start)
        while stack:
            cur = stack.pop()
            comp.add(cur)
            for nbr in adjset[cur]:
                if nbr not in seen:
                    seen.add(nbr)
                    stack.append(nbr)
        comps.append(comp)

    def comp_key(comp):
        keys = [min(pins[n]) for n in comp if n in pins]
        return (min(keys) if keys else 4, min(comp))

    comps.sort(key=comp_key)
    # Per component try every root (components are small); take the first
    # combination whose full cyclic order verifies.
    comp_orders = []
    for comp in comps:
        roots = sorted(comp, key=lambda n: (min(pins[n]) if n in pins else 4, n))
        comp_orders.append([preorder_from(r, comp) for r in roots])

    def crossing_free(order):
        posidx = {n: i for i, n in enumerate(order)}
        chords = []
        for edge in edges_list:
            a, b = posidx[edge[0]], posidx[edge[1]]
            lo, hi = min(a, b), max(a, b)
            if hi - lo > 1 and not (lo == 0 and hi == len(order) - 1):
                chords.append((lo, hi))
        for i in range(len(chords)):
            for j in range(i + 1, len(chords)):
                a, b = chords[i]
                c, d = chords[j]
                if len({a, b, c, d}) == 4 and (a < c < b < d or c < a < d < b):
                    return False
        return True

    def combos(idx, prefix, budget):
        if budget[0] <= 0:
            return None
        if idx == len(comp_orders):
            budget[0] -= 1
            if _ring_order_satisfies(prefix, pins) and crossing_free(prefix):
                return list(prefix)
            return None
        for cand in comp_orders[idx]:
            found = combos(idx + 1, prefix + cand, budget)
            if found is not None:
                return found
        return None

    order = None
    if fixed_rooms:
        # every valid order is known: take the one whose ring neighbours can
        # actually tile around the fixed rooms, ties to the lowest order
        exhaustive = _ring_orders_exhaustive(nodecnt, adjset, edges_list, pins)
        if exhaustive:
            order = min(
                exhaustive,
                key=lambda cand: (-_ring_neighbour_score(cand, adjset, nodes_list, fixed_rooms),
                                  tuple(cand)),
            )
    if order is None:
        # pin-only briefs keep the DFS-preorder ring they always had; the
        # exhaustive search only steps in where that family finds no ring at
        # all (a strict gain: the pinned room used to run unringed)
        order = combos(0, [], [500])
    if order is None:
        exhaustive = _ring_orders_exhaustive(nodecnt, adjset, edges_list, pins)
        if exhaustive:
            order = min(exhaustive, key=tuple)
    if order is None:
        return None
    ring_edges = []
    total = len(order)
    for i in range(total):
        a, b = order[i], order[(i + 1) % total]
        if b not in adjset[a]:
            ring_edges.append((a, b))
    coords = [None] * nodecnt
    for i, node in enumerate(order):
        ang = 2 * math.pi * i / total
        coords[node] = [0.5 + 0.4 * math.cos(ang), 0.5 + 0.4 * math.sin(ang)]
    return ring_edges, coords


def apply_cardinal_ring(graph, nodecnt, edges_list, cardinal_pairs,
                        non_adj_edge_list=None, nodes_list=None,
                        fixed_rooms=None):
    """Adds the cardinal outer ring to graph (matrix + coordinates) in place.

    Ring edges are engine augmentation, not user adjacencies - they are added
    to the matrix only, so no doors are placed on them (ui edges stay the user
    list). Skipped (returns False) when pins are absent, non-adjacency
    constraints exist (a ring edge could force a forbidden adjacency), or no
    valid ring order was found.
    """
    if not cardinal_pairs or non_adj_edge_list:
        return False
    ring = build_cardinal_ring(nodecnt, edges_list, cardinal_pairs,
                               nodes_list=nodes_list, fixed_rooms=fixed_rooms)
    if ring is None:
        return False
    ring_edges, coords = ring
    for a, b in ring_edges:
        if graph.matrix[a][b] != 1:
            graph.matrix[a][b] = 1
            graph.matrix[b][a] = 1
            graph.edgecnt += 1
    graph.coordinates = coords
    return True


def plan_satisfies_cardinal(plan_graph, cardinal_pairs):
    """True when every (node, dir_idx) pin is exterior-facing in the solved plan.

    Checks the geometry that is actually serialized to clients
    (final_traversal, rooms in node order): no other room's bounding box may
    intersect the strip between the pinned room's box and the plan bounds on
    the pinned side. Frame is y-down, dir_idx 0..3 = N,E,S,W (N = smaller y).
    """
    rooms = getattr(plan_graph, "final_traversal", None) or []
    if not rooms:
        return False
    rects = []
    for poly in rooms:
        xs = [pt[0] for pt in poly]
        ys = [pt[1] for pt in poly]
        if not xs:
            return False
        rects.append((min(xs), max(xs), min(ys), max(ys)))
    bx0 = min(r[0] for r in rects)
    bx1 = max(r[1] for r in rects)
    by0 = min(r[2] for r in rects)
    by1 = max(r[3] for r in rects)
    eps = max(bx1 - bx0, by1 - by0, 1e-6) * 1e-4
    for node, dir_idx in cardinal_pairs:
        if node >= len(rects):
            return False
        rx0, rx1, ry0, ry1 = rects[node]
        if dir_idx == 0:
            sx0, sx1, sy0, sy1 = rx0, rx1, by0, ry0
        elif dir_idx == 1:
            sx0, sx1, sy0, sy1 = rx1, bx1, ry0, ry1
        elif dir_idx == 2:
            sx0, sx1, sy0, sy1 = rx0, rx1, ry1, by1
        else:
            sx0, sx1, sy0, sy1 = bx0, rx0, ry0, ry1
        for j, (ox0, ox1, oy0, oy1) in enumerate(rects):
            if j == node:
                continue
            if (min(ox1, sx1) - max(ox0, sx0) > eps and
                    min(oy1, sy1) - max(oy0, sy0) > eps):
                return False
    return True


def plan_satisfies_fixed(plan_graph, fixed_rooms, tolerance=0.05,
                         require_absolute=True):
    """Hard gate for exact fixed-room extents and side/corner contacts.

    Unlike ordinary cardinal constraints, a fixed anchor is not a preference
    and is never interpreted as merely an unobstructed view to a side.  NW
    means west/north contact; S/E anchors are additionally checked against the
    request's authoritative plot height/width, not a shrunken result bbox. The
    small tolerance covers the legacy LP's three-decimal output only; it is
    less than one inch and cannot hide a material stair-size change.
    """
    if not fixed_rooms:
        return True
    rooms = getattr(plan_graph, "final_traversal", None) or []
    if not rooms:
        return False
    rects = []
    for poly in rooms:
        if not poly:
            return False
        xs = [float(pt[0]) for pt in poly]
        ys = [float(pt[1]) for pt in poly]
        rects.append((min(xs), min(ys), max(xs), max(ys)))
    bx0 = min(r[0] for r in rects)
    by0 = min(r[1] for r in rects)
    bx1 = max(r[2] for r in rects)
    by1 = max(r[3] for r in rects)
    for spec in fixed_rooms:
        index = spec["room"]
        if index < 0 or index >= len(rects):
            return False
        x0, y0, x1, y1 = rects[index]
        if abs((x1 - x0) - spec["width"]) > tolerance:
            return False
        if abs((y1 - y0) - spec["height"]) > tolerance:
            return False
        for direction in spec.get("directions", ()):
            side = (y0, x1, y1, x0)[direction]
            outer = (by0, bx1, by1, bx0)[direction]
            if abs(side - outer) > tolerance:
                return False
        # GPlan serializes floorplans in a normalized top-left coordinate
        # system.  Do not accept a translated whole-plan geometry merely
        # because the room still touches the translated bbox: N/W are hard
        # absolute top/left locks for fixed cores.
        if (require_absolute and 0 in spec.get("directions", ())
                and abs(y0) > tolerance):
            return False
        if (require_absolute and 3 in spec.get("directions", ())
                and abs(x0) > tolerance):
            return False
        if (require_absolute and 1 in spec.get("directions", ()) and
                spec.get("right") is not None and
                abs(x1 - spec["right"]) > tolerance):
            return False
        if (require_absolute and 2 in spec.get("directions", ()) and
                spec.get("bottom") is not None and
                abs(y1 - spec["bottom"]) > tolerance):
            return False
    return True


def _shared_wall_length(poly_a, poly_b, eps=0.05):
    """Total collinear overlap between two rectilinear room polygons, feet.

    E2a support (documentation/plans/VALIDITY_AND_TOPOLOGY_ENGINE_PLAN.md):
    the graph promises an adjacency, the dimensioning realises it as SOME
    shared boundary, and nothing below this function ever measured how much.
    A briefed edge realised as a 0 ft corner contact or a 0.3 ft sliver holds
    no door; the serializer uses this to disclose those pairs per plan.
    Polygons are the `final_traversal` point lists (axis-aligned, closed
    implicitly).
    """
    def edges(poly):
        n = len(poly)
        for idx in range(n):
            x1, y1 = float(poly[idx][0]), float(poly[idx][1])
            x2, y2 = float(poly[(idx + 1) % n][0]), float(poly[(idx + 1) % n][1])
            yield x1, y1, x2, y2

    total = 0.0
    for ax1, ay1, ax2, ay2 in edges(poly_a):
        for bx1, by1, bx2, by2 in edges(poly_b):
            if abs(ax1 - ax2) < eps and abs(bx1 - bx2) < eps and abs(ax1 - bx1) < eps:
                lo = max(min(ay1, ay2), min(by1, by2))
                hi = min(max(ay1, ay2), max(by1, by2))
                if hi - lo > eps:
                    total += hi - lo
            elif abs(ay1 - ay2) < eps and abs(by1 - by2) < eps and abs(ay1 - by1) < eps:
                lo = max(min(ax1, ax2), min(bx1, bx2))
                hi = min(max(ax1, ax2), max(bx1, bx2))
                if hi - lo > eps:
                    total += hi - lo
    return total


def filter_output_by_cardinal(ui, cardinal_pairs):
    """Keeps only geometrically satisfying plans in ui's output data.

    Returns the surviving list. The boundary pre-filter aims candidates at the
    right sides, but later stages (dimension-fit selection, optimal-resize
    fallback) can still emit violating plans - this is the hard gate.
    """
    satisfying = [plan for plan in ui.get_output_data()
                  if plan_satisfies_cardinal(plan, cardinal_pairs)]
    ui._set_output_data(satisfying)
    return satisfying


def filter_output_by_fixed(ui, fixed_rooms):
    """Drop every plan that moved, resized, rotated or unanchored a lock."""
    satisfying = [plan for plan in ui.get_output_data()
                  if plan_satisfies_fixed(plan, fixed_rooms)]
    ui._set_output_data(satisfying)
    return satisfying


def _fixed_alignment(plan_graph, fixed_rooms, tolerance=0.05):
    """Translation that puts relative fixed anchors in the plot frame.

    Topology generation knows that an SW stair touches the generated plan's
    south and west sides, but its natural compacted bbox can be smaller than
    the requested plot.  The physical core is at the plot boundary, so move
    the WHOLE dissection (never the fixed room alone) before exact-fill grows
    the free sides.  Multiple locks must prescribe the same translation.
    """
    if not plan_satisfies_fixed(plan_graph, fixed_rooms, tolerance,
                                require_absolute=False):
        return None
    rooms = getattr(plan_graph, "final_traversal", None) or []
    rects = []
    for poly in rooms:
        xs = [float(pt[0]) for pt in poly]
        ys = [float(pt[1]) for pt in poly]
        rects.append((min(xs), min(ys), max(xs), max(ys)))

    dx_values = []
    dy_values = []
    for spec in fixed_rooms:
        x0, y0, x1, y1 = rects[spec["room"]]
        directions = spec.get("directions", ())
        if 3 in directions:
            dx_values.append(-x0)
        if 1 in directions and spec.get("right") is not None:
            dx_values.append(float(spec["right"]) - x1)
        if 0 in directions:
            dy_values.append(-y0)
        if 2 in directions and spec.get("bottom") is not None:
            dy_values.append(float(spec["bottom"]) - y1)

    def one_axis(values):
        if not values:
            return 0.0
        if max(values) - min(values) > tolerance:
            return None
        return sum(values) / len(values)

    dx = one_axis(dx_values)
    dy = one_axis(dy_values)
    return None if dx is None or dy is None else (dx, dy)


def align_output_by_fixed(ui, fixed_rooms, tolerance=0.05):
    """Place fixed candidates in the absolute plot frame without scaling."""
    if not fixed_rooms:
        return ui.get_output_data()
    kept = []
    for plan in ui.get_output_data():
        shift = _fixed_alignment(plan, fixed_rooms, tolerance)
        if shift is None:
            continue
        dx, dy = shift
        if abs(dx) > 1e-9 or abs(dy) > 1e-9:
            plan.final_traversal = [
                [(float(x) + dx, float(y) + dy) for x, y in poly]
                for poly in plan.final_traversal
            ]
            if getattr(plan, "room_x", None):
                plan.room_x = [float(x) + dx for x in plan.room_x]
            if getattr(plan, "room_y", None):
                plan.room_y = [float(y) + dy for y in plan.room_y]
            if getattr(plan, "name_coords", None):
                plan.name_coords = [
                    [float(point[0]) + dx, float(point[1]) + dy]
                    for point in plan.name_coords
                ]
        if plan_satisfies_fixed(plan, fixed_rooms, tolerance):
            kept.append(plan)
    ui._set_output_data(kept)
    return kept


def plan_matches_plot_envelope(plan_graph, plot_width, plot_height,
                               tolerance=0.05):
    """True only for the normalized [0,width] x [0,height] plot frame."""
    rooms = getattr(plan_graph, "final_traversal", None) or []
    if not rooms:
        return False
    xs = [float(pt[0]) for poly in rooms for pt in poly]
    ys = [float(pt[1]) for poly in rooms for pt in poly]
    return (abs(min(xs)) <= tolerance and abs(min(ys)) <= tolerance
            and abs(max(xs) - float(plot_width)) <= tolerance
            and abs(max(ys) - float(plot_height)) <= tolerance)


def plan_fills_plot_exactly(plan_graph, plot_width, plot_height,
                            tolerance=0.05):
    """True only when rectangular rooms tile the requested plot frame.

    Matching the outer bbox alone misses the user-visible "Empty space" case:
    rooms can touch all four plot edges while retaining a notch or interior
    hole.  This gate also rejects overlaps so summed room area is a valid union
    area, then requires that union to cover the complete envelope.
    """
    if not plan_matches_plot_envelope(
            plan_graph, plot_width, plot_height, tolerance):
        return False
    rooms = getattr(plan_graph, "final_traversal", None) or []
    rects = []
    for poly in rooms:
        rect = _polyline_rect(poly, tolerance)
        if rect is None:
            return False
        rects.append(rect)
    eps = max(float(plot_width), float(plot_height), 1e-6) * 1e-5
    return not _rects_overlap(rects, eps) and _is_gapless(rects, eps)


def _polyline_rect(poly, eps):
    """(x0, y0, x1, y1) if the room polyline is an axis-aligned rectangle
    (shoelace area == bbox area), else None."""
    xs = [pt[0] for pt in poly]
    ys = [pt[1] for pt in poly]
    if not xs:
        return None
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    area2 = 0.0
    n = len(poly)
    for i in range(n):
        ax, ay = poly[i][0], poly[i][1]
        bx, by = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        area2 += ax * by - bx * ay
    if abs(abs(area2) / 2 - (x1 - x0) * (y1 - y0)) > eps:
        return None
    return (x0, y0, x1, y1)


_INF = float("inf")


def _room_bounds(ui, count):
    """Per-room {minw, minh, maxw, maxh, minarea, maxarea, aspect} or None.

    Everything here is already carried by the request: min/max width and height
    per node, and the ratio band that min-dim itself ignores. Area bounds are
    the products, the aspect cap is max_ratio.
    """
    params = getattr(ui, "min_dim_inputs", None)
    if params is None:
        return None

    def col(getter, default):
        try:
            values = getter() or []
        except Exception:
            return []
        out = []
        for v in values:
            try:
                f = float(v)
            except (TypeError, ValueError):
                f = default
            out.append(f)
        return out

    minw = col(params.get_min_width, 0.0)
    minh = col(params.get_min_height, 0.0)
    maxw = col(params.get_max_width, 99999.0)
    maxh = col(params.get_max_height, 99999.0)
    minr = col(params.get_min_aspect_ratio, 0.0)
    maxr = col(params.get_max_aspect_ratio, 0.0)
    mina = col(lambda: params.get_min_area() if hasattr(params, "get_min_area") else [], 0.0)
    maxa = col(lambda: params.get_max_area() if hasattr(params, "get_max_area") else [], 0.0)
    if not minw or not minh:
        return None

    def at(arr, i, default):
        if i >= len(arr):
            return default
        v = arr[i]
        return default if v is None else v

    bounds = []
    for i in range(count):
        lw, lh = at(minw, i, 0.0), at(minh, i, 0.0)
        hw, hh = at(maxw, i, 99999.0), at(maxh, i, 99999.0)
        if lw <= 0 or lh <= 0:
            bounds.append(None)           # dummy room, leave it alone
            continue
        if hw <= 0 or hw >= 99999:
            hw = _INF
        if hh <= 0 or hh >= 99999:
            hh = _INF
        # max_ratio is a slenderness cap (long/short <= ratio) applied
        # symmetrically by _axis_targets. min_ratio is honoured only as a
        # genuine sub-1 lower w/h band; the wire carries sentinels here
        # (designer sends min=1, one legacy default sends min=3) that must
        # not be read as "force landscape".
        ratio = at(maxr, i, 0.0)
        aspect = ratio if ratio and 1.0 <= ratio < 99999 else _INF
        lo_ratio = at(minr, i, 0.0)
        # Explicit area rule when the request carries one; the span product
        # is only the fallback (it runs 1.04x-1.42x looser than NBC for
        # every room type, which is why max_area exists as its own field).
        span_area_cap = (hw * hh) if (hw < _INF and hh < _INF) else _INF
        req_max_area = at(maxa, i, 0.0)
        max_area = min(span_area_cap, req_max_area) \
            if 0 < req_max_area < 99999 else span_area_cap
        req_min_area = at(mina, i, 0.0)
        min_area = max(lw * lh, req_min_area) \
            if 0 < req_min_area < 99999 else lw * lh
        entry = {
            "minw": lw, "minh": lh, "maxw": hw, "maxh": hh,
            "minarea": min_area,
            "maxarea": max(max_area, min_area),
            "aspect": aspect,
        }
        if aspect < _INF and 0.0 < lo_ratio < 1.0:
            entry["ar_lo"] = lo_ratio
            entry["ar_hi"] = aspect
        bounds.append(entry)
    return bounds if any(b is not None for b in bounds) else None


def _axis_targets(bounds, other_side, horizontal):
    """[lo, hi] for one room's span on this axis, given its span on the other.

    Area and aspect are bilinear, so they only become interval constraints once
    the other axis is held fixed. That is what makes the alternating solve below
    a pair of linear problems instead of one non-convex one. Minimums are hard,
    so when the two disagree the minimum wins and the ceiling is reported as
    unmet rather than silently violating a floor.
    """
    if bounds is None or other_side <= 0:
        return None
    lo = bounds["minw"] if horizontal else bounds["minh"]
    hi = bounds["maxw"] if horizontal else bounds["maxh"]
    lo = max(lo, bounds["minarea"] / other_side)
    if bounds["maxarea"] < _INF:
        hi = min(hi, bounds["maxarea"] / other_side)
    if bounds["aspect"] < _INF:
        lo = max(lo, other_side / bounds["aspect"])
        hi = min(hi, other_side * bounds["aspect"])
    # Optional asymmetric w/h band (ar_lo <= w/h <= ar_hi), used by the NBC
    # post-processor. The symmetric "aspect" cap above is the special case
    # ar_lo = 1/aspect, ar_hi = aspect; bounds without these keys are untouched.
    ar_lo = bounds.get("ar_lo")
    ar_hi = bounds.get("ar_hi")
    if ar_lo and ar_hi and 0 < ar_lo <= ar_hi:
        if horizontal:
            lo = max(lo, ar_lo * other_side)
            hi = min(hi, ar_hi * other_side)
        else:
            lo = max(lo, other_side / ar_hi)
            hi = min(hi, other_side / ar_lo)
    if hi < lo:
        hi = lo
    return (lo, hi)


def _relax_axis(rects, bounds, horizontal, rounds=60, damping=0.5):
    """Move the shared coordinate lines so every room lands inside its band.

    A rectangular dissection is fully described by a set of global coordinate
    lines plus, for each room, WHICH lines its edges sit on. Changing the line
    POSITIONS therefore keeps every room a rectangle, keeps the tiling exactly
    gapless, and keeps every adjacency (so every door survives). That is the
    whole trick: the layout is untouched, only the measurements move.

    Each room asks its span to grow or shrink; every gap inside that span is
    asked for an equal share; each gap averages the requests of the rooms that
    contain it and moves a damped step. Gaps never go below a small floor, so
    the ordering of the lines cannot invert.
    """
    lo_i, hi_i = (0, 2) if horizontal else (1, 3)
    coords = sorted({round(r[lo_i], 6) for r in rects} | {round(r[hi_i], 6) for r in rects})
    if len(coords) < 2:
        return rects
    index = {c: i for i, c in enumerate(coords)}
    spans = []
    for r in rects:
        spans.append((index[round(r[lo_i], 6)], index[round(r[hi_i], 6)]))
    gaps = [coords[i + 1] - coords[i] for i in range(len(coords) - 1)]
    floor = min([g for g in gaps if g > 1e-9] + [1.0]) * 0.05

    for _ in range(rounds):
        other = []
        for k, r in enumerate(rects):
            other.append(r[3] - r[1] if horizontal else r[2] - r[0])
        want = [[] for _ in gaps]
        moved = False
        for k, (a, b) in enumerate(spans):
            target = _axis_targets(bounds[k] if k < len(bounds) else None, other[k], horizontal)
            if target is None or b <= a:
                continue
            cur = sum(gaps[a:b])
            lo, hi = target
            goal = min(max(cur, lo), hi)
            if abs(goal - cur) < 1e-6:
                continue
            moved = True
            share = (goal - cur) / (b - a)
            for g in range(a, b):
                want[g].append(share)
        if not moved:
            break
        for g in range(len(gaps)):
            if want[g]:
                gaps[g] = max(floor, gaps[g] + damping * (sum(want[g]) / len(want[g])))
        # A gap is shared, so the compromise above can drag a room under its
        # floor even though every individual request respected it. Minimums are
        # NOT negotiable - they are the NBC line - so grow back any room that
        # fell short. Growth only, hence this terminates.
        for _ in range(40):
            short = False
            for k, (a, b) in enumerate(spans):
                target = _axis_targets(bounds[k] if k < len(bounds) else None,
                                       other[k], horizontal)
                if target is None or b <= a:
                    continue
                cur = sum(gaps[a:b])
                if cur < target[0] - 1e-6:
                    short = True
                    add = (target[0] - cur) / (b - a)
                    for g in range(a, b):
                        gaps[g] += add
            if not short:
                break
        # rebuild rects from the new lines so `other` is fresh next round
        pos = [coords[0]]
        for g in gaps:
            pos.append(pos[-1] + g)
        for k, (a, b) in enumerate(spans):
            r = rects[k]
            if horizontal:
                rects[k] = (pos[a], r[1], pos[b], r[3])
            else:
                rects[k] = (r[0], pos[a], r[2], pos[b])
    return rects


REPAIR_DIMENSIONS = os.environ.get("GPLAN_REPAIR_DIMS", "1") == "1"


def repair_dimensions(rects, bounds, passes=6):
    """Alternate the two axis relaxations until the plan stops improving.

    Returns the repaired rects. The tiling stays gapless and every room stays a
    rectangle by construction, so this can only change measurements, never the
    arrangement.
    """
    if not bounds or not REPAIR_DIMENSIONS:
        return rects
    work = list(rects)
    for _ in range(passes):
        before = list(work)
        work = _relax_axis(work, bounds, True)
        work = _relax_axis(work, bounds, False)
        if all(abs(a[i] - b[i]) < 1e-4 for a, b in zip(before, work) for i in range(4)):
            break
    return work


def _room_size_caps(ui, count):
    """Per-room (max span, max area) for the gap fill, or None when unbounded.

    The dimensioning solver honours per-axis maximums, but the greedy fill below
    can undo that by handing a whole leftover strip to whichever room happens to
    touch it (this is how a 5x7 ft balcony became 5x24 ft). The span cap is
    max(max_width, max_height) rather than the per-axis value because the
    rotation pass may already have swapped a plan's axes; the area cap does the
    real work of keeping a service room from outgrowing a habitable one.
    """
    params = getattr(ui, "min_dim_inputs", None)
    if params is None:
        return None
    max_w = params.get_max_width() or []
    max_h = params.get_max_height() or []
    max_a = (params.get_max_area() or []) \
        if hasattr(params, "get_max_area") else []
    if not max_w or not max_h:
        return None

    def value(arr, i):
        if i >= len(arr):
            return None
        try:
            v = float(arr[i])
        except (TypeError, ValueError):
            return None
        return v if 0 < v < 99999 else None

    caps = []
    for i in range(count):
        w, h = value(max_w, i), value(max_h, i)
        if w is None or h is None:
            # Dummy rooms from separating-triangle removal have no entry:
            # leave open.
            caps.append(None)
            continue
        # The explicit per-room area rule wins over the span product when
        # the request carries one (the product is 1.04x-1.42x looser than
        # NBC for every room type).
        area_cap = value(max_a, i)
        caps.append((max(w, h),
                     min(w * h, area_cap) if area_cap else w * h))
    return caps if any(c is not None for c in caps) else None


def _exceeds_caps(rects, caps, eps):
    """True when any room now runs past its own span or area ceiling."""
    for i, (x0, y0, x1, y1) in enumerate(rects):
        if i >= len(caps) or caps[i] is None:
            continue
        span_cap, area_cap = caps[i]
        w, h = x1 - x0, y1 - y0
        if max(w, h) > span_cap + eps or w * h > area_cap + eps:
            return True
    return False


def _span_limit(caps, i, other_side):
    """Longest this room may run on one axis given its current span on the other."""
    if not caps or i >= len(caps) or caps[i] is None:
        return None
    span_cap, area_cap = caps[i]
    if other_side > 0:
        return min(span_cap, area_cap / other_side)
    return span_cap


def _headroom(rects, caps, i):
    """How much more area room i may still take before hitting its ceiling.

    Uncapped rooms are treated as bottomless so they absorb first. A room that
    is already at or past its ceiling returns 0 and drops to the back of the
    queue, which is what keeps a leftover strip out of the bathroom.
    """
    if not caps or i >= len(caps) or caps[i] is None:
        return float('inf')
    x0, y0, x1, y1 = rects[i]
    return caps[i][1] - (x1 - x0) * (y1 - y0)


def _scaled_caps(caps, slack):
    """Caps with `slack` headroom for the bounded gap-closing pass (E3 of
    documentation/plans/VALIDITY_AND_TOPOLOGY_ENGINE_PLAN.md). The fully
    uncapped pass is what produced 1.5-3x WCs on 8+ room briefs: the hole had
    to close and whoever touched it took everything. Closing within a bounded
    multiple first keeps the overshoot small enough for post-processing to
    repair without carving neighbouring walls into door-less slivers."""
    if caps is None:
        return None
    return [None if c is None else (c[0] * slack, c[1] * slack) for c in caps]


# How far past a room's ceiling the bounded gap-closing pass may go. Must sit
# BELOW every gross-oversize threshold the client audits, with margin: the
# fill lands rooms exactly AT the bound (a 34 sqft toilet cap emitted a 51.1
# sqft toilet at the old 1.5, and the client's wet/service hard error starts
# at 1.5x - the 0.1 sqft overshoot flipped whole exact-fill batches to
# non-compliant, measured 2026-08-17). 1.4 keeps the closure property and
# every closed room inside warning territory.
GAP_FILL_SLACK = 1.4


def _fill_gaps(rects, eps, caps=None, enforce=True, locked=None):
    """Greedy wall extension: push each room's sides outward to the nearest
    obstruction (another room overlapping that side's span, else the plan
    bounds) until nothing moves. Rooms stay rectangles and never overlap;
    empty notches next to a full side get absorbed. Mutates and returns rects.

    `caps` (see _room_size_caps) drives two separate things:
      * ORDER, always. Rooms are visited in descending remaining headroom,
        recomputed every round, so leftover area flows to whoever can still
        take it (living rooms and bedrooms) rather than to whichever room
        happens to sit next to the notch.
      * LIMIT, only when `enforce`. The second, gap-closing pass runs with
        enforce=False: it must be free to break a ceiling to reach a gapless
        rectangle, but it should still hand the slack to the right rooms.
    """
    bx0 = min(r[0] for r in rects)
    by0 = min(r[1] for r in rects)
    bx1 = max(r[2] for r in rects)
    by1 = max(r[3] for r in rects)
    limits = caps if enforce else None
    locked = set(locked or ())
    changed = True
    guard = 0
    while changed and guard < 200:
        changed = False
        guard += 1
        order = sorted(range(len(rects)), key=lambda i: (-_headroom(rects, caps, i), i))
        for i in order:
            # Fixed rooms are occupied obstacles.  Other rooms may grow up to
            # their walls, but no gap-closing rung may grow the fixed room -
            # including the legacy uncapped last resort.
            if i in locked:
                continue
            x0, y0, x1, y1 = rects[i]
            wide = _span_limit(limits, i, y1 - y0)
            tall = _span_limit(limits, i, x1 - x0)
            # East
            obst = bx1
            for j, (a0, b0, a1, b1) in enumerate(rects):
                if j != i and min(y1, b1) - max(y0, b0) > eps and a0 >= x1 - eps:
                    obst = min(obst, a0)
            if wide is not None:
                obst = min(obst, x0 + wide)
            if obst - x1 > eps:
                rects[i] = (x0, y0, obst, y1)
                changed = True
                x1 = obst
            # West
            obst = bx0
            for j, (a0, b0, a1, b1) in enumerate(rects):
                if j != i and min(y1, b1) - max(y0, b0) > eps and a1 <= x0 + eps:
                    obst = max(obst, a1)
            if wide is not None:
                obst = max(obst, x1 - wide)
            if x0 - obst > eps:
                rects[i] = (obst, y0, x1, y1)
                changed = True
                x0 = obst
            # South (larger y)
            obst = by1
            for j, (a0, b0, a1, b1) in enumerate(rects):
                if j != i and min(x1, a1) - max(x0, a0) > eps and b0 >= y1 - eps:
                    obst = min(obst, b0)
            if tall is not None:
                obst = min(obst, y0 + tall)
            if obst - y1 > eps:
                rects[i] = (x0, y0, x1, obst)
                changed = True
                y1 = obst
            # North (smaller y)
            obst = by0
            for j, (a0, b0, a1, b1) in enumerate(rects):
                if j != i and min(x1, a1) - max(x0, a0) > eps and b1 <= y0 + eps:
                    obst = max(obst, b1)
            if tall is not None:
                obst = max(obst, y1 - tall)
            if y0 - obst > eps:
                rects[i] = (x0, obst, x1, y1)
                changed = True
                y0 = obst
    return rects


def _is_gapless(rects, eps):
    """Full-coverage test for non-overlapping axis-aligned rects: the union
    fills the bounding rectangle iff the areas sum to the bounds area."""
    bx0 = min(r[0] for r in rects)
    by0 = min(r[1] for r in rects)
    bx1 = max(r[2] for r in rects)
    by1 = max(r[3] for r in rects)
    bounds_area = (bx1 - bx0) * (by1 - by0)
    if bounds_area <= 0:
        return False
    total = sum((r[2] - r[0]) * (r[3] - r[1]) for r in rects)
    return abs(total - bounds_area) <= bounds_area * 1e-3


def _anchor_locked_rects(rects, fixed_rooms, eps):
    """Move each locked room onto its anchored edges of the plan bbox.

    The min-dim placement aligns rooms to the interior walls they share and
    leaves the outer strip for the gap-closing pass to grow into; a locked
    room cannot grow, so it came back 1-4 ft inside its west or south edge
    with nothing beside it and the whole plan was dropped. Translating the
    locked rectangle (never resizing it) onto the edge leaves the strip it
    vacated next to an ordinary room, which the fill closes. Skipped when the
    room is not at its exact size (an expanded plan) or the spot is occupied.
    """
    if not fixed_rooms or not rects:
        return rects
    rects = list(rects)
    bx0 = min(r[0] for r in rects)
    by0 = min(r[1] for r in rects)
    bx1 = max(r[2] for r in rects)
    by1 = max(r[3] for r in rects)
    for spec in fixed_rooms:
        i = spec["room"]
        if i < 0 or i >= len(rects):
            continue
        x0, y0, x1, y1 = rects[i]
        w, h = x1 - x0, y1 - y0
        if abs(w - spec["width"]) > 0.05 or abs(h - spec["height"]) > 0.05:
            continue
        nx0, ny0 = x0, y0
        directions = spec.get("directions", ())
        if 3 in directions:
            nx0 = bx0
        if 1 in directions:
            nx0 = bx1 - w
        if 0 in directions:
            ny0 = by0
        if 2 in directions:
            ny0 = by1 - h
        if abs(nx0 - x0) <= eps and abs(ny0 - y0) <= eps:
            continue
        moved = (nx0, ny0, nx0 + w, ny0 + h)
        blocked = False
        for j, other in enumerate(rects):
            if j == i:
                continue
            if (min(moved[2], other[2]) - max(moved[0], other[0]) > eps
                    and min(moved[3], other[3]) - max(moved[1], other[1]) > eps):
                blocked = True
                break
        if not blocked:
            rects[i] = moved
    return rects


def _rects_satisfy_fixed(rects, fixed_rooms, tolerance=0.05):
    if not fixed_rooms:
        return True
    if not rects:
        return False
    bx0 = min(r[0] for r in rects)
    by0 = min(r[1] for r in rects)
    bx1 = max(r[2] for r in rects)
    by1 = max(r[3] for r in rects)
    for spec in fixed_rooms:
        i = spec["room"]
        if i < 0 or i >= len(rects):
            return False
        x0, y0, x1, y1 = rects[i]
        if abs((x1 - x0) - spec["width"]) > tolerance \
                or abs((y1 - y0) - spec["height"]) > tolerance:
            return False
        for direction in spec.get("directions", ()):
            side = (y0, x1, y1, x0)[direction]
            outer = (by0, bx1, by1, bx0)[direction]
            if abs(side - outer) > tolerance:
                return False
        if 0 in spec.get("directions", ()) and abs(y0) > tolerance:
            return False
        if 3 in spec.get("directions", ()) and abs(x0) > tolerance:
            return False
    return True


def rectangularize_output(ui, fixed_rooms=None):
    """Keeps only plans that are (or can be made) gapless rectangles.

    For each generated plan whose rooms are all rectangles: fill boundary
    notches/holes by greedy wall extension, then require full coverage of the
    bounding rectangle. Plans with merged (non-rectangular) rooms, overlapping
    rooms, or residual holes are dropped. Surviving plans get their geometry
    rewritten to the filled rectangles. Returns the surviving list.
    """
    if fixed_rooms is None:
        params = getattr(ui, "min_dim_inputs", None)
        fixed_rooms = (getattr(params, "get_fixed_rooms", lambda: [])()
                       if params is not None else [])
    params = getattr(ui, "min_dim_inputs", None)
    if getattr(params, "strict_room_bounds", False):
        from GPLAN.source.dimensioning.room_constraints import constrain_plan
        kept = []
        for plan in ui.get_output_data():
            candidate, failures = constrain_plan(plan, params.room_nodes, ui.get_edges(), strict=True)
            if candidate is None:
                from GPLAN.source.dimensioning.room_constraints import unique_violations
                ui._room_constraint_failures = unique_violations(
                    getattr(ui, "_room_constraint_failures", []) + failures)
            if candidate is not None and (not fixed_rooms or plan_satisfies_fixed(candidate, fixed_rooms)):
                kept.append(candidate)
        ui._set_output_data(kept)
        return kept
    locked = {spec["room"] for spec in fixed_rooms or []}
    kept = []
    caps_broken = False
    for plan in ui.get_output_data():
        rooms = getattr(plan, "final_traversal", None) or []
        if not rooms:
            continue
        all_xs = [pt[0] for poly in rooms for pt in poly]
        all_ys = [pt[1] for poly in rooms for pt in poly]
        span = max(max(all_xs) - min(all_xs), max(all_ys) - min(all_ys), 1e-6)
        eps = span * 1e-4
        rects = []
        for poly in rooms:
            rect = _polyline_rect(poly, span * 1e-3)
            if rect is None:
                rects = None
                break
            rects.append(rect)
        if rects is not None and fixed_rooms:
            rects = _anchor_locked_rects(rects, fixed_rooms, eps)
        if (rects is None or _rects_overlap(rects, eps)
                or not _rects_satisfy_fixed(rects, fixed_rooms)):
            continue
        # Fill within the room ceilings first, then let whatever hole is left
        # close without them. A gapless rectangle is required; honouring every
        # ceiling is only preferred, and running the capped pass first means
        # the big rooms have already taken their share, so the residual a
        # bathroom or balcony can grab is small.
        caps = _room_size_caps(ui, len(rects))
        if caps is not None:
            _fill_gaps(rects, eps, caps, locked=locked)
        if not _is_gapless(rects, eps) and caps is not None:
            # E3 (2026-08-06): close the hole within a BOUNDED multiple of the
            # ceilings first. Most holes close here, and 1.5x overshoot is
            # repairable; the old jump straight to uncapped is what put WCs at
            # 1.5-3x their ceiling on every 8+ room brief.
            _fill_gaps(rects, eps, _scaled_caps(caps, GAP_FILL_SLACK),
                       locked=locked)
        if not _is_gapless(rects, eps):
            # Ceilings off, priority ordering still on: the hole must close, but
            # the room that closes it should be one that can carry the area.
            # Last resort only, kept so a pathological plan degrades to an
            # oversized room with a warning instead of vanishing from the batch.
            _fill_gaps(rects, eps, caps, enforce=False, locked=locked)
        if _rects_overlap(rects, eps) or not _is_gapless(rects, eps):
            continue
        # The tiling is now correct but the MEASUREMENTS are not: min-dim only
        # ever honoured minimums, and closing the gaps above pushed whatever
        # slack was left into whichever room could reach it. Re-solve the
        # coordinate lines against the full band (min and max width, height,
        # area and aspect). This moves shared walls only, so the arrangement,
        # the gaplessness and every adjacency survive untouched.
        repaired = repair_dimensions(rects, _room_bounds(ui, len(rects)))
        if not _rects_overlap(repaired, eps) and _is_gapless(repaired, eps):
            # repair_dimensions claims to move shared walls only, but the
            # relax iterations DO push the outer walls (measured +0.19 to
            # +3 ft on the bbox). Under enforce_plot that silently broke the
            # hard cap the solver had just honoured, so the repaired
            # geometry is accepted only while it stays inside the enforced
            # plot - or inside the plan's own entry bbox when that was
            # already larger (the labeled expanded plans keep their repair).
            accept = True
            params = getattr(ui, "min_dim_inputs", None)
            if params is not None and bool(getattr(params, "get_enforce_plot",
                                                   lambda: False)()):
                try:
                    limit_w = float(params.get_plot_width() or 0)
                    limit_h = float(params.get_plot_height() or 0)
                except (TypeError, ValueError):
                    limit_w = limit_h = 0.0
                if limit_w > 0 and limit_h > 0:
                    entry_w = max(r[2] for r in rects) - min(r[0] for r in rects)
                    entry_h = max(r[3] for r in rects) - min(r[1] for r in rects)
                    rep_w = max(r[2] for r in repaired) - min(r[0] for r in repaired)
                    rep_h = max(r[3] for r in repaired) - min(r[1] for r in repaired)
                    accept = (rep_w <= max(limit_w, entry_w) + eps
                              and rep_h <= max(limit_h, entry_h) + eps)
            if accept and _rects_satisfy_fixed(repaired, fixed_rooms):
                rects = repaired
        if not _rects_satisfy_fixed(rects, fixed_rooms):
            continue
        # The gap-closing passes can push a room past its ceiling (the bounded
        # E3 pass up to GAP_FILL_SLACK, the last-resort pass without limit).
        # That is the intended trade (a gapless rectangle is required,
        # honouring every ceiling is preferred), but it must not be silent.
        # Judged on the FINAL geometry only - not on which pass ran - because
        # repair_dimensions often shrinks the offender back inside its band,
        # and warning about a plan that ends up compliant taught clients to
        # ignore the warning. `ran_uncapped` no longer gates this: the bounded
        # pass overshoots too, and a silent 1.4x bathroom is still a lie.
        if caps is not None and _exceeds_caps(rects, caps, eps):
            caps_broken = True
        plan.final_traversal = [
            [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]
            for (x0, y0, x1, y1) in rects
        ]
        plan.room_width = [r[2] - r[0] for r in rects]
        plan.room_height = [r[3] - r[1] for r in rects]
        plan.area = [(r[2] - r[0]) * (r[3] - r[1]) for r in rects]
        if hasattr(plan, "name_coords") and plan.name_coords:
            for k, (x0, y0, x1, y1) in enumerate(rects):
                if k < len(plan.name_coords):
                    plan.name_coords[k] = [(x0 + x1) / 2, (y0 + y1) / 2]
        kept.append(plan)
    ui._set_output_data(kept)
    if caps_broken:
        ui.print_gui("Warning: Room maximum dimensions were exceeded while "
                     "closing gaps in some floorplans.")
    return kept


def _stacked_room_dict(name, color, x0, y0, x1, y1):
    """Serialized Room dict for one axis-aligned rectangle."""
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    walls = []
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        walls.append(Wall(str(uuid.uuid4()), ax, ay, bx, by))
    room = Room(str(uuid.uuid4()), name, color, walls,
                circular_coordinates=[list(c) for c in corners] + [[x0, y0]],
                name_coord=[(x0 + x1) / 2, (y0 + y1) / 2],
                area=(x1 - x0) * (y1 - y0), width=x1 - x0, height=y1 - y0)
    return room.to_dict()


def _plan_rects(plan):
    """name -> (x0, y0, x1, y1) bounding box for each room of a serialized plan."""
    rects = {}
    for room in plan:
        xs, ys = [], []
        for wall in room["walls"]:
            xs += [wall["x1"], wall["x2"]]
            ys += [wall["y1"], wall["y2"]]
        if xs:
            rects[room["name"]] = (min(xs), min(ys), max(xs), max(ys))
    return rects


def _rects_overlap(rect_list, eps):
    for i in range(len(rect_list)):
        x0, y0, x1, y1 = rect_list[i]
        for j in range(i + 1, len(rect_list)):
            a0, b0, a1, b1 = rect_list[j]
            if min(x1, a1) - max(x0, a0) > eps and min(y1, b1) - max(y0, b0) > eps:
                return True
    return False


def _rect_exterior(rect_list, target, dir_idx, eps):
    """Strip test on (x0,y0,x1,y1) rects; dir_idx 0..3 = N,E,S,W (y-down)."""
    bx0 = min(r[0] for r in rect_list)
    by0 = min(r[1] for r in rect_list)
    bx1 = max(r[2] for r in rect_list)
    by1 = max(r[3] for r in rect_list)
    x0, y0, x1, y1 = target
    if dir_idx == 0:
        sx0, sy0, sx1, sy1 = x0, by0, x1, y0
    elif dir_idx == 1:
        sx0, sy0, sx1, sy1 = x1, y0, bx1, y1
    elif dir_idx == 2:
        sx0, sy0, sx1, sy1 = x0, y1, x1, by1
    else:
        sx0, sy0, sx1, sy1 = bx0, y0, x0, y1
    for r in rect_list:
        if r == target:
            continue
        if min(r[2], sx1) - max(r[0], sx0) > eps and min(r[3], sy1) - max(r[1], sy0) > eps:
            return False
    return True


def stacked_oneconnected_floorplans(nodes_list, edges_list, count, dim_inputs,
                                    cardinal_pairs, corridor_thickness, limit):
    """Floorplans for a 1-connected graph, composed as stacked component duals.

    Applies when the graph is exactly two biconnected components sharing one
    cut vertex (e.g. two wings joined by a hall). Each component's floorplans
    are generated through the full pipeline with the cut-vertex room pinned to
    the seam side (E|W for a horizontal stack, S|N for vertical); the second
    component is scaled so its cut-room span matches the first and the two are
    fused into a single plan whose cut room spans the seam. Composites are
    kept only when no rooms overlap and every user cardinal pin holds
    geometrically. Returns (plans, note) or (None, None) when the topology
    doesn't fit or nothing valid could be composed.
    """
    if not dim_inputs:
        return None, None
    try:
        import networkx as nx
        g = nx.Graph()
        g.add_nodes_from(range(len(nodes_list)))
        g.add_edges_from((int(e[0]), int(e[1])) for e in edges_list)
        if len(nodes_list) < 4 or g.number_of_edges() == 0 or not nx.is_connected(g):
            return None, None
        arts = list(nx.articulation_points(g))
        if len(arts) != 1:
            return None, None
        cut = arts[0]
        comps = [sorted(c) for c in nx.biconnected_components(g)]
        if len(comps) != 2:
            return None, None
    except Exception:
        return None, None
    labels = [node["label"] for node in nodes_list]
    if len(set(labels)) != len(labels):
        return None, None  # name-keyed stitching needs unique room labels
    cut_name = labels[cut]

    def component_plans(comp, seam_dir):
        mapping = {old: new for new, old in enumerate(comp)}
        sub_nodes = []
        for old in comp:
            node = dict(nodes_list[old])
            node["id"] = mapping[old]
            sub_nodes.append(node)
        sub_edges = [[mapping[e[0]], mapping[e[1]], "black"] for e in edges_list
                     if e[0] in mapping and e[1] in mapping]

        def slice_arr(key, default):
            arr = dim_inputs.get(key) or []
            return [arr[i] if i < len(arr) else default for i in comp]

        sub_dims = {
            "min_width": slice_arr("min_width", 3), "max_width": slice_arr("max_width", 99999),
            "min_height": slice_arr("min_height", 3), "max_height": slice_arr("max_height", 99999),
            "min_ratio": slice_arr("min_ratio", 3), "max_ratio": slice_arr("max_ratio", 99999),
            "plot_width": 0, "plot_height": 0,
            "symmetric": False, "optimal_floorplan": 0, "rotation_enabled": 0,
        }
        # User pins for rooms in this component; the cut room belongs to the
        # seam (its user pin, if any, is enforced on the composite instead).
        sub_cardinal = [{"room": mapping[n], "direction": "NESW"[d]}
                        for n, d in cardinal_pairs if n in mapping and n != cut]
        sub_cardinal.append({"room": mapping[cut], "direction": "NESW"[seam_dir]})
        coords = [[node.get("x", 0), node.get("y", 0)] for node in sub_nodes]
        sub_graph = InputGraph(len(sub_nodes), len(sub_edges), sub_edges, coords)
        per = max(6, min(count, 10))
        try:
            fps, msg = Documents.get_floorplans(
                starting_from=0, count=per, caller="door_connectivity",
                nodes_list=sub_nodes, graph=sub_graph, rectangular=True,
                minDimEnabled=True, nonAdj=False, limit=per,
                corridor_thickness=corridor_thickness, dim_inputs=sub_dims,
                edges_list=sub_edges, non_adj_edge_list=[],
                cardinal_constraints=sub_cardinal)
        except Exception:
            return [], ""
        return fps.to_dict()["Documents"]["floorPlans"], msg

    memo = {}

    def plans_for(comp_idx, seam_dir):
        key = (comp_idx, seam_dir)
        if key not in memo:
            memo[key] = component_plans(comps[comp_idx], seam_dir)
        return memo[key]

    def color_of(plan, name):
        for room in plan:
            if room["name"] == name:
                return room.get("color", "#1C4C82")
        return "#1C4C82"

    composed = []
    relaxed = False
    name_by_node = {i: labels[i] for i in range(len(labels))}
    # (left/top component, right/bottom component) x (horizontal, vertical)
    for first, second in ((0, 1), (1, 0)):
        for axis in ("h", "v"):
            if len(composed) >= count:
                break
            l_dir, r_dir = (1, 3) if axis == "h" else (2, 0)
            left_plans, left_msg = plans_for(first, l_dir)
            right_plans, right_msg = plans_for(second, r_dir)
            if "relaxed" in (left_msg or "") or "relaxed" in (right_msg or ""):
                relaxed = True
            for lp in left_plans:
                if len(composed) >= count:
                    break
                lrects = _plan_rects(lp)
                if cut_name not in lrects:
                    continue
                lall = list(lrects.values())
                lw = max(r[2] for r in lall) - min(r[0] for r in lall)
                lh = max(r[3] for r in lall) - min(r[1] for r in lall)
                leps = max(lw, lh, 1e-6) * 1e-4
                lcut = lrects[cut_name]
                if not _rect_exterior(lall, lcut, l_dir, leps):
                    continue
                # Gapless composite: the cut room must span the FULL seam side
                # of its component, so the two rectangles fuse into one
                # rectangle with no empty corners at the seam.
                if axis == "h":
                    if (lcut[1] > min(r[1] for r in lall) + leps or
                            lcut[3] < max(r[3] for r in lall) - leps):
                        continue
                else:
                    if (lcut[0] > min(r[0] for r in lall) + leps or
                            lcut[2] < max(r[2] for r in lall) - leps):
                        continue
                for rp in right_plans:
                    rrects = _plan_rects(rp)
                    if cut_name not in rrects:
                        continue
                    rall = list(rrects.values())
                    rw = max(r[2] for r in rall) - min(r[0] for r in rall)
                    rh = max(r[3] for r in rall) - min(r[1] for r in rall)
                    reps = max(rw, rh, 1e-6) * 1e-4
                    rcut = rrects[cut_name]
                    if not _rect_exterior(rall, rcut, r_dir, reps):
                        continue
                    if axis == "h":
                        if (rcut[1] > min(r[1] for r in rall) + reps or
                                rcut[3] < max(r[3] for r in rall) - reps):
                            continue
                    else:
                        if (rcut[0] > min(r[0] for r in rall) + reps or
                                rcut[2] < max(r[2] for r in rall) - reps):
                            continue
                    if axis == "h":
                        span_l = lcut[3] - lcut[1]
                        span_r = rcut[3] - rcut[1]
                    else:
                        span_l = lcut[2] - lcut[0]
                        span_r = rcut[2] - rcut[0]
                    if span_l <= 0 or span_r <= 0:
                        continue
                    s = span_l / span_r
                    if axis == "h":
                        dx = lcut[2] - rcut[0] * s
                        dy = lcut[1] - rcut[1] * s
                    else:
                        dx = lcut[0] - rcut[0] * s
                        dy = lcut[3] - rcut[1] * s
                    rooms = []
                    for nm, (x0, y0, x1, y1) in lrects.items():
                        if nm == cut_name:
                            continue
                        rooms.append((nm, x0, y0, x1, y1, color_of(lp, nm)))
                    for nm, (x0, y0, x1, y1) in rrects.items():
                        if nm == cut_name:
                            continue
                        rooms.append((nm, x0 * s + dx, y0 * s + dy,
                                      x1 * s + dx, y1 * s + dy, color_of(rp, nm)))
                    tcut = (rcut[0] * s + dx, rcut[1] * s + dy,
                            rcut[2] * s + dx, rcut[3] * s + dy)
                    if axis == "h":
                        fused = (lcut[0], lcut[1], tcut[2], lcut[3])
                    else:
                        fused = (lcut[0], lcut[1], lcut[2], tcut[3])
                    rooms.append((cut_name, fused[0], fused[1], fused[2], fused[3],
                                  color_of(lp, cut_name)))
                    rect_list = [(r[1], r[2], r[3], r[4]) for r in rooms]
                    w = max(r[2] for r in rect_list) - min(r[0] for r in rect_list)
                    h = max(r[3] for r in rect_list) - min(r[1] for r in rect_list)
                    eps = max(w, h, 1e-6) * 1e-4
                    if _rects_overlap(rect_list, eps) or not _is_gapless(rect_list, eps):
                        continue
                    rect_by_name = {r[0]: (r[1], r[2], r[3], r[4]) for r in rooms}
                    ok = True
                    for n, d in cardinal_pairs:
                        target = rect_by_name.get(name_by_node[n])
                        if target is None or not _rect_exterior(rect_list, target, d, eps):
                            ok = False
                            break
                    if not ok:
                        continue
                    composed.append([_stacked_room_dict(nm, col, x0, y0, x1, y1)
                                     for nm, x0, y0, x1, y1, col in rooms])
                    if len(composed) >= count:
                        break
    if not composed:
        return None, None
    note = (" One-connected graph composed as two stacked component duals"
            " joined at '" + cut_name + "'.")
    if relaxed:
        note += " Adjacency constraints were relaxed to satisfy cardinal directions."
    return composed, note


def spanning_tree_edges(edges_list, nodecnt):
    """BFS spanning-tree subset of the given edges (adjacency relaxation).

    Keeps only existing user edges; roots at the highest-degree node so hub
    adjacencies survive. Returns edges in the same [src, dst, color] shape.
    """
    adj = {i: [] for i in range(nodecnt)}
    edge_by_pair = {}
    for edge in edges_list:
        a, b = edge[0], edge[1]
        adj[a].append(b)
        adj[b].append(a)
        edge_by_pair[(min(a, b), max(a, b))] = edge
    if not edge_by_pair:
        return list(edges_list)
    roots = sorted(adj, key=lambda n: len(adj[n]), reverse=True)
    visited = set()
    tree = []
    for root in roots:
        if root in visited:
            continue
        visited.add(root)
        queue = [root]
        while queue:
            cur = queue.pop(0)
            for nbr in adj[cur]:
                if nbr not in visited:
                    visited.add(nbr)
                    queue.append(nbr)
                    tree.append(edge_by_pair[(min(cur, nbr), max(cur, nbr))])
    return tree


def _serialize_graph_plan(plan_graph, nodes_list):
    """Serialize one finalized graph plan with the terminal wire contract."""
    rooms = []
    floorplan_data = plan_graph.final_traversal
    name_coord = plan_graph.name_coords
    area = plan_graph.area
    widths = plan_graph.room_width
    heights = plan_graph.room_height
    for index, room_data in enumerate(floorplan_data):
        x1 = room_data[0][0]
        y1 = room_data[0][1]
        wall_values = []
        for point_index in range(1, len(room_data)):
            x2 = room_data[point_index][0]
            y2 = room_data[point_index][1]
            wall_values.append(Wall(str(uuid.uuid4()), x1, y1, x2, y2))
            x1 = x2
            y1 = y2
        wall_values.append(
            Wall(str(uuid.uuid4()), x1, y1, room_data[0][0], room_data[0][1]))
        room = Room(
            str(uuid.uuid4()), nodes_list[index]["label"],
            nodes_list[index]["color"], wall_values,
            circular_coordinates=room_data,
            name_coord=name_coord[index], area=area[index],
            width=widths[index], height=heights[index])
        rooms.append(room.to_dict())
    return rooms


def _wire_plan_key(plan):
    """Geometry identity that ignores the random room and wall UUIDs."""
    keyed = []
    for room in plan:
        polygon = room.get("circular_coordinates") or []
        keyed.append((
            str(room.get("name", "")),
            tuple((round(float(point[0]), 5), round(float(point[1]), 5))
                  for point in polygon),
        ))
    return tuple(keyed)


def _metric_shortfalls(floorplan_data, nodes_list, edges_list, overlap_floor):
    shortfalls = []
    for edge in edges_list or []:
        try:
            a, b = int(edge[0]), int(edge[1])
        except (TypeError, ValueError, IndexError):
            continue
        if max(a, b) >= len(floorplan_data) or max(a, b) >= len(nodes_list):
            continue
        shared = _shared_wall_length(floorplan_data[a], floorplan_data[b])
        if shared + 1e-6 < overlap_floor:
            shortfalls.append({
                "a": a,
                "b": b,
                "a_name": nodes_list[a]["label"],
                "b_name": nodes_list[b]["label"],
                "shared_wall_ft": round(shared, 2),
            })
    return shortfalls


def _plot_fit_report(floorplan_data, plot_width, plot_height):
    xs = [point[0] for polygon in floorplan_data for point in polygon]
    ys = [point[1] for polygon in floorplan_data for point in polygon]
    plan_width = (max(xs) - min(xs)) if xs else 0.0
    plan_height = (max(ys) - min(ys)) if ys else 0.0
    eps = 0.05
    return {
        "fits": (plan_width <= plot_width + eps
                 and plan_height <= plot_height + eps),
        "plan_width": round(plan_width, 2),
        "plan_height": round(plan_height, 2),
        "plot_width": plot_width,
        "plot_height": plot_height,
        "overflow_width": round(max(0.0, plan_width - plot_width), 2),
        "overflow_height": round(max(0.0, plan_height - plot_height), 2),
    }


def _publish_progress_snapshot(callback, response, message, ready_count):
    """Publish one JSON-safe prefix using the normal Documents wire shape."""
    if not callable(callback) or ready_count <= 0:
        return
    try:
        documents = copy.deepcopy(response.to_dict()["Documents"])
        ready = min(int(ready_count), len(documents.get("floorPlans") or []))
        documents["count"] = ready
        documents["floorPlans"] = (documents.get("floorPlans") or [])[:ready]
        for field in ("postprocess", "adjacency_shortfalls", "plot_fit"):
            if isinstance(documents.get(field), list):
                documents[field] = documents[field][:ready]
        callback({
            "provisional": True,
            "message": message,
            "response": {"Documents": documents},
        })
    except Exception:
        # Serialization progress is observational and best effort.
        return


class _DoorProgressCollector:
    """Finalize a bounded provisional preview during topology solving.

    The handler reports candidates while it walks the topology pool, but none
    crosses this boundary in that raw form. Each candidate is deep-copied and
    put through the same rectilinear, fixed-room, cardinal, postprocess and
    exact-fill gates as the terminal batch. The preview is deliberately capped
    at eight plans and stops doing progress-only work once that window is full.
    Later candidates may outrank an emitted plan, so every snapshot is marked
    provisional and clients replace the whole window rather than append it.
    """

    def __init__(self, callback, ui, nodes_list, edges_list, fixed_rooms,
                 cardinal_pairs, postprocess_enabled, postprocess_options,
                 dim_inputs, document_id, name, count, limit):
        self.callback = callback
        self.ui = ui
        self.nodes_list = nodes_list
        self.edges_list = edges_list
        self.fixed_rooms = fixed_rooms
        self.cardinal_pairs = list(cardinal_pairs or [])
        self.postprocess_enabled = bool(postprocess_enabled)
        self.postprocess_options = postprocess_options
        self.dim_inputs = dim_inputs
        self.document_id = document_id
        self.name = name
        self.requested_count = int(count)
        self.cap = max(0, min(int(count), int(limit)))
        self.preview_cap = min(self.cap, 8)
        # Raw solved candidates are retained only until the bounded preview is
        # full. Finalization results are cached by sequence so a candidate is
        # never rectangularized or postprocessed twice solely for progress.
        self.entries = []
        self.finalized = {}
        self.published = {}
        self.sequence = 0
        self.last_signature = None
        self.last_emit_at = 0.0

        self.disclose_metric = True
        self.overlap_floor = 2.8
        if isinstance(postprocess_options, dict):
            try:
                requested = float(
                    postprocess_options.get("min_door_overlap", 2.0) or 2.0)
            except (TypeError, ValueError):
                requested = 2.0
            self.overlap_floor = max(self.overlap_floor, requested)

        self.disclose_plot_fit = (
            isinstance(dim_inputs, dict)
            and bool(dim_inputs.get("enforce_plot"))
            and float(dim_inputs.get("plot_width") or 0) > 0
            and float(dim_inputs.get("plot_height") or 0) > 0)
        self.plot_width = (
            float(dim_inputs.get("plot_width") or 0)
            if self.disclose_plot_fit else 0.0)
        self.plot_height = (
            float(dim_inputs.get("plot_height") or 0)
            if self.disclose_plot_fit else 0.0)

    def set_cardinal_pairs(self, cardinal_pairs):
        self.cardinal_pairs = list(cardinal_pairs or [])

    def reset_for_retry(self, cardinal_pairs):
        """Start a cardinal retry after the previous pass yielded no plans."""
        self.cardinal_pairs = list(cardinal_pairs or [])
        self.entries = []
        self.finalized = {}
        self.published = {}
        self.sequence = 0
        self.last_signature = None
        self.last_emit_at = 0.0

    def _finalize(self, candidate, sort_key):
        plan = copy.deepcopy(candidate)
        progress_ui = copy.copy(self.ui)
        progress_ui._set_output_data([plan])
        progress_ui.set_message("")
        # The terminal append helper mainly computes display label positions.
        # Seed those cheaply here; rectangularization rewrites every surviving
        # position to the final room center before serialization.
        if len(getattr(plan, "name_coords", []) or []) < len(
                plan.final_traversal):
            plan.name_coords = [
                [float(poly[0][0]), float(poly[0][1])]
                for poly in plan.final_traversal]

        rectangularize_output(progress_ui, self.fixed_rooms)
        if self.fixed_rooms:
            align_output_by_fixed(progress_ui, self.fixed_rooms)
        plans = progress_ui.get_output_data()
        if len(plans) != 1:
            return None
        plan = plans[0]
        if (self.cardinal_pairs
                and not plan_satisfies_cardinal(plan, self.cardinal_pairs)):
            return None
        if self.fixed_rooms and not plan_satisfies_fixed(plan, self.fixed_rooms):
            return None

        # Preserve a rejected postprocess/exact-fill result in the finalization
        # cache. The preview scanner can then skip it without repeating the
        # expensive production gates on every refresh milestone.
        entry = {
            "sort_key": tuple(sort_key),
            "sequence": self.sequence,
            "plan": None,
            "postprocess": None,
            "shortfalls": None,
            "plot_fit": None,
        }

        report = None
        if self.postprocess_enabled:
            from GPLAN.source.postprocessing.postprocess import postprocess_ui_output

            validator = None
            if self.cardinal_pairs or self.fixed_rooms:
                validator = (
                    lambda item:
                    (not self.cardinal_pairs
                     or plan_satisfies_cardinal(item, self.cardinal_pairs))
                    and plan_satisfies_fixed(item, self.fixed_rooms))
            reports, _note = postprocess_ui_output(
                progress_ui, self.nodes_list, self.edges_list,
                self.postprocess_options, 1, plan_validator=validator)
            report = reports[0] if reports else None
            plan = progress_ui.get_output_data()[0]

            exact_fill_enabled = not (
                isinstance(self.postprocess_options, dict)
                and self.postprocess_options.get("exact_fill") is False)
            hard_fixed_fill = (
                self.fixed_rooms and exact_fill_enabled
                and bool(self.dim_inputs.get("enforce_plot"))
                and float(self.dim_inputs.get("plot_width") or 0) > 0
                and float(self.dim_inputs.get("plot_height") or 0) > 0)
            if (hard_fixed_fill
                    and not plan_fills_plot_exactly(
                        plan, float(self.dim_inputs["plot_width"]),
                        float(self.dim_inputs["plot_height"]))):
                return entry

        if self.dim_inputs.get("strict_room_bounds"):
            from GPLAN.source.dimensioning.room_constraints import constrain_plan
            plan, _ = constrain_plan(plan, self.nodes_list, self.edges_list, strict=True)
            if plan is None:
                return entry

        serialized = _serialize_graph_plan(plan, self.nodes_list)
        shortfalls = _metric_shortfalls(
            plan.final_traversal, self.nodes_list, self.edges_list,
            self.overlap_floor)
        fit = (
            _plot_fit_report(
                plan.final_traversal, self.plot_width, self.plot_height)
            if self.disclose_plot_fit else None)
        entry.update({
            "geometry_key": _wire_plan_key(serialized),
            "plan": serialized,
            "postprocess": report,
            "shortfalls": shortfalls,
            "plot_fit": fit,
        })
        return entry

    def candidate(self, candidate, sort_key, remaining_attempts):
        """Accept one raw handler candidate, or a progress-only tick."""
        try:
            # Once eight fully gated previews exist, later callbacks are true
            # no-ops. This keeps terminal latency close to the legacy path.
            if (candidate is None or sort_key is None
                    or len(self.published) >= self.preview_cap):
                return
            self.sequence += 1
            self.entries.append({
                "candidate": candidate,
                "sort_key": tuple(sort_key),
                "sequence": self.sequence,
            })

            seen = len(self.entries)
            target = 1 if seen < 4 else (4 if seen < 8 else 8)
            target = min(target, self.preview_cap)
            if len(self.published) < target:
                self._refresh_preview(target)
        except Exception:
            # Progress is best effort. It must never change terminal generation.
            return

    def _refresh_preview(self, target):
        ranked = sorted(
            self.entries,
            key=lambda item: (item["sort_key"], item["sequence"]))
        preview = []
        for raw in ranked:
            sequence = raw["sequence"]
            if sequence not in self.finalized:
                finalized = self._finalize(
                    raw["candidate"], raw["sort_key"])
                if finalized is not None:
                    finalized["sequence"] = sequence
                self.finalized[sequence] = finalized
            entry = self.finalized[sequence]
            if entry is not None and entry["plan"] is not None:
                preview.append(entry)
            if len(preview) >= target:
                break
        self.published = {item["sequence"]: item for item in preview}
        # Emit only the deliberate 1, 4 and 8 milestones. When invalid raw
        # candidates make a target fill gradually, do not serialize and ship
        # every intermediate size (2, 3, 5, 6, 7).
        should_emit = (
            len(preview) >= target
            or (self.last_signature is None and len(preview) > 0))
        if should_emit:
            self._emit(force=True)

    def _snapshot(self):
        ranked = sorted(
            self.published.values(),
            key=lambda item: (item["sort_key"], item["sequence"]))
        documents = {
            "documentID": self.document_id,
            "name": self.name,
            "count": len(ranked),
            "floorPlans": [item["plan"] for item in ranked],
        }
        ptpg_graph = (
            self.ui.get_ptpg_graph()
            if hasattr(self.ui, "get_ptpg_graph") else None)
        if ptpg_graph:
            documents["ptpg_graph"] = ptpg_graph
        if self.postprocess_enabled:
            documents["postprocess"] = [
                item["postprocess"] for item in ranked]
        if self.disclose_metric:
            documents["adjacency_shortfalls"] = [
                item["shortfalls"] for item in ranked]
        if self.disclose_plot_fit:
            documents["plot_fit"] = [item["plot_fit"] for item in ranked]
        return {
            "provisional": True,
            "message": (
                "Generating Multiple Door connectivity floorplan. "
                + str(len(ranked)) + " floorplan(s) ready."),
            "response": {"Documents": documents},
        }

    def _emit(self, force=False):
        ready = len(self.published)
        signature = tuple(
            item["sequence"] for item in sorted(
                self.published.values(),
                key=lambda item: (item["sort_key"], item["sequence"])))
        if ready == 0 or signature == self.last_signature:
            return
        now = time.monotonic()
        if not force and now - self.last_emit_at < 0.75:
            return
        snapshot = self._snapshot()
        # Advance the throttle before invoking user code. A failing callback
        # must not be hammered with the same large snapshot on every solver
        # tick, and its failure still must not affect generation.
        self.last_signature = signature
        self.last_emit_at = now
        try:
            self.callback(copy.deepcopy(snapshot))
        except Exception:
            return


class Documents:
    @staticmethod
    def get_house_concepts(request_data, progress_callback=None):
        """Generate a versioned catalogue of complete validated house concepts.

        The separate integer-mm contract leaves legacy per-unit APIs unchanged.
        HTTP/task adapters wrap this result in response.HouseConcepts.
        """
        from GPLAN.housing import generate_house_concepts
        return generate_house_concepts(request_data, progress_callback=progress_callback)

    def __init__(self, hasMore, offset, documentID, name, count):
        self.hasMore = hasMore
        self.offset = offset
        self.documentID = documentID
        self.name = name
        self.count = count
        self.floorplans = []
        self.ptpg_graph = None
        self.postprocess = None
        self.adjacency_shortfalls = None
        self.plot_fit = None

    def append_floorplan(self,floorplan):
        if floorplan:
            self.floorplans.append([room.to_dict() for room in floorplan])

    def set_ptpg_graph(self, ptpg_graph):
        self.ptpg_graph = ptpg_graph

    def to_dict(self):
        doc_dict = {
            "documentID": self.documentID,
            "name": self.name,
            "count":self.count,
            # "hasMore": self.hasMore,
            # "offset": self.offset,
            "floorPlans": self.floorplans
        }
        if self.ptpg_graph is not None:
            doc_dict["ptpg_graph"] = self.ptpg_graph
        if self.postprocess is not None:
            # Per-plan NBC post-processing reports, index-aligned with
            # floorPlans (null for plans that were passed through untouched).
            doc_dict["postprocess"] = self.postprocess
        if self.adjacency_shortfalls is not None:
            # Per-plan metric adjacency disclosure, index-aligned with
            # floorPlans: requested adjacencies this plan realises with less
            # shared wall than a door needs (E2a of
            # documentation/plans/VALIDITY_AND_TOPOLOGY_ENGINE_PLAN.md). The
            # graph guarantee is combinatorial; a briefed edge can come back
            # as a 0 ft corner contact, and silence here is what made that
            # invisible to clients.
            doc_dict["adjacency_shortfalls"] = self.adjacency_shortfalls
        if self.plot_fit is not None:
            # Per-plan plot-fit disclosure, index-aligned with floorPlans.
            # Present only when the request opted into enforce_plot: each
            # entry says whether the plan's extent stays inside the enforced
            # plot and by how much it overflows when it does not. Clients
            # verify geometrically anyway; this is the engine owning its
            # answer rather than leaving it to be inferred.
            doc_dict["plot_fit"] = self.plot_fit

        if hasattr(self, "room_constraints"):
            doc_dict["room_constraints"] = self.room_constraints
        return {
            "Documents": doc_dict
        }

    @staticmethod
    def get_floorplans(starting_from: int, count: int, caller, nodes_list: list, graph: InputGraph, rectangular: bool, corridor=False,
                         dimensioned = False, dimensionedCirculation = False, minDimEnabled = False, removeAddCirculation = False, publicEnabled = False, nonAdj = False, normalize_const=40, limit=FLOORPLAN_LIMIT, corridor_thickness=None,documentID=None, name=None,circulationEnabled = 0, dim_inputs={},edges_list=[], non_adj_edge_list=[], cardinal_constraints=[], postProcessEnabled=False, postprocess_options=None, progress_callback=None):
        original_print = builtins.print

        def null_print(*args, **kwargs):
            pass

        message = ""
        dim_parameters: DimParameters = None
        dim_inputs = dict(dim_inputs or {})
        fixed_rooms = normalize_fixed_rooms(nodes_list, dim_inputs,
                                            bool(minDimEnabled))
        hard_cardinal_pairs = fixed_cardinal_pairs(fixed_rooms)
        # Validate the fixed-room wire contract before globally suppressing
        # the legacy engine's prints.  A malformed request must not leave
        # builtins.print disabled after normalize_fixed_rooms raises.
        builtins.print = null_print
        if minDimEnabled:
            # min_ratio/max_ratio ride along for the post-solve band
            # (_room_bounds / repair_dimensions / post-processing). The min-dim
            # SOLVER still ignores them; see minimum_dimensioning.input_constraints.
            dim_parameters = DimParameters(min_width=dim_inputs['min_width'], min_height=dim_inputs['min_height'],max_width=dim_inputs['max_width'], max_height=dim_inputs['max_height'], min_ratio=dim_inputs.get('min_ratio', []), max_ratio=dim_inputs.get('max_ratio', []), min_area=dim_inputs.get('min_area', []), max_area=dim_inputs.get('max_area', []), plot_width=dim_inputs['plot_width'], plot_height=dim_inputs['plot_height'], isOptimalEnabled=dim_inputs['optimal_floorplan'],isRotationAllowed = dim_inputs['rotation_enabled'], enforce_plot=dim_inputs.get('enforce_plot', False), fixed_rooms=fixed_rooms, strict_room_bounds=dim_inputs.get('strict_room_bounds', False), room_nodes=nodes_list, use_requested_room_bounds=dim_inputs.get('use_requested_room_bounds', False))
        elif dimensioned:
            dim_parameters = DimParameters(min_width=dim_inputs['min_width'], min_height=dim_inputs['min_height'], max_width=dim_inputs['max_width'], max_height=dim_inputs['max_height'], min_ratio=dim_inputs['min_ratio'], max_ratio=dim_inputs['max_ratio'], plot_width=dim_inputs['plot_width'], plot_height=dim_inputs['plot_height'], symmetric=dim_inputs['symmetric'], isOptimalEnabled=dim_inputs['optimal_floorplan'], strict_room_bounds=dim_inputs.get('strict_room_bounds', False), room_nodes=nodes_list, use_requested_room_bounds=dim_inputs.get('use_requested_room_bounds', False))
        ui = GuiParameters(graph=graph).set_isDimensioned(dimensioned).set_isDimensionedCirculation(
            dimensionedCirculation).set_isMinDimensioned(minDimEnabled).set_isRemoveAddCirculation(
            removeAddCirculation).set_isPublic(publicEnabled).set_min_dim_inputs(dim_parameters).set_isCirculation(circulationEnabled)
        original_print(ui)
        documentID = str(uuid.uuid4()) if documentID is None else documentID
        name = "Untitled Document" if name is None else name
        ui.set_message("")
        ui.set_fptype(caller)
        ui.set_edges(edges_list)
        roomColors = []
        roomNames = []
        for node in nodes_list:
            roomColors.append(node["color"])
            roomNames.append(node["label"])
        ui.set_roomNames(roomNames)
        ui.set_roomColors(roomColors)
        if corridor_thickness is not None:
            ui.set_corridor_thickness(corridor_thickness)
        if corridor:
            ui.set_command('circulation')
        if starting_from is None:
            starting_from = 0
        if count is None:
            count = 1
        nodes_data = []
        response = []
        # Optional, best-effort observer. Legacy callers omit it and retain
        # the exact synchronous behavior and return shape.
        progress_collector = None
        if count == 1:
            if caller == 'lshape':
                ui.set_letter("L Shape")
                for node in nodes_list:
                    node_obj = gui.gui_class.Nodes(node['id'], node['x'], node['y'])
                    nodes_data.append(node_obj)
                handle_letter_shape(ui, graph, nodes_data=nodes_data)
                message = 'Generated L shaped floorplan.'
                message += ui.get_message()
                print(message)
            elif caller == 'ushape':
                ui.set_letter("U Shape")
                handle_letter_shape(ui, graph)
                message = 'Generated U shaped floorplan.'
                message += ui.get_message()
                print(message)
            elif caller == 'tshape':
                ui.set_letter("T Shape")
                handle_letter_shape(ui, graph)
                message = 'Generated T shaped floorplan.'
                message += ui.get_message()
                print(message)
            elif caller == 'zshape':
                ui.set_letter("Z Shape")
                handle_letter_shape(ui, graph)
                message = 'Generated Z shaped floorplan.'
                message += ui.get_message()
                print(message)
            elif caller == 'staircaseshape':
                handle_staircase_shaped(ui, graph)
                message = 'Generated Staircase shaped floorplan.'
                message += ui.get_message()
                print(message)
            # elif caller == 'pentagonal': #Support Not added yet
            #     Pentagonal.PentagonalFloorplan(ui, graph, nodes_list)
            # elif caller == 'hexagonal': #Support Not added yet
            #     Hexagonal.HexagonalFloorplan(ui, graph, nodes_list)
            # elif caller == 'custom': #Support Not added yet
            # Customplot.CustomplotFloorplan(graph, nodes_list)
            elif caller == 'rectangular' or caller == 'irregular':
                if rectangular:
                    handle_single_oc(ui, graph)
                else:
                    handle_single(ui, graph)

            elif caller == "door_connectivity":
                ui.set_isNonAdj(nonAdj)
                ui.set_non_adj_list(non_adj_edge_list)
                ui.set_is_multiple_door(False)
                graph.cardinal_constraints = merge_cardinal_pairs(
                    normalize_cardinal_constraints(cardinal_constraints,
                                                   len(nodes_list)),
                    hard_cardinal_pairs)
                apply_cardinal_ring(graph, len(nodes_list), edges_list,
                                    graph.cardinal_constraints, non_adj_edge_list,
                                    nodes_list=nodes_list, fixed_rooms=fixed_rooms)
                handle_door_connectivity(ui, graph)
                if fixed_rooms:
                    align_output_by_fixed(ui, fixed_rooms)
            else:
                message = f"Support for {caller} Not yet Handled from Backend for Single Floorplan"
                print(message)
        else:
            if caller == 'lshape':
                print("Generating Multiple L shape")
                ui.set_letter("L Shape")
                for node in nodes_list:
                    node_obj = gui.gui_class.Nodes(node['id'], node['x'], node['y'])
                    nodes_data.append(node_obj)
                handle_multiple_l(ui, graph, nodes_data=nodes_data)
                message = 'Generated L shaped floorplan.'
                message += ui.get_message()
                print(message)
            elif caller == 'rectangular':
                handle_multiple_oc(ui, graph)
            elif caller == 'irregular':
                handle_multiple(ui, graph)
            elif caller == "door_connectivity":
                ui.set_isNonAdj(nonAdj)
                ui.set_non_adj_list(non_adj_edge_list)
                ui.set_is_multiple_door(True)
                requested_cardinal_pairs = normalize_cardinal_constraints(
                    cardinal_constraints, len(nodes_list))
                cardinal_pairs = merge_cardinal_pairs(requested_cardinal_pairs,
                                                       hard_cardinal_pairs)
                graph.cardinal_constraints = cardinal_pairs
                # Fixed-room catalogues are the bounded early-preview path:
                # their solver disables rotation and both expand/top-up, so a
                # small fully gated window can be produced without tracing
                # later candidate families. Other request shapes publish only
                # serialization-time prefixes below.
                if (callable(progress_callback) and fixed_rooms
                        and minDimEnabled
                        and int(dim_inputs.get("optimal_floorplan", 1) or 0) == 1):
                    progress_collector = _DoorProgressCollector(
                        progress_callback, ui, nodes_list, edges_list,
                        fixed_rooms, cardinal_pairs, postProcessEnabled,
                        postprocess_options, dim_inputs, documentID, name,
                        count, limit)
                if not non_adj_edge_list and not fixed_rooms and not dim_inputs.get("strict_room_bounds"):
                    # One-connected graphs (two wings joined by one cut-vertex
                    # room) compose better as stacked component duals than
                    # through biconnectivity augmentation, which invents a
                    # fake adjacency and merges dummy rooms.
                    stacked_plans, stacked_note = stacked_oneconnected_floorplans(
                        nodes_list, edges_list, count, dim_inputs,
                        cardinal_pairs, corridor_thickness, limit)
                    if stacked_plans:
                        message = ('Generated Multiple Door connectivity floorplan.'
                                   + stacked_note)
                        response = Documents(False, 0, documentID, name,
                                             min(len(stacked_plans), limit, count))
                        response.floorplans = stacked_plans[:min(limit, count)]
                        if postProcessEnabled and minDimEnabled:
                            # This path returns pre-serialized plans and skips
                            # the shared serialization loop below, so it gets
                            # the JSON-level post-processing adapter instead.
                            from GPLAN.source.postprocessing.postprocess import (
                                postprocess_serialized_plans)
                            # Stacked plans are name-keyed (labels are unique on
                            # this path), so hand the doors over as name pairs.
                            door_edges = [(nodes_list[int(e[0])]["label"],
                                           nodes_list[int(e[1])]["label"])
                                          for e in edges_list
                                          if e is not None and len(e) >= 2
                                          and int(e[0]) < len(nodes_list)
                                          and int(e[1]) < len(nodes_list)]
                            # Same plot priority as the ui path: the request's
                            # plot is the hard cap unless the caller sent one.
                            stacked_options = dict(postprocess_options or {})
                            for key in ("plot_width", "plot_height"):
                                if stacked_options.get(key) is None \
                                        and dim_inputs.get(key):
                                    stacked_options[key] = dim_inputs[key]
                            new_plans, pp_reports, pp_note = \
                                postprocess_serialized_plans(
                                    response.floorplans,
                                    stacked_options,
                                    edges=door_edges)
                            response.floorplans = new_plans
                            response.postprocess = pp_reports
                            message += pp_note
                        from GPLAN.source.dimensioning.room_constraints import validate
                        reports = []
                        by_name = {n["label"]: n for n in nodes_list}
                        for plan in response.floorplans:
                            ordered_nodes = [by_name.get(r["name"], {}) for r in plan]
                            failures = validate([r["circular_coordinates"] for r in plan], ordered_nodes)
                            reports.append({"satisfied": not failures, "violations": failures,
                                            "relaxed": bool(failures)})
                        response.room_constraints = {
                            "mode": "best_effort", "plans": reports,
                            "status": "relaxed" if any(r["violations"] for r in reports) else "satisfied",
                            "failed_bounds": [],
                        }
                        _publish_progress_snapshot(
                            progress_callback, response, message,
                            len(response.floorplans))
                        builtins.print = original_print
                        return response, message
                apply_cardinal_ring(graph, len(nodes_list), edges_list,
                                    cardinal_pairs, non_adj_edge_list,
                                    nodes_list=nodes_list, fixed_rooms=fixed_rooms)
                handle_door_connectivity(
                    ui, graph,
                    candidate_callback=(progress_collector.candidate
                                        if progress_collector else None))
                original_plans = list(ui.get_output_data())
                # Capture ui's message AFTER the gapless pass: that pass emits
                # its own warnings (a room pushed past its maximum to close a
                # hole), and reading the message first dropped them silently.
                gated_plans = rectangularize_output(ui, fixed_rooms)
                if fixed_rooms:
                    gated_plans = align_output_by_fixed(ui, fixed_rooms)
                message = 'Generated Multiple Door connectivity floorplan.'+ ui.get_message()
                if (not gated_plans and original_plans and not cardinal_pairs
                        and not fixed_rooms):
                    # The gapless gate emptied the batch - return the ungated
                    # plans with an honest note rather than nothing.
                    ui._set_output_data(original_plans)
                    message += ' Some floorplans have a non-rectangular outline.'
                if cardinal_pairs and len(filter_output_by_cardinal(ui, cardinal_pairs)) == 0:
                    # Cardinal constraints take priority over adjacency wishes:
                    # retry with the adjacency set relaxed to a spanning tree
                    # (drops the fewest edges that still keep every room reachable),
                    # then as a last resort drop the cardinal filter so the caller
                    # always receives floorplans. Every constrained attempt is
                    # verified geometrically - only plans that satisfy all pins
                    # count as output, so callers can populate their full batch
                    # from satisfying plans alone.
                    node_coordinates = [[node.get("x", 0), node.get("y", 0)]
                                        for node in nodes_list]
                    tree_edges = spanning_tree_edges(edges_list, len(nodes_list))
                    attempts = []

                    def add_attempt(attempt_edges, attempt_cardinal, note):
                        key = (tuple((int(e[0]), int(e[1])) for e in attempt_edges),
                               tuple(attempt_cardinal))
                        if all(existing[0] != key for existing in attempts):
                            attempts.append((key, attempt_edges,
                                             attempt_cardinal, note))

                    if len(tree_edges) < len(edges_list):
                        add_attempt(
                            tree_edges, cardinal_pairs,
                            " Adjacency constraints were relaxed to satisfy cardinal directions.")
                    if hard_cardinal_pairs:
                        # A fixed anchor is a hard geometric constraint.  Soft
                        # user cardinal wishes may still be dropped, but the
                        # fixed room's anchor directions survive every retry and
                        # there is deliberately no invalid last-resort plan.
                        if any(pair not in hard_cardinal_pairs
                               for pair in cardinal_pairs):
                            add_attempt(
                                edges_list, hard_cardinal_pairs,
                                " Non-fixed cardinal constraints could not be satisfied and were ignored.")
                            if len(tree_edges) < len(edges_list):
                                add_attempt(
                                    tree_edges, hard_cardinal_pairs,
                                    " Adjacency and non-fixed cardinal constraints were relaxed to keep fixed-room anchors.")
                    else:
                        add_attempt(
                            edges_list, [],
                            " Cardinal constraints could not be satisfied and were ignored.")
                    for _key, attempt_edges, attempt_cardinal, note in attempts:
                        retry_graph = InputGraph(len(nodes_list), len(attempt_edges),
                                                 attempt_edges, node_coordinates)
                        retry_graph.cardinal_constraints = attempt_cardinal
                        apply_cardinal_ring(retry_graph, len(nodes_list),
                                            attempt_edges, attempt_cardinal,
                                            non_adj_edge_list,
                                    nodes_list=nodes_list, fixed_rooms=fixed_rooms)
                        if progress_collector:
                            # This block is entered only when the preceding
                            # cardinal gate retained zero plans, so no prior
                            # externally published plan can be retracted.
                            progress_collector.reset_for_retry(
                                attempt_cardinal)
                        handle_door_connectivity(
                            ui, retry_graph,
                            candidate_callback=(progress_collector.candidate
                                                if progress_collector else None))
                        retry_original = list(ui.get_output_data())
                        rectangularize_output(ui, fixed_rooms)
                        if fixed_rooms:
                            align_output_by_fixed(ui, fixed_rooms)
                        if attempt_cardinal:
                            filter_output_by_cardinal(ui, attempt_cardinal)
                        elif (not ui.get_output_data() and retry_original
                              and not fixed_rooms):
                            ui._set_output_data(retry_original)
                            note += ' Some floorplans have a non-rectangular outline.'
                        if len(ui.get_output_data()) > 0:
                            graph = retry_graph
                            # Re-capture ui's accumulated message: the retry
                            # re-ran handle_door_connectivity, and warnings it
                            # emitted (e.g. "plot was expanded") must reach the
                            # caller - the capture above predates the retry.
                            message = ('Generated Multiple Door connectivity floorplan.'
                                       + ui.get_message() + note)
                            break
                    if fixed_rooms and not ui.get_output_data():
                        message += (' Fixed-room dimensions/anchors are hard;'
                                    ' no invalid fallback floorplan was returned.')
                elif fixed_rooms and not ui.get_output_data():
                    # Exact-size-only fixed rooms have no injected cardinal
                    # pair, so the cardinal retry block above is not entered.
                    # Their infeasibility must still be explicit and must not
                    # restore the legacy ungated batch.
                    message += (' Fixed-room dimensions/anchors are hard;'
                                ' no invalid fallback floorplan was returned.')
            elif caller == "multiple_l":
                handle_multiple_l(ui, graph)
            else:
                message = f"Support for {caller} Not yet Handled from Backend for Multiple Floorplan"
                print(message)
        if count == 1:
            outputData = ui.get_output_data()
        elif count > 1:
            outputData = ui.get_output_data()
        offset = 0
        hasMore = graph.fpcnt - offset - 1 > 0
        total_fp_count = min(min(len(outputData), limit),count)
        response = Documents(hasMore, offset, documentID, name, total_fp_count)

        # NBC post-processing: bring rooms inside their aspect bands and
        # service-room ceilings, trading the gapless rectangle for boundary
        # notches where needed. Only meaningful on dimensioned catalogues -
        # dimensionless duals are integer grids the rulebook does not apply to.
        postprocess_reports = None
        if postProcessEnabled and minDimEnabled and caller == "door_connectivity":
            from GPLAN.source.postprocessing.postprocess import postprocess_ui_output
            # Cardinal pins were verified BEFORE this point; a room absorbing
            # notch space could grow into the strip between a pinned room and
            # its side, so any post-processed plan must re-pass the pin check
            # or be reverted.
            pp_cardinal_pairs = merge_cardinal_pairs(
                normalize_cardinal_constraints(cardinal_constraints,
                                               len(nodes_list)),
                hard_cardinal_pairs)
            validator = None
            if pp_cardinal_pairs or fixed_rooms:
                validator = (
                    lambda plan:
                    (not pp_cardinal_pairs
                     or plan_satisfies_cardinal(plan, pp_cardinal_pairs))
                    and plan_satisfies_fixed(plan, fixed_rooms))
            postprocess_reports, pp_note = postprocess_ui_output(
                ui, nodes_list, edges_list, postprocess_options, total_fp_count,
                plan_validator=validator)
            message += pp_note
            response.postprocess = postprocess_reports

            # A fixed core turns the enforced plot into an absolute frame, and
            # exact-fill is a hard output invariant rather than a catalogue
            # preference.  Never serialize a validator rollback or a bbox-only
            # result with an internal notch (shown by housing as "Empty space").
            exact_fill_enabled = not (
                isinstance(postprocess_options, dict)
                and postprocess_options.get("exact_fill") is False)
            hard_fixed_fill = (
                fixed_rooms and exact_fill_enabled
                and isinstance(dim_inputs, dict)
                and bool(dim_inputs.get("enforce_plot"))
                and float(dim_inputs.get("plot_width") or 0) > 0
                and float(dim_inputs.get("plot_height") or 0) > 0)
            if hard_fixed_fill:
                exact_w = float(dim_inputs["plot_width"])
                exact_h = float(dim_inputs["plot_height"])
                kept_plans = []
                kept_reports = []
                considered = list(ui.get_output_data())[:total_fp_count]
                for plan, report in zip(considered, postprocess_reports):
                    if (plan_satisfies_fixed(plan, fixed_rooms)
                            and plan_fills_plot_exactly(plan, exact_w, exact_h)):
                        kept_plans.append(plan)
                        kept_reports.append(report)
                dropped = len(considered) - len(kept_plans)
                ui._set_output_data(kept_plans)
                outputData = kept_plans
                postprocess_reports = kept_reports
                total_fp_count = len(kept_plans)
                response.count = total_fp_count
                response.postprocess = postprocess_reports
                if dropped:
                    message += (
                        " %d floorplan(s) were dropped because hard fixed-room"
                        " exact-fill could not cover the complete plot; no"
                        " invalid fallback was returned." % dropped)

        # Add ptpg_graph if available
        if caller == "door_connectivity":
            ptpg_graph = ui.get_ptpg_graph() if hasattr(ui, 'get_ptpg_graph') else None
            if ptpg_graph:
                response.set_ptpg_graph(ptpg_graph)

        # Metric adjacency disclosure (E2a): the dimensioning can realise a
        # briefed edge as a corner contact or a sliver no door fits through
        # (measured: an 8-room brief realised one edge at 0.00 ft in every
        # plan of the batch). Judged against the REQUESTED edges, whatever
        # relaxation later stages applied, because the client asked for those.
        disclose_metric = caller == "door_connectivity" and minDimEnabled
        # Disclosure floor: at least the dresser's 2.8 ft door threshold, not
        # the 2.0 ft post-processing door-PROTECTION value - the solver's
        # relaxation rung can legally emit a 2.0-2.8 ft wall (see
        # handlers.solve_min_dim), and a wall in that band still holds no door.
        overlap_floor = 2.8
        if isinstance(postprocess_options, dict):
            try:
                requested = float(postprocess_options.get("min_door_overlap", 2.0) or 2.0)
            except (TypeError, ValueError):
                requested = 2.0
            overlap_floor = max(overlap_floor, requested)
        adjacency_shortfalls = []

        # Plot-fit disclosure (enforce_plot only): judged on the FINAL
        # geometry, i.e. after post-processing grew or trimmed the plans.
        disclose_plot_fit = (
            caller == "door_connectivity" and minDimEnabled
            and isinstance(dim_inputs, dict) and dim_inputs.get("enforce_plot")
            and (dim_inputs.get("plot_width") or 0) > 0
            and (dim_inputs.get("plot_height") or 0) > 0)
        fit_plot_w = float(dim_inputs.get("plot_width") or 0) if disclose_plot_fit else 0.0
        fit_plot_h = float(dim_inputs.get("plot_height") or 0) if disclose_plot_fit else 0.0
        plot_fit_reports = []

        if disclose_metric:
            response.adjacency_shortfalls = adjacency_shortfalls
        if disclose_plot_fit:
            response.plot_fit = plot_fit_reports

        from GPLAN.source.dimensioning.room_constraints import constrain_plan, unique_violations
        strict_rooms = bool(dim_inputs.get("strict_room_bounds"))
        constraint_reports, rejected, accepted = [], [], []
        accepted_indices = []
        for item_index, item in enumerate(outputData):
            checked, violations = constrain_plan(item, nodes_list, edges_list, strict=strict_rooms)
            if checked is None:
                rejected.extend(violations)
            else:
                accepted.append(checked)
                accepted_indices.append(item_index)
                constraint_reports.append({"satisfied": not violations, "violations": violations,
                                           "relaxed": bool(violations) and not strict_rooms})
        outputData = accepted
        if response.postprocess is not None:
            response.postprocess = [response.postprocess[i] for i in accepted_indices
                                    if i < len(response.postprocess)]
        response.room_constraints = {
            "mode": "strict" if strict_rooms else "best_effort",
            "status": "infeasible" if not accepted else ("relaxed" if any(r["violations"] for r in constraint_reports) else "satisfied"),
            "plans": constraint_reports[:min(limit, count)],
            "failed_bounds": unique_violations(rejected + getattr(ui, "_room_constraint_failures", [])) if not accepted else [],
        }
        if strict_rooms and not accepted:
            # A solver-infeasible topology has no final dimensions to measure.
            # Report the requested bounds as candidates, without claiming an IIS.
            response.room_constraints["requested_bounds"] = [
                {"room_id": n.get("id"), "label": n.get("label"),
                 "width": n.get("width"), "height": n.get("height"),
                 "min_area": n.get("min_area"), "max_area": n.get("max_area")}
                for n in nodes_list]
            message += " No plan satisfies the requested hard room bounds and required connections."

        serialization_count = min(min(len(outputData), limit), count)
        response.count = serialization_count
        for index in range(serialization_count):
            floorplanData = outputData[index].final_traversal
            response.floorplans.append(
                _serialize_graph_plan(outputData[index], nodes_list))

            if disclose_metric:
                adjacency_shortfalls.append(_metric_shortfalls(
                    floorplanData, nodes_list, edges_list, overlap_floor))

            if disclose_plot_fit:
                plot_fit_reports.append(_plot_fit_report(
                    floorplanData, fit_plot_w, fit_plot_h))

            ready = index + 1
            if (progress_collector is None and callable(progress_callback)
                    and (ready == 1 or ready % 4 == 0
                         or ready == serialization_count)):
                _publish_progress_snapshot(
                    progress_callback, response, message, ready)

        if disclose_metric:
            response.adjacency_shortfalls = adjacency_shortfalls
            affected = sum(1 for s in adjacency_shortfalls if s)
            if affected:
                message += (f" {affected} floorplan(s) realise at least one requested "
                            f"adjacency with under {overlap_floor:g} ft of shared wall "
                            f"(no room for a door there); the pairs are listed per plan "
                            f"in adjacency_shortfalls.")

        if disclose_plot_fit:
            response.plot_fit = plot_fit_reports
            fitting = sum(1 for r in plot_fit_reports if r["fits"])
            message += (f" {fitting} of {len(plot_fit_reports)} floorplan(s) fit within "
                        f"the {fit_plot_w:g} x {fit_plot_h:g} ft plot; per-plan extents "
                        f"are listed in plot_fit.")

        builtins.print = original_print

        return response, message

    @staticmethod
    def postprocess_floorplans(request_data):
        """Standalone NBC post-processing of already-generated floorplans.

        Accepts either the documented response envelope or a bare batch:
          {"floorPlans": [[room, ...], ...]}                       - bare
          {"response": {"Documents": {"floorPlans": [...]}}}      - envelope
          {"Documents": {"floorPlans": [...]}}                    - envelope

        Optional fields:
          "edges":   requested adjacencies (doors) as index pairs or
                     room-name pairs; protected pairs keep >= the configured
                     door overlap of shared wall.
          "options": see postprocess.DEFAULT_OPTIONS - phases on/off, the
                     aspect band, per-type rule overrides, door overlap.

        Returns (response_dict, message). response_dict carries the same
        Documents envelope with rewritten geometry plus a "postprocess" list
        of per-plan reports (null entries = plan passed through untouched).
        """
        from GPLAN.source.postprocessing.postprocess import (
            postprocess_serialized_plans)

        data = request_data or {}
        container = data
        if isinstance(container.get("response"), dict):
            container = container["response"]
        if isinstance(container.get("Documents"), dict):
            container = container["Documents"]
        floorplans = container.get("floorPlans") or data.get("floorPlans") or []
        if not isinstance(floorplans, list):
            raise ValueError("floorPlans must be a list of floorplans")
        # A single plan (list of room dicts) is accepted and re-wrapped.
        if floorplans and isinstance(floorplans[0], dict):
            floorplans = [floorplans]

        new_plans, reports, note = postprocess_serialized_plans(
            floorplans, data.get("options"), edges=data.get("edges"))

        document_id = container.get("documentID") or str(uuid.uuid4())
        doc_name = container.get("name") or "Untitled Document"
        response = {
            "Documents": {
                "documentID": document_id,
                "name": doc_name,
                "count": len(new_plans),
                "floorPlans": new_plans,
                "postprocess": reports,
            }
        }
        message = ("Post-processed %d floorplan(s)." % len(new_plans)) + note
        return response, message

    @staticmethod
    def allocate_program(request_data):
        """Area-budget allocation preview: the size envelope generation will
        use, without generating anything. Synchronous and pure (no solver).

        Request:
          {"rooms": [{"name", optional clear-feet overrides
                      min_width/min_height/max_width/max_height}, ...],
           "plot_width": 36, "plot_height": 28,
           "wall_allowance_ft": 0.4,          # optional
           "tier_share": [1.0, 1.0, 1.0]}     # optional

        Returns (response_dict, message); response_dict is
        {"allocation": {...}} - see source/dimensioning/allocator.allocate.
        Raises ValueError (a 400 at the view) for a malformed program, e.g.
        a user maximum below the NBC minimum.
        """
        from GPLAN.source.dimensioning.allocator import allocate

        data = request_data or {}
        allocation = allocate(
            data.get("rooms"),
            data.get("plot_width"),
            data.get("plot_height"),
            wall_allowance_ft=data.get("wall_allowance_ft"),
            tier_share=data.get("tier_share"),
        )
        if allocation["fits"]:
            message = ("Allocated %d rooms across %s sqft;"
                       " %s sqft of surplus distributed."
                       % (len(allocation["rooms"]),
                          allocation["plot"]["area"],
                          allocation["surplus"]))
        else:
            message = ("Program minimum %s sqft exceeds the %s sqft plot by"
                       " %s sqft; every room is at its minimum and generation"
                       " will overflow the plot."
                       % (allocation["program_min_area"],
                          allocation["plot"]["area"],
                          allocation["shortfall_sqft"]))
        return {"allocation": allocation}, message

    @staticmethod
    def get_space_optimized_floorplan(request_data):
        """
        Generate a space-optimized floorplan using the negNew FloorPlan engine.
        
        Args:
            request_data: Dictionary containing:
                - request_id: Unique request identifier
                - params: Dictionary with:
                    - regions: List of rectangular regions (optional if boundary is provided)
                    - boundary: List of [x,y] points defining polygonal boundary (optional if regions provided)
                    - rooms: List of room specifications
                    - fixed_rooms: List of fixed room specifications
                    - adjacency: List of adjacency requirements
                    - non_adjacency: List of non-adjacency requirements
                    - entrance_coords: Entrance location
                - ops: List of operations to perform (e.g., ["place", "compact", "expand", "score"])
        
        Returns:
            Dictionary with response data matching the expected output format
        """
        original_print = builtins.print

        def null_print(*args, **kwargs):
            pass

        builtins.print = null_print

        try:
            request_id = request_data.get('request_id', str(uuid.uuid4()))
            params = request_data.get('params', {})
            ops = request_data.get('ops', ['place', 'compact', 'expand', 'score'])
            
            # Extract parameters
            # Support EITHER regions OR boundary
            regions = params.get('regions', None)
            boundary = params.get('boundary', None)
            rooms = params.get('rooms', [])
            fixed_rooms = params.get('fixed_rooms', [])
            adjacency = params.get('adjacency', [])
            non_adjacency = params.get('non_adjacency', [])
            entrance_coords = params.get('entrance_coords', None)

            fixed_room_polygons = []
            for fixed_room in fixed_rooms:
                if not isinstance(fixed_room, dict):
                    continue
                polygon = fixed_room.get('polygon')
                if polygon:
                    fixed_room_polygons.append(polygon)
                    continue
                try:
                    fx = int(fixed_room['x'])
                    fy = int(fixed_room['y'])
                    fw = int(fixed_room['width'])
                    fh = int(fixed_room['height'])
                except Exception:
                    continue
                fixed_room_polygons.append([
                    [fx, fy],
                    [fx + fw, fy],
                    [fx + fw, fy + fh],
                    [fx, fy + fh],
                ])
            
            # If boundary is provided instead of regions, convert it
            if boundary and not regions:
                try:
                    regions = boundary_to_regions(boundary, fixed_rooms=fixed_room_polygons)
                    print(f"Converted boundary with {len(boundary)} points to {len(regions)} regions")
                except Exception as e:
                    builtins.print = original_print
                    return {
                        'request_id': request_id,
                        'status': 'error',
                        'data': {},
                        'error': {'message': f'Failed to convert boundary to regions: {str(e)}'}
                    }

            boundary_bounds = None
            if boundary:
                boundary_bounds = {
                    'min_x': min(point[0] for point in boundary),
                    'min_y': min(point[1] for point in boundary),
                    'max_x': max(point[0] for point in boundary),
                    'max_y': max(point[1] for point in boundary),
                }
            
            # Validate that we have regions (either provided or converted)
            if not regions:
                builtins.print = original_print
                return {
                    'request_id': request_id,
                    'status': 'error',
                    'data': {},
                    'error': {'message': 'Either regions or boundary must be provided'}
                }
            
            # Operation flags
            enable_expansion = 'expand' in ops
            enable_compaction = 'compact' in ops
            max_attempts = params.get('max_attempts', 100)
            
            # Create UI object for output storage
            from GPLAN.pythongui.GuiParameters import GuiParameters
            ui = GuiParameters(graph=None)
            
            # Call the handler
            success = handle_space_optimization(
                ui=ui,
                regions=regions,
                rooms=rooms,
                fixed_rooms=fixed_rooms,
                adjacency=adjacency,
                non_adjacency=non_adjacency,
                entrance_coords=entrance_coords,
                max_attempts=max_attempts,
                enable_expansion=enable_expansion,
                enable_compaction=enable_compaction,
                boundary_bounds=boundary_bounds
            )
            
            if not success:
                builtins.print = original_print
                return {
                    'request_id': request_id,
                    'status': 'error',
                    'data': {},
                    'error': {'message': ui.get_message()}
                }
            
            # Extract output data
            output_data = ui.get_output_data()
            if not output_data or len(output_data) == 0:
                builtins.print = original_print
                return {
                    'request_id': request_id,
                    'status': 'error',
                    'data': {},
                    'error': {'message': 'No output data generated'}
                }
            
            result = output_data[0]
            
            # Calculate floor dimensions
            floor_width = max(r['x'] + r['width'] for r in regions) if regions else 0
            floor_height = max(r['y'] + r['height'] for r in regions) if regions else 0
            
            # Format response
            response = {
                'request_id': request_id,
                'status': 'ok',
                'data': {
                    'floor': {
                        'width': floor_width,
                        'height': floor_height,
                        'regions': regions,
                        'entrance_coords': entrance_coords
                    },
                    'placements': result['rooms'],
                    'metrics': {
                        'ops_run': ops,
                        'adjacency_score': result['metrics']['adjacency_score'] / len(adjacency) if adjacency else 1.0,
                        'satisfied_pairs': result['metrics']['satisfied_pairs'],
                        'non_adjacency_violations': result['metrics']['non_adjacency_violations'],
                        'entrance_adjacent_rooms': result['metrics'].get('entrance_adjacent_rooms', []),
                        'area_utilization': result['metrics']['area_utilization'],
                        'integrity': {
                            'overlaps': [],
                            'out_of_bounds': [],
                            'invalid_adjacent_edges': []
                        }
                    }
                },
                'error': {}
            }
            
            builtins.print = original_print
            return response
            
        except Exception as e:
            builtins.print = original_print
            import traceback
            traceback.print_exc()
            return {
                'request_id': request_data.get('request_id', 'unknown'),
                'status': 'error',
                'data': {},
                'error': {'message': str(e)}
            }

    @staticmethod
    def get_ga_optimized_floorplan(request_data):
        """
        Generate a GA-optimized floorplan using genetic algorithm with corridor generation.
        
        Args:
            request_data: Dictionary containing:
                - request_id: Unique request identifier
                - engine: Should be "GA_FloorPlan"
                - params: Dictionary with:
                    - plot_width: Width of the plot
                    - plot_height: Height of the plot
                    - corridor_width: Width of corridors (default: 3)
                    - rooms: List of room objects with walls
                    - walls: List of all walls
                    - labels: List of labels (optional)
                    - ga_config: GA configuration (optional)
                        - max_workers: Number of workers for parallel processing (default: 1)
        
        Returns:
            Dictionary with response data matching temp2.json format
        """
        original_print = builtins.print

        def null_print(*args, **kwargs):
            pass

        builtins.print = null_print

        try:
            request_id = request_data.get('request_id', str(uuid.uuid4()))
            params = request_data.get('params', {})
            
            # Build floorplan_data in the format ga_current expects
            floorplan_data = {
                'plot_width': params.get('plot_width', 40),
                'plot_height': params.get('plot_height', 30),
                'rooms': params.get('rooms', []),
                'walls': params.get('walls', []),
                'labels': params.get('labels', []),
                'windows': params.get('windows', []),
                'doors': params.get('doors', [])
            }
            
            corridor_width = params.get('corridor_width', 3)
            ga_config = params.get('ga_config', None)
            
            # Create UI object for output storage
            from GPLAN.pythongui.GuiParameters import GuiParameters
            ui = GuiParameters(graph=None)
            
            # Call the GA handler
            success = handle_ga_optimization(
                ui=ui,
                floorplan_data=floorplan_data,
                corridor_width=corridor_width,
                ga_config=ga_config
            )
            
            if not success:
                builtins.print = original_print
                return {
                    'request_id': request_id,
                    'status': 'error',
                    'engine': 'GA_FloorPlan',
                    'data': {},
                    'error': {'message': ui.get_message()}
                }
            
            # Extract output data
            outputData = ui.get_output_data() if hasattr(ui, 'get_output_data') else []
            if not outputData:
                outputData = []
            
            if not outputData or len(outputData) == 0:
                builtins.print = original_print
                return {
                    'request_id': request_id,
                    'status': 'error',
                    'engine': 'GA_FloorPlan',
                    'data': {},
                    'error': {'message': 'No output data generated'}
                }
            
            result = outputData[0]
            
            # Format response matching temp2.json structure
            response = {
                'request_id': request_id,
                'status': 'success',
                'engine': 'GA_FloorPlan',
                'data': {
                    'floor': {
                        'plot_width': result['plot_width'],
                        'plot_height': result['plot_height']
                    },
                    'ga_optimization': {
                        'status': 'converged',
                        'solution_chromosome': result['ga_solution']
                    },
                    'rooms': result['rooms'],
                    'walls': result['walls'],
                    'labels': result['labels'],
                    'windows': result.get('windows', []),
                    'doors': result.get('doors', []),
                    'layout_matrix': result['layout_matrix']
                },
                'error': {}
            }
            
            builtins.print = original_print
            return response
            
        except Exception as e:
            builtins.print = original_print
            import traceback
            traceback.print_exc()
            return {
                'request_id': request_data.get('request_id', 'unknown'),
                'status': 'error',
                'engine': 'GA_FloorPlan',
                'data': {},
                'error': {'message': str(e)}
            }

    @staticmethod
    def get_multi_ptpg_floorplans(request_data):
        """Dimensioned floorplans for every topological PTPG variant of one graph.

        Combines the two GPLAN-team branches: ``QA_multi_ptpg`` enumerates the
        PTPG variants of the input adjacency graph, and the ``ptpg_floorplanner``
        exact-dimension placer runs over each of them. See
        :mod:`GPLAN.source.multi_ptpg_pipeline` for the request/response shape.

        Args:
            request_data: ``{"request_id": str, "params": {...}}`` where params
                carries ``nodes`` (each with an exact ``width``/``height``),
                ``edges``, and the optional variant/strictness knobs.

        Returns:
            ``{"request_id", "status", "engine", "data", "error"}``
        """
        from GPLAN.source import multi_ptpg_pipeline

        original_print = builtins.print

        def null_print(*args, **kwargs):
            pass

        builtins.print = null_print
        try:
            return multi_ptpg_pipeline.run(request_data)
        finally:
            builtins.print = original_print

    # -----------------------------------------------------------------
    # structural module (documentation/structural_api.md)
    #
    # Four thin delegations and nothing else: GPLAN/structural/api.py owns
    # every rule, and these exist so the bridge, the Django views and the
    # Celery tasks all reach the engine through the same facade the rest of
    # this class provides. The import sits INSIDE each method on purpose -
    # the structural package pulls in numpy, shapely and the YAML code
    # tables, and a package that fails to import must not take
    # get_floorplans down with it.
    #
    # Every one of them returns the full engine envelope
    # {status, message, response: {Documents: {structural: [...],
    # batch_summary, ...}}, disclaimer} rather than the (response, message)
    # pair the older methods return: the structural responses carry a
    # status of their own (a refusal is still a 200 with an ERROR envelope)
    # and the verbatim disclaimer, and splitting that into a tuple would
    # lose both.
    # -----------------------------------------------------------------

    @staticmethod
    def layout_structure(payload, **options):
        """Placement only: grid, columns, beams, slabs, walls, bands, and
        UNSIZED footing markers at the ground column stacks.

        Fast and synchronous (sub-second to a few seconds): the same adapt /
        validate / place prefix ``design_structure`` runs, stopping before the
        load model, so layout and design can never disagree on geometry. A
        footing is sized from a takedown layout never runs, so none is sized
        here.

        Args:
            payload: the structural request - ``source`` plus the matching
                ``plan`` / ``building`` / ``housing`` block, optional
                ``params`` and ``output``. May also arrive wrapped in the
                engine envelope.
            options: per-call overrides merged over ``payload["output"]``
                (and ``params=`` / ``output=`` sub-dicts).

        Returns:
            The engine envelope. ``status`` is ``"ERROR"`` for a refusal, and
            the response still carries the disclaimer and the reason.
        """
        from GPLAN.structural.api import run_layout
        return run_layout(payload, **options)

    @staticmethod
    def design_structure(payload, **options):
        """The full pipeline: placement, loads, takedown, foundations,
        diaphragm, member design, quantities and the clause-traced report.

        Seconds to a minute, which is why the HTTP surface runs it through
        Celery. The orchestration sequence is fixed in
        ``GPLAN/structural/api.py`` and is not this facade's business.

        Args:
            payload: as ``layout_structure``.
            options: as ``layout_structure``; ``output={"validate_only": True}``
                short-circuits after validation for a cheap pre-dispatch check.

        Returns:
            The engine envelope, footings sized, one entry per plan or per
            built plot stack.
        """
        from GPLAN.structural.api import run_design
        return run_design(payload, **options)

    @staticmethod
    def check_structure(payload, **options):
        """Re-validate a ``structural_model`` this engine returned earlier,
        possibly hand-edited.

        Nothing here moves an element. ``scope: "placement"`` re-runs the hard
        rules and the score; ``scope: "full"`` adds the loads, the gravity
        takedown and the per-member checks against the edited geometry.

        Args:
            payload: ``{"source": "model", "model": {...}, "scope": ...}``.
            options: as ``layout_structure``.

        Returns:
            The engine envelope carrying ``verdict``, ``hard_violations``,
            ``layout_score`` and ``element_checks``.
        """
        from GPLAN.structural.api import run_check
        return run_check(payload, **options)

    @staticmethod
    def structural_options():
        """Static capability discovery: sources, systems, code profiles,
        seismic zones, soils, grades, spans, limits and the disclosure
        registry.

        Every list is generated from the constant the pipeline enforces, so a
        client that reads this can never drift from the engine. Pure and
        cheap; takes no request.

        Returns:
            The engine envelope with one entry describing the engine.
        """
        from GPLAN.structural.api import run_options
        return run_options()
