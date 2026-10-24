import GPLAN.source.polygonal.poly as poly
import GPLAN.input.input_for_min_dim as input_for_min_dim
import json
# from system_functions.os_functions import delete_fileimport input.input_for_min_dim as input_for_min_dim
from GPLAN.source.graphoperations.operations import get_encoded_matrix
from GPLAN.source.graphoperations.graph_crossings1 import check_intersection as check_intersection
from GPLAN.source.graphoperations import connect_graph
from GPLAN.system_functions.os_functions import delete_file
import GPLAN.source.dimensioning.minimum_dimensioning as min_dim
import GPLAN.source.lettershape.lshape.Lshaped as Lshaped
import GPLAN.source.lettershape.tshape.tshape as Tshaped
import GPLAN.source.lettershape.ushape.ushape as Ushaped
import GPLAN.source.lettershape.zshape.zshape as Zshaped
import GPLAN.source.polygonal.limits as lim
import GPLAN.source.polygonal.newcoord as nc
import GPLAN.pythongui.dimensiongui as dimgui
import GPLAN.pythongui.mindimensiongui as mindimgui
import GPLAN.pythongui.nonadjgui as nonadjgui
import time
# tkinter messagebox only used in GUI/drawGUI paths; lazy-load to avoid display requirement on server
class _LazyMessageBox:
    def __getattr__(self, name):
        from tkinter import messagebox as _mb
        return getattr(_mb, name)
messagebox = _LazyMessageBox()
import networkx as nx
import numpy as np
import GPLAN.pythongui.gui as gui
import GPLAN.source.inputgraph as inputgraph
import GPLAN.pythongui.drawing as draw
import GPLAN.circulation as cir
# matplotlib imported lazily inside drawGUI branches only (avoid module-level overhead on server)
import copy

from dataclasses import is_dataclass
# import checker
# import triangularity as trng

origin = 0

def drawFunction(ui ,graph, origin, roomName, isPoly = False, gclass = None):
    if not isPoly:
        draw.draw_rdg(graph
                    , 1
                    , gclass.pen
                    , 1
                    , ui.get_roomColors()
                    , roomName
                    , origin)
    else:
        draw.draw_poly(gclass.canonicalObject.graph_data,1
            ,gclass.pen
            ,1
            ,ui.get_roomColors()
            ,[]
            ,origin,gclass.outer_boundary, gclass.shape,graph.matrix)

def make_dissection_corridor(gclass):
    dis = nx.Graph()
    dis = nx.from_numpy_array(gclass.dclass.mat)
    m = len(dis)
    spanned = circulation.BFS(dis, gclass.e1.get(), gclass.e2.get())
    gclass.cir_dim_mat = nx.to_numpy_matrix(spanned)
    # colors = ['#4BC0D9','#76E5FC','#6457A6','#5C2751','#7D8491','#BBBE64','#64F58D','#9DFFF9','#AB4E68','#C4A287','#6F9283','#696D7D','#1B1F3B','#454ADE','#FB6376','#6C969D','#519872','#3B5249','#A4B494','#CCFF66','#FFC800','#FF8427','#0F7173','#EF8354','#795663','#AF5B5B','#667761','#CF5C36','#F0BCD4','#ADB2D3','#FF1B1C','#6A994E','#386641','#8B2635','#2E3532','#124E78']*10
    colors = ['#4BC0D9'] * 10
    rnames = []
    for i in range(1, m + 1):
        rnames.append('Room' + str(i))
    rnames.append("Corridor")
    for i in range(1, 10):
        colors[m + i - 1] = '#FF4C4C'
        rnames.append("")
    parameters = [len(spanned), spanned.size(), spanned.edges(), 0, 0, rnames, colors]
    C = ptpg.PTPG(parameters)
    C.create_single_dual(1, gclass.pen, gclass.textbox)
    gclass.ocan.add_cir_tab()
    gclass.dclass.add_cir()

def call_circulation_new(ui, graph_data, graph, coord, is_dimensioned, dim_constraints, remove_corridor,
                         public_private=False, is_minimum_dimensioned=False, is_optimal=False, drawGUI=False, gclass=None):
    g = nx.Graph()
    edge_set = ui.get_edges()
    entry = gclass.entry_door

    for x in edge_set:
        g.add_edge(x[0], x[1])

    n = len(g)

    rooms = []
    if graph_data is not None:
        for i in range(n):
            rooms.append(
                cir.Room(i, graph_data.get("room_x")[i], graph_data.get("room_y")[i] + graph_data.get("room_height")[i],
                         graph_data.get("room_x")[i] + graph_data.get("room_width")[i], graph_data.get("room_y")[i]))
    else:
        for i in range(n):
            rooms.append(cir.Room(i, graph.room_x[i], graph.room_y[i] + graph.room_height[i],
                                  graph.room_x[i] + graph.room_width[i], graph.room_y[i]))

    rfp = cir.RFP(g, rooms)

    cir.plot(g, n)
    circulation_obj = cir.circulation(g, ui.get_corridor_thickness(), rfp, gclass.rem)

    # Add dimensional constraints if chosen option is "dimensioned circulation"
    if is_dimensioned == True:
        circulation_obj.is_dimensioned = True
        circulation_obj.dimension_constraints = dim_constraints
    elif is_minimum_dimensioned == True:
        circulation_obj.is_minimum_dimensioned = True
        circulation_obj.dimension_constraints = dim_constraints
        circulation_obj.is_optimal = is_optimal

    # Apply circulation algorithm
    circulation_result = circulation_obj.circulation_algorithm(entry[0], entry[1])
    cir.plot(circulation_obj.circulation_graph, len(circulation_obj.circulation_graph))
    if circulation_result == 0:
        return None
    if public_private == True:
        circ = copy.deepcopy(circulation_obj)
        circ.adjust_RFP_to_circulation()

        # Printing how much shift was done for each room
        for room in circ.RFP.rooms:
            print("Room ", room.id, ":")
            print("Push top edge by: ", room.rel_push_T)
            print("Push bottom edge by: ", room.rel_push_B)
            print("Push left edge by: ", room.rel_push_L)
            print("Push right edge by: ", room.rel_push_R)
            print(room.target)
            print('\n')

        room_x1 = []
        room_y1 = []
        room_height1 = []
        room_width1 = []

        # Getting the required values
        for room in circ.RFP.rooms:
            room_x1.append(room.top_left_x)
            room_y1.append(room.bottom_right_y)
            room_height1.append(abs(room.top_left_y - room.bottom_right_y))
            room_width1.append(abs(room.top_left_x - room.bottom_right_x))

        new_graph = copy.deepcopy(graph)
        new_graph.room_x = room_x1
        new_graph.room_y = room_y1
        new_graph.room_height = room_height1
        new_graph.room_width = room_width1
        new_graph.final_traversal = inputgraph.get_final_traversal(new_graph)
        if drawGUI:
            drawFunction(ui, new_graph, origin,ui.get_roomNames(), gclass=gclass)

        # Now going back to flow of removing circulation
        corridors = circulation_obj.adjacency
        rem_edges = gclass.public_rooms(corridors)

        for x in rem_edges:
            circulation_obj.remove_corridor(circulation_obj.circulation_graph, x[0], x[1])

    if remove_corridor == True:
        # Created a deepcopy of object to display circulation before
        # we display GUI for removing corridor
        circ = copy.deepcopy(circulation_obj)
        circ.adjust_RFP_to_circulation()

        # Printing how much shift was done for each room
        # print("\t\tT\tB\t\L\tR\t\tTarget")
        for room in circ.RFP.rooms:
            print(
                f"{room.id}\t{room.rel_push_T}\t{room.rel_push_B}\t{room.rel_push_L}\t{room.rel_push_R}\t\t{room.target}")
            # print("Room ",room.id, ":")
            # print("Push top edge by: ", room.rel_push_T)
            # print("Push bottom edge by: ", room.rel_push_B)
            # print("Push left edge by: ", room.rel_push_L)
            # print("Push right edge by: ", room.rel_push_R)
            # print(room.target)
            # print('\n')

        room_x1 = []
        room_y1 = []
        room_height1 = []
        room_width1 = []

        # Getting the required values
        for room in circ.RFP.rooms:
            room_x1.append(room.top_left_x)
            room_y1.append(room.bottom_right_y)
            room_height1.append(abs(room.top_left_y - room.bottom_right_y))
            room_width1.append(abs(room.top_left_x - room.bottom_right_x))

        if graph_data is not None:
            new_graph = copy.deepcopy(graph)
            new_graph.room_x = room_x1
            new_graph.room_y = room_y1
            new_graph.room_height = room_height1
            new_graph.room_width = room_width1
            new_graph.final_traversal = inputgraph.get_final_traversal(new_graph)
            # graph_data1 = {}
            # graph_data1['room_x'] = np.array(room_x1)
            # graph_data1['room_y'] = np.array(room_y1)
            # graph_data1['room_height'] = np.array(room_height1)
            # graph_data1['room_width'] = np.array(room_width1)
            # graph_data1['area'] = np.array(circulation_obj.room_area)
            # graph_data1['extranodes'] = graph_data['extranodes']
            # graph_data1['mergednodes'] = graph_data['mergednodes']
            # graph_data1['irreg_nodes'] = graph_data['irreg_nodes']
            # draw.draw_rdg(graph_data1, 1, gclass.pen, 1, ui.get_roomColors(), ui.get_roomNames(), origin)
        else:
            new_graph = copy.deepcopy(graph)
            new_graph.room_x = room_x1
            new_graph.room_y = room_y1
            new_graph.room_height = room_height1
            new_graph.room_width = room_width1
            new_graph.final_traversal = inputgraph.get_final_traversal(new_graph)

        if drawGUI:
            drawFunction(ui, new_graph, origin, ui.get_roomNames(), gclass=gclass)

        # Now going back to flow of removing circulation
        corridors = circulation_obj.adjacency
        rem_edges = gclass.remove_corridor_gui(corridors)

        for x in rem_edges:
            circulation_obj.remove_corridor(circulation_obj.circulation_graph, x[0], x[1])

    # To remove entry corridor alone we are just shifting rooms by looking at second corridor vertex
    # Done by shifting the range left bound in for loop of adjust_RFP_to_circulation()
    circulation_obj.adjust_RFP_to_circulation()

    if is_minimum_dimensioned == True:
        new_min_width = []
        new_min_height = []
        for i in range(len(circulation_obj.dimensions)):
            min_width = circulation_obj.dimension_constraints[0][i]
            min_height = circulation_obj.dimension_constraints[1][i]
            width = circulation_obj.dimensions[i][0]
            height = circulation_obj.dimensions[i][1]
            if width < min_width:
                circulation_obj.dimensions[i][0] = 2 * min_width - width
            else:
                circulation_obj.dimensions[i][0] = min(min_width, width)
            new_min_width.append(circulation_obj.dimensions[i][0])
            if height < min_height:
                circulation_obj.dimensions[i][1] = 2 * min_height - height
            else:
                circulation_obj.dimensions[i][0] = min(min_height, height)
            new_min_height.append(circulation_obj.dimensions[i][1])

        generate_mindim_rfp(ui, gclass.ptpg, gclass, new_min_width, new_min_height, dim_constraints[2], dim_constraints[3],
                            is_optimal)
        rooms = []
        graph_data = gclass.output_data[0]
        for i in range(n):
            rooms.append(
                cir.Room(i, graph_data.get("room_x")[i], graph_data.get("room_y")[i] + graph_data.get("room_height")[i],
                         graph_data.get("room_x")[i] + graph_data.get("room_width")[i], graph_data.get("room_y")[i]))
        rfp = cir.RFP(g, rooms)
        circulation_obj.RFP = rfp
        circulation_obj.room_area = []
        circulation_obj.adjust_RFP_to_circulation()

    # Printing how much shift was done for each room
    print("\tT\tB\tL\tR")
    for room in circulation_obj.RFP.rooms:
        print(f"{room.id}\t{room.rel_push_T}\t{room.rel_push_B}\t{room.rel_push_L}\t{room.rel_push_R}")
        # print("Room ",room.id, ":")
        # print("Push top edge by: ", room.rel_push_T)
        # print("Push bottom edge by: ", room.rel_push_B)
        # print("Push left edge by: ", room.rel_push_L)
        # print("Push right edge by: ", room.rel_push_R)
        # print(room.target)
        # print('\n')

    room_x = []
    room_y = []
    room_height = []
    room_width = []

    # Getting the required values
    for room in circulation_obj.RFP.rooms:
        room_x.append(room.top_left_x)
        room_y.append(room.bottom_right_y)
        room_height.append(abs(room.top_left_y - room.bottom_right_y))
        room_width.append(abs(room.top_left_x - room.bottom_right_x))

    if graph_data is not None:
        graph_data['room_x'] = np.array(room_x)
        graph.room_x = room_x
        graph_data['room_y'] = np.array(room_y)
        graph.room_y = room_y
        graph_data['room_height'] = np.array(room_height)
        graph.room_height = room_height
        graph_data['room_width'] = np.array(room_width)
        graph.room_width = room_width
        graph_data['area'] = np.array(circulation_obj.room_area)
        graph.area = circulation_obj.room_area
        return (graph_data, circulation_obj.is_dimensioning_successful)
    else:
        graph.room_x = room_x
        graph.room_y = room_y
        graph.room_height = room_height
        graph.room_width = room_width
        graph.area = circulation_obj.room_area
        return (graph, circulation_obj.is_dimensioning_successful)


def get_max_dims(ui):
    """Per-room maximum width/height lists supplied by the caller, else (None, None).

    Minimum dimensioning bounds every room at DEFAULT_UB_FACTOR x its own
    minimum when no ceiling is given, which is what lets a small service room
    stretch until it outgrows a habitable one. Callers that send maximums get
    them applied; empty lists (the GUI path, and API clients that do not opt in)
    keep the old behaviour.
    """
    params = getattr(ui, "min_dim_inputs", None)
    if params is None:
        return None, None

    def usable(values):
        # 99999 is the "open" sentinel every legacy client sends; a list made
        # only of sentinels carries no ceiling at all.
        if not values:
            return None
        for value in values:
            try:
                if 0 < float(value) < 99999:
                    return values
            except (TypeError, ValueError):
                continue
        return None

    return usable(params.get_max_width()), usable(params.get_max_height())


# enforce_plot top-up: after the capped pass, keep expanding the catalogue with
# plot-overflowing (labeled) plans until this many are available, so a plot
# that only a few topologies fit still returns a full page. Matches the API
# response cap (api.FLOORPLAN_LIMIT) rather than the 500-strong search bound.
ENFORCE_PLOT_TOPUP_TARGET = 30


def _allocation_rooms(ui):
    from GPLAN.source.dimensioning.allocator import WALL_ALLOWANCE_FT
    nodes = getattr(ui.min_dim_inputs, "room_nodes", [])
    if not nodes or (not getattr(ui.min_dim_inputs, "strict_room_bounds", False)
                     and not getattr(ui.min_dim_inputs, "use_requested_room_bounds", False)
                     and not any("priority" in n for n in nodes)):
        return [{"name": name} for name in ui.get_roomNames()]
    rooms = []
    for node in nodes:
        room = {"name": node.get("label", "Room")}
        if "priority" in node:
            room["priority"] = node["priority"]
        for axis in ("width", "height"):
            for bound in ("min", "max"):
                value = (node.get(axis) or {}).get(bound)
                if isinstance(value, (int, float)) and 0 < value < 99999:
                    room[bound + "_" + axis] = max(0.001, value - WALL_ALLOWANCE_FT)
        for key in ("min_area", "max_area"):
            if node.get(key):
                room[key] = node[key]
        rooms.append(room)
    return rooms


def solve_min_dim(floorplan_data, plot_width, plot_height, capped,
                  fixed_indices=None):
    """min_dim.main with a relaxation ladder.

    Returns (status, out_data, released). The caller prefers a plan that
    honours every room maximum AND gives every briefed (red, door) adjacency a
    door-width shared wall (DOOR_OVERLAP_FLOOR), but a plan that breaks one
    still beats no plan at all. Rungs, in order:

      1. full: ceilings + door-width overlap floor;
      2. ceilings + the legacy 0.1 overlap - a sliver wall holds no door,
         but the plan ships and E2a disclosure names the pair per plan;
      3. ceilings stripped, door-width floor (sets `released`);
      4. ceilings stripped + legacy overlap, the pre-2026-08-06 behaviour.

    min_dim.main MUTATES its input in place (1-based id bumps), so every rung
    gets its own deepcopy of the PRISTINE data - re-solving a dict a previous
    rung touched double-bumps the ids and crashes tblr_rooms.
    """
    LEGACY_OVERLAP = 0.1
    fixed_indices = set(fixed_indices or ())
    # minimum_dimensioning normally widens every maximum to preserve the
    # topology catalogue.  Mark hard rooms in the solver input so its upper
    # bound remains exactly equal to the lower bound.
    for index, node in enumerate(floorplan_data.get('nodes', [])):
        if index in fixed_indices:
            node['is_fixed'] = True
    retry_data = copy.deepcopy(floorplan_data)
    status, out_data = min_dim.main(floorplan_data, plot_width, plot_height)
    if status:
        return status, out_data, False
    if retry_data.get('strict_room_bounds'):
        # Hard ceilings and mandatory door lengths survive every retry.
        return status, out_data, False
    # Overlap relaxes BEFORE the ceilings release: a sliver wall is a
    # disclosed shortfall, a released ceiling can inflate a room past the
    # point any client shows it. It also keeps the release warning honest - a
    # topology blocked only by the door-width floor must not report
    # "maximum dimensions were released".
    status, out_data = min_dim.main(
        copy.deepcopy(retry_data), plot_width, plot_height,
        overlap_floor=LEGACY_OVERLAP)
    if status or not capped:
        return status, out_data, False
    stripped = copy.deepcopy(retry_data)
    for index, node in enumerate(stripped['nodes']):
        if index in fixed_indices:
            continue
        node.pop('max_width', None)
        node.pop('max_height', None)
    status, out_data = min_dim.main(
        copy.deepcopy(stripped), plot_width, plot_height)
    if status:
        return status, out_data, True
    status, out_data = min_dim.main(
        stripped, plot_width, plot_height, overlap_floor=LEGACY_OVERLAP)
    return status, out_data, bool(status)


def _minimums_toward_floors(base, floors, lam, fixed_indices=None):
    """Copy of `base` with every room minimum interpolated `lam` of the way
    from the request's band minimum DOWN to the rulebook floor (lam=1 IS the
    floor). A floor at or above the request minimum is left alone."""
    data = copy.deepcopy(base)
    fixed_indices = set(fixed_indices or ())
    for i, node in enumerate(data['nodes']):
        if i in fixed_indices:
            continue
        if floors is None or i >= len(floors) or floors[i] is None:
            continue
        fw, fh = floors[i]
        for key_min, floor_value in (("min_width", fw), ("min_height", fh)):
            try:
                lo = float(node.get(key_min) or 0)
            except (TypeError, ValueError):
                continue
            if lo <= 0 or floor_value is None or float(floor_value) >= lo:
                continue
            node[key_min] = lo + lam * (float(floor_value) - lo)
    return data


def solve_min_dim_fill(floorplan_data, plot_width, plot_height, capped,
                       targets=None, area_caps=None, floors=None,
                       fixed_indices=None):
    """solve_min_dim, then raise the minimums TOWARD THE ALLOCATOR TARGETS as
    far as this topology can fit them (enforce_plot exact fill, 2026-08-17).

    `targets` is the tier-aware allocation for this plot: per room the
    (target_width, target_height) the functionality rule assigns - living
    and bedrooms take the surplus, a toilet stays a toilet. The bisection
    finds the largest interpolation lambda in [0, 1] between the band
    minimums and those targets that still solves INSIDE the plot cap, so
    the tiling stretches coherently toward the allocation instead of a
    post-hoc fill inflating whichever room touches the leftover strip.
    A per-axis uniform-factor fallback covers programs the allocator does
    not know.

    `floors` is the per-room (min_width, min_height) RULEBOOK floor. It turns
    the search into a two-sided one: a topology that does not fit the enforced
    plot even at the request's band minimums first relaxes those minimums
    DOWNWARD toward the floors (2026-08-29, the plot-wins rule) and the
    bisection toward the targets then runs from whatever it landed on. Without
    it the behaviour is exactly as before: such a topology fails here and
    re-enters as a labeled expanded plan.

    Returns (status, out_data, released) exactly like solve_min_dim.
    """
    fixed_indices = set(fixed_indices or ())
    base = copy.deepcopy(floorplan_data)
    if base.get("strict_room_bounds"):
        floors = None
    status, out_data, released = solve_min_dim(
        copy.deepcopy(base), plot_width, plot_height, capped,
        fixed_indices=fixed_indices)
    if not status and floors is not None \
            and plot_width > 0 and plot_height > 0:
        # The topology does not fit the ENFORCED plot at the request's band
        # minimums. Those minimums are a proportional PREFERENCE (the client
        # aims each room at ~75% of its thumb-rule share); the rulebook floor
        # underneath them is the hard number. The user rule is that the typed
        # plot wins over room preferences, so relax the minimums downward
        # toward those floors - never below one - and keep the SMALLEST
        # relaxation that fits. A plan a hair under its preferred room sizes
        # inside the plot beats a labeled expanded plan outside it, and phase
        # 5 of post-processing grows it back out to the plot exactly.
        s, o, r = solve_min_dim(
            _minimums_toward_floors(base, floors, 1.0, fixed_indices),
            plot_width, plot_height, capped, fixed_indices=fixed_indices)
        if s:
            lo_s, hi_s = 0.0, 1.0        # lo fails, hi solves
            best_lam = 1.0
            status, out_data, released = s, o, r
            for _ in range(5):
                mid = (lo_s + hi_s) / 2.0
                s, o, r = solve_min_dim(
                    _minimums_toward_floors(base, floors, mid, fixed_indices),
                    plot_width, plot_height, capped,
                    fixed_indices=fixed_indices)
                if s:
                    hi_s = mid
                    best_lam = mid
                    status, out_data, released = s, o, r
                else:
                    lo_s = mid
            base = _minimums_toward_floors(base, floors, best_lam,
                                           fixed_indices)
    if not status or plot_width <= 0 or plot_height <= 0:
        return status, out_data, released

    def clamp_to_node_max(node, key_max, value):
        hi = node.get(key_max)
        if hi is not None:
            try:
                hi = float(hi)
                if 0 < hi < 99999:
                    value = min(value, hi)
            except (TypeError, ValueError):
                pass
        return value

    def scaled_toward_targets(lam):
        data = copy.deepcopy(base)
        for i, node in enumerate(data['nodes']):
            if targets is None or i >= len(targets) or targets[i] is None:
                continue
            tw, th = targets[i]
            for key_min, key_max, tgt in (("min_width", "max_width", tw),
                                          ("min_height", "max_height", th)):
                try:
                    lo = float(node.get(key_min) or 0)
                except (TypeError, ValueError):
                    continue
                if lo <= 0 or tgt is None or float(tgt) <= lo:
                    continue
                grown = lo + lam * (float(tgt) - lo)
                node[key_min] = max(lo, clamp_to_node_max(node, key_max, grown))
        return data

    def solution_ok(out):
        # Quality gate on a probe's SOLUTION: solver feasibility alone lets a
        # spanning-cell room absorb a stretched row (SOLVER_UB_SLACK), so a
        # probe only counts when every room stays inside its tiered area cap.
        if area_caps is None:
            return True
        for i, node in enumerate(out.get("nodes", [])):
            if i >= len(area_caps) or area_caps[i] is None:
                continue
            try:
                area = float(node["width"]) * float(node["height"])
            except (KeyError, TypeError, ValueError):
                continue
            if area > area_caps[i] + 0.5:
                return False
        return True

    def scaled_uniform(fw, fh):
        data = copy.deepcopy(base)
        for node in data['nodes']:
            for key_min, key_max, factor in (("min_width", "max_width", fw),
                                             ("min_height", "max_height", fh)):
                try:
                    lo = float(node.get(key_min) or 0)
                except (TypeError, ValueError):
                    continue
                if lo <= 0:
                    continue
                node[key_min] = max(lo, clamp_to_node_max(node, key_max,
                                                          lo * factor))
        return data

    best = (status, out_data, released)

    if targets is not None:
        # Lambda toward the allocation, solved ENTIRELY under a tight solver
        # slack: the normal SOLVER_UB_SLACK (2.5x the minimum) lets one
        # arbitrary room take a stretched column's whole residual (a
        # spanning-cell toilet measured 38.0 x 4.1 ft) and then every probe
        # fails the area gate, collapsing lambda to zero. At 1.3x each room
        # stays near its interpolated minimum, the allocation's balance does
        # the equalising, and the gate only has to catch real spanning-cell
        # topologies. lam=1 IS the allocation; most topologies bind earlier
        # and the bisection keeps the largest lambda that solves inside the
        # plot with every room inside its tiered cap. The residual to the
        # exact plot is post-processing's job; when NO lambda passes, the
        # plain minimum solution stands and the plan simply fills less.
        old_slack = min_dim.SOLVER_UB_SLACK
        min_dim.SOLVER_UB_SLACK = 1.3
        try:
            lo_l, hi_l = 0.0, 1.0
            s, o, r = solve_min_dim(scaled_toward_targets(1.0),
                                    plot_width, plot_height, capped,
                                    fixed_indices=fixed_indices)
            if s and solution_ok(o):
                best = (s, o, r)
            else:
                for _ in range(6):
                    mid = (lo_l + hi_l) / 2.0
                    s, o, r = solve_min_dim(scaled_toward_targets(mid),
                                            plot_width, plot_height, capped,
                                            fixed_indices=fixed_indices)
                    if s and solution_ok(o):
                        lo_l = mid
                        best = (s, o, r)
                    else:
                        hi_l = mid
        finally:
            min_dim.SOLVER_UB_SLACK = old_slack
        if not solution_ok(best[1]):
            # Every lambda failed the gate AND the plain solve itself violates
            # a tiered area cap - the ladder released this topology's ceilings
            # and handed a spanning cell the residual (the 295.8 sqft
            # Bedroom 2). Under enforce_plot that plan would break the area
            # hierarchy the client audits, so the topology is DROPPED from the
            # fitted pool (it re-enters through the labeled expanded top-up).
            # An honest thinner catalogue beats a page-one broken plan.
            return False, best[1], best[2]
        return best

    # No allocation available: per-axis uniform factors (a single factor
    # stalls once the tighter axis binds; independent factors let each axis
    # stretch to its own plot line).
    best_fw = 1.0
    lo_f, hi_f = 1.0, 2.5
    for _ in range(6):
        mid = (lo_f + hi_f) / 2.0
        s, o, r = solve_min_dim(scaled_uniform(mid, 1.0),
                                plot_width, plot_height, capped,
                                fixed_indices=fixed_indices)
        if s:
            lo_f = mid
            best = (s, o, r)
            best_fw = mid
        else:
            hi_f = mid
    lo_f, hi_f = 1.0, 2.5
    for _ in range(6):
        mid = (lo_f + hi_f) / 2.0
        s, o, r = solve_min_dim(scaled_uniform(best_fw, mid),
                                plot_width, plot_height, capped,
                                fixed_indices=fixed_indices)
        if s:
            lo_f = mid
            best = (s, o, r)
        else:
            hi_f = mid
    return best


def generate_mindim_rfp(ui, graph, gclass, min_width, min_height, plot_width, plot_height, optimal_floorplan):
    try:
        graph.oneconnected_dual("multiple")
    except inputgraph.OCError:
        show_warning("Can not generate rectangular floorplan.")
        graph.irreg_multiple_dual()
    except inputgraph.BCNError:
        graph.irreg_multiple_dual()
    number_of_floorplans = graph.fpcnt
    floorplan_found = False

    # Resets the data already present for downloading catalogues
    ui._set_output_data([])
    ui._set_multiple_output_found(0)

    # Variables for storing the data of the floorplan with minimal area
    min_area = -1
    min_graph = None
    areas = []

    # Iterate through all possible floorplans to find one which satisfies the given conditions
    for i in range(number_of_floorplans):
        print("Trying floorplan number", i + 1, "to see if minimum dimension floorplan can be constructed.")
        floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                    ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(), ui.get_corridor_thickness())
        enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x, graph.graph_list[i].room_y,
                                     graph.graph_list[i].room_width, graph.graph_list[i].room_height)
        floorplan_data = floorplan_obj.get_floorplan_details(
            ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height, graph.graph_list[i].room_x,
            graph.graph_list[i].room_y, graph.graph_list[i].room_width, graph.graph_list[i].room_height,
            ui.get_edges(), enc_mat
        )
        floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

        # If floorplan satisfying the given constraints is satisfied
        [status, out_data] = min_dim.main(floorplan_data, plot_width, plot_height)
        if status == True:
            room_x = []
            room_y = []
            room_width = []
            room_height = []
            room_area = []
            room_name = []
            for room_detail in out_data["nodes"]:
                room_x.append(room_detail["room_x"])
                room_y.append(room_detail["room_y"])
                room_width.append(room_detail["width"])
                room_height.append(room_detail["height"])
                room_area.append(room_detail["width"] * room_detail["height"])

            # Store the room labels if they have been entered
            for room_id in range(len(out_data["nodes"])):
                if "label" not in out_data["nodes"][room_id]:
                    room_name.append(str(room_id))
                else:
                    room_name.append(out_data["nodes"][room_id]["label"])

            # room_x = np.array(room_x)
            # room_y = np.array(room_y)
            # room_width = np.array(room_width)
            # room_height = np.array(room_height)

            graph.graph_list[i].room_x = room_x
            graph.graph_list[i].room_y = room_y
            graph.graph_list[i].room_width = room_width
            graph.graph_list[i].room_height = room_height
            graph.graph_list[i].area = room_area

            ui._set_multiple_output_found(1)

            floorplan_found = True

            # If optimal area not required, store graph data
            if optimal_floorplan == 0:
                '''
                Adds the graph data to output_data for downloading the catalogue and 
                multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                '''
                graph_data = {
                    'room_x': graph.graph_list[i].room_x,
                    'room_y': graph.graph_list[i].room_y,
                    'room_width': graph.graph_list[i].room_width,
                    'room_height': graph.graph_list[i].room_height,
                    'area': graph.graph_list[i].area,
                    'extranodes': graph.graph_list[i].extranodes,
                    'mergednodes': graph.graph_list[i].mergednodes,
                    'irreg_nodes': graph.graph_list[i].irreg_nodes1
                }
                ui._append_output_data(graph_data)#Fix this almost certainly there would be error here
                # ui._append_output_data(graph_data) No need for this because it was alread appended
                break

            # Store graph data if graph area is less than current minimal area
            area_sum = sum(room_area)
            areas.append(area_sum)
            # graph.graph_list[i].area = area_sum
            if min_area < 0 or area_sum < min_area:
                min_area = area_sum
                min_graph = graph.graph_list[i]

    if not floorplan_found:
        print("No floorplan found which satisfies the minimum dimensions input by user.")

    # Store graph data with optimal area if required
    elif optimal_floorplan == 1:
        graph = min_graph
        graph_data = {
            'room_x': graph.room_x,
            'room_y': graph.room_y,
            'room_width': graph.room_width,
            'room_height': graph.room_height,
            'area': graph.area,
            'extranodes': graph.extranodes,
            'mergednodes': graph.mergednodes,
            'irreg_nodes': graph.irreg_nodes1
        }
        ui._append_output_data(graph_data)#Fix this almost certainly there would be error here


def plot(graph: nx.Graph, m: int) -> None:
    """Plots the graph using matplotlib

    Args:
        graph (Networkx graph): The graph to plot
        m (integer): Number of vertices in the graph
    """
    pos = nx.spring_layout(graph)  # positions for all nodes
    nx.draw_networkx(graph, pos, label=None, node_size=400, node_color='#4b8bc8', font_size=12, font_color='k',
                     font_family='sans-serif', font_weight='normal', alpha=1, bbox=None, ax=None)
    nx.draw_networkx_edges(graph, pos)
    nx.draw_networkx_nodes(graph, pos,
                           nodelist=list(range(m, len(graph))),
                           node_color='r',
                           node_size=500,
                           alpha=1)
    #plt.show()

def handle_circulation(ui, graph, drawGUI=False, gclass=None):
    is_dimensioned = False
    remove_corridor = False
    public_private = False
    node_coord = graph.coordinates
    dim_constraints = []

    if ui.get_isMinDimensioned() == 1 and ui.get_isRemoveAddCirculation() == 0:  # Minimum dimensioned circulation
        old_dims = [[3] * ui.get_noOfNodes()
            , [3] * ui.get_noOfNodes()]

        # If the graph came from an input file, the default values are set
        if gclass.open and len(ui.get_dim_constraints()) > 0:
            [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
            print("Dim Constraints before old dims:", ui.get_dim_constraints())
            for i in range(len(old_min_height)):
                old_dims[0][i] = old_min_width[i]
                old_dims[1][i] = old_min_height[i]
            old_dims.extend([plot_width, plot_height])
            gclass.open = False
        min_width, min_height, plot_width, plot_height, optimal_floorplan = mindimgui.gui_fnc(ui,old_dims,
                                                                                              ui.get_noOfNodes(),
                                                                                              ui.get_roomNames(), gclass)
        start = time.time()
        generate_mindim_rfp(ui, graph, gclass, min_width, min_height, plot_width, plot_height,
                            optimal_floorplan)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        # Sets the ptpg object to the current graph to use for download catalogue and also stores the dimensional constraints of the graph
        ui._set_multiple_output_found(1)
        gclass.ptpg = graph  # Fix this Merge problem
        ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])

        mindim_graph_data = gclass.output_data[0]
        (new_graph_data, success) = call_circulation_new(ui, mindim_graph_data, graph, node_coord,
                                                         is_dimensioned, ui.get_dim_constraints(),
                                                         remove_corridor, is_minimum_dimensioned=True,
                                                         is_optimal=optimal_floorplan, drawGUI=drawGUI,gclass=gclass)
        print("Constraints: ", dim_constraints)
        print("New graph data: ", new_graph_data)
        print("success: ", success)

        # Resets the flag for minimum dimensioned circulation as it is the same flag as minimum dimensioned floorplans
        gclass.checkvar4.set(0)

        # If there was some error in algorithm execution new_graph_data will be empty
        # we display the pop-up error message
        if new_graph_data == None:
            messagebox.showerror("Error", "ERROR!! THE INITIAL CHOSEN ENTRY EDGE MUST BE EXTERIOR EDGE")

        # If no issues we continue to draw the corridor
        elif success == True:
            graph.room_x = new_graph_data['room_x']
            graph.room_y = new_graph_data['room_y']
            graph.room_width = new_graph_data['room_width']
            graph.room_height = new_graph_data['room_height']
            graph.final_traversal = inputgraph.get_final_traversal(graph)
            if drawGUI:
                drawFunction(ui, graph, origin, ui.get_roomNames(), gclass=gclass)
            feasible_dim = 1

    #                 elif (ui.get_isDimensionedCirculation() == 0 and ui.get_isRemoveAddCirculation() == 0): #Non-dimensioned single circulation
    #                     print("panyamshtest: nondim_mode, rem_mode:", gclass.checkvar3.get())
    # Check here for merge conflicts
    # =======
    elif ui.get_isDimensionedCirculation() == 0 and ui.get_isRemoveAddCirculation() == 0 and ui.get_isPublic() == 0:  # Non-dimensioned single circulation
        # print("panyamshtest: nondim_mode, rem_mode:", gclass.checkvar3.get())
        start = time.time()
        graph.irreg_single_dual()
        ui.print_gui('Generated Single Irregular floorplan with Circulation')
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        print("type of roomx " + str(type(graph.room_x)))
        graph_data = {
            'room_x': graph.room_x,
            'room_y': graph.room_y,
            'room_width': graph.room_width,
            'room_height': graph.room_height,
            'area': graph.area,
            'extranodes': graph.extranodes,
            'mergednodes': graph.mergednodes,
            'irreg_nodes': graph.irreg_nodes1
        }
        # new_graph_data = call_circulation(graph_data, ui.get_edges(), gclass.entry_door, ui.get_corridor_thickness())
        (new_graph_data, success) = call_circulation_new(ui, graph_data, graph, node_coord,
                                                         is_dimensioned, dim_constraints, remove_corridor, drawGUI=drawGUI,gclass=gclass)
        # If there was some error in algorithm execution new_graph_data will be empty
        # we display the pop-up error message
        if new_graph_data == None:
            messagebox.showerror("Error", "ERROR!! THE INITIAL CHOSEN ENTRY EDGE MUST BE EXTERIOR EDGE")

        # If no issues we continue to draw the corridor
        else:
            graph.room_x = new_graph_data['room_x']
            graph.room_y = new_graph_data['room_y']
            graph.room_width = new_graph_data['room_width']
            graph.room_height = new_graph_data['room_height']
            graph.final_traversal = inputgraph.get_final_traversal(graph)
            if drawGUI:
                drawFunction(ui, graph, origin, ui.get_roomNames(), gclass=gclass)
            # draw.draw_rdg(new_graph, 1, gclass.pen, 1, ui.get_roomColors(), [],origin)

    elif ui.get_isDimensionedCirculation() == 1 and ui.get_isRemoveAddCirculation() == 0 and ui.get_isPublic() == 0:  # Dimensioned single circulation
        is_dimensioned = True
        feasible_dim = 0
        old_dims = [[0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , ""
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()]
        min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
            old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
        dimensional_constraints = [min_width, max_width, min_height, max_height, symm_string, min_aspect,
                                   max_aspect, plot_width, plot_height]
        start = time.time()
        try:
            graph.oneconnected_dual("multiple")
            ui.print_gui('Generated Dimensioned Multiple Rectangular floorplan with Circulation')
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan.")
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Dimensioned Multiple Irregular floorplan instead of Rectangular with Circulation. OCERROR')
        except inputgraph.BCNError:
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Dimensioned Multiple Irregular floorplan instead of Rectangular with Circulation. BCNError')

        graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect,
                               max_aspect, plot_width, plot_height)
        while (graph.floorplan_exist == False):
            old_dims = [min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect]
            min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
                old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
            graph.irreg_multiple_dual()
            graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect,
                                   max_aspect, plot_width, plot_height)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        dim_constraints = [min_width, max_width, min_height, max_height, min_aspect, max_aspect]
        (new_graph, success) = call_circulation_new(ui ,None, graph, node_coord, is_dimensioned,
                                                    dim_constraints, remove_corridor, public_private, drawGUI=drawGUI,gclass=gclass)
        print("Constraints: ", dim_constraints)
        print("New graph data: ", new_graph)
        print("success: ", success)
        # If there was some error in algorithm execution new_graph will be empty
        # we display the pop-up error message
        if new_graph == None:
            messagebox.showerror("Error", "ERROR!! THE INITIAL CHOSEN ENTRY EDGE MUST BE EXTERIOR EDGE")
        # If no issues we continue to draw the corridor
        else:
            if (success == False):
                return
            new_graph.final_traversal = inputgraph.get_final_traversal(new_graph)
            if drawGUI:
                drawFunction(ui, new_graph, origin,ui.get_roomNames(), gclass=gclass)

            feasible_dim = 1 #TODO AYush Check
            return
        if (feasible_dim == 0):
            messagebox.showerror("Error", "ERROR!! NO CIRCULATION POSSIBLE FOR GIVEN DIMENSIONS")


    elif ui.get_isMinDimensioned() == 1 and ui.get_isRemoveAddCirculation() == 1:  # Remove corridors for mindim
        old_dims = [[3] * ui.get_noOfNodes(), [3] * ui.get_noOfNodes()]

        # If the graph came from an input file, the default values are set
        if gclass.open and len(ui.get_dim_constraints()) > 0:
            [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
            print("Dim Constraints before old dims:", ui.get_dim_constraints())
            for i in range(len(old_min_height)):
                old_dims[0][i] = old_min_width[i]
                old_dims[1][i] = old_min_height[i]
            old_dims.extend([plot_width, plot_height])
            gclass.open = False
        min_width, min_height, plot_width, plot_height, optimal_floorplan = mindimgui.gui_fnc(ui,old_dims,
                                                                                              ui.get_noOfNodes(),
                                                                                              ui.get_roomNames(), gclass)
        remove_corridor = True
        start = time.time()
        generate_mindim_rfp(ui, graph, gclass, min_width, min_height, plot_width, plot_height,
                            optimal_floorplan)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        # Sets the ptpg object to the current graph to use for download catalogue and also stores the dimensional constraints of the graph
        ui._set_multiple_output_found(1)
        gclass.ptpg = graph
        ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])

        mindim_graph_data = gclass.output_data[0]
        (new_graph_data, success) = call_circulation_new(ui, mindim_graph_data, graph, node_coord,
                                                         is_dimensioned, ui.get_dim_constraints(),
                                                         remove_corridor, is_minimum_dimensioned=True,
                                                         is_optimal=optimal_floorplan, drawGUI=drawGUI,gclass=gclass)
        print("Constraints: ", dim_constraints)
        print("New graph data: ", new_graph_data)
        print("success: ", success)

        # Resets the flag for minimum dimensioned circulation as it is the same flag as minimum dimensioned floorplans
        gclass.checkvar4.set(0)

        # If there was some error in algorithm execution new_graph_data will be empty
        # we display the pop-up error message
        if new_graph_data == None:
            messagebox.showerror("Error", "ERROR!! THE INITIAL CHOSEN ENTRY EDGE MUST BE EXTERIOR EDGE")

        # If no issues we continue to draw the corridor
        elif success == True:
            if drawGUI:
                drawFunction(ui, new_graph_data, origin, ui.get_roomNames(), gclass=gclass) #TODO FIx if remove circulation is not required on the same tab.

    elif (ui.get_isDimensionedCirculation() == 0 and ui.get_isRemoveAddCirculation() == 1 and ui.get_isPublic() == 0):  # Add/remove
        remove_corridor = True
        start = time.time()
        graph.irreg_single_dual()
        ui.print_gui('Generated Single Irregular floorplan with Circulation. Remove/Add Circulation:Enabled and Public: disabled')
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        print("type of roomx " + str(type(graph.room_x)))

        (new_graph, success) = call_circulation_new(ui, None, graph, node_coord, is_dimensioned,
                                                    dim_constraints, remove_corridor, public_private, drawGUI=drawGUI,gclass=gclass)
        new_graph.final_traversal = inputgraph.get_final_traversal(new_graph)
        # If there was some error in algorithm execution new_graph will be empty
        # we display the pop-up error message
        if new_graph == None:
            messagebox.showerror("Error", "ERROR!! THE INITIAL CHOSEN ENTRY EDGE MUST BE EXTERIOR EDGE")

        # If no issues we continue to draw the corridor
        else:
            print(new_graph.final_traversal)
            if drawGUI:
                drawFunction(ui, new_graph, origin,ui.get_roomNames(), gclass=gclass)

    elif (ui.get_isDimensionedCirculation() == 0 and ui.get_isPublic() == 1):
        public_private = True
        start = time.time()
        graph.irreg_single_dual()
        ui.print_gui('Generated Single Irregular floorplan with Circulation. Dimensioned: Disabled and Public: Enabled')
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        (new_graph, success) = call_circulation_new(ui, None, graph, node_coord, is_dimensioned,
                                                    dim_constraints, remove_corridor, public_private, drawGUI=drawGUI,gclass=gclass)

        # If there was some error in algorithm execution new_graph will be empty
        # we display the pop-up error message
        if new_graph == None:
            messagebox.showerror("Error", "ERROR!! THE INITIAL CHOSEN ENTRY EDGE MUST BE EXTERIOR EDGE")

        # If no issues we continue to draw the corridor
        else:
            new_graph.final_traversal = inputgraph.get_final_traversal(new_graph)
            if drawGUI:
                drawFunction(ui, new_graph, origin - 300, ui.get_roomNames(), gclass=gclass)

def handle_single(ui, graph, drawGUI = False, gclass = None):
    if not graph.is_connected():#Check if the graph is connected or not
                connect_graph.one_connected(graph.matrix)
    if (ui.get_isDimensioned() == 0 and ui.get_isMinDimensioned() == 0 ):  # Non-Dimensioned single dual
        start = time.time()
        # Resets the data already present for downloading catalogues
        ui._set_output_data([])
        ui._set_multiple_output_found(0)
        
        graph.irreg_single_dual()
        ui.print_gui('Generated Single Irregular floorplan')
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        graph.final_traversal = inputgraph.get_final_traversal(graph)
        if drawGUI:
            drawFunction(ui, graph, origin,ui.get_roomNames(), gclass=gclass)
        ui._append_output_data(graph) #Keep OTUPUT DATA AT The END OTHERWISE Final Traversal would run twice

    elif (ui.get_isMinDimensioned() == 1):  # Minimum dimensioned floorplan
        if(gclass is not None):
            old_dims = [[3] * ui.get_noOfNodes()
                , [3] * ui.get_noOfNodes()]

            # If the graph came from an input file, the default values are set
            if gclass.open and len(ui.get_dim_constraints()) > 0:
                [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
                print("Dim Constraints before old dims:", ui.get_dim_constraints())
                for i in range(len(old_min_height)):
                    old_dims[0][i] = old_min_width[i]
                    old_dims[1][i] = old_min_height[i]
                old_dims.extend([plot_width, plot_height])
                gclass.open = False
            min_width, min_height, plot_width, plot_height, optimal_floorplan = mindimgui.gui_fnc(ui,old_dims,
                                                                                                ui.get_noOfNodes(),
                                                                                                ui.get_roomNames(), gclass)
        else:
            min_width, min_height, plot_width, plot_height, optimal_floorplan = ui.min_dim_inputs.get_min_width(), ui.min_dim_inputs.get_min_height(), ui.min_dim_inputs.get_plot_width(), ui.min_dim_inputs.get_plot_height(), ui.min_dim_inputs.get_isOptimalEnabled()

        ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])

        start = time.time()
        graph.irreg_multiple_dual()
        number_of_floorplans = graph.fpcnt
        floorplan_found = False

        # Resets the data already present for downloading catalogues
        ui._set_output_data([])
        ui._set_multiple_output_found(0)

        min_area = -1
        min_graph = None
        areas = []

        # Iterate through all possible floorplans to find one which satisfies the given conditions
        for i in range(number_of_floorplans):
            print("Trying floorplan number", i + 1, "to see if minimum dimension floorplan can be constructed.")
            floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                        ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                        ui.get_corridor_thickness())
            enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                        graph.graph_list[i].room_height)
            floorplan_data = floorplan_obj.get_floorplan_details(
                ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height, graph.graph_list[i].room_x,
                graph.graph_list[i].room_y, graph.graph_list[i].room_width, graph.graph_list[i].room_height,
                ui.get_edges(), enc_mat
            )
            floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

            # Storing the dummy merge nodes, and the irregular room they will be adjacent to
            dummy_node_data = {'mergednodes': graph.graph_list[i].mergednodes, 'irreg_nodes1': graph.graph_list[i].irreg_nodes1,
                            'irreg_nodes2': graph.graph_list[i].irreg_nodes2,
                            'dummy_node_adj': graph.graph_list[i].dummy_node_adjacencies}
            floorplan_data.update(dummy_node_data)
            for merge_node in graph.graph_list[i].mergednodes:
                node_min_width = 1
                node_min_height = 1
                for j in range(0,graph.graph_list[i].nodecnt):
                    if(graph.graph_list[i].matrix[merge_node][j] == 1):
                        node_min_width = max(node_min_width, floorplan_data['nodes'][j]['min_width'])
                        node_min_height = max(node_min_height,floorplan_data['nodes'][j]['min_height'])
                floorplan_data['nodes'][merge_node]['min_width'] = node_min_width/2
                floorplan_data['nodes'][merge_node]['min_height'] = node_min_height/2

            # If floorplan satisfying the given constraints is satisfied
            [status, out_data] = min_dim.main(floorplan_data, plot_width, plot_height)
            if status == True:
                room_x = []
                room_y = []
                room_width = []
                room_height = []
                room_area = []
                room_name = []
                for room_detail in out_data["nodes"]:
                    room_x.append(room_detail["room_x"])
                    room_y.append(room_detail["room_y"])
                    room_width.append(room_detail["width"])
                    room_height.append(room_detail["height"])
                    room_area.append(room_detail["width"] * room_detail["height"])

                # Store the room labels if they have been entered
                for room_id in range(len(out_data["nodes"])):
                    if "label" not in out_data["nodes"][room_id]:
                        room_name.append(str(room_id))
                    else:
                        room_name.append(out_data["nodes"][room_id]["label"])

                # room_x = np.array(room_x)
                # room_y = np.array(room_y)
                # room_width = np.array(room_width)
                # room_height = np.array(room_height)

                graph.graph_list[i].room_x = room_x
                graph.graph_list[i].room_y = room_y
                graph.graph_list[i].room_width = room_width
                graph.graph_list[i].room_height = room_height
                graph.graph_list[i].area = room_area

                floorplan_found = True

                if optimal_floorplan == 0:
                    graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                    ui._append_output_data(graph.graph_list[i]) 
                    ui._set_multiple_output_found(1)
                    if drawGUI:
                        drawFunction(ui, graph.graph_list[i], origin, ui.get_roomNames(), gclass=gclass)
                    break

                '''
                Adds the graph data to output_data for downloading the catalogue and
                multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                '''
                # Store graph data if graph area is less than current minimal area
                area_sum = sum(room_area)
                areas.append(area_sum)
                if min_area < 0 or area_sum < min_area:
                    min_area = area_sum
                    min_graph = graph.graph_list[i]
                    inputgraph.get_final_traversal(min_graph)
                    min_area = area_sum
                # ui._append_output_data(graph.graph_list[i])
                # ui._set_multiple_output_found(1)

        if not floorplan_found:
            print("No floorplan found which satisfies the minimum dimensions input by user.")
        elif optimal_floorplan == 1:
            ui._append_output_data(min_graph)#Keep OTUPUT DATA AT The END OTHERWISE Final Traversal would run twice
            ui._set_multiple_output_found(1)
            print("Floorplan Areas Possible:", areas, "\nOptimal Area:", min_area)
            if drawGUI:
                drawFunction(ui, min_graph, origin, room_name, gclass=gclass)


        ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        # Sets the ptpg object to the current graph to use for download catalogue and also stores the dimensional constraints of the graph
        # gclass.ptpg = graph#Look into this 

    else:  # Dimensioned single floorplan
        old_dims = [[0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , ""
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()]
        min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
            old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
        dimensional_constraints = [min_width, max_width, min_height, max_height, symm_string, min_aspect,
                                   max_aspect, plot_width, plot_height]
        ui._set_dim_constraints(dimensional_constraints)
        start = time.time()
        graph.irreg_multiple_dual()
        graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect, max_aspect,
                               plot_width, plot_height)
        while (graph.floorplan_exist == False):
            old_dims = [min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect]
            min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
                old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
            graph.irreg_multiple_dual()
            graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect, max_aspect,
                                   plot_width, plot_height)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        graph.final_traversal = inputgraph.get_final_traversal(graph)
        if drawGUI:
            drawFunction(ui, graph, origin,ui.get_roomNames(), gclass=gclass)

def handle_letter_shape(ui, graph, drawGUI = False, gclass = None, nodes_data = None):
    assert ui.get_letter is not None
    if nodes_data is None:
        # gclass is None on the API path (api.py passes nodes_data only for the
        # L shape, which is the only generator that consumes it), so this must
        # not dereference gclass unconditionally.
        nodes_data = gclass.app.nodes_data if gclass is not None else []
    if(ui.get_isDimensioned() == 0): #Non-Dimensioned Letter Shape
        start = time.time()
        if(ui.get_letter() == "L Shape"):
            Lshaped.LShapedFloorplan(graph, nodes_data)
        elif(ui.get_letter() == "T Shape"):
            Tshaped.TShapedFloorplan(graph)
        elif(ui.get_letter() == "Z Shape"):
            Zshaped.ZShapedFloorplan(graph)
        elif(ui.get_letter() == "U Shape"):
            Ushaped.UShapedFloorplan(graph)
        end = time.time()
        print("REL MATRIX \n", graph.matrix)
        graph.final_traversal=inputgraph.get_final_traversal(graph)
        if drawGUI:
            drawFunction(ui, graph, origin,ui.get_roomNames(), gclass = gclass)
    else:
        old_dims = [[0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , ""
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()]
        min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
            old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
        start = time.time()
        if(ui.get_letter() == "L Shape"):
            Lshaped.LShapedFloorplan(graph, nodes_data)
        else:
            # The T, Z and U dimensioned generators below were commented out at
            # some point, which left the U call dangling inside the L branch (so
            # an L shape ran both generators) and left T/Z/U running no generator
            # at all, dimensioning whatever matrix the graph arrived with. Fail
            # loudly instead of returning a floorplan built from a stale matrix.
            # elif(ui.get_letter() == "T Shape"):
            #     Tshaped.TShapedFloorplan(graph)
            # elif(ui.get_letter() == "Z Shape"):
            #     Zshaped.ZShapedFloorplan(graph)
            # elif(ui.get_letter() == "U Shape"):
            #     Ushaped.UShapedFloorplan(graph)
            raise ValueError(
                "Dimensioned letter-shape floorplans are implemented for the L shape only; "
                + str(ui.get_letter()) + " is not supported.")
        graph.rel_matrix_list.append(graph.matrix) #All rels are in it. Currently we have only 1.

        # Since multiple rfp are generated before calling single_floorplan, all the parameters need to be
        # converted list of lists
        temp_lst = []
        temp_lst.append(graph.extranodes)
        graph.extranodes = temp_lst
        temp_lst = []
        temp_lst.append(graph.mergednodes)
        graph.mergednodes = temp_lst
        temp_lst = []
        temp_lst.append(graph.irreg_nodes1)
        graph.irreg_nodes1 = temp_lst
        temp_lst = []
        temp_lst.append(graph.room_x)
        graph.room_x = temp_lst
        temp_lst = []
        temp_lst.append(graph.room_y)
        graph.room_y = temp_lst
        temp_lst = []
        temp_lst.append(graph.room_width)
        graph.room_width = temp_lst
        temp_lst = []
        temp_lst.append(graph.room_height)
        graph.room_height = temp_lst

        graph.nodecnt -= 4 # Because Lshaped was counting NESW as well

        graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect,
                                max_aspect, plot_width, plot_height)

        # Add code here in case multiple RELs get generated.

        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        graph.final_traversal=inputgraph.get_final_traversal(graph)
        if drawGUI:
            drawFunction(ui, graph, origin, ui.get_roomNames(), gclass = gclass)

def handle_multiple_l(ui, graph, gclass = None, nodes_data = None):
    nodes_data = nodes_data if nodes_data is not None else gclass.app.nodes_data
    if(ui.get_isDimensioned() == 0):#Non-Dimensioned multiple dual
        start = time.time()
        Lshaped.multipleLshapedFloorplans(graph, nodes_data)
        end = time.time()
        graph.fpcnt = len(graph.graph_list)
        ui.print_gui("Average Time taken: " + str(((end - start) * 1000) / graph.fpcnt) + " ms")
        ui.print_gui("Number of floorplans: " + str(graph.fpcnt))

        for idx in range(graph.fpcnt):
            graph_new = graph.graph_list[idx]
            graph_new.final_traversal = inputgraph.get_final_traversal(graph_new)
            ui._set_multiple_output_found(1)
            ui._append_output_data(graph_new)#Keep OTUPUT DATA AT The END OTHERWISE Final Traversal would run twice
         

def handle_staircase_shaped(ui, graph, drawGUI = False, gclass = None):
    start = time.time()
    inputgraph.staircaseshaped(graph)
    end = time.time()
    graph.final_traversal=inputgraph.get_final_traversal(graph)
    if drawGUI:
        drawFunction(ui, graph, origin, ui.get_roomNames(), gclass = gclass)
    return graph
#
def handle_multiple(ui, graph, gclass = None):
    if not graph.is_connected():#Check if the graph is connected or not
            connect_graph.one_connected(graph.matrix)
    if(ui.get_isDimensioned() == 0):#Non-Dimensioned multiple dual
        start = time.time()
        # Resets the data already present for downloading catalogues
        ui._set_output_data([])
        ui._set_multiple_output_found(0)

        graph.irreg_multiple_dual()
        end = time.time()
        ui.print_gui("Average Time taken: " + str(((end - start) * 1000) / graph.fpcnt) + " ms")
        ui.print_gui("Number of floorplans: " + str(graph.fpcnt))
        for idx in range(graph.fpcnt):
            graph_new = graph.graph_list[idx]
            ui._append_output_data(graph_new)#Don't put get final travesal here
            ui._set_multiple_output_found(1)
    else:#Dimensioned multiple floorplans
        old_dims = [[0] * ui.get_noOfNodes()
                    , [0] * ui.get_noOfNodes()
                    , [0] * ui.get_noOfNodes()
                    , [0] * ui.get_noOfNodes()
                    , ""
                    , [0] * ui.get_noOfNodes()
                    , [0] * ui.get_noOfNodes()]
        min_width,max_width,min_height,max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height  = dimgui.gui_fnc(ui, old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
        start = time.time()
        graph.irreg_multiple_dual()
        graph.multiple_floorplan(min_width,min_height,max_width,max_height,symm_string, min_aspect, max_aspect, plot_width, plot_height)
        end = time.time()
        ui.print_gui("Time taken: " + str((end-start)*1000) + " ms")
        ui.print_gui("Number of floorplans: " +  str(len(graph.graph_list)))
        ui._set_multiple_output_found(1)
        for idx in range(len(graph.fpcnt)):
            graph_new = graph.graph_list[idx]
            ui._append_output_data(graph_new) 

def handle_single_oc(ui, graph, drawGUI = False, gclass = None):
    if (ui.get_isDimensioned() == 0 and ui.get_isMinDimensioned() == 0):  # Non-Dimensioned single rectangular dual
        start = time.time()
        try:
            graph.oneconnected_dual("single")
            ui.print_gui('Generated single rectangular floorplan')
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan.")
            graph.irreg_single_dual()
            ui.print_gui('Generated Single Irregular floorplan instead of Rectangular. OCERROR')
        except inputgraph.BCNError:
            graph.irreg_single_dual()
            ui.print_gui('Generated Single Irregular floorplan instead of Rectangular. BCNERROR')

        temp_graph_data = {}
        temp_graph_data["nodecnt"] = ui.get_noOfNodes()
        temp_graph_data["edgecnt"] = ui.get_edgeCount()
        temp_graph_data["edgeset"] = ui.get_edges()
        temp_graph_data["node_coordinate"] = ui.get_nodeCoordinates()
        graph_data = {
            'room_x': graph.room_x,
            'room_y': graph.room_y,
            'room_width': graph.room_width,
            'room_height': graph.room_height,
            'area': graph.area,
            'extranodes': graph.extranodes,
            'mergednodes': graph.mergednodes,
            'irreg_nodes': graph.irreg_nodes1
        }

        for key, value in graph_data.items():
            if isinstance(value, np.ndarray):
                temp_graph_data[key] = value.tolist()
            else:
                temp_graph_data[key] = value

        input_path = "./saved_files/input_to_limits.json"
        with open(input_path, 'w') as json_file:
            json_data = json.dump(temp_graph_data, json_file, indent=2)
        print(f"JSON data has been written to {input_path}")

        # min_dim.main(input_path)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        room_name = []
        if len(ui.get_roomNames()) > 0:
            room_name = ui.get_roomNames()
        graph.final_traversal = inputgraph.get_final_traversal(graph)
        if drawGUI:
            drawFunction(ui, graph, origin, room_name, gclass=gclass)
        ui._set_dim_constraints([])

    elif (ui.get_isDimensioned() == 1):  # Dimensioned single floorplan
        old_dims = [[0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , ""
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()]

        # If the graph came from an input file, the default values are set
        if gclass.open and len(ui.get_dim_constraints()) > 0:
            [old_min_width, old_max_width, old_min_height, old_max_height, old_symm_string, old_min_aspect,
             old_max_aspect, old_plot_width, old_plot_height] = ui.get_dim_constraints()
            for i in range(len(old_min_width)):
                old_dims[0][i] = old_min_width[i]
                old_dims[1][i] = old_max_width[i]
                old_dims[2][i] = old_min_height[i]
                old_dims[3][i] = old_max_height[i]
                old_dims[4] = old_symm_string
                old_dims[5][i] = old_min_aspect[i]
                old_dims[6][i] = old_max_aspect[i]
            old_dims.extend([old_plot_width, old_plot_height])
            gclass.open = False
        min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
            old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
        start = time.time()
        try:
            graph.oneconnected_dual("multiple")
            ui.print_gui('Generated single rectangular floorplan. Dimensioned')
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan. Dimensioned")
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular floorplan instead of Rectangular. OCError')
        except inputgraph.BCNError:
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular floorplan instead of Rectangular. BCNError')
        graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect,
                               max_aspect, plot_width, plot_height)
        while (graph.floorplan_exist == False):
            old_dims = [min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect]
            min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
                old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
            graph.multiple_dual()
            graph.single_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect,
                                   max_aspect, plot_width, plot_height)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        room_name = []
        if len(ui.get_roomNames()) > 0:
            room_name = ui.get_roomNames()
        graph.final_traversal = inputgraph.get_final_traversal(graph)
        if drawGUI:
            drawFunction(ui, graph, origin, room_name, gclass=gclass)

        dimensional_constraints = [min_width, max_width, min_height, max_height, symm_string,
                                   min_aspect, max_aspect, plot_width, plot_height]
        ui._append_output_data(graph)# Keep this after get final traversal
        ui._set_dim_constraints(dimensional_constraints)

    elif (ui.get_isMinDimensioned() == 1):  # Minimum dimensioned floorplan
        old_dims = [[3] * ui.get_noOfNodes()
            , [3] * ui.get_noOfNodes()]
        # If the graph came from an input file, the default values are set
        if gclass.open and len(ui.get_dim_constraints()) > 0:
            [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
            print("Dim Constraints before old dims:", ui.get_dim_constraints())
            for i in range(len(old_min_height)):
                old_dims[0][i] = old_min_width[i]
                old_dims[1][i] = old_min_height[i]
            old_dims.extend([plot_width, plot_height])
            gclass.open = False
        min_width, min_height, plot_width, plot_height, optimal_floorplan = mindimgui.gui_fnc(ui,old_dims,
                                                                                              ui.get_noOfNodes(),
                                                                                              ui.get_roomNames(), gclass)
        start = time.time()
        try:
            graph.oneconnected_dual("multiple")
            ui.print_gui('Generated multiple rectangular floorplan. Min Dim Enabled')
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan.")
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular floorplan instead of Rectangular. Min Dim Enabled. OCError')
        except inputgraph.BCNError:
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular floorplan instead of Rectangular. Min Dim Enabled. BCNError')
        number_of_floorplans = graph.fpcnt
        floorplan_found = False

        # Resets the data already present for downloading catalogues
        ui._set_output_data([])
        ui._set_multiple_output_found(0)

        # Variables for storing the data of the floorplan with minimal area
        min_area = -1
        min_graph = None
        areas = []

        # Iterate through all possible floorplans to find one which satisfies the given conditions
        for i in range(number_of_floorplans):
            print("Trying floorplan number", i + 1, "to see if minimum dimension floorplan can be constructed.")
            floorplan_obj = input_for_min_dim.floorplan(ui.get_command(), ui.get_isDimensioned(),
                                                        ui.get_isDimensionedCirculation(),
                                                        ui.get_isRemoveAddCirculation(), ui.get_corridor_thickness())
            enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x, graph.graph_list[i].room_y,
                                         graph.graph_list[i].room_width, graph.graph_list[i].room_height)
            floorplan_data = floorplan_obj.get_floorplan_details(
                ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height,
                graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                graph.graph_list[i].room_height,
                ui.get_edges(), enc_mat
            )
            floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))
            print(floorplan_data)
            input_path = "input_to_min_dim.json"
            json_data = json.dumps(floorplan_data, indent=2)

            with open(input_path, 'w') as json_file:
                json_file.write(json_data)
            print(f"JSON data has been written to {input_path}")

            # If floorplan satisfying the given constraints is satisfied
            [status, out_data] = min_dim.main(floorplan_data, plot_width, plot_height)
            if status == True:
                room_x = []
                room_y = []
                room_width = []
                room_height = []
                room_area = []
                room_name = []
                for room_detail in out_data["nodes"]:
                    room_x.append(room_detail["room_x"])
                    room_y.append(room_detail["room_y"])
                    room_width.append(room_detail["width"])
                    room_height.append(room_detail["height"])
                    room_area.append(room_detail["width"] * room_detail["height"])

                # Store the room labels if they have been entered
                for room_id in range(len(out_data["nodes"])):
                    if "label" not in out_data["nodes"][room_id]:
                        room_name.append(str(room_id))
                    else:
                        room_name.append(out_data["nodes"][room_id]["label"])

                # room_x = np.array(room_x)
                # room_y = np.array(room_y)
                # room_width = np.array(room_width)
                # room_height = np.array(room_height)

                graph.graph_list[i].room_x = room_x
                graph.graph_list[i].room_y = room_y
                graph.graph_list[i].room_width = room_width
                graph.graph_list[i].room_height = room_height
                graph.graph_list[i].area = room_area

                '''
                Adds the graph data to output_data for downloading the catalogue and
                multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                '''
                

                floorplan_found = True

                # If optimal area not required, display floorplan
                if optimal_floorplan == 0:
                    graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                    ui._append_output_data(graph.graph_list[i]) 
                    ui._set_multiple_output_found(1)
                    if drawGUI:
                        drawFunction(ui, graph.graph_list[i], origin, room_name, gclass=gclass)
                    break

                # Store graph data if graph area is less than current minimal area
                area_sum = sum(room_area)
                areas.append(area_sum)
                if min_area < 0 or area_sum < min_area:
                    min_area = area_sum
                    min_graph = graph.graph_list[i]
                    min_area = area_sum
                ui._append_output_data(graph.graph_list[i]) 
                ui._set_multiple_output_found(1)
        
        if not floorplan_found:
            print("No floorplan found which satisfies the minimum dimensions input by user.")

        # Display floorplan with optimal area if required
        elif optimal_floorplan == 1:
            print("Floorplan Areas Possible:", areas, "\nOptimal Area:", min_area)
            if drawGUI:
                drawFunction(ui, min_graph, origin, room_name, gclass=gclass)

        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        # Sets the ptpg object to the current graph to use for download catalogue and also stores the dimensional constraints of the graph
        gclass.ptpg = graph
        ui._set_multiple_output_found(1)
        ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])

def handle_multiple_oc(ui, graph, drawGUI = False, gclass = None):
    if (ui.get_isDimensioned() == 0 and ui.get_isMinDimensioned() == 0):  # Non-Dimensioned multiple dual
        start = time.time()
        try:
            graph.oneconnected_dual("multiple")
            ui.print_gui('Generated multiple rectangular floorplan')
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan.")
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular floorplan instead of Rectangular. OCError')
        except inputgraph.BCNError:
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular floorplan instead of Rectangular. BCNError')
        end = time.time()
        ui.print_gui("Average Time taken: " + str(((end - start) * 1000) / graph.fpcnt) + " ms")
        ui.print_gui("Number of floorplans: " + str(graph.fpcnt))
        # CHeck the underneath 100ish lines for merge conflicts #ARSH remove comment

        for idx in range(graph.fpcnt):
            graph_new = graph.graph_list[idx]
            ui._set_multiple_output_found(1)
            ui._append_output_data(graph_new)

    elif (ui.get_isMinDimensioned() == 1):  # Minimum dimensioned rectangular floorplans
        old_dims = [[3] * ui.get_noOfNodes()
            , [3] * ui.get_noOfNodes()]
        # If the graph came from an input file, the default values are set
        if gclass.open and len(ui.get_dim_constraints()) > 0:
            [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
            print("Dim Constraints before old dims:", ui.get_dim_constraints())
            for i in range(len(old_min_height)):
                old_dims[0][i] = old_min_width[i]
                old_dims[1][i] = old_min_height[i]
            old_dims.extend([plot_width, plot_height])
            gclass.open = False
        min_width, min_height, plot_width, plot_height, optimal_floorplan = mindimgui.gui_fnc(ui,old_dims, ui.get_noOfNodes(), ui.get_roomNames(), ui.get_roomNames(), gclass)
        start = time.time()
        try:
            graph.oneconnected_dual("multiple")
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan.")
            graph.irreg_multiple_dual()
        except inputgraph.BCNError:
            graph.irreg_multiple_dual()
        number_of_floorplans = graph.fpcnt
        floorplan_found = False

        # Resets the data already present for downloading catalogues
        ui._set_output_data([])
        ui._set_multiple_output_found(0)

        # Iterate through all possible floorplans to find one which satisfies the given conditions
        for i in range(number_of_floorplans):
            print("Trying floorplan number", i + 1,
                  "to see if minimum dimension floorplan can be constructed.")
            floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                        ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                        ui.get_corridor_thickness())
            enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.room_x[i], graph.room_y[i],
                                         graph.room_width[i], graph.room_height[i])
            floorplan_data = floorplan_obj.get_floorplan_details(
                ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height, graph.room_x[i],
                graph.room_y[i], graph.room_width[i], graph.room_height[i],
                ui.get_edges(), enc_mat
            )
            floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

            # If floorplan satisfying the given constraints is satisfied
            [status, out_data] = min_dim.main(floorplan_data, plot_width, plot_height)
            if status == True:
                room_x = []
                room_y = []
                room_width = []
                room_height = []
                room_area = []
                room_name = []
                for room_detail in out_data["nodes"]:
                    room_x.append(room_detail["room_x"])
                    room_y.append(room_detail["room_y"])
                    room_width.append(room_detail["width"])
                    room_height.append(room_detail["height"])
                    room_area.append(room_detail["width"] * room_detail["height"])

                # Store the room labels if they have been entered
                for room_id in range(len(out_data["nodes"])):
                    if "label" not in out_data["nodes"][room_id]:
                        room_name.append(str(room_id))
                    else:
                        room_name.append(out_data["nodes"][room_id]["label"])

                room_x = np.array(room_x)
                room_y = np.array(room_y)
                room_width = np.array(room_width)
                room_height = np.array(room_height)

                graph_data = {
                    'room_x': room_x,
                    'room_y': room_y,
                    'room_width': room_width,
                    'room_height': room_height,
                    'area': room_area,
                    'extranodes': graph.extranodes,
                    'mergednodes': graph.mergednodes,
                    'irreg_nodes': graph.irreg_nodes1
                }

                '''
                Adds the graph data to output_data for downloading the catalogue and
                multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                '''
                ui._append_output_data(graph_data)
                ui._set_multiple_output_found(1)
                floorplan_found = True
                if drawGUI:
                    drawFunction(ui, graph_data, origin, room_name, gclass=gclass)

        if not floorplan_found:
            print("No floorplan found which satisfies the minimum dimensions input by user.")

        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

        # Sets the ptpg object to the current graph to use for download catalogue and also stores the dimensional constraints of the graph
        gclass.ptpg = graph
        ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])
    # Here might be some errors compare carefully in case of errors because of merge conflicts #ARSH remove this comment if everything is fine
    else: # Dimensioned rectangular floorplan
        old_dims = [[0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()
            , ""
            , [0] * ui.get_noOfNodes()
            , [0] * ui.get_noOfNodes()]
        min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height = dimgui.gui_fnc(ui,
            old_dims, ui.get_noOfNodes(), ui.get_roomNames(), gclass)
        ui._set_dim_constraints([min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height])
        start = time.time()
        try:
            graph.oneconnected_dual("multiple")
            ui.print_gui('Generated multiple Dimensioned Rectangular floorplan')
        except inputgraph.OCError:
            show_warning("Can not generate rectangular floorplan.")
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular Dimensioned floorplan instead of Rectangular. OCError')
        except inputgraph.BCNError:
            graph.irreg_multiple_dual()
            ui.print_gui('Generated Multiple Irregular Dimensioned floorplan instead of Rectangular. BCNError')
        graph.multiple_floorplan(min_width, min_height, max_width, max_height, symm_string, min_aspect,
                                max_aspect, plot_width, plot_height)
        end = time.time()
        ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
        ui.print_gui("Number of floorplans: " + str(len(graph.room_x)))
        ui._set_multiple_output_found(1)

        room_name = []
        if len(ui.get_roomNames()) > 0:
            room_name = ui.get_roomNames()

        for idx in range(len(graph.room_x)):
            graph_new = graph.graph_list[idx]
            ui._set_dim_constraints([min_width, max_width, min_height, max_height, symm_string, min_aspect, max_aspect, plot_width, plot_height])
            gclass.ptpg = graph
            print_all_rfp = False
            if print_all_rfp == True:
                gclass.ocan.add_tab()
                gclass.pen = gclass.ocan.getpen()
                gclass.pen.speed(0)
                graph_new.final_traversal=inputgraph.get_final_traversal(graph_new)
                if drawGUI:
                    drawFunction(ui, graph_new, origin, room_name, gclass = gclass)
            ui._append_output_data(graph_new)

def handle_poly(ui, graph, drawGUI = False, gclass = None):
    start = time.time()
    # graph.irreg_single_dual()
    graph.polyonalinput(gclass.canonicalObject, gclass.v1, gclass.v2, gclass.vn, gclass.po, ui.get_edges(),
                        gclass.debugcano)
    end = time.time()
    if drawGUI:
        drawFunction(ui, graph, origin, ui.get_roomNames(), isPoly=True, gclass = gclass)


def handle_limits(ui, graph, drawGUI = False, gclass = None):
    input_json = {}
    graph_data = {}
    start = time.time()
    input_path = "./saved_files/input_to_limits.json"

    try:
        with open(input_path, 'r') as file:
            input_json = json.load(file)
    except:
        show_warning("Draw a floorplan before applying Limits algorithm.")


    for key, value in input_json.items():
        if isinstance(value, list):
            graph_data[key] = np.array(value)
        else:
            graph_data[key] = value

    # delete_file(input_path)

    end = time.time()


    # graph1 = deepcopy(self.graph)

    # graph1 = nx.Graph(edgeset)
    nodecnt = graph_data['nodecnt']
    edgecnt = graph_data['edgecnt']
    edgeset = graph_data['edgeset']
    node_coordinate = graph_data['node_coordinate']

    adj_matrix = np.zeros((nodecnt,nodecnt), int)

    for edges in (edgeset):
        x = (int)(edges[0])
        y = (int)(edges[1])
        adj_matrix[x][y] = 1
        adj_matrix[y][x] = 1


    graph_data = {
                'room_x': graph_data["room_x"],
                'room_y': graph_data["room_y"],
                'room_width': graph_data["room_width"],
                'room_height': graph_data["room_height"],
                'area': graph_data["area"],
                'extranodes': graph_data["extranodes"],
                'mergednodes': graph_data["mergednodes"],
                'irreg_nodes': graph_data["irreg_nodes"]
            }
    cr_data = []
    rooms = []
    coordinates = {}
    print()
    for i in range(graph_data['room_x'].shape[0]):
        temp=[]
        data = []
        data.append([(graph_data['room_x'][i],graph_data['room_y'][i]),
                        (graph_data['room_x'][i] + graph_data['room_width'][i],graph_data['room_y'][i])])
        data.append([(graph_data['room_x'][i] + graph_data['room_width'][i],graph_data['room_y'][i]),
                        (graph_data['room_x'][i] + graph_data['room_width'][i],graph_data['room_y'][i] + graph_data['room_height'][i])])
        data.append([(graph_data['room_x'][i] + graph_data['room_width'][i],graph_data['room_y'][i] + graph_data['room_height'][i]),
                        (graph_data['room_x'][i],graph_data['room_y'][i] + graph_data['room_height'][i])])
        data.append([(graph_data['room_x'][i],graph_data['room_y'][i] + graph_data['room_height'][i]),
                        (graph_data['room_x'][i],graph_data['room_y'][i])])
        coordinates[i] = data
        # print(data)
        Room_i = poly.Room()
        for i in data:
            temp.append(i[0])

        temp2 = [temp[0]] + temp[1:][::-1]

        print(temp2)
        for i in temp2:
            Room_i.coords.append(i)
        rooms.append(Room_i)

    s_dir = ""
    dist = 0
    if(gclass.side == 0):
        s_dir = gclass.left_dropdown_value
        dist = gclass.left_shift_value
    elif(gclass.side == 1):
        s_dir = gclass.right_dropdown_value
        dist = gclass.right_shift_value
    elif(gclass.side == 2):
        s_dir = gclass.top_dropdown_value
        dist = gclass.top_shift_value
    elif(gclass.side == 3):
        s_dir = gclass.bottom_dropdown_value
        dist = gclass.bottom_shift_value

    limits_instance = lim.LimitsAlgorithm(gclass.side,gclass.room_limits,rooms,adj_matrix,s_dir,dist)
    old_unchanged_coordinates = []
    if(limits_instance.proceed==1):
        newRoomSet = []
        exact_RoomSet = []
        for i in range(len(adj_matrix[gclass.room_limits])):
            if adj_matrix[gclass.room_limits][i]==1:
                newRoomSet.append(rooms[i])
                exact_RoomSet.append(i)
                old_unchanged_coordinates.append(False)
            else:
                old_unchanged_coordinates.append(True)
        newCoordsInstance = nc.NewCoordinateAlgorithm(exact_RoomSet,newRoomSet,s_dir,dist,limits_instance.coords_input1,limits_instance.coords_input2, gclass.side,rooms[gclass.room_limits])
        k=0
        _circular_traversal = []
        new_graph  = inputgraph.InputGraph(ui.get_noOfNodes()
            , ui.get_edgeCount()
            , ui.get_edges()
            , ui.get_nodeCoordinates())
        ui.set_graph(new_graph) 

        for i in range(len(old_unchanged_coordinates)):
            if(i==gclass.room_limits):
                _circular_traversal.append(newCoordsInstance.circularTraversalOfSelectedRoomForMain())

            elif(old_unchanged_coordinates[i]==True):
                _circular_traversal.append(inputgraph.get_circular_traversal(graph_data['room_x'],graph_data['room_y'],graph_data['room_width'],graph_data['room_height'],i))
            else:
                _circular_traversal.append(newCoordsInstance.adjrooms[k].coord())
                k = k + 1
            new_graph.final_traversal.append(_circular_traversal[i])

        # graph_data = {
        #             'room_x': new_graph_data["room_x"],
        #             'room_y': new_graph_data["room_y"],
        #             'room_width': new_graph_data["room_width"],
        #             'room_height': new_graph_data["room_height"],
        #             'area': [],
        #             'extranodes': [],
        #             'mergednodes': [],
        #             'irreg_nodes': []
        #         }

        # temp_graph_data = {}
        # temp_graph_data["nodecnt"] = ui.get_noOfNodes()
        # temp_graph_data["edgecnt"] = ui.get_edgeCount()
        # temp_graph_data["edgeset"] = ui.get_edges()
        # temp_graph_data["node_coordinate"] = ui.get_nodeCoordinates()

        # for key, value in graph_data.items():
        #     if isinstance(value, np.ndarray):
        #         temp_graph_data[key] = value.tolist()
        #     else:
        #         temp_graph_data[key] = value

        # input_path = "./saved_files/input_to_limits.json"
        # with open(input_path, 'w') as json_file:
        #     json_data = json.dump(temp_graph_data,json_file, indent=2)
        # print(f"JSON data has been written to {input_path}")

        # for key, value in new_graph_data.items():
        #     if isinstance(value, list):
        #         new_graph_data[key] = np.array(value)
        #     else:
        #         new_graph_data[key] = value

        # new_graph.circular_traversal =
        print("circular traversal for all rooms: ", new_graph.final_traversal)
        # new_graph.room_x = new_graph_data['room_x']
        # new_graph.room_y = new_graph_data['room_y']
        # new_graph.room_width = new_graph_data['room_width']
        # new_graph.room_height = new_graph_data['room_height']
        # new_graph.area = new_graph_data['area']
        # new_graph.extranodes = new_graph_data['extranodes']
        # new_graph.mergednodes = new_graph_data['mergednodes']
        # new_graph.irreg_nodes1 = new_graph_data['irreg_nodes']

        if drawGUI:
            drawFunction(ui, new_graph, origin, ui.get_roomNames(), gclass = gclass)
    else:
        print("Limit Exceeded")
        # show_warning(newCoordsInstance.error_message)

def handle_door_connectivity(ui, graph, drawGUI=False, gclass=None,
                             candidate_callback=None):
    """Generate door-connectivity plans.

    ``candidate_callback``, when supplied by the API layer, is an internal
    progress hook with the shape ``(graph, sort_key, remaining_attempts)``.
    It sees only solved PTPG candidates, and any exception it raises is
    ignored so observing progress can never change generation.
    """

    def notify_candidate(candidate, sort_key, remaining_attempts):
        if candidate_callback is None:
            return
        try:
            candidate_callback(candidate, sort_key, remaining_attempts)
        except Exception:
            # Progress is best effort and must never affect the solver.
            pass

    if not graph.is_connected():#Check if the graph is connected or not
                connect_graph.one_connected(graph.matrix)
    # Check for edge intersections
    # nxgraph = nx.from_numpy_array(graph.matrix)
    # nx.draw_networkx(nxgraph,graph.coordinates , label=None,node_size=400 ,node_color='#4b8bc8',font_size=12, font_color='k', font_family='sans-serif', font_weight='normal', alpha=1, bbox=None, ax=None)
    # x_coords = []
    # y_coords = []
    # for coord in enumerate(graph.coordinates):
    #         x_coords.append(coord[1][0])
    #         y_coords.append(coord[1][1])

    # is_not_planar_embedding = check_intersection(x_coords,y_coords, graph.matrix)
    # if is_not_planar_embedding:
    #         nxgraph = nx.from_numpy_array(graph.matrix)
    #         graph.coordinates = nx.planar_layout(nxgraph)  # Recalculate positions
  
    if ui.get_isNonAdj() == 1:#If Non adjacency edges are entered.
        if gclass is not None:#using glcass here because nonadjgui might be present
            non_adj_list = nonadjgui.gui_non_adj(ui)
            print("Non-Adjacency List Applied:", non_adj_list)
        else:
            non_adj_list = ui.get_non_adj_list()
        start = time.time()
        result = graph.door_connectivity(show_graph=drawGUI, non_adj_list=non_adj_list)

        if len(result) == 3:
            graph, checkPTPG, possible = result
            if not possible:
                show_warning("No augmentation found")
                return
        elif len(result) == 2:
            graph, checkPTPG = result
        else:
            show_warning("Unexpected return value from door_connectivity")
            return
        
        # Set PTPG graph for output
        ui.set_ptpg_graph(graph.matrix.tolist())
        
        ui.set_edgeCount(graph.edgecnt)
        ui.set_edges(graph.update_gclass_with_edges(gclass))
        ui.set_nodeCoordinates(graph.coordinates)

    else:
        start = time.time()
        graph, checkPTPG = graph.door_connectivity(show_graph=drawGUI)
        
        # Set PTPG graph for output
        ui.set_ptpg_graph(graph.matrix.tolist())
        
        ui.set_edgeCount(graph.edgecnt)
        ui.set_edges(graph.update_gclass_with_edges(gclass))
        ui.set_nodeCoordinates(graph.coordinates)
    
    multiple_door = False
    ui._set_output_data([])
    ui._set_multiple_output_found(0)
    if (checkPTPG):
        print("PTPG going with RFP")
        if(ui.get_isCirculation() == 1):
            handle_circulation(ui,graph,drawGUI,gclass)
            return

        if (ui.get_isMinDimensioned() == 0):  # Non-Dimensioned single rectangular dual
            if gclass is None:
                multiple_door = ui.get_is_multiple_door()
            if not multiple_door:
                try:
                    graph.oneconnected_dual("single")
                except inputgraph.OCError:
                    show_warning("Can not generate rectangular floorplan.")
                    graph.irreg_single_dual()
                except inputgraph.BCNError:
                    graph.irreg_single_dual()


                end = time.time()
                ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
                room_name = []
                if len(ui.get_roomNames()) > 0:
                    room_name = ui.get_roomNames()
                graph.final_traversal = inputgraph.get_final_traversal(graph)
                ui._append_output_data(graph)
                if drawGUI:
                    drawFunction(ui, graph, origin, room_name, gclass=gclass)
            else:
                # Resets the data already present for downloading catalogues
                ui._set_output_data([])
                ui._set_multiple_output_found(0)

                graph.irreg_multiple_dual()
                end = time.time()
                ui.print_gui("Average Time taken: " + str(((end - start) * 1000) / graph.fpcnt) + " ms")
                ui.print_gui("Number of floorplans: " + str(graph.fpcnt))
                for idx in range(graph.fpcnt):
                    graph_new = graph.graph_list[idx]
                    ui._append_output_data(graph_new)#Don't put get final travesal here
                    ui._set_multiple_output_found(1)

        elif (ui.get_isMinDimensioned() == 1):  # Min dimension floorplan
            if(gclass is not None):
                old_dims = [[3] * ui.get_noOfNodes()
                    , [3] * ui.get_noOfNodes()]

                # If the graph came from an input file, the default values are set
                if gclass.open and len(ui.get_dim_constraints()) > 0:
                    [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
                    print("Dim Constraints before old dims:", ui.get_dim_constraints())
                    for i in range(len(old_min_height)):
                        old_dims[0][i] = old_min_width[i]
                        old_dims[1][i] = old_min_height[i]
                    old_dims.extend([plot_width, plot_height])
                    gclass.open = False
                min_width, min_height, plot_width, plot_height, optimal_floorplan,allow_rotation= mindimgui.gui_fnc(ui,old_dims,
                                                                                                    ui.get_noOfNodes(),
                                                                                                    ui.get_roomNames(), gclass)
                multiple_door = False
            else:
                min_width, min_height, plot_width, plot_height, optimal_floorplan,allow_rotation,multiple_door = ui.min_dim_inputs.get_min_width(), ui.min_dim_inputs.get_min_height(), ui.min_dim_inputs.get_plot_width(), ui.min_dim_inputs.get_plot_height(), ui.min_dim_inputs.get_isOptimalEnabled(),ui.min_dim_inputs.get_isRotationAllowed(),ui.get_is_multiple_door()
            start = time.time()
            max_width, max_height = get_max_dims(ui)
            fixed_rooms = getattr(ui.min_dim_inputs, "get_fixed_rooms",
                                  lambda: [])() or []
            fixed_indices = {spec["room"] for spec in fixed_rooms}
            # Axis-specific equality cannot survive the legacy rotate/swap
            # pass, and an anchored room must not rotate away from its side.
            if fixed_indices:
                allow_rotation = 0
            max_dims_released = False
            # Hard plot-fit mode (API opt-in): plot_width/plot_height carry the
            # caller's REAL footprint. The solver cap below already enforces it
            # per topology; what changes here is the boundary pool and the
            # batch composition (see the two enforce_plot branches).
            enforce_plot = bool(getattr(ui.min_dim_inputs, "get_enforce_plot",
                                        lambda: False)())
            if enforce_plot and (plot_width <= 0 or plot_height <= 0):
                enforce_plot = False
            fill_targets = None
            fill_floors = None
            use_fill = enforce_plot or (any("priority" in n for n in getattr(ui.min_dim_inputs, "room_nodes", [])) and plot_width > 0 and plot_height > 0)
            if use_fill:
                # Tier-aware per-room targets for the fill solver: the
                # allocator hands the plot's surplus to living/bedrooms and
                # keeps service rooms small - the functionality rule the
                # exact fill distributes by. Unknown programs degrade to the
                # uniform-factor fallback inside solve_min_dim_fill.
                try:
                    from GPLAN.source.dimensioning.allocator import allocate
                    _alloc = allocate(_allocation_rooms(ui), plot_width, plot_height)
                    fill_targets = [(room["target_width"], room["target_height"])
                                    for room in _alloc["rooms"]]
                    # Per-room AREA quality caps for the fill probes, tiered
                    # like the client's gross-oversize audit: a probe whose
                    # solution blows a room past these is treated as
                    # infeasible, so spanning-cell topologies settle at a
                    # smaller lambda (a slightly under-filled VALID plan)
                    # instead of reaching the plot through a 2.2x bedroom.
                    fill_area_caps = [
                        float(room["max_area"]) * (1.5 if room.get("tier", 3) <= 2
                                                   else 1.4)
                        for room in _alloc["rooms"]]
                    # Per-room RULEBOOK floors (rect feet, walls included).
                    # The two-sided fill search relaxes DOWN to these when a
                    # topology cannot fit the enforced plot at the request's
                    # band minimums - the typed plot outranks a room's
                    # preferred size, never its NBC floor.
                    fill_floors = [(room["min_width"], room["min_height"])
                                   for room in _alloc["rooms"]]
                    # The allocator knows program-room tiers, not hard cores.
                    # Replace its generic "Staircase" envelope with the exact
                    # engine lock before the fill bisection/gate sees it.
                    for spec in fixed_rooms:
                        fi = spec["room"]
                        if fi < len(fill_targets):
                            fill_targets[fi] = (spec["width"], spec["height"])
                        if fi < len(fill_area_caps):
                            fill_area_caps[fi] = spec["width"] * spec["height"]
                        if fi < len(fill_floors):
                            fill_floors[fi] = (spec["width"], spec["height"])
                except Exception as _exc:
                    ui.print_gui("Room allocation could not use the requested sizing controls: %s. "
                                 "Generation will report final room-bound violations." % _exc)
                    print("enforce_plot: allocator unavailable (%s); uniform"
                          " fill fallback" % _exc)
                    fill_targets = None
                    fill_area_caps = None
                    fill_floors = None
            input_dims = [min_width, min_height, plot_width, plot_height]
            input_dims = copy.deepcopy(input_dims)
            if plot_width == 0 and plot_height == 0:
                input_dims = []
            elif enforce_plot:
                # The dim_on_paths_bdy pre-selector tunes the boundary pool
                # for a slack rejection cap and COLLAPSES it at a real plot
                # (measured 2/14/30 plans, 2026-07-24). Enumerate the full
                # pool instead, exactly like the cardinal path, and let the
                # capped solver do the selecting.
                input_dims = []
            if allow_rotation and getattr(graph, "cardinal_constraints", []):
                # The rotation pass swaps x/y, which would rotate pinned rooms
                # off their required compass side - disable it under cardinal
                # constraints.
                allow_rotation = 0
            if allow_rotation:
                print("Will try rotation the floorplans")
            if getattr(graph, "cardinal_constraints", []):
                # Cardinal constraints are enforced by the 8-symmetry boundary
                # filter inside irreg_multiple_dual (which on a PTPG produces
                # exact rectangular duals - no dummy vertices). The
                # one-connected construction below never applies here: by this
                # point door_connectivity() has augmented the graph to
                # biconnected, so oneconnected_dual would always raise
                # BCNError. One-connected inputs are handled upstream by the
                # stacked composition in api.py.
                graph.irreg_multiple_dual(input_dims)
            else:
                try:
                    graph.oneconnected_dual("multiple")
                except inputgraph.OCError:
                    show_warning("Can not generate rectangular floorplan.")
                    graph.irreg_multiple_dual(input_dims)
                except inputgraph.BCNError:
                    graph.irreg_multiple_dual(input_dims)
            number_of_floorplans = graph.fpcnt
            floorplan_found = False
            graph.graph_list = []

            # Resets the data already present for downloading catalogues
            ui._set_output_data([])
            ui._set_multiple_output_found(0)

            # Variables for storing the data of the floorplan with minimal area
            min_area = -1
            min_graph = None
            areas = []
            areas_mapping = []

            # Iterate through all possible floorplans to find one which satisfies the given conditions
            i = -1
            print("Total possible floorplans = ",number_of_floorplans)

            original_graph_list = copy.deepcopy(graph.graph_list)
            # Topologies whose capped solve succeeded, keyed by their slot in
            # graph_list_by_bdy - the enforce_plot top-up pass skips these so
            # the expanded catalogue never duplicates a fitting plan.
            fitted_topologies = set()

            # Upper bound on future candidates for the progress collector.
            # Count every topology, not merely floorplan_per_bdy_limit: failed
            # solves do not advance that limit and could make every entry in a
            # boundary group get tried. Skipped tail entries are removed from
            # the budget at the end of each group.
            remaining_topology_attempts = sum(
                len(group) for group in graph.graph_list_by_bdy)

            for bdy_itr in range(len(graph.graph_list_by_bdy)):
                bdy_fplans = 0
                attempted_in_boundary = 0
                for rel_itr in range(len(graph.graph_list_by_bdy[bdy_itr])):
                    if bdy_fplans >= graph.floorplan_per_bdy_limit:
                        break
                    attempted_in_boundary += 1
                    remaining_topology_attempts = max(
                        0, remaining_topology_attempts - 1)
                    # print("Trying floorplan number", i + 1,
                    #       "to see if minimum dimension floorplan can be constructed.")
                    graph.graph_list.append(graph.graph_list_by_bdy[bdy_itr][rel_itr])#may cause issues later might need to deepcopy
                    temp_graph_copy = copy.deepcopy(graph.graph_list_by_bdy[bdy_itr][rel_itr])
                    original_graph_list.append(temp_graph_copy)
                    i = i+1
                    floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                                ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                                ui.get_corridor_thickness())
                    enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x,
                                                graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                                graph.graph_list[i].room_height)
                    floorplan_data = floorplan_obj.get_floorplan_details(
                        ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height,
                        graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                        graph.graph_list[i].room_height,
                        ui.get_edges(), enc_mat,
                        max_width=max_width, max_height=max_height
                    )
                    floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))


                    # print(floorplan_data)
                    # input_path = "input_to_min_dim.json"
                    # json_data = json.dumps(floorplan_data, indent=2)

                    # with open(input_path, 'w') as json_file:
                    #     json_file.write(json_data)
                    # print(f"JSON data has been written to {input_path}")

                    # If floorplan satisfying the given constraints is satisfied.
                    # enforce_plot: the fill variant bisects a minimum-scale
                    # factor so the tiling stretches toward the plot in the
                    # SOLVER, room by room until each ceiling clamps.
                    _solver = solve_min_dim_fill if use_fill else solve_min_dim
                    _kw = ({"targets": fill_targets,
                            "area_caps": fill_area_caps,
                            "floors": None if getattr(ui.min_dim_inputs, "strict_room_bounds", False) else fill_floors} if use_fill else {})
                    status, out_data, released = _solver(
                        floorplan_data, plot_width, plot_height,
                        max_width is not None or max_height is not None,
                        fixed_indices=fixed_indices, **_kw)
                    max_dims_released = max_dims_released or released
                    if status == True:
                        bdy_fplans += 1
                        fitted_topologies.add((bdy_itr, rel_itr))
                        room_x = []
                        room_y = []
                        room_width = []
                        room_height = []
                        room_area = []
                        room_name = []
                        for room_detail in out_data["nodes"]:
                            room_x.append(room_detail["room_x"])
                            room_y.append(room_detail["room_y"])
                            room_width.append(room_detail["width"])
                            room_height.append(room_detail["height"])
                            room_area.append(room_detail["width"] * room_detail["height"])

                        # Store the room labels if they have been entered
                        # for room_id in range(len(out_data["nodes"])):
                        #     if "label" not in out_data["nodes"][room_id]:
                        #         room_name.append(str(room_id))
                        #     else:
                        #         room_name.append(out_data["nodes"][room_id]["label"])

                        graph.graph_list[i].room_x = room_x
                        graph.graph_list[i].room_y = room_y
                        graph.graph_list[i].room_width = room_width
                        graph.graph_list[i].room_height = room_height
                        graph.graph_list[i].area = room_area          

                        # If optimal area not required, display floorplan
                        if optimal_floorplan == 0:
                            graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(
                                graph.graph_list[i])
                            ui._append_output_data(graph.graph_list[i])
                            floorplan_found = True
                            if drawGUI:
                                drawFunction(ui, graph.graph_list[i], origin, ui.get_roomNames(), gclass=gclass)
                            break
                        
                        #Calculate bound area
                        hMax = 0
                        wMax = 0
                        for itr2 in range(len(room_x)):
                            if room_x[itr2] + room_width[itr2] > wMax:
                                wMax = room_x[itr2] + room_width[itr2]
                            if room_y[itr2] + room_height[itr2] > hMax:
                                hMax = room_y[itr2] + room_height[itr2]

                        boundArea = hMax * wMax
                        # Store graph data if graph area is less than current minimal area
                        area_sum = sum(room_area)
                        areas.append(area_sum)
                        areas_mapping.append(((boundArea - area_sum,area_sum),i))
                        # graph.graph_list[i].area = area_sum
                        graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                        notify_candidate(
                            graph.graph_list[i],
                            (boundArea - area_sum, area_sum),
                            remaining_topology_attempts)
                        if min_area < 0 or area_sum < min_area:
                            min_graph = i
                            min_area = area_sum
                        
                        floorplan_found = True

                        '''
                        Adds the graph data to output_data for downloading the catalogue and
                        multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                        '''
                        # ui._append_output_data(graph.graph_list[i])
                        # ui._set_multiple_output_found(1)
                    else:
                        i = i-1
                        graph.graph_list.pop()
                        original_graph_list.pop()
                        notify_candidate(
                            None, None, remaining_topology_attempts)

                # Once the per-boundary success cap is reached, the unvisited
                # tail can no longer displace an already-seen candidate.
                skipped = (len(graph.graph_list_by_bdy[bdy_itr])
                           - attempted_in_boundary)
                remaining_topology_attempts = max(
                    0, remaining_topology_attempts - skipped)
                notify_candidate(None, None, remaining_topology_attempts)

            # Final progress tick before batch composition. The bounded API
            # preview may already be full, in which case this is a no-op.
            notify_candidate(None, None, 0)

            if allow_rotation :
                print("Rotation is allowed duplicating all floorplans")
                number_of_floorplans = len(graph.graph_list)
                for itr in range(number_of_floorplans):
                    curr_graph = copy.deepcopy(original_graph_list[itr])
                    curr_graph.final_traversal = []
                    curr_graph.circular_traversal = []
                    graph.graph_list.append(curr_graph)
                    i = itr + number_of_floorplans
                    # print("Trying floorplan number", i + 1,
                    #       "to see if minimum dimension floorplan can be constructed.")
                    floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                                ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                                ui.get_corridor_thickness())
                    enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x,
                                                graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                                graph.graph_list[i].room_height)
                    floorplan_data = floorplan_obj.get_floorplan_details(
                        ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height,
                        graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                        graph.graph_list[i].room_height,
                        ui.get_edges(), enc_mat,
                        max_width=max_width, max_height=max_height
                    )
                    floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

                    # print(floorplan_data)
                    # input_path = "input_to_min_dim.json"
                    # json_data = json.dumps(floorplan_data, indent=2)

                    # with open(input_path, 'w') as json_file:
                    #     json_file.write(json_data)
                    # print(f"JSON data has been written to {input_path}")

                    # If floorplan satisfying the given constraints is satisfied
                    # reverse order of plot width and height for the rotated pass
                    _solver = solve_min_dim_fill if use_fill else solve_min_dim
                    _kw = ({"targets": [None if t is None else (t[1], t[0])
                                        for t in fill_targets],
                            "area_caps": fill_area_caps,
                            "floors": [None if f is None else (f[1], f[0])
                                       for f in fill_floors]
                            if fill_floors else None}
                           if use_fill and fill_targets else
                           ({"targets": None} if use_fill else {}))
                    status, out_data, released = _solver(
                        floorplan_data, plot_height, plot_width,
                        max_width is not None or max_height is not None, **_kw)
                    max_dims_released = max_dims_released or released
                    if status == True:
                        room_x = []
                        room_y = []
                        room_width = []
                        room_height = []
                        room_area = []
                        room_name = []
                        for room_detail in out_data["nodes"]:
                            room_x.append(room_detail["room_x"])
                            room_y.append(room_detail["room_y"])
                            room_width.append(room_detail["width"])
                            room_height.append(room_detail["height"])
                            room_area.append(room_detail["width"] * room_detail["height"])

                        # Store the room labels if they have been entered
                        # for room_id in range(len(out_data["nodes"])):
                        #     if "label" not in out_data["nodes"][room_id]:
                        #         room_name.append(str(room_id))
                        #     else:
                        #         room_name.append(out_data["nodes"][room_id]["label"])

                        # changing x with y ahd width by height here to re rotate the entire floorplan
                        graph.graph_list[i].room_x = room_y
                        graph.graph_list[i].room_y = room_x
                        graph.graph_list[i].room_width = room_height
                        graph.graph_list[i].room_height = room_width
                        graph.graph_list[i].area = room_area

                        # If optimal area not required, display floorplan
                        if optimal_floorplan == 0:
                            graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(
                                graph.graph_list[i])
                            ui._append_output_data(graph.graph_list[i])
                            floorplan_found = True
                            if drawGUI:
                                drawFunction(ui, graph.graph_list[i], origin, ui.get_roomNames(), gclass=gclass)
                            break
                        
                        #Calculate bound area
                        hMax = 0
                        wMax = 0
                        for itr2 in range(len(room_x)):
                            if room_x[itr2] + room_width[itr2] > wMax:
                                wMax = room_x[itr2] + room_width[itr2]
                            if room_y[itr2] + room_height[itr2] > hMax:
                                hMax = room_y[itr2] + room_height[itr2]

                        boundArea = hMax * wMax
                        # Store graph data if graph area is less than current minimal area
                        area_sum = sum(room_area)
                        areas.append(area_sum)
                        areas_mapping.append(((boundArea - area_sum,area_sum),i))
                        # graph.graph_list[i].area = area_sum
                        graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                        if min_area < 0 or area_sum < min_area:
                            min_graph = i
                            min_area = area_sum
                        
                        floorplan_found = True


            # ── batch composition ─────────────────────────────────────────
            # Fitting plans first (the two floorplan_found branches below);
            # then, when the caller opted into enforce_plot and the capped
            # pass came up short, a top-up of expanded plans; with no fitting
            # plan at all the legacy expand fallback runs for every caller.
            if floorplan_found and optimal_floorplan == 1 and multiple_door is not True: # Display floorplan with optimal area if required
                areas_mapping.sort()#sort all bound areas
                min_graph = areas_mapping[0][1]
                min_area = areas_mapping[0][0][1]
                print("Area mapping", areas_mapping)
                print("Floorplan Areas Possible:", areas, "\nOptimal Area:", min_area)
                ui._append_output_data(graph.graph_list[min_graph])
                # graph.graph_list[min_graph].final_traversal=inputgraph.get_final_traversal(graph.graph_list[min_graph])
                if drawGUI:
                    drawFunction(ui, graph.graph_list[min_graph], origin, ui.get_roomNames(), gclass=gclass)
            elif floorplan_found and multiple_door == 1:#Sort all the given floorplans based on area then append them at the end of the output list
                areas_mapping.sort()#sort all bound areas
                print("plot height",plot_height,"plot width",plot_width)
                print("area mapping", areas_mapping)
                for floorplan in areas_mapping:
                    ui._append_output_data(graph.graph_list[floorplan[1]])
                    ui._set_multiple_output_found(1)

            fitted_count = len(ui.get_output_data()) if floorplan_found else 0
            # A fixed room is exact by contract; the expand and top-up paths
            # scale every room, so their plans can never pass the fixed gate
            # and only cost time (measured: 197 of 208 plans on a fixed-stair
            # 2BHK, all discarded later). Fitting plans are the whole answer.
            fixed_present = bool(getattr(getattr(ui, "min_dim_inputs", None),
                                         "get_fixed_rooms", lambda: [])())
            need_expand = not floorplan_found and not fixed_present
            topup = (enforce_plot and floorplan_found and multiple_door == 1
                     and fitted_count < ENFORCE_PLOT_TOPUP_TARGET
                     and not fixed_present)
            if need_expand or topup:
                if need_expand:
                    print("No floorplan found which satisfies the minimum dimensions input by user.")
                    print("Getting optimal floorplan with same info just plot data is 0")
                    # This retry drops the PLOT cap only. Per-room maximums still
                    # apply and are released per topology inside solve_min_dim.
                    warning = ("No floorplan fits the given plot dimensions; room dimensions were kept "
                               "and the plot was expanded to fit.")
                    if gclass is not None:
                        messagebox.showwarning("Warning", warning)
                    else:
                        ui.print_gui("Warning: " + warning)

                    ui._set_output_data([])
                    ui._set_multiple_output_found(0)
                else:
                    # enforce_plot top-up: every fitting plan is already in the
                    # output; the remaining topologies are solved uncapped and
                    # appended AFTER them, expanded toward the plot and named
                    # in the message so no client mistakes them for fits.
                    ui.print_gui("Only " + str(fitted_count) + " floorplan(s) fit the plot; "
                                 "the catalogue was topped up with plans that overflow it.")

                # Variables for storing the data of the floorplan with minimal area
                min_area = -1
                min_graph = None
                areas = []

                print("Total possible floorplans = ",number_of_floorplans)
                valid = []
                rotated_mapping = []
                skip_keys = fitted_topologies if topup else set()
                for bdy_itr in range(len(graph.graph_list_by_bdy)):
                    bdy_fplans = 0
                    for rel_itr in range(len(graph.graph_list_by_bdy[bdy_itr])):
                        if bdy_fplans >= graph.floorplan_per_bdy_limit:
                            break
                        if (bdy_itr, rel_itr) in skip_keys:
                            continue
                        graph.graph_list.append(graph.graph_list_by_bdy[bdy_itr][rel_itr])#may cause issues later might need to deepcopy
                        i = len(graph.graph_list) - 1
                        floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                                    ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                                    ui.get_corridor_thickness())
                        enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x,
                                                    graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                                    graph.graph_list[i].room_height)
                        floorplan_data = floorplan_obj.get_floorplan_details(
                            ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height,
                            graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                            graph.graph_list[i].room_height,
                            ui.get_edges(), enc_mat,
                            max_width=max_width, max_height=max_height
                        )
                        floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

                        if enforce_plot and allow_rotation:
                            # A topology that overflows W x H may still fit
                            # H x W: solve against the swapped plot and swap
                            # the geometry back, exactly like the rotation
                            # pass above. Tried BEFORE the uncapped solve so
                            # a rotated FIT beats an expanded plan. The solver
                            # mutates its input, so it gets its own deepcopy.
                            status, out_data, released = solve_min_dim_fill(
                                copy.deepcopy(floorplan_data), plot_height, plot_width,
                                max_width is not None or max_height is not None,
                                targets=([None if t is None else (t[1], t[0])
                                          for t in fill_targets]
                                         if fill_targets else None),
                                area_caps=fill_area_caps,
                                floors=([None if f is None else (f[1], f[0])
                                         for f in fill_floors]
                                        if fill_floors else None),
                                fixed_indices=fixed_indices)
                            if status == True:
                                max_dims_released = max_dims_released or released
                                bdy_fplans += 1
                                room_x = []
                                room_y = []
                                room_width = []
                                room_height = []
                                room_area = []
                                for room_detail in out_data["nodes"]:
                                    room_x.append(room_detail["room_x"])
                                    room_y.append(room_detail["room_y"])
                                    room_width.append(room_detail["width"])
                                    room_height.append(room_detail["height"])
                                    room_area.append(room_detail["width"] * room_detail["height"])
                                # x/y and width/height swapped back into the
                                # plot frame (mirrors the rotation pass).
                                graph.graph_list[i].room_x = room_y
                                graph.graph_list[i].room_y = room_x
                                graph.graph_list[i].room_width = room_height
                                graph.graph_list[i].room_height = room_width
                                graph.graph_list[i].area = room_area
                                wMax = 0
                                hMax = 0
                                for itr2 in range(len(room_x)):
                                    if room_y[itr2] + room_height[itr2] > wMax:
                                        wMax = room_y[itr2] + room_height[itr2]
                                    if room_x[itr2] + room_width[itr2] > hMax:
                                        hMax = room_x[itr2] + room_width[itr2]
                                area_sum = sum(room_area)
                                graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(
                                        graph.graph_list[i])
                                rotated_mapping.append(((wMax * hMax - area_sum, area_sum), i))
                                continue

                        # This pass drops only the PLOT cap; the room ceilings
                        # still apply and are released per topology only when
                        # they are what blocks it.
                        status, out_data, released = solve_min_dim(
                            floorplan_data, 0, 0,
                            max_width is not None or max_height is not None,
                            fixed_indices=fixed_indices)
                        max_dims_released = max_dims_released or released
                        if status == True:
                            bdy_fplans += 1
                            room_x = []
                            room_y = []
                            room_width = []
                            room_height = []
                            room_area = []
                            room_name = []
                            for room_detail in out_data["nodes"]:
                                room_x.append(room_detail["room_x"])
                                room_y.append(room_detail["room_y"])
                                room_width.append(room_detail["width"])
                                room_height.append(room_detail["height"])
                                room_area.append(room_detail["width"] * room_detail["height"])

                            # Store the room labels if they have been entered
                            for room_id in range(len(out_data["nodes"])):
                                if "label" not in out_data["nodes"][room_id]:
                                    room_name.append(str(room_id))
                                else:
                                    room_name.append(out_data["nodes"][room_id]["label"])

                            graph.graph_list[i].room_x = room_x
                            graph.graph_list[i].room_y = room_y
                            graph.graph_list[i].room_width = room_width
                            graph.graph_list[i].room_height = room_height
                            graph.graph_list[i].area = room_area

                            # Store graph data if graph area is less than current minimal area
                            area_sum = sum(room_area)
                            areas.append(area_sum)
                            # graph.graph_list[i].area = area_sum #Check if this line is really needed also look into gui function not writing areas
                            graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(
                                    graph.graph_list[i])
                            valid.append(i)
                            if min_area < 0 or area_sum < min_area:
                                min_graph = i
                                min_area = area_sum

                            '''
                            Adds the graph data to output_data for downloading the catalogue and
                            multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                            '''
                        else:
                            graph.graph_list.pop()

                # Rotated fits are real fits: they land BEFORE anything
                # expanded, least bounding-box waste first.
                rotated_mapping.sort()
                for entry in rotated_mapping:
                    ui._append_output_data(graph.graph_list[entry[1]])
                    ui._set_multiple_output_found(1)
                print("Floorplan Areas Possible:", areas, "\nOptimal Area:", min_area)

                # Expand-only fit toward the plot; least growth first, appended
                # after every fitting plan.
                expanded = graph.scale_plot_dimension(plot_width, plot_height, valid)
                for graphz in expanded:
                    ui._append_output_data(graphz)
                    ui._set_multiple_output_found(1)
                if drawGUI and len(ui.get_output_data()) > 0:
                    drawFunction(ui, ui.get_output_data()[0], origin, ui.get_roomNames(), gclass=gclass)
            end = time.time()
            ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
            if max_dims_released:
                ui.print_gui("Warning: Room maximum dimensions were released for some floorplans.")
            # ui._set_multiple_output_found(1)
            
            # gclass.ptpg = graph#FIX THIS
            ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])
    else:
        ui._set_output_data([])
        ui._set_multiple_output_found(0)
        if gclass is not None:
            messagebox.showwarning("Warning",
                                      "The given graph is not PTPG.Currently using irregular correctness not garunteed.")
        else:
            multiple_door = ui.get_is_multiple_door()#would be better to move this down
            pass#Later on return the error message to front

        # use irregular because not ptpg
        if (ui.get_isDimensioned() == 0 and ui.get_isMinDimensioned() == 0 ):  # Non-Dimensioned single dual
            if not multiple_door :
                graph.irreg_single_dual()
                end = time.time()
                ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
                graph.final_traversal = inputgraph.get_final_traversal(graph)
                if drawGUI:
                    drawFunction(ui, graph, origin, ui.get_roomNames(), gclass=gclass)
                ui._append_output_data(graph)
            else:
                ui._set_output_data([])
                ui._set_multiple_output_found(0)

                graph.irreg_multiple_dual()
                end = time.time()
                ui.print_gui("Average Time taken: " + str(((end - start) * 1000) / graph.fpcnt) + " ms")
                ui.print_gui("Number of floorplans: " + str(graph.fpcnt))
                for idx in range(graph.fpcnt):
                    graph_new = graph.graph_list[idx]
                    ui._append_output_data(graph_new)#Don't put get final travesal here
                    ui._set_multiple_output_found(1)
        elif (ui.get_isMinDimensioned() == 1):  # Minimum dimensioned floorplan
            old_dims = [[3] * ui.get_noOfNodes()
                , [3] * ui.get_noOfNodes()]
            if gclass is not None:
                # If the graph came from an input file, the default values are set
                if gclass.open and len(ui.get_dim_constraints()) > 0:
                    [old_min_width, old_min_height, plot_width, plot_height] = ui.get_dim_constraints()
                    print("Dim Constraints before old dims:", ui.get_dim_constraints())
                    for i in range(len(old_min_height)):
                        old_dims[0][i] = old_min_width[i]
                        old_dims[1][i] = old_min_height[i]
                    old_dims.extend([plot_width, plot_height])
                    gclass.open = False
                min_width, min_height, plot_width, plot_height, optimal_floorplan,allow_rotation = mindimgui.gui_fnc(ui,old_dims,
                                                                                                    ui.get_noOfNodes(),
                                                                                                    ui.get_roomNames(), gclass)
            else:
                min_width, min_height, plot_width, plot_height, optimal_floorplan,allow_rotation,multiple_door = ui.min_dim_inputs.get_min_width(), ui.min_dim_inputs.get_min_height(), ui.min_dim_inputs.get_plot_width(), ui.min_dim_inputs.get_plot_height(), ui.min_dim_inputs.get_isOptimalEnabled(),ui.min_dim_inputs.get_isRotationAllowed(),ui.get_is_multiple_door()

            ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])
            max_width, max_height = get_max_dims(ui)
            max_dims_released = False
            # Same hard plot-fit opt-in as the PTPG branch: full boundary pool
            # + the fill solver. The non-PTPG (irregular) branch shares the
            # solve loop below, so the flag must exist here too.
            enforce_plot = bool(getattr(ui.min_dim_inputs, "get_enforce_plot",
                                        lambda: False)())
            if enforce_plot and (plot_width <= 0 or plot_height <= 0):
                enforce_plot = False
            fill_targets = None
            fill_floors = None
            use_fill = enforce_plot or (any("priority" in n for n in getattr(ui.min_dim_inputs, "room_nodes", [])) and plot_width > 0 and plot_height > 0)
            if use_fill:
                # Tier-aware per-room targets for the fill solver: the
                # allocator hands the plot's surplus to living/bedrooms and
                # keeps service rooms small - the functionality rule the
                # exact fill distributes by. Unknown programs degrade to the
                # uniform-factor fallback inside solve_min_dim_fill.
                try:
                    from GPLAN.source.dimensioning.allocator import allocate
                    _alloc = allocate(_allocation_rooms(ui), plot_width, plot_height)
                    fill_targets = [(room["target_width"], room["target_height"])
                                    for room in _alloc["rooms"]]
                    # Per-room AREA quality caps for the fill probes, tiered
                    # like the client's gross-oversize audit: a probe whose
                    # solution blows a room past these is treated as
                    # infeasible, so spanning-cell topologies settle at a
                    # smaller lambda (a slightly under-filled VALID plan)
                    # instead of reaching the plot through a 2.2x bedroom.
                    fill_area_caps = [
                        float(room["max_area"]) * (1.5 if room.get("tier", 3) <= 2
                                                   else 1.4)
                        for room in _alloc["rooms"]]
                    # Per-room RULEBOOK floors (rect feet, walls included).
                    # The two-sided fill search relaxes DOWN to these when a
                    # topology cannot fit the enforced plot at the request's
                    # band minimums - the typed plot outranks a room's
                    # preferred size, never its NBC floor.
                    fill_floors = [(room["min_width"], room["min_height"])
                                   for room in _alloc["rooms"]]
                except Exception as _exc:
                    ui.print_gui("Room allocation could not use the requested sizing controls: %s. "
                                 "Generation will report final room-bound violations." % _exc)
                    print("enforce_plot: allocator unavailable (%s); uniform"
                          " fill fallback" % _exc)
                    fill_targets = None
                    fill_area_caps = None
                    fill_floors = None
            input_dims = [min_width, min_height, plot_width, plot_height]
            input_dims = copy.deepcopy(input_dims)
            if plot_width == 0 and plot_height == 0:
                input_dims = []
            elif enforce_plot:
                input_dims = []

            start = time.time()
            graph.irreg_multiple_dual(input_dims)
            number_of_floorplans = graph.fpcnt
            floorplan_found = False
            graph.graph_list = []

            # Resets the data already present for downloading catalogues
            ui._set_output_data([])
            ui._set_multiple_output_found(0)

            min_area = -1
            min_graph = None
            areas = []
            areas_mapping = []
            i = -1
            # Iterate through all possible floorplans to find one which satisfies the given conditions
            original_graph_list = copy.deepcopy(graph.graph_list)
            for bdy_itr in range(len(graph.graph_list_by_bdy)):
                bdy_fplans = 0
                for rel_itr in range(len(graph.graph_list_by_bdy[bdy_itr])):
                    if bdy_fplans >= graph.floorplan_per_bdy_limit:
                        break
                    print("Trying floorplan number", i + 1, "to see if minimum dimension floorplan can be constructed.")
                    graph.graph_list.append(graph.graph_list_by_bdy[bdy_itr][rel_itr])#may cause issues later might need to deepcopy
                    temp_graph_copy = copy.deepcopy(graph.graph_list_by_bdy[bdy_itr][rel_itr])
                    original_graph_list.append(temp_graph_copy)
                    i = i+1
                    floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                                ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                                ui.get_corridor_thickness())
                    enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                                graph.graph_list[i].room_height)
                    floorplan_data = floorplan_obj.get_floorplan_details(
                        ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height, graph.graph_list[i].room_x,
                        graph.graph_list[i].room_y, graph.graph_list[i].room_width, graph.graph_list[i].room_height,
                        ui.get_edges(), enc_mat,
                        max_width=max_width, max_height=max_height
                    )
                    floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

                    # Storing the dummy merge nodes, and the irregular room they will be adjacent to
                    dummy_node_data = {'mergednodes': graph.graph_list[i].mergednodes, 'irreg_nodes1': graph.graph_list[i].irreg_nodes1,
                                    'irreg_nodes2': graph.graph_list[i].irreg_nodes2,
                                    'dummy_node_adj': graph.graph_list[i].dummy_node_adjacencies}
                    floorplan_data.update(dummy_node_data)
                    for merge_node in graph.graph_list[i].mergednodes:
                        node_min_width = 1
                        node_min_height = 1
                        for j in range(0,graph.graph_list[i].nodecnt):
                            if(graph.graph_list[i].matrix[merge_node][j] == 1):
                                node_min_width = max(node_min_width, floorplan_data['nodes'][j]['min_width'])
                                node_min_height = max(node_min_height,floorplan_data['nodes'][j]['min_height'])
                        floorplan_data['nodes'][merge_node]['min_width'] = node_min_width/2
                        floorplan_data['nodes'][merge_node]['min_height'] = node_min_height/2

                    # If floorplan satisfying the given constraints is satisfied.
                    # enforce_plot: the fill variant bisects a minimum-scale
                    # factor so the tiling stretches toward the plot in the
                    # SOLVER, room by room until each ceiling clamps.
                    _solver = solve_min_dim_fill if use_fill else solve_min_dim
                    _kw = ({"targets": fill_targets,
                            "area_caps": fill_area_caps,
                            "floors": None if getattr(ui.min_dim_inputs, "strict_room_bounds", False) else fill_floors} if use_fill else {})
                    status, out_data, released = _solver(
                        floorplan_data, plot_width, plot_height,
                        max_width is not None or max_height is not None, **_kw)
                    max_dims_released = max_dims_released or released
                    if status == True:
                        bdy_fplans += 1
                        room_x = []
                        room_y = []
                        room_width = []
                        room_height = []
                        room_area = []
                        room_name = []
                        for room_detail in out_data["nodes"]:
                            room_x.append(room_detail["room_x"])
                            room_y.append(room_detail["room_y"])
                            room_width.append(room_detail["width"])
                            room_height.append(room_detail["height"])
                            room_area.append(room_detail["width"] * room_detail["height"])

                        # Store the room labels if they have been entered
                        for room_id in range(len(out_data["nodes"])):
                            if "label" not in out_data["nodes"][room_id]:
                                room_name.append(str(room_id))
                            else:
                                room_name.append(out_data["nodes"][room_id]["label"])

                        # room_x = np.array(room_x)
                        # room_y = np.array(room_y)
                        # room_width = np.array(room_width)
                        # room_height = np.array(room_height)

                        graph.graph_list[i].room_x = room_x
                        graph.graph_list[i].room_y = room_y
                        graph.graph_list[i].room_width = room_width
                        graph.graph_list[i].room_height = room_height
                        graph.graph_list[i].area = room_area

                        floorplan_found = True

                        if optimal_floorplan == 0:
                            graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                            ui._append_output_data(graph.graph_list[i]) 
                            if drawGUI:
                                drawFunction(ui, graph.graph_list[i], origin, ui.get_roomNames(), gclass=gclass)
                            break

                        '''
                        Adds the graph data to output_data for downloading the catalogue and
                        multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                        '''
                        #Calculate bound area
                        hMax = 0
                        wMax = 0
                        for j in range(len(room_x)):
                            if room_x[j] + room_width[j] > wMax:
                                wMax = room_x[j] + room_width[j]
                            if room_y[j] + room_height[j] > hMax:
                                hMax = room_y[j] + room_height[j]

                        boundArea = hMax * wMax
                        # Store graph data if graph area is less than current minimal area
                        area_sum = sum(room_area)
                        areas.append(area_sum)
                        areas_mapping.append(((boundArea-area_sum,area_sum),i))
                        # graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                        if min_area < 0 or area_sum < min_area:
                            min_graph = i
                            min_area = area_sum
                        # ui._append_output_data(graph.graph_list[i])
                        # ui._set_multiple_output_found(1)
                    else:
                        i = i-1
                        graph.graph_list.pop()
                        original_graph_list.pop()
            if allow_rotation :
                print("Rotation is allowed duplicating all floorplans")
                number_of_floorplans = len(graph.graph_list)
                for itr in range(number_of_floorplans):
                    curr_graph = copy.deepcopy(original_graph_list[itr])
                    curr_graph.circular_traversal = []
                    curr_graph.final_traversal = []
                    graph.graph_list.append(curr_graph)
                    i = itr + number_of_floorplans
                    print("Trying floorplan number", i + 1, "to see if minimum dimension floorplan can be constructed.")
                    floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                                ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                                ui.get_corridor_thickness())
                    enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                                graph.graph_list[i].room_height)
                    floorplan_data = floorplan_obj.get_floorplan_details(
                        ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height, graph.graph_list[i].room_x,
                        graph.graph_list[i].room_y, graph.graph_list[i].room_width, graph.graph_list[i].room_height,
                        ui.get_edges(), enc_mat,
                        max_width=max_width, max_height=max_height
                    )
                    floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

                    # Storing the dummy merge nodes, and the irregular room they will be adjacent to
                    dummy_node_data = {'mergednodes': graph.graph_list[i].mergednodes, 'irreg_nodes1': graph.graph_list[i].irreg_nodes1,
                                    'irreg_nodes2': graph.graph_list[i].irreg_nodes2,
                                    'dummy_node_adj': graph.graph_list[i].dummy_node_adjacencies}
                    floorplan_data.update(dummy_node_data)
                    for merge_node in graph.graph_list[i].mergednodes:
                        node_min_width = 1
                        node_min_height = 1
                        for itr2 in range(0,graph.graph_list[i].nodecnt):
                            if(graph.graph_list[i].matrix[merge_node][itr2] == 1):
                                node_min_width = max(node_min_width, floorplan_data['nodes'][itr2]['min_width'])
                                node_min_height = max(node_min_height,floorplan_data['nodes'][itr2]['min_height'])
                        floorplan_data['nodes'][merge_node]['min_width'] = node_min_width/2
                        floorplan_data['nodes'][merge_node]['min_height'] = node_min_height/2

                    # If floorplan satisfying the given constraints is satisfied
                    # swapping plot width and height for the rotated pass
                    status, out_data, released = solve_min_dim(
                        floorplan_data, plot_height, plot_width,
                        max_width is not None or max_height is not None)
                    max_dims_released = max_dims_released or released
                    if status == True:
                        room_x = []
                        room_y = []
                        room_width = []
                        room_height = []
                        room_area = []
                        room_name = []
                        for room_detail in out_data["nodes"]:
                            room_x.append(room_detail["room_x"])
                            room_y.append(room_detail["room_y"])
                            room_width.append(room_detail["width"])
                            room_height.append(room_detail["height"])
                            room_area.append(room_detail["width"] * room_detail["height"])

                        # Store the room labels if they have been entered
                        for room_id in range(len(out_data["nodes"])):
                            if "label" not in out_data["nodes"][room_id]:
                                room_name.append(str(room_id))
                            else:
                                room_name.append(out_data["nodes"][room_id]["label"])

                        # room_x = np.array(room_x)
                        # room_y = np.array(room_y)
                        # room_width = np.array(room_width)
                        # room_height = np.array(room_height)

                        graph.graph_list[i].room_x = room_y
                        graph.graph_list[i].room_y = room_x
                        graph.graph_list[i].room_width = room_height
                        graph.graph_list[i].room_height = room_width
                        graph.graph_list[i].area = room_area

                        floorplan_found = True

                        if optimal_floorplan == 0:
                            graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                            ui._append_output_data(graph.graph_list[i]) 
                            if drawGUI:
                                drawFunction(ui, graph.graph_list[i], origin, ui.get_roomNames(), gclass=gclass)
                            break

                        '''
                        Adds the graph data to output_data for downloading the catalogue and
                        multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                        '''
                        #Calculate bound area
                        hMax = 0
                        wMax = 0
                        for itr3 in range(len(room_x)):
                            if room_x[itr3] + room_width[itr3] > wMax:
                                wMax = room_x[itr3] + room_width[itr3]
                            if room_y[itr3] + room_height[itr3] > hMax:
                                hMax = room_y[itr3] + room_height[itr3]

                        boundArea = hMax * wMax
                        # Store graph data if graph area is less than current minimal area
                        area_sum = sum(room_area)
                        areas.append(area_sum)
                        areas_mapping.append(((boundArea-area_sum,area_sum),i))
                        # graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                        if min_area < 0 or area_sum < min_area:
                            min_graph = i
                            min_area = area_sum
                        # ui._append_output_data(graph.graph_list[i])
                        # ui._set_multiple_output_found(1)

            if not floorplan_found:
                #Here tell front that no floorplan was made
                print("No floorplan found which satisfies the minimum dimensions input by user.")
                print("Getting optimal floorplan with same info just plot data is 0")
                # This retry drops the PLOT cap only. Per-room maximums still
                # apply and are released per topology inside solve_min_dim.
                warning = ("No floorplan fits the given plot dimensions; room dimensions were kept "
                           "and the plot was expanded to fit.")
                if gclass is not None:
                    messagebox.showwarning("Warning", warning)
                else:
                    ui.print_gui("Warning: " + warning)
        
                ui._set_output_data([])
                ui._set_multiple_output_found(0)

                # Variables for storing the data of the floorplan with minimal area
                min_area = -1
                min_graph = None
                areas = []
                valid = []

                i = -1
                # Iterate through all possible floorplans to find one which satisfies the given conditions
                for bdy_itr in range(len(graph.graph_list_by_bdy)):
                    bdy_fplans = 0
                    for rel_itr in range(len(graph.graph_list_by_bdy[bdy_itr])):
                        if bdy_fplans >= graph.floorplan_per_bdy_limit:
                            break
                        print("Trying floorplan number", i + 1, "to see if minimum dimension floorplan can be constructed.")
                        graph.graph_list.append(graph.graph_list_by_bdy[bdy_itr][rel_itr])
                        i = i+1
                        floorplan_obj = input_for_min_dim.floorplan(ui.get_fptype(), ui.get_isDimensioned(),
                                                                    ui.get_isDimensionedCirculation(), ui.get_isRemoveAddCirculation(),
                                                                    ui.get_corridor_thickness())
                        enc_mat = get_encoded_matrix(ui.get_noOfNodes(), graph.graph_list[i].room_x, graph.graph_list[i].room_y, graph.graph_list[i].room_width,
                                                    graph.graph_list[i].room_height)
                        floorplan_data = floorplan_obj.get_floorplan_details(
                            ui.get_roomNames(), ui.get_roomColors(), ui.get_nodeCoordinates(), min_width, min_height, graph.graph_list[i].room_x,
                            graph.graph_list[i].room_y, graph.graph_list[i].room_width, graph.graph_list[i].room_height,
                            ui.get_edges(), enc_mat,
                            max_width=max_width, max_height=max_height
                        )
                        floorplan_data["strict_room_bounds"] = bool(getattr(ui.min_dim_inputs, "strict_room_bounds", False))

                        # Storing the dummy merge nodes, and the irregular room they will be adjacent to
                        dummy_node_data = {'mergednodes': graph.graph_list[i].mergednodes, 'irreg_nodes1': graph.graph_list[i].irreg_nodes1,
                                        'irreg_nodes2': graph.graph_list[i].irreg_nodes2,
                                        'dummy_node_adj': graph.graph_list[i].dummy_node_adjacencies}
                        floorplan_data.update(dummy_node_data)
                        for merge_node in graph.graph_list[i].mergednodes:
                            node_min_width = 1
                            node_min_height = 1
                            for j in range(0,graph.graph_list[i].nodecnt):
                                if(graph.graph_list[i].matrix[merge_node][j] == 1):
                                    node_min_width = max(node_min_width, floorplan_data['nodes'][j]['min_width'])
                                    node_min_height = max(node_min_height,floorplan_data['nodes'][j]['min_height'])
                            floorplan_data['nodes'][merge_node]['min_width'] = node_min_width/2
                            floorplan_data['nodes'][merge_node]['min_height'] = node_min_height/2

                        # This pass drops only the PLOT cap; the room ceilings
                        # still apply and are released per topology only when
                        # they are what blocks it.
                        status, out_data, released = solve_min_dim(
                            floorplan_data, 0, 0, max_width is not None or max_height is not None)
                        max_dims_released = max_dims_released or released
                        if status == True:
                            bdy_fplans += 1
                            room_x = []
                            room_y = []
                            room_width = []
                            room_height = []
                            room_area = []
                            room_name = []
                            for room_detail in out_data["nodes"]:
                                room_x.append(room_detail["room_x"])
                                room_y.append(room_detail["room_y"])
                                room_width.append(room_detail["width"])
                                room_height.append(room_detail["height"])
                                room_area.append(room_detail["width"] * room_detail["height"])

                            # Store the room labels if they have been entered
                            for room_id in range(len(out_data["nodes"])):
                                if "label" not in out_data["nodes"][room_id]:
                                    room_name.append(str(room_id))
                                else:
                                    room_name.append(out_data["nodes"][room_id]["label"])


                            graph.graph_list[i].room_x = room_x
                            graph.graph_list[i].room_y = room_y
                            graph.graph_list[i].room_width = room_width
                            graph.graph_list[i].room_height = room_height
                            graph.graph_list[i].area = room_area


                            area_sum = sum(room_area)
                            areas.append(area_sum)
                            
                            graph.graph_list[i].final_traversal = inputgraph.get_final_traversal(graph.graph_list[i])
                            valid.append(i)
                            # ui._append_output_data(graph.graph_list[i]) 

                            if min_area < 0 or area_sum < min_area:
                                min_graph = i
                                min_area = area_sum
                            '''
                            Adds the graph data to output_data for downloading the catalogue and
                            multiple_output_found flag is set which indicates that catalogue can be downloaded for this output
                            '''
                        else:
                            i = i-1
                            graph.graph_list.pop()

                print("Floorplan Areas Possible:", areas, "\nOptimal Area:", min_area)
                graph.scale_plot_dimension(plot_width, plot_height,valid)
                for i in range(len(graph.graph_list)):
                    graphz = graph.graph_list[i]
                    if(i in valid):
                        ui._append_output_data(graphz)
                        ui._set_multiple_output_found(1)
                min_graph = 0
                if drawGUI:
                    drawFunction(ui, graph.graph_list[min_graph], origin, ui.get_roomNames(), gclass=gclass)



            elif optimal_floorplan == 1 and multiple_door is not True:
                areas_mapping.sort()
                min_graph = areas_mapping[0][1]
                min_area = areas_mapping[0][0][1]
                ui._append_output_data(graph.graph_list[min_graph])
                print("Floorplan Areas Possible:", areas, "\nOptimal Area:", min_area)
                if drawGUI:
                    drawFunction(ui,graph.graph_list[min_graph], origin, room_name, gclass=gclass)
            elif multiple_door == 1:
                areas_mapping.sort()#sort all areas
                for floorplan in areas_mapping:
                    ui._append_output_data(graph.graph_list[floorplan[1]])
                    ui._set_multiple_output_found(1)

            # ui._append_output_data(graph)#Keep OTUPUT DATA AT The END OTHERWISE Final Traversal would run twice
            # ui._set_multiple_output_found(1)
            ui._set_dim_constraints([min_width, min_height, plot_width, plot_height])
            end = time.time()
            ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")
            if max_dims_released:
                ui.print_gui("Warning: Room maximum dimensions were released for some floorplans.")

            # Sets the ptpg object to the current graph to use for download catalogue and also stores the dimensional constraints of the graph
            # gclass.ptpg = graph
        
    end = time.time()
    ui.print_gui("Time taken: " + str((end - start) * 1000) + " ms")

def handle_space_optimization(ui, regions, rooms, fixed_rooms, adjacency, non_adjacency, entrance_coords=None, 
                               max_attempts=100, enable_expansion=True, enable_compaction=True,
                               boundary_bounds=None):
    """
    Handler for space optimization using the negNew FloorPlan engine.
    
    Args:
        ui: GuiParameters object for storing output
        regions: List of region dicts with x, y, width, height
        rooms: List of room dicts with name, width, height, max_expansion
        fixed_rooms: List of fixed room dicts with name, x, y, width, height, is_fixed, max_expansion
        adjacency: List of [room1_name, room2_name] pairs
        non_adjacency: List of [room1_name, room2_name] pairs
        entrance_coords: List of [x, y] coordinate pairs for entrance line
        max_attempts: Maximum placement attempts
        enable_expansion: Whether to enable room expansion
        enable_compaction: Whether to enable room compaction
        boundary_bounds: Optional dict with raw boundary min/max values used to align fixed-room Y coordinates
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        # Import the FloorPlan class from Space_Optimization
        import sys
        import os
        space_opt_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'Space_Optimization')
        if space_opt_path not in sys.path:
            sys.path.insert(0, space_opt_path)
        
        from negNew import FloorPlan

        boundary_min_x = int(boundary_bounds['min_x']) if boundary_bounds and boundary_bounds.get('min_x') is not None else 0
        boundary_max_y = int(boundary_bounds['max_y']) if boundary_bounds and boundary_bounds.get('max_y') is not None else None
        
        # Create FloorPlan instance - DO NOT pass fixed_rooms to constructor
        # We'll add them via add_room() instead
        floor_plan = FloorPlan(region_specs=regions, fixed_rooms=None)
        
        # Set entrance location if provided
        if entrance_coords:
            if boundary_max_y is not None:
                try:
                    entrance_coords = [
                        (int(point[0]) - boundary_min_x, int(boundary_max_y - point[1]))
                        if isinstance(point, (list, tuple)) and len(point) >= 2 else point
                        for point in entrance_coords
                    ]
                except Exception:
                    pass
            floor_plan.set_entrance_location(entrance_coords)
        
        # Add regular rooms
        for room in rooms:
            floor_plan.add_room(
                name=room['name'],
                width=room['width'],
                height=room['height'],
                max_expansion=room.get('max_expansion', 20)
            )
        
        # Add fixed rooms
        for fixed_room in fixed_rooms:
            fixed_y = fixed_room['y']
            fixed_x = fixed_room['x']
            if boundary_max_y is not None:
                try:
                    fixed_x = int(fixed_room['x']) - boundary_min_x
                    fixed_y = boundary_max_y - (int(fixed_room['y']) + int(fixed_room['height']))
                except Exception:
                    fixed_x = fixed_room['x']
                    fixed_y = fixed_room['y']
            floor_plan.add_fixed_room(
                width=fixed_room['width'],
                height=fixed_room['height'],
                fixed_x=fixed_x,
                fixed_y=fixed_y,
                name=fixed_room['name'],
                max_expansion=fixed_room.get('max_expansion', 0),
                polygon_coords=fixed_room.get('polygon'),
                occupied_cells=fixed_room.get('occupied_cells')
            )
        
        # Add adjacency constraints
        for adj_pair in adjacency:
            floor_plan.add_adjacency(adj_pair[0], adj_pair[1])
        
        # Add non-adjacency constraints
        for non_adj_pair in non_adjacency:
            floor_plan.add_non_adjacency(non_adj_pair[0], non_adj_pair[1])
        
        # Generate layout
        success = floor_plan.generate_layout(
            max_attempts=max_attempts,
            enable_expansion=enable_expansion,
            enable_space_optimization=enable_compaction
        )
        
        if not success:
            ui.set_message("Failed to generate space-optimized layout")
            return False
        
        # Compact rooms if enabled
        if enable_compaction:
            floor_plan.compact_rooms()
        
        # Calculate adjacency score and metrics
        score, satisfied_pairs, violations = floor_plan.evaluate_adjacency_score()
        
        # Check entrance adjacency if entrance_coords provided
        entrance_adjacent_rooms = []
        if entrance_coords:
            for room in floor_plan.rooms:
                if room.is_adjacent_to_entrance(entrance_coords):
                    entrance_adjacent_rooms.append(room.name)
        
        # Prepare output data
        output_data = {
            'rooms': [],
            'floor_regions': regions,
            'entrance_coords': entrance_coords,
            'metrics': {
                'adjacency_score': score,
                'satisfied_pairs': satisfied_pairs,
                'non_adjacency_violations': violations,
                'entrance_adjacent_rooms': entrance_adjacent_rooms
            }
        }
        
        # Extract room data
        total_region_area = 0
        for region in floor_plan.floor_regions:
            total_region_area += region['width'] * region['height']
        
        occupied_room_area = 0
        occupied_fixed_area = 0
        
        for room in floor_plan.rooms:
            room_dict = {
                'name': room.name,
                'x': room.x,
                'y': room.y,
                'width': room.width,
                'height': room.height,
                'rotated': room.rotated,
                'is_fixed': getattr(room, 'is_fixed', False),
                'original_width': room.original_width,
                'original_height': room.original_height,
                'area': room.get_area()
            }
            output_data['rooms'].append(room_dict)
            
            if getattr(room, 'is_fixed', False):
                occupied_fixed_area += room.get_area()
            else:
                occupied_room_area += room.get_area()
        
        # Calculate area utilization
        occupied_total_area = occupied_room_area + occupied_fixed_area
        remaining_area = max(0, total_region_area - occupied_total_area)
        remaining_percent = (remaining_area / total_region_area * 100.0) if total_region_area > 0 else 0.0
        utilization_ratio = occupied_total_area / total_region_area if total_region_area > 0 else 0.0
        
        output_data['metrics']['area_utilization'] = {
            'total_region_area': total_region_area,
            'occupied_room_area': occupied_room_area,
            'occupied_fixed_area': occupied_fixed_area,
            'occupied_total_area': occupied_total_area,
            'remaining_area': remaining_area,
            'remaining_percent': remaining_percent,
            'utilization_ratio': utilization_ratio
        }
        
        # Store output in UI
        ui._set_output_data([output_data])
        ui.set_message(f"Space optimization successful. Adjacency score: {score}/{len(adjacency)}")
        
        return True
        
    except Exception as e:
        ui.set_message(f"Space optimization error: {str(e)}")
        print(f"Error in handle_space_optimization: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def handle_ga_optimization(ui, floorplan_data, corridor_width=3, ga_config=None):
    """
    Handler for GA-based floor plan optimization using ga_current.py.
    
    Args:
        ui: GuiParameters object for storing output
        floorplan_data: Dict with plot_width, plot_height, rooms (with walls)
        corridor_width: Width of corridors (default: 3)
        ga_config: Dict with population_size, num_generations, mutation_rate (optional)
        
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        import sys
        import os
        
        # Add Space_Optimization to path if needed
        ga_path = os.path.join(os.path.dirname(__file__), '..', 'Space_Optimization')
        if ga_path not in sys.path:
            sys.path.insert(0, ga_path)
        
        # Import GA functions
        from ga_current import parse_json_floorplan, run_ga, apply_chromosome, corridor_creator
        import ga_current
        
        # Apply GA configuration if provided
        max_workers = 1  # Default to single process
        if ga_config:
            if 'population_size' in ga_config:
                ga_current.POPULATION_SIZE = ga_config['population_size']
            if 'num_generations' in ga_config:
                ga_current.NUM_GENERATIONS = ga_config['num_generations']
            if 'mutation_rate' in ga_config:
                ga_current.INITIAL_MUTATION_RATE = ga_config['mutation_rate']
            if 'max_workers' in ga_config:
                max_workers = ga_config['max_workers']
        
        # Adjust TOURNAMENT_SIZE to be safe for small populations
        # Must be less than population size for tournament selection to work
        ga_current.TOURNAMENT_SIZE = min(8, max(2, ga_current.POPULATION_SIZE // 4))
        
        print(f"\n=== Starting GA Optimization ===")
        print(f"Corridor width: {corridor_width}")
        print(f"Population size: {ga_current.POPULATION_SIZE}")
        print(f"Generations: {ga_current.NUM_GENERATIONS}")
        print(f"Mutation rate: {ga_current.INITIAL_MUTATION_RATE}")
        print(f"Tournament size: {ga_current.TOURNAMENT_SIZE}")
        print(f"Max workers: {max_workers}")
        
        # Build complete walls list from all rooms
        # This ensures all walls referenced by rooms are in the master walls list
        all_walls_dict = {}
        
        # First add any walls from the master list
        for wall in floorplan_data.get('walls', []):
            all_walls_dict[wall['id']] = wall
        
        # Then add all walls from rooms (this will overwrite/add missing ones)
        for room in floorplan_data.get('rooms', []):
            for wall in room.get('walls', []):
                all_walls_dict[wall['id']] = wall
        
        # Update floorplan_data with complete walls list
        floorplan_data['walls'] = list(all_walls_dict.values())
        
        print(f"Built complete walls list: {len(floorplan_data['walls'])} walls")
        
        # Parse input floorplan
        rooms, initial_grid, region_matrix, room_names = parse_json_floorplan(floorplan_data)
        
        print(f"Parsed {len(rooms)} rooms from input")
        print(f"Grid size: {initial_grid.shape}")
        
        # Suppress matplotlib visualization when using API
        # Monkey-patch visualize_layout to do nothing
        original_visualize = ga_current.visualize_layout
        
        def null_visualize(*args, **kwargs):
            pass  # Do nothing - suppress visualization
        
        ga_current.visualize_layout = null_visualize
        
        try:
            # Run GA optimization
            print(f"\n--- Running Genetic Algorithm ---")
            print(f"Configuration: {ga_current.NUM_GENERATIONS} generations, "
                  f"{ga_current.POPULATION_SIZE} population size, "
                  f"{ga_current.INITIAL_MUTATION_RATE} mutation rate")
            print("This may take some time depending on configuration...")
            best_solution = run_ga(
                initial_grid, 
                region_matrix, 
                corridor_width=corridor_width,
                room_names=room_names,
                max_workers=max_workers
            )
        finally:
            # Restore original function
            ga_current.visualize_layout = original_visualize
        
        if not best_solution:
            ui.set_message("GA optimization failed to find a solution")
            return False
        
        print(f"\n--- GA completed successfully! Best solution: {best_solution} ---")
        
        # Apply chromosome to get displaced coordinates
        displaced_coords = apply_chromosome(rooms, best_solution)
        
        # Create layout matrix for corridors
        region_height, region_width = region_matrix.shape
        layout_matrix = [[0 for _ in range(region_width)] for _ in range(region_height)]
        
        # Mark unusable areas
        for r in range(region_height):
            for c in range(region_width):
                if region_matrix[r, c] == '#':
                    layout_matrix[r][c] = -1
        
        # Place rooms in layout matrix
        for room_id, coords in displaced_coords.items():
            for r, c in coords:
                if 0 <= r < region_height and 0 <= c < region_width:
                    if region_matrix[r, c] != '#':
                        layout_matrix[r][c] = room_id
        
        # Generate corridors
        layout_matrix = corridor_creator(layout_matrix, corridor_width)
        
        # Convert numpy array to plain Python list for JSON serialization
        if hasattr(layout_matrix, 'tolist'):
            layout_matrix = layout_matrix.tolist()
        
        # Build output data structure
        sorted_room_ids = sorted(rooms.keys())
        room_id_to_chromosome_idx = {room_id: idx for idx, room_id in enumerate(sorted_room_ids)}
        
        optimized_rooms = []
        for room_idx, coords in displaced_coords.items():
            if room_idx in rooms:
                room = rooms[room_idx]
                chromosome_idx = room_id_to_chromosome_idx[room_idx]
                dr, dc = best_solution[chromosome_idx]
                
                # Transform wall coordinates by applying displacement
                transformed_walls = []
                for wall_data in room.original_data["walls"]:
                    wall = wall_data.copy()
                    wall["x1"] += dc
                    wall["x2"] += dc
                    wall["y1"] += dr
                    wall["y2"] += dr
                    transformed_walls.append(wall)
                
                # Get optimized bounds
                min_r = min(r for r, c in coords)
                max_r = max(r for r, c in coords)
                min_c = min(c for r, c in coords)
                max_c = max(c for r, c in coords)
                
                optimized_rooms.append({
                    "id": room.original_data["id"],
                    "name": room.name,
                    "color": room.original_data["color"],
                    "walls": transformed_walls,
                    "optimized_bounds": {
                        "min_r": int(min_r),
                        "max_r": int(max_r),
                        "min_c": int(min_c),
                        "max_c": int(max_c)
                    }
                })
        
        # Create output matching temp2.json format
        # Convert best_solution to plain list (in case it has numpy types)
        ga_solution_list = []
        for item in best_solution:
            if isinstance(item, tuple):
                ga_solution_list.append((int(item[0]), int(item[1])))
            else:
                ga_solution_list.append(item)
        
        output_data = {
            "plot_width": int(floorplan_data["plot_width"]),
            "plot_height": int(floorplan_data["plot_height"]),
            "rooms": optimized_rooms,
            "walls": floorplan_data["walls"],
            "labels": floorplan_data.get("labels", []),
            "windows": floorplan_data.get("windows", []),
            "doors": floorplan_data.get("doors", []),
            "ga_solution": ga_solution_list,
            "layout_matrix": layout_matrix
        }
        
        # Store output in UI
        ui._set_output_data([output_data])
        ui.set_message(f"GA optimization successful. {len(optimized_rooms)} rooms optimized.")
        
        print(f"\n=== GA Optimization Complete ===")
        print(f"Optimized {len(optimized_rooms)} rooms")
        
        return True
        
    except Exception as e:
        ui.set_message(f"GA optimization error: {str(e)}")
        print(f"Error in handle_ga_optimization: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

def show_warning(str):
    messagebox.showinfo("Warning", str)
