"""B4 regression fixtures: hard ceilings, priorities and occupied polygons."""

import copy
from types import SimpleNamespace
from unittest.mock import patch

import pytest
from shapely.geometry import Polygon

from GPLAN import handlers
from GPLAN.source.dimensioning.minimum_dimensioning import upper_bound
from GPLAN.source.dimensioning.allocator import allocate
from GPLAN.source.dimensioning.room_constraints import validate, repair, constrain_plan


def node(i=0, **kwargs):
    return {"id": i, "label": "Bathroom" if i == 0 else "Bedroom", **kwargs}


def test_strict_upper_bound_is_exact_without_changing_legacy_slack():
    assert upper_bound(7, 5, strict=True) == 7
    assert upper_bound(7, 5) == 12.5
    assert upper_bound(7, 5, exact=True, strict=True) == 5


def test_strict_ladder_never_releases_ceiling_or_door_overlap():
    data = {"strict_room_bounds": True, "nodes": [{"min_width": 5, "max_width": 7}]}
    with patch.object(handlers.min_dim, "main", return_value=(False, None)) as solve:
        assert handlers.solve_min_dim(copy.deepcopy(data), 20, 20, True) == (False, None, False)
        assert solve.call_count == 1
        assert solve.call_args.args[0]["nodes"][0]["max_width"] == 7


def test_best_effort_retains_four_rungs_and_release_signal():
    data = {"nodes": [{"min_width": 5, "max_width": 7}]}
    with patch.object(handlers.min_dim, "main", side_effect=[(False, None)]*3+[(True, {})]) as solve:
        assert handlers.solve_min_dim(data, 20, 20, True)[2] is True
        assert solve.call_count == 4
        assert "max_width" not in solve.call_args.args[0]["nodes"][0]


def test_priorities_change_surplus_direction_without_breaking_caps():
    rooms = [{"name": "Bedroom", "priority": 0}, {"name": "Bathroom", "priority": 5}]
    bath_first = allocate(rooms, 16, 16)
    rooms[0]["priority"], rooms[1]["priority"] = 5, 0
    bed_first = allocate(rooms, 16, 16)
    assert bed_first["rooms"][0]["allocated_area"] > bath_first["rooms"][0]["allocated_area"]
    assert bed_first["rooms"][1]["allocated_area"] < bath_first["rooms"][1]["allocated_area"]
    for result in (bed_first, bath_first):
        assert all(r["min_area"] <= r["allocated_area"] <= r["max_area"] for r in result["rooms"])
        assert all(r["min_area"] - 0.02 <= r["target_width"] * r["target_height"] <= r["max_area"] + 0.02 for r in result["rooms"])


def test_scarce_area_does_not_buy_priority_by_breaking_floors():
    result = allocate([{"name": "Bedroom", "priority": 100}, {"name": "Bathroom", "priority": 0}], 8, 8)
    assert not result["fits"]
    assert result["shortfall_sqft"] > 0
    assert all(r["allocated_area"] == r["min_area"] for r in result["rooms"])


@pytest.mark.parametrize("priority", [-1, float("nan"), float("inf"), True, "high"])
def test_invalid_priorities_are_rejected(priority):
    with pytest.raises(ValueError, match="priority"):
        allocate([{"name": "Bedroom", "priority": priority}], 20, 20)


def test_area_maximum_is_hard_in_allocation():
    result = allocate([{"name": "Bedroom", "max_area": 160, "priority": 10}], 30, 30)
    assert result["rooms"][0]["allocated_area"] == 160
    assert result["rooms"][0]["target_width"] * result["rooms"][0]["target_height"] <= 160.0001
    with pytest.raises(ValueError, match="max_area"):
        allocate([{"name": "Bedroom", "max_area": 1}], 30, 30)


def test_generation_allocation_keeps_explicit_caps_and_priorities():
    params = SimpleNamespace(strict_room_bounds=False, use_requested_room_bounds=True,
                             room_nodes=[node(width={"min": 5, "max": 7}, max_area=50, priority=0)])
    ui = SimpleNamespace(min_dim_inputs=params, get_roomNames=lambda: ["Bathroom"])
    room = handlers._allocation_rooms(ui)[0]
    assert room["max_width"] == pytest.approx(6.6)
    assert room["max_area"] == 50
    assert room["priority"] == 0


L_BATH = [(0, 0), (12, 0), (12, 3), (3, 3), (3, 8), (0, 8)]


def test_l_room_area_is_occupied_area_not_bounding_box():
    assert Polygon(L_BATH).area == 51
    assert validate([L_BATH], [node(max_area=55)]) == []


def test_l_bathroom_can_be_trimmed_while_preserving_bedroom_door():
    bed = [(0, 8), (8, 8), (8, 16), (0, 16)]
    nodes = [node(width={"min": 3, "max": 8}, max_area=45), node(1)]
    edges = [[0, 1, "black"]]
    repaired, failures = repair([L_BATH, bed], nodes, edges)
    assert repaired is not None, failures
    assert not validate(repaired, nodes, edges)
    assert Polygon(repaired[0]).area <= 45.0001
    assert Polygon(repaired[0]).boundary.intersection(Polygon(repaired[1]).boundary).length >= 2.8
    assert Polygon(repaired[0]).intersection(Polygon(repaired[1])).area == 0


def test_impossible_repair_returns_bounds_instead_of_releasing_them():
    room = [(0, 0), (10, 0), (10, 10), (0, 10)]
    repaired, failures = repair([room], [node(width={"min": 8, "max": 7})])
    assert repaired is None
    assert any(f["bound"] == "max_width" for f in failures)


def test_strict_gate_never_mutates_source_plan():
    plan = SimpleNamespace(final_traversal=[L_BATH], area=[51], room_width=[12], room_height=[8], name_coords=[[1, 1]])
    kept, failures = constrain_plan(plan, [node(width={"min": 3, "max": 8})], [], True)
    assert kept is not None and not failures
    assert plan.final_traversal == [L_BATH]
    assert kept.room_width[0] <= 8.0001


def test_production_pipeline_strict_results_are_compliant_or_bounded_failure():
    from test_max_dimensions import build_request, TIGHT_CAPS
    from GPLAN.api import Documents
    request = build_request(TIGHT_CAPS)
    request["dim_inputs"]["strict_room_bounds"] = True
    request["dim_inputs"]["rotation_enabled"] = 0
    response, message = Documents.get_floorplans(**request)
    documents = response.to_dict()["Documents"]
    for plan in documents["floorPlans"]:
        polygons = [r["circular_coordinates"] for r in plan]
        assert not validate(polygons, request["nodes_list"], request["edges_list"])
    assert documents["room_constraints"]["mode"] == "strict"
    if not documents["floorPlans"]:
        assert documents["room_constraints"]["status"] == "infeasible"
        assert documents["room_constraints"]["requested_bounds"]


def test_production_pipeline_returns_valid_plans_for_a_feasible_strict_brief():
    from test_max_dimensions import build_request, LOOSE_CAPS
    from GPLAN.api import Documents
    request = build_request(LOOSE_CAPS)
    request["dim_inputs"]["strict_room_bounds"] = True
    request["dim_inputs"]["rotation_enabled"] = 0
    response, _ = Documents.get_floorplans(**request)
    documents = response.to_dict()["Documents"]
    assert documents["floorPlans"]
    assert documents["count"] == len(documents["floorPlans"])
    assert documents["room_constraints"]["status"] == "satisfied"
    for plan in documents["floorPlans"]:
        assert not validate([r["circular_coordinates"] for r in plan], request["nodes_list"], request["edges_list"])
