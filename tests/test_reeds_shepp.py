import pytest
import math
from reeds_shepp import (
    PathElement, 
    Steering, 
    Gear, 
    ReedsSheppCurve,
    ReedsSheppCurveWaypoint,
    ReedsSheppCurvePath
)
from reeds_shepp.core import path_length, timeflip, reflect, get_optimal_path

def test_path_element_creation():
    element = PathElement.create(13, Steering.LEFT, Gear.FORWARD)
    assert repr(element) == "{ Steering: LEFT\tGear: FORWARD\tdistance: 13 }"

def test_reverse_gear():
    element = PathElement.create(13, Steering.LEFT, Gear.FORWARD)
    assert element.reverse_gear().gear == Gear.BACKWARD

def test_reverse_steering():
    element = PathElement.create(13, Steering.LEFT, Gear.FORWARD)
    assert element.reverse_steering().steering == Steering.RIGHT

def test_negative_parameter():
    element = PathElement.create(-1, Steering.LEFT, Gear.FORWARD)
    assert element == PathElement.create(1, Steering.LEFT, Gear.BACKWARD)

def test_path_length():
    path = [PathElement.create(1, Steering.LEFT, Gear.FORWARD) for _ in range(2)]
    assert path_length(path) == 2

def test_timeflip():
    path = [PathElement.create(1, Steering.LEFT, g) for g in (Gear.FORWARD, Gear.BACKWARD)]
    timeflipped = timeflip(path)
    assert timeflipped[0].gear == Gear.BACKWARD
    assert timeflipped[1].gear == Gear.FORWARD
    assert path[0].gear == Gear.FORWARD  # Immutability check

def test_reflect():
    path = [PathElement.create(1, s, Gear.FORWARD) for s in (Steering.LEFT, Steering.STRAIGHT, Steering.RIGHT)]
    reflected = reflect(path)
    assert reflected[0].steering == Steering.RIGHT
    assert reflected[1].steering == Steering.STRAIGHT
    assert reflected[2].steering == Steering.LEFT
    assert path[0].steering == Steering.LEFT  # Immutability check

def test_get_optimal_path():
    path = get_optimal_path((0, 0, 0), (1, 0, 0))
    assert path == [PathElement.create(1.0, Steering.STRAIGHT, Gear.FORWARD)]

def test_planner_interface():
    planner = ReedsSheppCurve()
    waypoints = [
        ReedsSheppCurveWaypoint(0, 0, 0),
        ReedsSheppCurveWaypoint(1, 0, 0)
    ]
    paths = planner.plan(waypoints)
    
    assert len(paths) > 0
    assert isinstance(paths[0], ReedsSheppCurvePath)
    # The shortest path for (0,0,0) to (1,0,0) should be STRAIGHT FORWARD
    assert paths[0].elements == [PathElement.create(1.0, Steering.STRAIGHT, Gear.FORWARD)]
    assert math.isclose(paths[0].total_length, 1.0)
    assert paths[0].waypoints == waypoints

def test_planner_sorting():
    planner = ReedsSheppCurve()
    waypoints = [
        ReedsSheppCurveWaypoint(0, 0, 0),
        ReedsSheppCurveWaypoint(2, 2, 90)
    ]
    paths = planner.plan(waypoints)
    
    assert len(paths) > 1
    # Check if they are sorted by length
    for i in range(len(paths) - 1):
        assert paths[i].total_length <= paths[i+1].total_length

def test_planner_multiple_waypoints():
    planner = ReedsSheppCurve()
    waypoints = [
        ReedsSheppCurveWaypoint(0, 0, 0),
        ReedsSheppCurveWaypoint(1, 0, 0),
        ReedsSheppCurveWaypoint(2, 0, 0)
    ]
    paths = planner.plan(waypoints)
    
    # Each path in 'paths' should be a full path from W0 to W2
    assert len(paths) > 0
    assert math.isclose(paths[0].total_length, 2.0)
    assert len(paths[0].elements) >= 2
    assert paths[0].waypoints == waypoints