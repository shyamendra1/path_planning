import pytest
from Utils.generate_path import Path_plan
import json
from Utils.helper.geodesy import Geodesy
from Utils.helper.turn_generation import GenerateTurn
from Utils.helper.generate_headland import GenerateHeadland
from math import atan2, radians, cos, sin, asin, sqrt , degrees, acos, pi, tan
import csv


#Driving direction tests on the generated paths for each farm in the farms.json file
        
def total_track_length(flat_track):
    return sum(
        Geodesy.distancebet(flat_track[i], flat_track[i + 1])
        for i in range(len(flat_track) - 1)
    )

SEARCH_ANGLES = [0, 30, 60, 90, 120, 150]


def run_path(field, bearing=None):

    planner = Path_plan(
        gcp=field["gcp"],
        save_path="data/paths_test.json",
        Application_width=2.2,
        turning_radius=4.4,
        tractor_wheelbase=2.2,
        field_name=field["field_name"],
    )

    flat_track, _, actual_long_bearing = planner.path_planning(
        gcp=field["gcp"],
        Application_width=2.2,
        turning_radius=4.4,
        tractor_wheelbase=2.2,
        bearing_override=bearing,
    )

    return total_track_length(flat_track), actual_long_bearing


def angle_diff(a, b):
    diff = abs(a - b)
    return min(diff, 180 - diff)


with open("tests/farms.json") as f:
    FARMS = json.load(f)


@pytest.fixture(
    params=FARMS,
    ids=lambda x: x["name"]
)
def farm_data(request):

    farm = request.param

    field_name = farm["name"]
    boundary = farm["boundary"]

    gcp = []

    for n in range(len(boundary)):
        gcp.append([
            [boundary[n - 1][0], boundary[n - 1][1]],
            [boundary[n][0], boundary[n][1]]
        ])

    return {
        "field_name": field_name,
        "gcp": gcp
    }

def angle_diff(a, b):
    return abs((a - b + 90) % 180 - 90)

def test_best_angle_vs_long_bearing(farm_data):

    # Cost for actual long bearing
    long_bearing_length, actual_long_bearing = run_path(farm_data, None)

    # Cost for fixed angles
    results = {}

    for angle in SEARCH_ANGLES:
        length,_ = run_path(farm_data, angle)
        results[angle] = length

    # Ignore invalid paths (0 m)
    valid_results = {
        angle: length
        for angle, length in results.items()
        if int(length) > 0
    }

    assert valid_results, (
        f"No valid path found for {farm_data['field_name']}"
    )

    # Best angle = shortest NON-ZERO path
    best_angle = min(valid_results, key=valid_results.get)
    best_length = valid_results[best_angle]

    diff = angle_diff(actual_long_bearing, best_angle)

    print(f"\nField: {farm_data['field_name']}")
    print(f"Long Bearing : {actual_long_bearing:.2f}°")
    print(f"Long Bearing Cost : {long_bearing_length:.2f} m")

    for angle, length in sorted(results.items()):
        status = "VALID" if length > 0 else "INVALID"
        print(f"{angle:3d}° -> {length:.2f} m [{status}]")

    print(f"Best Angle : {best_angle}°")
    print(f"Best Cost  : {best_length:.2f} m")
    print(f"Angular Difference : {diff:.2f}°")

    with open("tests/bearing_test_results2.csv", "a", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            farm_data['field_name'],
            actual_long_bearing,
            long_bearing_length,
            best_angle,
            best_length,
            diff
        ])
    # If you expect long bearing to be the best
    assert diff == 0, (
        f"{farm_data['field_name']} : "
        f"Best angle is {best_angle}°, not long bearing"
    )