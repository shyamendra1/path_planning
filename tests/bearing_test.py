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


with open("data/farms.json") as f:
    FARMS = json.load(f)


@pytest.fixture(params=FARMS)
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

    return field_name, gcp

@pytest.mark.parametrize(
    "bearing",
    [0, 30, 60, 90, 120, 150]
)


# def find_best_direction(field, iterations=5):

#     tested = {}

#     step = 30

#     candidates = [0, 30, 60, 90, 120, 150]

#     for _ in range(iterations):

#         for angle in candidates:
#             if angle in tested:
#                 continue

#             tested[angle] = generate_length(field, angle)

#         best_three = sorted(
#             tested.items(),
#             key=lambda x: x[1],
#             reverse=True
#         )[:3]

#         step /= 2

#         new_candidates = set()

#         for angle, _ in best_three:
#             new_candidates.add(angle - step)
#             new_candidates.add(angle + step)

#         candidates = [
#             a for a in new_candidates
#             if 0 <= a <= 180
#         ]

#     best_angle, best_length = max(
#         tested.items(),
#         key=lambda x: x[1]
#     )

#     return best_angle, best_length, tested


def test_long_bearing(farm_data, bearing):

    field_name, gcp = farm_data
    field = {"field_name": field_name, "gcp": gcp}

    planner = Path_plan(
        gcp=field["gcp"],
        save_path="data/paths_test.json",
        Application_width=2.2,
        turning_radius=4.4,
        tractor_wheelbase=2.2,
        field_name=field["field_name"],
    )

    baseline_tracks, _ = planner.path_planning(
        gcp=field["gcp"],
        Application_width=2.2,
        turning_radius=4.4,
        tractor_wheelbase=2.2,
        bearing_override=None,
    )
    
    baseline_length = total_track_length(baseline_tracks)

    angle_tracks, _ = planner.path_planning(
        field["gcp"],
        2.2,
        4.4,
        2.2,
        bearing_override=bearing,
    )   

    test_length = total_track_length(angle_tracks)

    assert baseline_length <= test_length, (
        f"Long bearing length {baseline_length:.2f} "
        f"should be >= bearing {bearing} length {test_length:.2f}"
    )