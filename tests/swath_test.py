import pytest
import json
import csv
import os

from Utils.generate_path import Path_plan
from Utils.helper.geodesy import Geodesy


def farm_data():

    farm_file = "data/farms.json"

    with open(farm_file, "r") as f:
        farms = json.load(f)

    test_cases = []

    for farm in farms:

        field_name = farm["name"]
        boundary = farm["boundary"]

        gcp = []

        for n in range(len(boundary)):
            gcp.append([
                [boundary[n - 1][0], boundary[n - 1][1]],
                [boundary[n][0], boundary[n][1]]
            ])

        application_width = 2.2
        turning_radius = 4.4
        tractor_wheelbase = 2.2

        planner = Path_plan(
            gcp,
            "data/paths_test.json",
            application_width,
            turning_radius,
            tractor_wheelbase,
            field_name
        )

        tracks, _, _ = planner.path_planning(
            gcp,
            application_width,
            turning_radius,
            tractor_wheelbase
        )

        test_cases.append(
            (
                field_name,
                tracks,
                application_width
            )
        )

    return test_cases


@pytest.mark.parametrize(
    "field_name,tracks,expected_width",
    farm_data()
)
def test_swath_width(
    field_name,
    tracks,
    expected_width
):

    csv_file = "tests/swath_width/swath_width.csv"

    file_exists = os.path.exists(csv_file)

    with open(csv_file, "a", newline="") as file:

        writer = csv.writer(file)

        if not file_exists:
            writer.writerow([
                "Field",
                "Track Pair",
                "Measured Width",
                "Expected Width",
                "Error",
                "Result"
            ])

        for i in range(len(tracks) - 1):

            prev_track = tracks[i]
            next_track = tracks[i + 1]

            dist = abs(
                Geodesy.cross_track_distance(
                    next_track[0],
                    prev_track[0],
                    prev_track[1]
                )
            )

            error = abs(dist - expected_width)

            passed = error <= expected_width * 0.01

            writer.writerow([
                field_name,
                i + 1,
                round(dist, 3),
                expected_width,
                round(error, 3),
                "PASS" if passed else "FAIL"
            ])

            assert dist == pytest.approx(
                expected_width,
                rel=1e-2
            )

