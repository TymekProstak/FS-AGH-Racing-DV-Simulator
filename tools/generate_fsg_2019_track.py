#!/usr/bin/env python3
"""Generate the FSG 2019 centerline and constant-width cone boundaries."""

import argparse
import csv
import math
from pathlib import Path
from typing import List, Sequence, Tuple


Point = Tuple[float, float]

TRACK_WIDTH_M = 4.0
CONE_SPACING_M = 2.0
CENTERLINE_SPACING_M = 1.0
DENSE_SPACING_M = 0.1
CHAIKIN_ITERATIONS = 4
START_POINT = (0.0, 0.0)

# Centerline anchors digitized from the supplied FSG 2019 reference drawing.
# The driving direction starts on the upper straight and proceeds towards +X.
DIGITIZED_CENTERLINE_ANCHORS: Sequence[Point] = (
    (0.0, 0.0),
    (6.0, 0.0),
    (12.0, 0.0),
    (18.0, 0.0),
    (24.0, 0.0),
    (30.0, 0.0),
    (36.0, 0.0),
    (40.5, -2.0),
    (44.0, -6.0),
    (45.0, -11.5),
    (44.0, -16.5),
    (41.0, -20.5),
    (35.5, -23.5),
    (29.0, -26.0),
    (23.0, -27.0),
    (18.5, -25.5),
    (13.0, -22.0),
    (8.0, -18.5),
    (4.0, -16.0),
    (0.0, -16.0),
    (-3.5, -18.5),
    (-5.5, -23.0),
    (-5.0, -28.5),
    (-6.5, -34.5),
    (-9.0, -39.5),
    (-9.0, -43.0),
    (-6.5, -47.0),
    (-2.5, -50.0),
    (3.0, -51.5),
    (9.0, -52.0),
    (14.5, -51.0),
    (19.5, -53.0),
    (23.0, -57.0),
    (26.5, -63.0),
    (28.5, -68.0),
    (27.5, -71.0),
    (24.0, -73.0),
    (18.0, -73.0),
    (11.0, -71.0),
    (4.0, -68.0),
    (-3.0, -65.0),
    (-10.0, -60.0),
    (-16.0, -53.5),
    (-21.0, -47.0),
    (-24.0, -40.5),
    (-24.5, -33.0),
    (-23.5, -25.0),
    (-21.0, -17.0),
    (-17.0, -10.0),
    (-12.0, -5.5),
    (-6.0, -2.0),
)


def distance(a: Point, b: Point) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def smooth_closed(points: Sequence[Point], iterations: int) -> List[Point]:
    """Round a closed anchor polygon without spline overshoot."""
    smoothed = list(points)

    for _ in range(iterations):
        refined: List[Point] = []

        for index, current in enumerate(smoothed):
            following = smoothed[(index + 1) % len(smoothed)]
            refined.append(
                (
                    0.75 * current[0] + 0.25 * following[0],
                    0.75 * current[1] + 0.25 * following[1],
                )
            )
            refined.append(
                (
                    0.25 * current[0] + 0.75 * following[0],
                    0.25 * current[1] + 0.75 * following[1],
                )
            )

        smoothed = refined

    start_index = min(
        range(len(smoothed)),
        key=lambda index: distance(smoothed[index], START_POINT),
    )
    return smoothed[start_index:] + smoothed[:start_index]


def resample_closed(
    points: Sequence[Point],
    requested_spacing_m: float,
    duplicate_first_point: bool = False,
) -> Tuple[List[Point], float, float]:
    """Resample a closed polyline with one uniform spacing around the loop."""
    segment_lengths = [
        distance(point, points[(index + 1) % len(points)])
        for index, point in enumerate(points)
    ]

    cumulative_lengths = [0.0]
    for segment_length in segment_lengths:
        cumulative_lengths.append(cumulative_lengths[-1] + segment_length)

    total_length_m = cumulative_lengths[-1]
    sample_count = max(3, round(total_length_m / requested_spacing_m))
    actual_spacing_m = total_length_m / sample_count

    samples: List[Point] = []
    segment_index = 0

    for sample_index in range(sample_count):
        target_length = sample_index * actual_spacing_m

        while cumulative_lengths[segment_index + 1] < target_length:
            segment_index += 1

        start = points[segment_index]
        end = points[(segment_index + 1) % len(points)]
        segment_start = cumulative_lengths[segment_index]
        fraction = (
            (target_length - segment_start)
            / segment_lengths[segment_index]
        )

        samples.append(
            (
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
            )
        )

    if duplicate_first_point:
        samples.append(samples[0])

    return samples, total_length_m, actual_spacing_m


def offset_boundaries(
    centerline: Sequence[Point],
    half_width_m: float,
) -> Tuple[List[Point], List[Point]]:
    """Create left and right boundaries from centered tangent normals."""
    left: List[Point] = []
    right: List[Point] = []

    for index, point in enumerate(centerline):
        previous = centerline[(index - 2) % len(centerline)]
        following = centerline[(index + 2) % len(centerline)]

        tangent_x = following[0] - previous[0]
        tangent_y = following[1] - previous[1]
        tangent_length = math.hypot(tangent_x, tangent_y)

        normal_x = -tangent_y / tangent_length
        normal_y = tangent_x / tangent_length

        left.append(
            (
                point[0] + half_width_m * normal_x,
                point[1] + half_width_m * normal_y,
            )
        )
        right.append(
            (
                point[0] - half_width_m * normal_x,
                point[1] - half_width_m * normal_y,
            )
        )

    return left, right


def consecutive_spacing_range(points: Sequence[Point]) -> Tuple[float, float]:
    spacings = [
        distance(point, points[(index + 1) % len(points)])
        for index, point in enumerate(points)
    ]
    return min(spacings), max(spacings)


def write_centerline(path: Path, points: Sequence[Point]) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(("x", "y"))
        writer.writerows(
            (f"{point[0]:.3f}", f"{point[1]:.3f}")
            for point in points
        )


def write_cones(
    path: Path,
    blue_cones: Sequence[Point],
    yellow_cones: Sequence[Point],
) -> None:
    with path.open("w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output, lineterminator="\n")
        writer.writerow(("x", "y", "color"))

        for point in blue_cones:
            writer.writerow((f"{point[0]:.3f}", f"{point[1]:.3f}", "blue"))

        for point in yellow_cones:
            writer.writerow((f"{point[0]:.3f}", f"{point[1]:.3f}", "yellow"))


def generate(output_directory: Path) -> None:
    output_directory.mkdir(parents=True, exist_ok=True)

    smooth_curve = smooth_closed(
        DIGITIZED_CENTERLINE_ANCHORS,
        CHAIKIN_ITERATIONS,
    )
    dense_centerline, total_length_m, _ = resample_closed(
        smooth_curve,
        DENSE_SPACING_M,
    )
    centerline, _, centerline_spacing_m = resample_closed(
        smooth_curve,
        CENTERLINE_SPACING_M,
        duplicate_first_point=True,
    )

    dense_blue_boundary, dense_yellow_boundary = offset_boundaries(
        dense_centerline,
        TRACK_WIDTH_M / 2.0,
    )
    blue_cones, _, blue_spacing_m = resample_closed(
        dense_blue_boundary,
        CONE_SPACING_M,
    )
    yellow_cones, _, yellow_spacing_m = resample_closed(
        dense_yellow_boundary,
        CONE_SPACING_M,
    )

    blue_min_spacing, blue_max_spacing = consecutive_spacing_range(blue_cones)
    yellow_min_spacing, yellow_max_spacing = consecutive_spacing_range(
        yellow_cones
    )

    if min(blue_min_spacing, yellow_min_spacing) < 1.8:
        raise RuntimeError("Generated cones are too close together.")

    write_centerline(
        output_directory / "fsg_2019_centerline.csv",
        centerline,
    )
    write_cones(
        output_directory / "fsg_2019_cones.csv",
        blue_cones,
        yellow_cones,
    )

    print(f"centerline length: {total_length_m:.3f} m")
    print(
        "centerline: "
        f"{len(centerline) - 1} unique points, "
        f"{centerline_spacing_m:.3f} m spacing"
    )
    print(
        "blue cones: "
        f"{len(blue_cones)}, nominal spacing {blue_spacing_m:.3f} m, "
        f"chord range {blue_min_spacing:.3f}-{blue_max_spacing:.3f} m"
    )
    print(
        "yellow cones: "
        f"{len(yellow_cones)}, nominal spacing {yellow_spacing_m:.3f} m, "
        f"chord range {yellow_min_spacing:.3f}-{yellow_max_spacing:.3f} m"
    )
    print(f"track width: {TRACK_WIDTH_M:.3f} m")


def main() -> None:
    repository_root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-directory",
        type=Path,
        default=repository_root / "tracks",
        help="directory receiving fsg_2019_centerline.csv and fsg_2019_cones.csv",
    )
    arguments = parser.parse_args()
    generate(arguments.output_directory)


if __name__ == "__main__":
    main()
