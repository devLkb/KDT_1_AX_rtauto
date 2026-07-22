#!/usr/bin/env python3
"""Validate the deterministic DG5F GraspPoint reach evaluation ledger."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import NamedTuple


REQUIRED_COLUMNS = (
    "episode",
    "seed",
    "success",
    "final_distance_meters",
    "grasp_point_speed_mps",
    "palm_alignment",
    "upper_cone_alignment",
    "success_hold_seconds",
    "elapsed_seconds",
    "minimum_transit_clearance_meters",
    "unsafe_surface_contact",
    "premature_descent",
    "workspace_safe",
    "finite_physics",
    "termination_reason",
)
EXPECTED_EPISODES = 500
EXPECTED_BASE_SEED = 500_000
MINIMUM_SUCCESS_RATE = 0.90
MAXIMUM_DISTANCE_METERS = 0.01
MAXIMUM_SPEED_MPS = 0.05
MINIMUM_HOLD_SECONDS = 0.25
MINIMUM_PALM_ALIGNMENT = 0.965925826
MINIMUM_UPPER_CONE_ALIGNMENT = 0.707106781
MINIMUM_TRANSIT_CLEARANCE_METERS = 0.10
MAXIMUM_EPISODE_SECONDS = 20.0
EPSILON = 1e-9


class EvaluationSummary(NamedTuple):
    success_rate: float
    mean_final_distance_meters: float
    median_success_seconds: float
    p95_success_seconds: float


def _parse_binary(row: dict[str, str], column: str, row_number: int) -> int:
    value = row[column]
    if value not in {"0", "1"}:
        raise ValueError(f"row {row_number}: {column} must be 0 or 1, got {value!r}")
    return int(value)


def _parse_int(row: dict[str, str], column: str, row_number: int) -> int:
    try:
        return int(row[column])
    except ValueError as exc:
        raise ValueError(f"row {row_number}: {column} must be an integer") from exc


def _parse_finite(row: dict[str, str], column: str, row_number: int) -> float:
    try:
        value = float(row[column])
    except ValueError as exc:
        raise ValueError(
            f"row {row_number}: {column} is not a number: {row[column]!r}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(f"row {row_number}: {column} is not finite")
    return value


def _percentile_95(values: list[float]) -> float:
    ordered = sorted(values)
    index = math.ceil(0.95 * len(ordered)) - 1
    return ordered[max(index, 0)]


def validate(
    csv_path: Path,
    expected_episodes: int = EXPECTED_EPISODES,
    expected_base_seed: int = EXPECTED_BASE_SEED,
    minimum_success_rate: float = MINIMUM_SUCCESS_RATE,
) -> EvaluationSummary:
    if expected_episodes <= 0:
        raise ValueError("expected_episodes must be positive")
    if not 0.0 <= minimum_success_rate <= 1.0:
        raise ValueError("minimum_success_rate must be between 0 and 1")

    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        fields = tuple(reader.fieldnames or ())
        if fields != REQUIRED_COLUMNS:
            raise ValueError(
                "CSV columns must exactly match: " + ",".join(REQUIRED_COLUMNS)
            )
        rows = list(reader)

    if len(rows) != expected_episodes:
        raise ValueError(f"expected {expected_episodes} rows, found {len(rows)}")

    episodes: set[int] = set()
    seeds: set[int] = set()
    successes = 0
    final_distances: list[float] = []
    success_seconds: list[float] = []

    for row_number, row in enumerate(rows, start=2):
        episode = _parse_int(row, "episode", row_number)
        seed = _parse_int(row, "seed", row_number)
        if episode in episodes:
            raise ValueError(f"row {row_number}: duplicate episode {episode}")
        if seed in seeds:
            raise ValueError(f"row {row_number}: duplicate seed {seed}")
        if seed != expected_base_seed + episode:
            raise ValueError(
                f"row {row_number}: seed must equal base seed + episode"
            )
        episodes.add(episode)
        seeds.add(seed)

        success = _parse_binary(row, "success", row_number)
        workspace_safe = _parse_binary(row, "workspace_safe", row_number)
        finite_physics = _parse_binary(row, "finite_physics", row_number)
        unsafe_contact = _parse_binary(
            row, "unsafe_surface_contact", row_number
        )
        premature_descent = _parse_binary(row, "premature_descent", row_number)
        distance = _parse_finite(row, "final_distance_meters", row_number)
        speed = _parse_finite(row, "grasp_point_speed_mps", row_number)
        palm_alignment = _parse_finite(row, "palm_alignment", row_number)
        upper_cone_alignment = _parse_finite(
            row, "upper_cone_alignment", row_number
        )
        hold = _parse_finite(row, "success_hold_seconds", row_number)
        elapsed = _parse_finite(row, "elapsed_seconds", row_number)
        minimum_clearance = _parse_finite(
            row, "minimum_transit_clearance_meters", row_number
        )
        reason = row["termination_reason"].strip()

        if min(distance, speed, hold, elapsed) < 0:
            raise ValueError(
                f"row {row_number}: distance, speed, hold, and elapsed must be non-negative"
            )
        if elapsed > MAXIMUM_EPISODE_SECONDS + EPSILON:
            raise ValueError(
                f"row {row_number}: elapsed_seconds exceeds "
                f"{MAXIMUM_EPISODE_SECONDS:g}"
            )
        if hold > elapsed + EPSILON:
            raise ValueError(
                f"row {row_number}: success_hold_seconds exceeds elapsed_seconds"
            )
        if not workspace_safe:
            raise ValueError(f"row {row_number}: workspace safety failure")
        if not finite_physics:
            raise ValueError(f"row {row_number}: non-finite physics")
        if unsafe_contact:
            raise ValueError(f"row {row_number}: unsafe surface contact")
        if premature_descent:
            raise ValueError(f"row {row_number}: premature descent")
        if minimum_clearance + EPSILON < MINIMUM_TRANSIT_CLEARANCE_METERS:
            raise ValueError(
                f"row {row_number}: minimum transit clearance below "
                f"{MINIMUM_TRANSIT_CLEARANCE_METERS:g}m"
            )
        if not reason:
            raise ValueError(f"row {row_number}: termination_reason is empty")

        if success:
            successes += 1
            checks = {
                f"final_distance_meters <= {MAXIMUM_DISTANCE_METERS:g}":
                    distance <= MAXIMUM_DISTANCE_METERS + EPSILON,
                f"grasp_point_speed_mps <= {MAXIMUM_SPEED_MPS:g}":
                    speed <= MAXIMUM_SPEED_MPS + EPSILON,
                f"success_hold_seconds >= {MINIMUM_HOLD_SECONDS:g}":
                    hold + EPSILON >= MINIMUM_HOLD_SECONDS,
                f"palm_alignment >= {MINIMUM_PALM_ALIGNMENT:g}":
                    palm_alignment + EPSILON >= MINIMUM_PALM_ALIGNMENT,
                f"upper_cone_alignment >= {MINIMUM_UPPER_CONE_ALIGNMENT:g}":
                    upper_cone_alignment + EPSILON
                    >= MINIMUM_UPPER_CONE_ALIGNMENT,
                "termination_reason == Success": reason == "Success",
            }
            failed = [name for name, passed in checks.items() if not passed]
            if failed:
                raise ValueError(
                    f"row {row_number}: successful episode violates "
                    + ", ".join(failed)
                )
            success_seconds.append(elapsed)
        elif reason != "Timeout":
            raise ValueError(
                f"row {row_number}: safe finite failed episode must terminate by Timeout"
            )

        final_distances.append(distance)

    expected_episode_values = set(range(expected_episodes))
    if episodes != expected_episode_values:
        raise ValueError("episode values must be exactly 0..expected_episodes-1")
    expected_seeds = set(
        range(expected_base_seed, expected_base_seed + expected_episodes)
    )
    if seeds != expected_seeds:
        raise ValueError(
            "seed values must be the exact sequential held-out evaluation range"
        )

    success_rate = successes / expected_episodes
    if success_rate + EPSILON < minimum_success_rate:
        raise ValueError(
            f"success rate {success_rate:.1%} is below {minimum_success_rate:.1%}"
        )

    return EvaluationSummary(
        success_rate=success_rate,
        mean_final_distance_meters=statistics.fmean(final_distances),
        median_success_seconds=statistics.median(success_seconds),
        p95_success_seconds=_percentile_95(success_seconds),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--expected-episodes", type=int, default=EXPECTED_EPISODES)
    parser.add_argument("--expected-base-seed", type=int, default=EXPECTED_BASE_SEED)
    parser.add_argument(
        "--minimum-success-rate", type=float, default=MINIMUM_SUCCESS_RATE
    )
    args = parser.parse_args()
    try:
        summary = validate(
            args.csv_path,
            args.expected_episodes,
            args.expected_base_seed,
            args.minimum_success_rate,
        )
    except (OSError, ValueError) as exc:
        print(f"[FAIL] DG5F GraspPoint reach evaluation: {exc}", file=sys.stderr)
        return 1
    print(
        f"[PASS] {args.expected_episodes} unique held-out seeds; "
        f"success={summary.success_rate:.1%}; "
        f"mean_final_error={summary.mean_final_distance_meters:.6f}m; "
        f"median_time={summary.median_success_seconds:.3f}s; "
        f"p95_time={summary.p95_success_seconds:.3f}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
