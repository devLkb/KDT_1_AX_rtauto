#!/usr/bin/env python3
"""Validate the deterministic DG5F v2 evaluation CSV ledger."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import Counter
from pathlib import Path


REQUIRED_COLUMNS = {
    "episode_id",
    "seed",
    "area",
    "success",
    "failure_reason",
    "completion_seconds",
    "reach_success",
    "first_reach_seconds",
    "final_distance_meters",
    "best_distance_meters",
    "max_contact_hold_seconds",
}


def parse_binary(row: dict[str, str], column: str, row_number: int) -> int:
    value = row[column]
    if value not in {"0", "1"}:
        raise ValueError(f"row {row_number}: {column} must be 0 or 1, got {value!r}")
    return int(value)


def parse_finite(row: dict[str, str], column: str, row_number: int) -> float:
    try:
        value = float(row[column])
    except ValueError as exc:
        raise ValueError(
            f"row {row_number}: {column} is not a number: {row[column]!r}"
        ) from exc
    if not math.isfinite(value):
        raise ValueError(f"row {row_number}: {column} is not finite")
    return value


def validate(
    csv_path: Path,
    expected_episodes: int,
    area_count: int,
    required_hold_seconds: float,
    minimum_success_rate: float,
    minimum_reach_rate: float,
) -> tuple[float, float]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        columns = set(reader.fieldnames or ())
        missing = REQUIRED_COLUMNS - columns
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(sorted(missing))}")
        rows = list(reader)

    if len(rows) != expected_episodes:
        raise ValueError(
            f"expected {expected_episodes} rows, found {len(rows)}"
        )

    episode_ids: set[int] = set()
    seeds: set[int] = set()
    area_counts: Counter[int] = Counter()
    successes = 0
    reaches = 0
    non_finite_physics_failures = 0

    for row_number, row in enumerate(rows, start=2):
        try:
            episode_id = int(row["episode_id"])
            seed = int(row["seed"])
            area = int(row["area"])
        except ValueError as exc:
            raise ValueError(
                f"row {row_number}: episode_id, seed, and area must be integers"
            ) from exc
        if episode_id in episode_ids:
            raise ValueError(f"row {row_number}: duplicate episode_id {episode_id}")
        if seed in seeds:
            raise ValueError(f"row {row_number}: duplicate seed {seed}")
        if not 0 <= area < area_count:
            raise ValueError(f"row {row_number}: area {area} outside 0..{area_count - 1}")
        episode_ids.add(episode_id)
        seeds.add(seed)
        area_counts[area] += 1

        success = parse_binary(row, "success", row_number)
        reach_success = parse_binary(row, "reach_success", row_number)
        completion_seconds = parse_finite(row, "completion_seconds", row_number)
        first_reach_seconds = parse_finite(row, "first_reach_seconds", row_number)
        final_distance = parse_finite(row, "final_distance_meters", row_number)
        best_distance = parse_finite(row, "best_distance_meters", row_number)
        max_hold = parse_finite(row, "max_contact_hold_seconds", row_number)

        if completion_seconds < 0 or final_distance < 0 or best_distance < 0 or max_hold < 0:
            raise ValueError(f"row {row_number}: time and distance values must be non-negative")
        if reach_success and first_reach_seconds < 0:
            raise ValueError(
                f"row {row_number}: reached episode has negative first_reach_seconds"
            )
        if success:
            successes += 1
            if max_hold < required_hold_seconds:
                raise ValueError(
                    f"row {row_number}: successful episode held contact for "
                    f"{max_hold:.9g}s, below {required_hold_seconds:.9g}s"
                )
            if row["failure_reason"] != "None":
                raise ValueError(
                    f"row {row_number}: successful episode has failure reason "
                    f"{row['failure_reason']!r}"
                )
        elif not row["failure_reason"] or row["failure_reason"] == "None":
            raise ValueError(f"row {row_number}: failed episode has no failure reason")

        reaches += reach_success
        non_finite_physics_failures += row["failure_reason"] == "NonFinitePhysics"

    expected_ids = set(range(expected_episodes))
    if episode_ids != expected_ids:
        raise ValueError("episode_id values must be exactly 0..expected_episodes-1")
    for area in range(area_count):
        expected_for_area = (
            (expected_episodes - 1 - area) // area_count + 1
            if area < expected_episodes
            else 0
        )
        if area_counts[area] != expected_for_area:
            raise ValueError(
                f"area {area} has {area_counts[area]} rows, expected {expected_for_area}"
            )
    if non_finite_physics_failures:
        raise ValueError(
            f"NonFinitePhysics occurred in {non_finite_physics_failures} episodes"
        )

    success_rate = successes / expected_episodes
    reach_rate = reaches / expected_episodes
    if success_rate < minimum_success_rate:
        raise ValueError(
            f"grasp success rate {success_rate:.1%} is below "
            f"{minimum_success_rate:.1%}"
        )
    if reach_rate < minimum_reach_rate:
        raise ValueError(
            f"reach success rate {reach_rate:.1%} is below {minimum_reach_rate:.1%}"
        )
    return success_rate, reach_rate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", type=Path)
    parser.add_argument("--expected-episodes", type=int, default=200)
    parser.add_argument("--area-count", type=int, default=20)
    parser.add_argument("--required-hold-seconds", type=float, default=0.5)
    parser.add_argument("--minimum-success-rate", type=float, default=0.8)
    parser.add_argument("--minimum-reach-rate", type=float, default=0.8)
    args = parser.parse_args()
    try:
        success_rate, reach_rate = validate(
            args.csv_path,
            args.expected_episodes,
            args.area_count,
            args.required_hold_seconds,
            args.minimum_success_rate,
            args.minimum_reach_rate,
        )
    except (OSError, ValueError) as exc:
        print(f"[FAIL] DG5F v2 evaluation: {exc}", file=sys.stderr)
        return 1
    print(
        f"[PASS] {args.expected_episodes} unique episodes; "
        f"Grasp/Success={success_rate:.1%}; Reach/Success={reach_rate:.1%}; "
        f"successful holds >= {args.required_hold_seconds:g}s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
