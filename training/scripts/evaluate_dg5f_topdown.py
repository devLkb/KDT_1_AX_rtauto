#!/usr/bin/env python3
"""Validate matched 500-seed baseline/candidate DG5F top-down evaluations."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import sys
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path


DIRECTIONS = ("front", "right", "back", "left")
REQUIRED_COLUMNS = (
    "episode",
    "seed",
    "direction",
    "aligned_success",
    "distance_success",
    "hold_success",
    "final_palm_angle_degrees",
    "final_surface_clearance_meters",
    "hold_seconds",
    "hold_position_error_meters",
    "unsafe_surface_contact",
    "misaligned_descent",
    "premature_descent",
    "termination_reason",
)
EXPECTED_EPISODES = 500
EXPECTED_PER_DIRECTION = 125
EXPECTED_BASE_SEED = 700_000
MINIMUM_FOCUS_SUCCESS_RATE = 0.90
MINIMUM_OVERALL_SUCCESS_RATE = 0.90
MINIMUM_DIRECTION_SUCCESS_RATE = 0.85
MAXIMUM_REGRESSION = 0.05
MAXIMUM_PALM_ANGLE_DEGREES = 15.0
MAXIMUM_SURFACE_CLEARANCE_METERS = 0.03
MINIMUM_HOLD_SECONDS = 3.0
MAXIMUM_HOLD_POSITION_ERROR_METERS = 0.01
EPSILON = 1e-9


@dataclass(frozen=True)
class DirectionSummary:
    aligned_success_rate: float
    distance_success_rate: float
    hold_success_rate: float


@dataclass(frozen=True)
class EvaluationSummary:
    overall_aligned_success_rate: float
    directions: dict[str, DirectionSummary]


def _binary(row: dict[str, str], column: str, row_number: int) -> int:
    value = row[column]
    if value not in {"0", "1"}:
        raise ValueError(f"row {row_number}: {column} must be 0 or 1")
    return int(value)


def _integer(row: dict[str, str], column: str, row_number: int) -> int:
    try:
        return int(row[column])
    except ValueError as exc:
        raise ValueError(f"row {row_number}: {column} must be an integer") from exc


def _finite(row: dict[str, str], column: str, row_number: int) -> float:
    try:
        value = float(row[column])
    except ValueError as exc:
        raise ValueError(f"row {row_number}: {column} must be numeric") from exc
    if not math.isfinite(value):
        raise ValueError(f"row {row_number}: {column} must be finite")
    return value


def _read_ledger(
    path: Path,
    *,
    enforce_candidate_contract: bool,
) -> tuple[dict[tuple[int, int], str], EvaluationSummary]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if tuple(reader.fieldnames or ()) != REQUIRED_COLUMNS:
            raise ValueError(
                f"{path}: CSV columns must exactly match "
                + ",".join(REQUIRED_COLUMNS)
            )
        rows = list(reader)
    if len(rows) != EXPECTED_EPISODES:
        raise ValueError(
            f"{path}: expected {EXPECTED_EPISODES} episodes, found {len(rows)}"
        )

    episode_keys: dict[tuple[int, int], str] = {}
    counts = {
        direction: {
            "episodes": 0,
            "aligned_success": 0,
            "distance_success": 0,
            "hold_success": 0,
        }
        for direction in DIRECTIONS
    }
    total_aligned_success = 0

    for row_number, row in enumerate(rows, start=2):
        episode = _integer(row, "episode", row_number)
        seed = _integer(row, "seed", row_number)
        direction = row["direction"].strip()
        if episode not in range(EXPECTED_EPISODES):
            raise ValueError(f"row {row_number}: episode must be in 0..499")
        if seed != EXPECTED_BASE_SEED + episode:
            raise ValueError(
                f"row {row_number}: seed must equal {EXPECTED_BASE_SEED} + episode"
            )
        if direction not in DIRECTIONS:
            raise ValueError(f"row {row_number}: unknown direction {direction!r}")
        key = (episode, seed)
        if key in episode_keys:
            raise ValueError(f"row {row_number}: duplicate episode/seed {key}")
        episode_keys[key] = direction

        aligned = _binary(row, "aligned_success", row_number)
        distance = _binary(row, "distance_success", row_number)
        hold = _binary(row, "hold_success", row_number)
        unsafe = _binary(row, "unsafe_surface_contact", row_number)
        misaligned = _binary(row, "misaligned_descent", row_number)
        premature = _binary(row, "premature_descent", row_number)
        angle = _finite(row, "final_palm_angle_degrees", row_number)
        clearance = _finite(
            row, "final_surface_clearance_meters", row_number
        )
        hold_seconds = _finite(row, "hold_seconds", row_number)
        hold_error = _finite(
            row, "hold_position_error_meters", row_number
        )
        reason = row["termination_reason"].strip()

        if not 0.0 <= angle <= 180.0:
            raise ValueError(f"row {row_number}: palm angle must be in [0, 180]")
        if min(clearance, hold_seconds, hold_error) < 0.0:
            raise ValueError(
                f"row {row_number}: clearance, hold, and hold error must be non-negative"
            )
        if not reason:
            raise ValueError(f"row {row_number}: termination_reason is empty")
        if aligned and not (distance and hold):
            raise ValueError(
                f"row {row_number}: aligned success requires distance and hold success"
            )

        if enforce_candidate_contract:
            if unsafe or misaligned or premature:
                raise ValueError(
                    f"row {row_number}: candidate contains an unsafe or descent failure"
                )
            if aligned:
                checks = {
                    "final palm angle": angle
                    <= MAXIMUM_PALM_ANGLE_DEGREES + EPSILON,
                    "surface clearance": clearance
                    <= MAXIMUM_SURFACE_CLEARANCE_METERS + EPSILON,
                    "hold duration": hold_seconds + EPSILON
                    >= MINIMUM_HOLD_SECONDS,
                    "hold position error": hold_error
                    <= MAXIMUM_HOLD_POSITION_ERROR_METERS + EPSILON,
                    "termination reason": reason == "Success",
                }
                failed = [name for name, passed in checks.items() if not passed]
                if failed:
                    raise ValueError(
                        f"row {row_number}: successful candidate episode violates "
                        + ", ".join(failed)
                    )

        counts[direction]["episodes"] += 1
        counts[direction]["aligned_success"] += aligned
        counts[direction]["distance_success"] += distance
        counts[direction]["hold_success"] += hold
        total_aligned_success += aligned

    if set(episode_keys) != {
        (episode, EXPECTED_BASE_SEED + episode)
        for episode in range(EXPECTED_EPISODES)
    }:
        raise ValueError(f"{path}: episode/seed set is incomplete")
    for direction in DIRECTIONS:
        if counts[direction]["episodes"] != EXPECTED_PER_DIRECTION:
            raise ValueError(
                f"{path}: {direction} must contain exactly "
                f"{EXPECTED_PER_DIRECTION} episodes"
            )

    summaries = {
        direction: DirectionSummary(
            aligned_success_rate=counts[direction]["aligned_success"]
            / EXPECTED_PER_DIRECTION,
            distance_success_rate=counts[direction]["distance_success"]
            / EXPECTED_PER_DIRECTION,
            hold_success_rate=counts[direction]["hold_success"]
            / EXPECTED_PER_DIRECTION,
        )
        for direction in DIRECTIONS
    }
    return episode_keys, EvaluationSummary(
        overall_aligned_success_rate=total_aligned_success / EXPECTED_EPISODES,
        directions=summaries,
    )


def validate(
    baseline_csv: Path,
    candidate_csv: Path,
) -> tuple[EvaluationSummary, EvaluationSummary]:
    baseline_keys, baseline = _read_ledger(
        baseline_csv,
        enforce_candidate_contract=False,
    )
    candidate_keys, candidate = _read_ledger(
        candidate_csv,
        enforce_candidate_contract=True,
    )
    if baseline_keys != candidate_keys:
        raise ValueError(
            "baseline and candidate must use identical seed-to-direction assignments"
        )

    if (
        candidate.overall_aligned_success_rate + EPSILON
        < MINIMUM_OVERALL_SUCCESS_RATE
    ):
        raise ValueError("candidate overall aligned success rate is below 90%")
    for direction in DIRECTIONS:
        rate = candidate.directions[direction].aligned_success_rate
        if rate + EPSILON < MINIMUM_DIRECTION_SUCCESS_RATE:
            raise ValueError(
                f"candidate {direction} aligned success rate is below 85%"
            )
    for direction in ("front", "right"):
        rate = candidate.directions[direction].aligned_success_rate
        if rate + EPSILON < MINIMUM_FOCUS_SUCCESS_RATE:
            raise ValueError(
                f"candidate {direction} aligned success rate is below 90%"
            )
    for direction in ("left", "back"):
        for metric in ("distance_success_rate", "hold_success_rate"):
            original = getattr(baseline.directions[direction], metric)
            transferred = getattr(candidate.directions[direction], metric)
            if transferred + MAXIMUM_REGRESSION + EPSILON < original:
                raise ValueError(
                    f"candidate {direction} {metric} regressed by more than 5%p"
                )
    return baseline, candidate


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def freeze_first_passing_candidate(
    checkpoint: Path,
    candidate_csv: Path,
    approved_dir: Path,
    baseline: EvaluationSummary,
    candidate: EvaluationSummary,
) -> Path:
    if not checkpoint.is_file():
        raise FileNotFoundError(checkpoint)
    approved_manifest = approved_dir / "approval.json"
    if approved_manifest.exists():
        raise FileExistsError(
            f"an approved candidate is already frozen: {approved_manifest}"
        )

    approved_dir.mkdir(parents=True, exist_ok=True)
    destination = approved_dir / "checkpoint.pt"
    if destination.exists():
        raise FileExistsError(destination)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    try:
        shutil.copyfile(checkpoint, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)
    ledger_destination = approved_dir / "evaluation.csv"
    shutil.copyfile(candidate_csv, ledger_destination)

    manifest = {
        "schema_version": 1,
        "approved_utc": datetime.now(timezone.utc).isoformat(),
        "selection_policy": "first_checkpoint_passing_matched_500_seed_gate",
        "source_checkpoint": str(checkpoint.resolve()),
        "approved_checkpoint": str(destination.resolve()),
        "checkpoint_sha256": _sha256(destination),
        "baseline": asdict(baseline),
        "candidate": asdict(candidate),
    }
    temporary_manifest = approved_manifest.with_name(
        f".{approved_manifest.name}.{os.getpid()}.tmp"
    )
    temporary_manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_manifest, approved_manifest)
    return approved_manifest


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_csv", type=Path)
    parser.add_argument("candidate_csv", type=Path)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--approved-dir", type=Path)
    args = parser.parse_args()
    if (args.checkpoint is None) != (args.approved_dir is None):
        parser.error("--checkpoint and --approved-dir must be provided together")

    try:
        baseline, candidate = validate(args.baseline_csv, args.candidate_csv)
        if args.checkpoint is not None:
            manifest = freeze_first_passing_candidate(
                args.checkpoint,
                args.candidate_csv,
                args.approved_dir,
                baseline,
                candidate,
            )
            print(f"[PASS] first passing candidate frozen: {manifest}")
        print(
            "[PASS] matched 500-seed top-down gate: "
            f"overall={candidate.overall_aligned_success_rate:.1%}, "
            + ", ".join(
                f"{direction}="
                f"{candidate.directions[direction].aligned_success_rate:.1%}"
                for direction in DIRECTIONS
            )
        )
    except (OSError, ValueError) as exc:
        print(f"[FAIL] DG5F top-down evaluation: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
