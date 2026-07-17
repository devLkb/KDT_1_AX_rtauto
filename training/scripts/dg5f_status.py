#!/usr/bin/env python3
"""Print the latest ML-Agents scalar values for a DG5F run."""

from __future__ import annotations

import argparse
from pathlib import Path

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


DISPLAY_TAGS = (
    "Environment/Cumulative Reward",
    "Environment/Episode Length",
    "Grasp/Success",
    "Grasp/MaxContactHoldSeconds",
    "Grasp/ThumbContactReached",
    "Grasp/OpposingContactReached",
    "Grasp/DualContactReached",
    "Reach/Success",
    "Reach/FirstSuccessSeconds",
    "Reach/FinalDistanceMeters",
    "Reach/BestDistanceMeters",
    "Failure/Timeout",
    "Failure/BallOutOfBounds",
    "Failure/NonFinitePhysics",
    "Policy/Entropy",
    "Policy/Extrinsic Value Estimate",
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()

    event_files = sorted(args.run_dir.glob("*/events.out.tfevents.*"))
    if not event_files:
        print(f"metrics: 아직 event 파일이 없습니다 ({args.run_dir})")
        return 0

    latest: dict[str, object] = {}
    for event_file in event_files:
        accumulator = EventAccumulator(
            str(event_file), size_guidance={"scalars": 0}
        )
        try:
            accumulator.Reload()
        except Exception:
            continue
        for tag in accumulator.Tags().get("scalars", []):
            values = accumulator.Scalars(tag)
            if values and (
                tag not in latest or values[-1].wall_time > latest[tag].wall_time
            ):
                latest[tag] = values[-1]

    if not latest:
        print("metrics: 아직 기록된 scalar가 없습니다")
        return 0

    max_step = max(value.step for value in latest.values())
    print(f"latest metric step: {max_step:,}")
    for tag in DISPLAY_TAGS:
        if tag in latest:
            value = latest[tag]
            print(f"  {tag:<38} step={value.step:>8,}  value={value.value:.6g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
