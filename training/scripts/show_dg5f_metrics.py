#!/usr/bin/env python3
"""Print the live DG5F V1 training metrics without a web browser."""

from __future__ import annotations

import argparse
import datetime as dt
import re
from pathlib import Path

import numpy as np
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


METRICS = (
    ("Success", "Reach/Success", 100.0, "%"),
    ("Completion", "Reach/CompletionSeconds", 1.0, "s"),
    ("Final distance", "Reach/FinalDistanceMeters", 100.0, "cm"),
    ("Reward", "Environment/Cumulative Reward", 1.0, ""),
    ("Episode length", "Environment/Episode Length", 1.0, "steps"),
    ("Entropy", "Policy/Entropy", 1.0, ""),
)


def load_scalars(directory: Path) -> dict[str, list[tuple[int, float]]]:
    """Load and merge TensorBoard scalar files, preferring newer restarts."""
    merged: dict[str, dict[int, float]] = {}
    files = sorted(directory.glob("events.out.tfevents.*"), key=lambda p: p.stat().st_mtime)
    for path in files:
        accumulator = EventAccumulator(str(path), size_guidance={"scalars": 0})
        accumulator.Reload()
        for tag in accumulator.Tags().get("scalars", []):
            values = merged.setdefault(tag, {})
            for event in accumulator.Scalars(tag):
                values[event.step] = event.value
    return {tag: sorted(values.items()) for tag, values in merged.items()}


def tail_mean(
    data: dict[str, list[tuple[int, float]]],
    tag: str,
    window: int,
    cutoff: int | None = None,
) -> float | None:
    points = data.get(tag, [])
    if cutoff is not None:
        points = [(step, value) for step, value in points if step <= cutoff]
    if not points:
        return None
    return float(np.mean([value for _, value in points[-window:]]))


def latest(data: dict[str, list[tuple[int, float]]], tag: str) -> tuple[int, float] | None:
    points = data.get(tag, [])
    return points[-1] if points else None


def read_progress(log_path: Path) -> tuple[int, float | None]:
    pattern = re.compile(r"Step: (\d+)\. Time Elapsed: ([0-9.]+) s")
    segment: list[tuple[int, float]] = []
    previous_time = -1.0
    for line in log_path.read_text(errors="ignore").splitlines():
        match = pattern.search(line)
        if not match:
            continue
        point = (int(match.group(1)), float(match.group(2)))
        if point[1] < previous_time:
            segment = []
        segment.append(point)
        previous_time = point[1]
    if not segment:
        return 0, None
    sample = segment[-min(100, len(segment)) :]
    elapsed = sample[-1][1] - sample[0][1]
    rate = (sample[-1][0] - sample[0][0]) / elapsed if elapsed > 0 else None
    return segment[-1][0], rate


def format_value(value: float | None, unit: str) -> str:
    if value is None:
        return "n/a"
    if unit == "%":
        return f"{value:8.2f}%"
    if unit in {"s", "cm"}:
        return f"{value:8.3f} {unit}"
    if unit == "steps":
        return f"{value:8.2f}"
    return f"{value:8.4f}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=int, default=100, help="number of summaries to average")
    parser.add_argument("--target", type=int, default=1_000_000)
    parser.add_argument("--baseline-step", type=int, default=526_647)
    parser.add_argument(
        "--current",
        type=Path,
        default=Path("training/results/dg5f_v1_continued_gpu_20260721/DG5FGrasp"),
    )
    parser.add_argument(
        "--baseline",
        type=Path,
        default=Path("training/results/dg5f_v1_gpu_fixed/DG5FGrasp"),
    )
    parser.add_argument(
        "--log",
        type=Path,
        default=Path("training/logs/dg5f_v1_continued_gpu_20260721.log"),
    )
    args = parser.parse_args()

    current = load_scalars(args.current)
    baseline = load_scalars(args.baseline)
    step, rate = read_progress(args.log)
    progress = min(100.0, 100.0 * step / args.target)
    eta = (args.target - step) / rate / 60.0 if rate and step < args.target else 0.0

    print(f"DG5F V1 live metrics  {dt.datetime.now(dt.timezone.utc):%Y-%m-%d %H:%M:%S UTC}")
    print("=" * 76)
    rate_text = f"{rate:.1f} steps/s" if rate else "n/a"
    print(
        f"Step {step:,} / {args.target:,} ({progress:.2f}%)  |  "
        f"rate {rate_text}  |  ETA {eta:.1f} min"
    )
    print(f"Averages use the latest {args.window} TensorBoard summaries (about {args.window * 200:,} steps).")
    print()
    print(f"{'Metric':<19} {'Frozen V1':>15} {'Continued':>15} {'Delta':>15}")
    print("-" * 68)
    for label, tag, scale, unit in METRICS:
        old = tail_mean(baseline, tag, args.window, args.baseline_step)
        new = tail_mean(current, tag, args.window)
        old_scaled = old * scale if old is not None else None
        new_scaled = new * scale if new is not None else None
        delta = new_scaled - old_scaled if old_scaled is not None and new_scaled is not None else None
        delta_text = f"{delta:+.4f}" if delta is not None else "n/a"
        print(
            f"{label:<19} {format_value(old_scaled, unit):>15} "
            f"{format_value(new_scaled, unit):>15} {delta_text:>15}"
        )

    print("\nLatest optimizer update")
    for label, tag in (
        ("Policy loss", "Losses/Policy Loss"),
        ("Value loss", "Losses/Value Loss"),
        ("Learning rate", "Policy/Learning Rate"),
    ):
        point = latest(current, tag)
        print(f"  {label:<14}: {point[1]:.8f} at step {point[0]:,}" if point else f"  {label:<14}: n/a")

    failures = current.get("Failure/BallOutOfBounds", [])
    log_text = args.log.read_text(errors="ignore")
    print("\nHealth")
    print(f"  NaN/non-finite : {len(re.findall(r'nan|non.?finite', log_text, re.I))}")
    print(f"  Error/traceback: {len(re.findall(r'error|traceback|exception', log_text, re.I))}")
    print(
        f"  Ball out       : {len(failures)} summaries"
        + (f" (last at step {failures[-1][0]:,})" if failures else "")
    )


if __name__ == "__main__":
    main()
