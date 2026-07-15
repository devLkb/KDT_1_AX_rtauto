#!/usr/bin/env python3
"""Emit the next DG5FGraspV4 lesson config after an exact success-rate gate."""

import argparse
from pathlib import Path
import sys

import yaml

MIN_EPISODES = 200
MIN_SUCCESS_RATE = 0.80
FINAL_STAGE = 4


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--current-stage", type=int, required=True)
    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--successes", type=int, required=True)
    parser.add_argument(
        "--base-config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "dg5f_grasp.yaml",
    )
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0 <= args.current_stage < FINAL_STAGE:
        raise SystemExit("current-stage must be 0..3; stage 4 is final")
    if args.episodes < MIN_EPISODES:
        raise SystemExit(f"promotion denied: need at least {MIN_EPISODES} episodes")
    if not 0 <= args.successes <= args.episodes:
        raise SystemExit("successes must be between 0 and episodes")

    success_rate = args.successes / args.episodes
    if success_rate < MIN_SUCCESS_RATE:
        raise SystemExit(
            f"promotion denied: success rate {success_rate:.3f} < {MIN_SUCCESS_RATE:.2f}"
        )

    config = yaml.safe_load(args.base_config.read_text(encoding="utf-8"))
    configured_stage = int(
        config["environment_parameters"]["lesson"]["sampler_parameters"]["value"]
    )
    if configured_stage != args.current_stage:
        raise SystemExit(
            f"base config lesson is {configured_stage}, expected {args.current_stage}"
        )

    next_stage = args.current_stage + 1
    config["environment_parameters"]["lesson"]["sampler_parameters"]["value"] = float(
        next_stage
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(config, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    print(
        f"promotion accepted: stage {args.current_stage} -> {next_stage}; "
        f"{args.successes}/{args.episodes} = {success_rate:.3f}; wrote {args.output}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
