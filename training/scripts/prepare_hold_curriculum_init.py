#!/usr/bin/env python3
"""Prepare a low-noise, observation-safe checkpoint for hold curriculum transfer."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import torch


NEW_OBSERVATION_COLUMNS = (50, 51, 52)
ARM_ACTIONS = 6


def prepare(source: Path, output: Path, arm_sigma: float, ignored_sigma: float) -> None:
    if not 0.0 < arm_sigma <= 1.0 or not 0.0 < ignored_sigma <= 1.0:
        raise ValueError("sigmas must be in (0, 1]")
    checkpoint = torch.load(source, map_location="cpu", weights_only=False)

    policy = checkpoint["Policy"]
    input_weight = policy["network_body._body_endoder.seq_layers.0.weight"]
    if tuple(input_weight.shape)[1] != 57:
        raise ValueError(f"expected 57 policy inputs, got {tuple(input_weight.shape)}")
    input_weight[:, NEW_OBSERVATION_COLUMNS] = 0.0

    log_sigma = policy["action_model._continuous_distribution.log_sigma"]
    if log_sigma.numel() != 7:
        raise ValueError(f"expected 7 actions, got {tuple(log_sigma.shape)}")
    log_sigma[..., :ARM_ACTIONS] = math.log(arm_sigma)
    log_sigma[..., ARM_ACTIONS:] = math.log(ignored_sigma)

    critic = checkpoint.get("Optimizer:critic", {})
    critic_input = critic.get("network_body._body_endoder.seq_layers.0.weight")
    if critic_input is not None:
        if tuple(critic_input.shape)[1] != 57:
            raise ValueError(
                f"expected 57 critic inputs, got {tuple(critic_input.shape)}"
            )
        critic_input[:, NEW_OBSERVATION_COLUMNS] = 0.0

    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    torch.save(checkpoint, temporary)
    temporary.replace(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--arm-sigma", type=float, default=0.2)
    parser.add_argument("--ignored-sigma", type=float, default=0.05)
    args = parser.parse_args()
    prepare(args.source, args.output, args.arm_sigma, args.ignored_sigma)


if __name__ == "__main__":
    main()
