#!/usr/bin/env python3
"""Convert the frozen 57x7 V1 reach policy to the 26x6 PointReach policy.

Only semantically equivalent arm/task inputs and the first six arm outputs are
transferred. Optimizer moments are intentionally discarded and step starts at 0.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path

import torch


SOURCE_BEHAVIOR = "DG5FGrasp"
TARGET_BEHAVIOR = "DG5FGraspPointReach"
SOURCE_SHA256 = "75f5b5a6c88601b90fb9fb44e21d883a48df7ed3f6e8a23d4bba80f82768e066"
ENCODER = "network_body._body_endoder.seq_layers.0.weight"
ENCODER_BIAS = "network_body._body_endoder.seq_layers.0.bias"
MU_WEIGHT = "action_model._continuous_distribution.mu.weight"
MU_BIAS = "action_model._continuous_distribution.mu.bias"
LOG_SIGMA = "action_model._continuous_distribution.log_sigma"
STEP_KEY = "_GlobalSteps__global_step"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def convert_encoder(weight: torch.Tensor, bias: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
    if tuple(weight.shape) != (256, 57) or tuple(bias.shape) != (256,):
        raise ValueError("unexpected frozen V1 encoder shape")
    target = torch.zeros((256, 26), dtype=weight.dtype)
    target_bias = bias.detach().cpu().clone()

    # Identical arm position and velocity observations.
    target[:, 0:12] = weight[:, 0:12]
    # V1's commanded arm targets 43..48 become PointReach 12..17.
    target[:, 12:18] = weight[:, 43:49]
    # V1 offsets were divided by 1.0; PointReach divides by workspace radius 1.05.
    target[:, 18:21] = weight[:, 13:16] * 1.05

    # V1 current/best potentials were 1-distance/0.85. PointReach input 21 is
    # distance/1.05, so preserve the affine contribution in weight and bias.
    potential_weight = weight[:, 53] + weight[:, 54]
    target[:, 21] = potential_weight * (-1.05 / 0.85)
    target_bias += potential_weight

    # Fold constant V1 inputs: open grip, reach objective one-hot, and 5 cm gate.
    target_bias += weight[:, 12] * -1.0
    target_bias += weight[:, 49]
    target_bias += weight[:, 55] * (0.05 / 0.85)
    return target, target_bias


def convert(source: dict) -> dict:
    if set(source) != {"Policy", "global_step", "Optimizer:value_optimizer", "Optimizer:critic"}:
        raise ValueError("unexpected checkpoint modules")
    if int(source["global_step"][STEP_KEY].item()) != 526647:
        raise ValueError("source is not the frozen 526647-step V1 checkpoint")

    policy = copy.deepcopy(source["Policy"])
    policy[ENCODER], policy[ENCODER_BIAS] = convert_encoder(
        source["Policy"][ENCODER], source["Policy"][ENCODER_BIAS]
    )
    policy["continuous_act_size_vector"] = torch.tensor([6.0])
    policy["act_size_vector_deprecated"] = torch.tensor([6.0])
    policy[MU_WEIGHT] = source["Policy"][MU_WEIGHT][0:6].detach().cpu().clone()
    policy[MU_BIAS] = source["Policy"][MU_BIAS][0:6].detach().cpu().clone()
    policy[LOG_SIGMA] = source["Policy"][LOG_SIGMA][:, 0:6].detach().cpu().clone()

    critic = copy.deepcopy(source["Optimizer:critic"])
    critic[ENCODER], critic[ENCODER_BIAS] = convert_encoder(
        source["Optimizer:critic"][ENCODER],
        source["Optimizer:critic"][ENCODER_BIAS],
    )

    groups = copy.deepcopy(source["Optimizer:value_optimizer"]["param_groups"])
    for group in groups:
        group["lr"] = 5e-5
    return {
        "Policy": policy,
        "global_step": {STEP_KEY: torch.zeros(1, dtype=torch.int64)},
        "Optimizer:value_optimizer": {"state": {}, "param_groups": groups},
        "Optimizer:critic": critic,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output-run", required=True, type=Path)
    args = parser.parse_args()
    source_hash = sha256(args.source)
    if source_hash != SOURCE_SHA256:
        raise SystemExit(f"source SHA-256 mismatch: {source_hash}")
    source = torch.load(args.source, map_location="cpu", weights_only=False)
    output = args.output_run / TARGET_BEHAVIOR / "checkpoint.pt"
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(convert(source), output)
    manifest = {
        "format": "dg5f-v1-to-point-reach-bootstrap",
        "source_behavior": SOURCE_BEHAVIOR,
        "target_behavior": TARGET_BEHAVIOR,
        "source_sha256": source_hash,
        "source_observations": 57,
        "target_observations": 26,
        "source_actions": 7,
        "target_actions": 6,
        "global_step": 0,
        "optimizer_state_copied": False,
    }
    manifest_path = args.output_run / "bootstrap_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    os.chmod(output, 0o444)
    os.chmod(manifest_path, 0o444)
    print(f"[OK] PointReach bootstrap: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
