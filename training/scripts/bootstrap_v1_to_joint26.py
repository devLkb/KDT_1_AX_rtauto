#!/usr/bin/env python3
"""Expand the frozen 57-observation/7-action V1 checkpoint to joint26.

The output is an ML-Agents ``--initialize-from`` run directory.  It deliberately
contains no learned optimizer moments and its global step is zero.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Mapping

import torch


SOURCE_BEHAVIOR = "DG5FGrasp"
TARGET_BEHAVIOR = "DG5FGraspJoint"
SOURCE_OBSERVATIONS = 57
TARGET_OBSERVATIONS = 116
SOURCE_ACTIONS = 7
TARGET_ACTIONS = 26
DEFAULT_SEED = 21026
EXPECTED_SOURCE_STEP = 526647
EXPECTED_SOURCE_SHA256 = (
    "75f5b5a6c88601b90fb9fb44e21d883a48df7ed3f6e8a23d4bba80f82768e066"
)

ENCODER_WEIGHT = "network_body._body_endoder.seq_layers.0.weight"
MU_WEIGHT = "action_model._continuous_distribution.mu.weight"
MU_BIAS = "action_model._continuous_distribution.mu.bias"
LOG_SIGMA = "action_model._continuous_distribution.log_sigma"
STEP_KEY = "_GlobalSteps__global_step"

POLICY_SHAPES = {
    "version_number": (1,),
    "is_continuous_int_deprecated": (1,),
    "continuous_act_size_vector": (1,),
    "discrete_act_size_vector": (1, 0),
    "act_size_vector_deprecated": (1,),
    "memory_size_vector": (1,),
    ENCODER_WEIGHT: (256, SOURCE_OBSERVATIONS),
    "network_body._body_endoder.seq_layers.0.bias": (256,),
    "network_body._body_endoder.seq_layers.2.weight": (256, 256),
    "network_body._body_endoder.seq_layers.2.bias": (256,),
    "network_body._body_endoder.seq_layers.4.weight": (256, 256),
    "network_body._body_endoder.seq_layers.4.bias": (256,),
    LOG_SIGMA: (1, SOURCE_ACTIONS),
    MU_WEIGHT: (SOURCE_ACTIONS, 256),
    MU_BIAS: (SOURCE_ACTIONS,),
}

CRITIC_SHAPES = {
    ENCODER_WEIGHT: (256, SOURCE_OBSERVATIONS),
    "network_body._body_endoder.seq_layers.0.bias": (256,),
    "network_body._body_endoder.seq_layers.2.weight": (256, 256),
    "network_body._body_endoder.seq_layers.2.bias": (256,),
    "network_body._body_endoder.seq_layers.4.weight": (256, 256),
    "network_body._body_endoder.seq_layers.4.bias": (256,),
    "value_heads.value_heads.extrinsic.weight": (1, 256),
    "value_heads.value_heads.extrinsic.bias": (1,),
}


class ConversionError(RuntimeError):
    pass


def _validate_state_dict(
    name: str, state: Mapping[str, Any], expected_shapes: Mapping[str, tuple[int, ...]]
) -> None:
    actual_names = set(state)
    expected_names = set(expected_shapes)
    if actual_names != expected_names:
        missing = sorted(expected_names - actual_names)
        unexpected = sorted(actual_names - expected_names)
        raise ConversionError(
            f"{name} tensor names differ; missing={missing}, unexpected={unexpected}"
        )
    for tensor_name, shape in expected_shapes.items():
        value = state[tensor_name]
        if not isinstance(value, torch.Tensor):
            raise ConversionError(f"{name}.{tensor_name} is not a tensor")
        if tuple(value.shape) != shape:
            raise ConversionError(
                f"{name}.{tensor_name}: expected {shape}, got {tuple(value.shape)}"
            )


def validate_source(checkpoint: Mapping[str, Any]) -> None:
    expected_modules = {
        "Policy", "global_step", "Optimizer:value_optimizer", "Optimizer:critic"
    }
    if set(checkpoint) != expected_modules:
        raise ConversionError(
            f"checkpoint modules differ: expected={sorted(expected_modules)}, "
            f"actual={sorted(checkpoint)}"
        )
    _validate_state_dict("Policy", checkpoint["Policy"], POLICY_SHAPES)
    _validate_state_dict("Optimizer:critic", checkpoint["Optimizer:critic"], CRITIC_SHAPES)
    step = checkpoint["global_step"]
    if set(step) != {STEP_KEY} or tuple(step[STEP_KEY].shape) != (1,):
        raise ConversionError("unexpected global_step tensor name or shape")
    if int(step[STEP_KEY].item()) != EXPECTED_SOURCE_STEP:
        raise ConversionError(
            f"expected frozen V1 step {EXPECTED_SOURCE_STEP}, "
            f"got {int(step[STEP_KEY].item())}"
        )
    optimizer = checkpoint["Optimizer:value_optimizer"]
    if not isinstance(optimizer, dict) or set(optimizer) != {"state", "param_groups"}:
        raise ConversionError("unexpected PPO optimizer state structure")
    groups = optimizer["param_groups"]
    if len(groups) != 1 or len(groups[0].get("params", ())) != 23:
        raise ConversionError("unexpected PPO optimizer parameter-group topology")


def _expanded_encoder(
    source: torch.Tensor, generator: torch.Generator, random_std: float
) -> torch.Tensor:
    target = torch.empty(
        (source.shape[0], TARGET_OBSERVATIONS), dtype=source.dtype, device="cpu"
    )
    target.normal_(0.0, random_std, generator=generator)
    target[:, 0:12] = source[:, 0:12].cpu()
    target[:, 72:116] = source[:, 13:57].cpu()
    return target


def convert_checkpoint(
    source: Mapping[str, Any],
    *,
    seed: int = DEFAULT_SEED,
    random_std: float = 0.01,
    hand_log_sigma: float = -2.0,
) -> dict[str, Any]:
    validate_source(source)
    generator = torch.Generator(device="cpu")
    generator.manual_seed(seed)

    policy = copy.deepcopy(source["Policy"])
    policy[ENCODER_WEIGHT] = _expanded_encoder(
        source["Policy"][ENCODER_WEIGHT], generator, random_std
    )
    policy["continuous_act_size_vector"] = torch.tensor(
        [TARGET_ACTIONS],
        dtype=policy["continuous_act_size_vector"].dtype,
        device="cpu",
    )
    policy["act_size_vector_deprecated"] = torch.tensor(
        [TARGET_ACTIONS],
        dtype=policy["act_size_vector_deprecated"].dtype,
        device="cpu",
    )

    source_mu_weight = source["Policy"][MU_WEIGHT].cpu()
    target_mu_weight = torch.empty(
        (TARGET_ACTIONS, source_mu_weight.shape[1]),
        dtype=source_mu_weight.dtype,
        device="cpu",
    )
    target_mu_weight.normal_(0.0, random_std, generator=generator)
    target_mu_weight[0:6] = source_mu_weight[0:6]
    policy[MU_WEIGHT] = target_mu_weight

    source_mu_bias = source["Policy"][MU_BIAS].cpu()
    target_mu_bias = torch.empty(
        TARGET_ACTIONS, dtype=source_mu_bias.dtype, device="cpu"
    )
    target_mu_bias.normal_(0.0, random_std, generator=generator)
    target_mu_bias[0:6] = source_mu_bias[0:6]
    policy[MU_BIAS] = target_mu_bias

    source_log_sigma = source["Policy"][LOG_SIGMA].cpu()
    target_log_sigma = torch.full(
        (1, TARGET_ACTIONS),
        hand_log_sigma,
        dtype=source_log_sigma.dtype,
        device="cpu",
    )
    target_log_sigma[:, 0:6] = source_log_sigma[:, 0:6]
    policy[LOG_SIGMA] = target_log_sigma

    critic = copy.deepcopy(source["Optimizer:critic"])
    critic[ENCODER_WEIGHT] = _expanded_encoder(
        source["Optimizer:critic"][ENCODER_WEIGHT], generator, random_std
    )

    # Keep only the optimizer parameter-group topology so ML-Agents can load the
    # module without warnings. No momentum, variance, or update counter is copied.
    optimizer = {
        "state": {},
        "param_groups": copy.deepcopy(
            source["Optimizer:value_optimizer"]["param_groups"]
        ),
    }
    for group in optimizer["param_groups"]:
        group["lr"] = 1e-4

    converted = {
        "Policy": policy,
        "global_step": {
            STEP_KEY: torch.zeros(1, dtype=torch.int64, device="cpu")
        },
        "Optimizer:value_optimizer": optimizer,
        "Optimizer:critic": critic,
    }
    verify_conversion(source, converted, hand_log_sigma=hand_log_sigma)
    return converted


def verify_conversion(
    source: Mapping[str, Any],
    converted: Mapping[str, Any],
    *,
    hand_log_sigma: float = -2.0,
) -> None:
    validate_source(source)
    target_policy_shapes = dict(POLICY_SHAPES)
    target_policy_shapes[ENCODER_WEIGHT] = (256, TARGET_OBSERVATIONS)
    target_policy_shapes[LOG_SIGMA] = (1, TARGET_ACTIONS)
    target_policy_shapes[MU_WEIGHT] = (TARGET_ACTIONS, 256)
    target_policy_shapes[MU_BIAS] = (TARGET_ACTIONS,)
    target_critic_shapes = dict(CRITIC_SHAPES)
    target_critic_shapes[ENCODER_WEIGHT] = (256, TARGET_OBSERVATIONS)

    if set(converted) != {
        "Policy", "global_step", "Optimizer:value_optimizer", "Optimizer:critic"
    }:
        raise ConversionError("converted checkpoint has unexpected modules")
    _validate_state_dict("Policy", converted["Policy"], target_policy_shapes)
    _validate_state_dict(
        "Optimizer:critic", converted["Optimizer:critic"], target_critic_shapes
    )

    for module in ("Policy", "Optimizer:critic"):
        old_encoder = source[module][ENCODER_WEIGHT].cpu()
        new_encoder = converted[module][ENCODER_WEIGHT].cpu()
        if not torch.equal(new_encoder[:, 0:12], old_encoder[:, 0:12]):
            raise ConversionError(f"{module} arm observation columns were not copied exactly")
        if not torch.equal(new_encoder[:, 72:116], old_encoder[:, 13:57]):
            raise ConversionError(f"{module} task observation columns were not copied exactly")
        for name in (
            "network_body._body_endoder.seq_layers.0.bias",
            "network_body._body_endoder.seq_layers.2.weight",
            "network_body._body_endoder.seq_layers.2.bias",
            "network_body._body_endoder.seq_layers.4.weight",
            "network_body._body_endoder.seq_layers.4.bias",
        ):
            if not torch.equal(converted[module][name].cpu(), source[module][name].cpu()):
                raise ConversionError(f"{module}.{name} was not copied exactly")

    for name in (
        "value_heads.value_heads.extrinsic.weight",
        "value_heads.value_heads.extrinsic.bias",
    ):
        if not torch.equal(
            converted["Optimizer:critic"][name].cpu(),
            source["Optimizer:critic"][name].cpu(),
        ):
            raise ConversionError(f"critic {name} was not copied exactly")

    for name in (MU_WEIGHT, MU_BIAS):
        if not torch.equal(
            converted["Policy"][name][0:6].cpu(),
            source["Policy"][name][0:6].cpu(),
        ):
            raise ConversionError(f"arm action tensor {name} was not copied exactly")
    if not torch.equal(
        converted["Policy"][LOG_SIGMA][:, 0:6].cpu(),
        source["Policy"][LOG_SIGMA][:, 0:6].cpu(),
    ):
        raise ConversionError("arm log-sigma was not copied exactly")
    if not torch.all(
        converted["Policy"][LOG_SIGMA][:, 6:] == hand_log_sigma
    ):
        raise ConversionError("hand log-sigma does not use the conservative initializer")
    if converted["Optimizer:value_optimizer"]["state"]:
        raise ConversionError("optimizer state was copied")
    if int(converted["global_step"][STEP_KEY].item()) != 0:
        raise ConversionError("converted global step is not zero")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def create_bootstrap_run(
    source_checkpoint: Path,
    output_run: Path,
    *,
    seed: int = DEFAULT_SEED,
    force: bool = False,
    expected_source_sha256: str | None = None,
    target_behavior: str = TARGET_BEHAVIOR,
    spec_version: str = "2.1.0",
) -> Path:
    checkpoint_path = output_run / target_behavior / "checkpoint.pt"
    manifest_path = output_run / "bootstrap_manifest.json"
    source_hash = _sha256(source_checkpoint)
    if expected_source_sha256 is not None and source_hash != expected_source_sha256:
        raise ConversionError(
            "frozen V1 checkpoint SHA-256 mismatch: "
            f"expected {expected_source_sha256}, got {source_hash}"
        )

    if checkpoint_path.exists() and manifest_path.exists() and not force:
        source = torch.load(source_checkpoint, map_location="cpu", weights_only=False)
        converted = torch.load(checkpoint_path, map_location="cpu", weights_only=False)
        verify_conversion(source, converted)
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        if manifest.get("source_sha256") != source_hash:
            raise ConversionError("existing bootstrap was built from a different source")
        if manifest.get("target_behavior") != target_behavior:
            raise ConversionError("existing bootstrap targets a different behavior")
        if manifest.get("spec_version") != spec_version:
            raise ConversionError("existing bootstrap targets a different spec version")
        return checkpoint_path

    source = torch.load(source_checkpoint, map_location="cpu", weights_only=False)
    converted = convert_checkpoint(source, seed=seed)
    checkpoint_path.parent.mkdir(parents=True, exist_ok=True)
    torch.save(converted, checkpoint_path)
    verify_conversion(
        source,
        torch.load(checkpoint_path, map_location="cpu", weights_only=False),
    )
    manifest = {
        "format": "dg5f-v1-to-joint26-bootstrap",
        "spec_version": spec_version,
        "source_behavior": SOURCE_BEHAVIOR,
        "target_behavior": target_behavior,
        "source_checkpoint": str(source_checkpoint.resolve()),
        "source_sha256": source_hash,
        "source_observations": SOURCE_OBSERVATIONS,
        "target_observations": TARGET_OBSERVATIONS,
        "source_actions": SOURCE_ACTIONS,
        "target_actions": TARGET_ACTIONS,
        "seed": seed,
        "global_step": 0,
        "optimizer_state_copied": False,
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    os.chmod(checkpoint_path, 0o444)
    os.chmod(manifest_path, 0o444)
    return checkpoint_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output-run", required=True, type=Path)
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED)
    parser.add_argument(
        "--expected-source-sha256",
        default=EXPECTED_SOURCE_SHA256,
    )
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--target-behavior", default=TARGET_BEHAVIOR)
    parser.add_argument("--spec-version", default="2.1.0")
    args = parser.parse_args()
    output = create_bootstrap_run(
        args.source,
        args.output_run,
        seed=args.seed,
        force=args.force,
        expected_source_sha256=args.expected_source_sha256,
        target_behavior=args.target_behavior,
        spec_version=args.spec_version,
    )
    print(f"[OK] verified joint26 bootstrap: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
