#!/usr/bin/env python3
"""Stage a 57/7 reach checkpoint as an --initialize-from source for DG5FGraspLift.

The grasp+lift task deliberately keeps the reach task's 57-observation /
7-action policy shape so the already-trained pre-grasp positioning policy can
seed it. `mlagents-learn --initialize-from RUN` looks for
`<results>/RUN/<behavior>/checkpoint.pt`, so the source checkpoint has to be
copied under the *new* behavior name. Only the file location changes; the
weights are copied byte-for-byte and the source file is never modified.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import torch


BEHAVIOR = "DG5FGraspLift"
EXPECTED_OBSERVATIONS = 57
EXPECTED_ACTIONS = 7
EXPECTED_HIDDEN_UNITS = 256
EXPECTED_HIDDEN_LAYERS = 3
DEFAULT_SOURCE_RUN_ID = "dg5f_grasp_lift_transfer_source_599887"

# The reach policy sampled a 7th "grip" action that the reach agent ignored, so its
# output head carries arbitrary weights and — worse — the run converged to a nearly
# deterministic policy (log_sigma around -2, entropy ~ -0.16). Loaded as-is it barely
# explores at all, and never explores the grip axis, which is precisely the axis this
# task has to learn. Two targeted edits fix that without touching the arm skill:
#   * zero the grip row of the action mean head, so grip starts unbiased at 0;
#   * re-inflate log_sigma — moderately on the 6 arm axes (the arm behaviour is good
#     and should only be perturbed, not destroyed) and fully on the grip axis.
GRIP_ACTION_INDEX = 6
DEFAULT_ARM_LOG_SIGMA = -0.7  # sigma ~ 0.50 of the action range
DEFAULT_GRIP_LOG_SIGMA = 0.0  # sigma = 1.0: explore grip from scratch

BODY_LAYER_PREFIX = "network_body._body_endoder.seq_layers"


@dataclass(frozen=True)
class CheckpointContract:
    behavior: str
    global_step: int
    observations: int
    continuous_actions: int
    hidden_units: int
    hidden_layers: int
    sha256: str


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _shape(state: dict[str, Any], name: str) -> tuple[int, ...]:
    try:
        tensor = state[name]
    except KeyError as exc:
        raise ValueError(f"checkpoint is missing tensor {name!r}") from exc
    try:
        return tuple(tensor.shape)
    except AttributeError as exc:
        raise ValueError(f"checkpoint value {name!r} is not a tensor") from exc


def _validate_body(state: dict[str, Any], label: str) -> None:
    expected = (
        (f"{BODY_LAYER_PREFIX}.0.weight", (EXPECTED_HIDDEN_UNITS, EXPECTED_OBSERVATIONS)),
        (f"{BODY_LAYER_PREFIX}.0.bias", (EXPECTED_HIDDEN_UNITS,)),
        (f"{BODY_LAYER_PREFIX}.2.weight", (EXPECTED_HIDDEN_UNITS, EXPECTED_HIDDEN_UNITS)),
        (f"{BODY_LAYER_PREFIX}.2.bias", (EXPECTED_HIDDEN_UNITS,)),
        (f"{BODY_LAYER_PREFIX}.4.weight", (EXPECTED_HIDDEN_UNITS, EXPECTED_HIDDEN_UNITS)),
        (f"{BODY_LAYER_PREFIX}.4.bias", (EXPECTED_HIDDEN_UNITS,)),
    )
    for name, expected_shape in expected:
        actual = _shape(state, name)
        if actual != expected_shape:
            raise ValueError(
                f"{label} tensor {name!r}: expected {expected_shape}, got {actual}"
            )

    layer_weights = sorted(
        name
        for name in state
        if name.startswith(f"{BODY_LAYER_PREFIX}.") and name.endswith(".weight")
    )
    expected_weights = sorted(name for name, _ in expected if name.endswith(".weight"))
    if layer_weights != expected_weights:
        raise ValueError(
            f"{label} must have exactly {EXPECTED_HIDDEN_LAYERS} hidden layers; "
            f"found {layer_weights}"
        )


def validate_checkpoint(path: Path) -> CheckpointContract:
    if not path.is_file():
        raise FileNotFoundError(path)
    source_hash = sha256_file(path)

    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict):
        raise ValueError("checkpoint root must be a mapping")

    try:
        policy = checkpoint["Policy"]
        critic = checkpoint["Optimizer:critic"]
        global_step_state = checkpoint["global_step"]
    except KeyError as exc:
        raise ValueError(f"checkpoint is missing module {exc.args[0]!r}") from exc
    if not all(isinstance(item, dict) for item in (policy, critic, global_step_state)):
        raise ValueError("policy, critic, and global_step modules must be mappings")

    _validate_body(policy, "policy")
    _validate_body(critic, "critic")

    mu_shape = _shape(policy, "action_model._continuous_distribution.mu.weight")
    sigma_shape = _shape(policy, "action_model._continuous_distribution.log_sigma")
    if mu_shape != (EXPECTED_ACTIONS, EXPECTED_HIDDEN_UNITS):
        raise ValueError(
            f"expected {EXPECTED_ACTIONS} continuous action outputs, got {mu_shape}"
        )
    if sigma_shape != (1, EXPECTED_ACTIONS):
        raise ValueError(f"expected {EXPECTED_ACTIONS} action sigmas, got {sigma_shape}")

    step_tensor = global_step_state.get("_GlobalSteps__global_step")
    try:
        global_step = int(step_tensor.item())
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("checkpoint global step is not a scalar tensor") from exc

    return CheckpointContract(
        behavior=BEHAVIOR,
        global_step=global_step,
        observations=EXPECTED_OBSERVATIONS,
        continuous_actions=EXPECTED_ACTIONS,
        hidden_units=EXPECTED_HIDDEN_UNITS,
        hidden_layers=EXPECTED_HIDDEN_LAYERS,
        sha256=source_hash,
    )


def retune_exploration(
    checkpoint: dict[str, Any],
    arm_log_sigma: float,
    grip_log_sigma: float,
    reset_grip_head: bool,
) -> dict[str, float | bool]:
    """Zero the unused grip mean head and re-inflate the action sigmas in place."""
    policy = checkpoint["Policy"]
    mu_weight = policy["action_model._continuous_distribution.mu.weight"]
    mu_bias = policy["action_model._continuous_distribution.mu.bias"]
    log_sigma = policy["action_model._continuous_distribution.log_sigma"]

    previous_log_sigma = [float(value) for value in log_sigma.reshape(-1)]

    if reset_grip_head:
        with torch.no_grad():
            mu_weight[GRIP_ACTION_INDEX].zero_()
            mu_bias[GRIP_ACTION_INDEX].zero_()

    with torch.no_grad():
        for index in range(EXPECTED_ACTIONS):
            value = grip_log_sigma if index == GRIP_ACTION_INDEX else arm_log_sigma
            log_sigma.reshape(-1)[index] = value

    return {
        "grip_head_reset": reset_grip_head,
        "arm_log_sigma": arm_log_sigma,
        "grip_log_sigma": grip_log_sigma,
        "previous_log_sigma": previous_log_sigma,
    }


def prepare_source_run(
    source: Path,
    results_dir: Path,
    source_run_id: str = DEFAULT_SOURCE_RUN_ID,
    arm_log_sigma: float = DEFAULT_ARM_LOG_SIGMA,
    grip_log_sigma: float = DEFAULT_GRIP_LOG_SIGMA,
    reset_grip_head: bool = True,
) -> tuple[Path, Path]:
    if not source_run_id or Path(source_run_id).name != source_run_id:
        raise ValueError("source run ID must be one path component")

    contract = validate_checkpoint(source)
    destination = results_dir / source_run_id / BEHAVIOR / "checkpoint.pt"
    if source.resolve() == destination.resolve():
        raise ValueError("source checkpoint and prepared checkpoint must be separate")

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    try:
        checkpoint = torch.load(source, map_location="cpu", weights_only=False)
        exploration = retune_exploration(
            checkpoint, arm_log_sigma, grip_log_sigma, reset_grip_head
        )
        torch.save(checkpoint, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)

    if sha256_file(source) != contract.sha256:
        raise RuntimeError("source checkpoint changed during preparation")

    manifest_path = destination.parents[1] / "provenance.json"
    manifest = {
        "schema_version": 1,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "transfer_mode": "mlagents_initialize_from",
        "source_checkpoint": str(source.resolve()),
        "prepared_checkpoint": str(destination.resolve()),
        "source_run_id": source_run_id,
        "source_unchanged": True,
        "observation_weights_modified": False,
        "exploration_retuned": exploration,
        "note": (
            "Reach (DG5FGrasp) policy re-hosted under the DG5FGraspLift behavior "
            "name. Observation slots 0..48 keep their reach meaning; 49..56 are "
            "repurposed for grasp/lift state and are re-learned during fine-tuning. "
            "The grip action head is zeroed and the action sigmas re-inflated so the "
            "nearly deterministic reach policy explores again."
        ),
        "contract": asdict(contract),
    }
    temporary_manifest = manifest_path.with_name(f".{manifest_path.name}.{os.getpid()}.tmp")
    temporary_manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    os.replace(temporary_manifest, manifest_path)
    return destination, manifest_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--results-dir", type=Path)
    parser.add_argument("--source-run-id", default=DEFAULT_SOURCE_RUN_ID)
    parser.add_argument("--arm-log-sigma", type=float, default=DEFAULT_ARM_LOG_SIGMA)
    parser.add_argument("--grip-log-sigma", type=float, default=DEFAULT_GRIP_LOG_SIGMA)
    parser.add_argument("--keep-grip-head", action="store_true")
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()

    try:
        if args.verify_only:
            print(json.dumps(asdict(validate_checkpoint(args.source)), indent=2, sort_keys=True))
        else:
            if args.results_dir is None:
                parser.error("--results-dir is required unless --verify-only is used")
            checkpoint, manifest = prepare_source_run(
                args.source,
                args.results_dir,
                args.source_run_id,
                arm_log_sigma=args.arm_log_sigma,
                grip_log_sigma=args.grip_log_sigma,
                reset_grip_head=not args.keep_grip_head,
            )
            print(f"[PASS] prepared transfer checkpoint: {checkpoint}")
            print(f"[PASS] provenance manifest: {manifest}")
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
