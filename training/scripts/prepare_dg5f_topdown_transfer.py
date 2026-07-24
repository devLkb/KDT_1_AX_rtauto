#!/usr/bin/env python3
"""Validate and stage the immutable 599887-step DG5F transfer checkpoint."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import torch


BEHAVIOR = "DG5FGrasp"
EXPECTED_SHA256 = (
    "38340bbbe20a994f6ea5db2792bb5f2b29eb51024c3667782f1ead17873d2cd1"
)
EXPECTED_GLOBAL_STEP = 599_887
EXPECTED_OBSERVATIONS = 57
EXPECTED_ACTIONS = 7
EXPECTED_HIDDEN_UNITS = 256
EXPECTED_HIDDEN_LAYERS = 3
DEFAULT_SOURCE_RUN_ID = "dg5f_topdown_transfer_source_599887"

POLICY_LAYER_PREFIX = "network_body._body_endoder.seq_layers"
CRITIC_LAYER_PREFIX = "network_body._body_endoder.seq_layers"


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


def _validate_body(
    state: dict[str, Any],
    prefix: str,
    input_size: int,
    label: str,
) -> None:
    expected = (
        (f"{prefix}.0.weight", (EXPECTED_HIDDEN_UNITS, input_size)),
        (f"{prefix}.0.bias", (EXPECTED_HIDDEN_UNITS,)),
        (
            f"{prefix}.2.weight",
            (EXPECTED_HIDDEN_UNITS, EXPECTED_HIDDEN_UNITS),
        ),
        (f"{prefix}.2.bias", (EXPECTED_HIDDEN_UNITS,)),
        (
            f"{prefix}.4.weight",
            (EXPECTED_HIDDEN_UNITS, EXPECTED_HIDDEN_UNITS),
        ),
        (f"{prefix}.4.bias", (EXPECTED_HIDDEN_UNITS,)),
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
        if name.startswith(f"{prefix}.") and name.endswith(".weight")
    )
    expected_weights = [name for name, _ in expected if name.endswith(".weight")]
    if layer_weights != expected_weights:
        raise ValueError(
            f"{label} must have exactly {EXPECTED_HIDDEN_LAYERS} hidden layers; "
            f"found {layer_weights}"
        )


def validate_checkpoint(path: Path) -> CheckpointContract:
    if not path.is_file():
        raise FileNotFoundError(path)
    source_hash = sha256_file(path)
    if source_hash != EXPECTED_SHA256:
        raise ValueError(
            f"checkpoint SHA-256 mismatch: expected {EXPECTED_SHA256}, "
            f"got {source_hash}"
        )

    checkpoint = torch.load(path, map_location="cpu", weights_only=False)
    if not isinstance(checkpoint, dict):
        raise ValueError("checkpoint root must be a mapping")

    try:
        policy = checkpoint["Policy"]
        critic = checkpoint["Optimizer:critic"]
        global_step_state = checkpoint["global_step"]
        value_optimizer = checkpoint["Optimizer:value_optimizer"]
    except KeyError as exc:
        raise ValueError(f"checkpoint is missing module {exc.args[0]!r}") from exc
    if not all(isinstance(item, dict) for item in (policy, critic, global_step_state)):
        raise ValueError("policy, critic, and global_step modules must be mappings")
    if not isinstance(value_optimizer, dict) or not {
        "state",
        "param_groups",
    }.issubset(value_optimizer):
        raise ValueError("checkpoint is missing PPO optimizer state")

    _validate_body(policy, POLICY_LAYER_PREFIX, EXPECTED_OBSERVATIONS, "policy")
    _validate_body(critic, CRITIC_LAYER_PREFIX, EXPECTED_OBSERVATIONS, "critic")

    mu_shape = _shape(
        policy,
        "action_model._continuous_distribution.mu.weight",
    )
    sigma_shape = _shape(
        policy,
        "action_model._continuous_distribution.log_sigma",
    )
    if mu_shape != (EXPECTED_ACTIONS, EXPECTED_HIDDEN_UNITS):
        raise ValueError(
            f"expected {EXPECTED_ACTIONS} continuous action outputs, got {mu_shape}"
        )
    if sigma_shape != (1, EXPECTED_ACTIONS):
        raise ValueError(
            f"expected {EXPECTED_ACTIONS} action sigmas, got {sigma_shape}"
        )

    step_tensor = global_step_state.get("_GlobalSteps__global_step")
    try:
        global_step = int(step_tensor.item())
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("checkpoint global step is not a scalar tensor") from exc
    if global_step != EXPECTED_GLOBAL_STEP:
        raise ValueError(
            f"expected global step {EXPECTED_GLOBAL_STEP}, got {global_step}"
        )

    return CheckpointContract(
        behavior=BEHAVIOR,
        global_step=global_step,
        observations=EXPECTED_OBSERVATIONS,
        continuous_actions=EXPECTED_ACTIONS,
        hidden_units=EXPECTED_HIDDEN_UNITS,
        hidden_layers=EXPECTED_HIDDEN_LAYERS,
        sha256=source_hash,
    )


def prepare_source_run(
    source: Path,
    results_dir: Path,
    source_run_id: str = DEFAULT_SOURCE_RUN_ID,
) -> tuple[Path, Path]:
    if not source_run_id or Path(source_run_id).name != source_run_id:
        raise ValueError("source run ID must be one path component")

    contract = validate_checkpoint(source)
    source_hash_before = contract.sha256
    destination = results_dir / source_run_id / BEHAVIOR / "checkpoint.pt"
    if source.resolve() == destination.resolve():
        raise ValueError("source checkpoint and prepared checkpoint must be separate")

    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists():
        if sha256_file(destination) != EXPECTED_SHA256:
            raise ValueError(
                f"refusing to overwrite non-matching prepared checkpoint: {destination}"
            )
    else:
        temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
        try:
            shutil.copyfile(source, temporary)
            if sha256_file(temporary) != EXPECTED_SHA256:
                raise ValueError("staged checkpoint changed while copying")
            os.replace(temporary, destination)
        finally:
            temporary.unlink(missing_ok=True)

    source_hash_after = sha256_file(source)
    if source_hash_after != source_hash_before:
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
        "contract": asdict(contract),
    }
    temporary_manifest = manifest_path.with_name(
        f".{manifest_path.name}.{os.getpid()}.tmp"
    )
    temporary_manifest.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_manifest, manifest_path)
    return destination, manifest_path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--results-dir", type=Path)
    parser.add_argument("--source-run-id", default=DEFAULT_SOURCE_RUN_ID)
    parser.add_argument("--verify-only", action="store_true")
    args = parser.parse_args()

    try:
        if args.verify_only:
            contract = validate_checkpoint(args.source)
            print(json.dumps(asdict(contract), indent=2, sort_keys=True))
        else:
            if args.results_dir is None:
                parser.error("--results-dir is required unless --verify-only is used")
            checkpoint, manifest = prepare_source_run(
                args.source,
                args.results_dir,
                args.source_run_id,
            )
            print(f"[PASS] prepared immutable transfer checkpoint: {checkpoint}")
            print(f"[PASS] provenance manifest: {manifest}")
    except (OSError, RuntimeError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
