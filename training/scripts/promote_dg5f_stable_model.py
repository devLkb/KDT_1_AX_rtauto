#!/usr/bin/env python3
"""Gate and stage a stable-grasp ONNX for explicit Unity assignment."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
import shutil
import sys
from pathlib import Path

import onnx


OBSERVATIONS = 116
ACTIONS = 26
CANONICAL_NAME = "DG5FStableGrasp.onnx"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def concrete_dimensions(value_info: object) -> list[int]:
    tensor_type = value_info.type.tensor_type
    return [
        dimension.dim_value
        for dimension in tensor_type.shape.dim
        if dimension.HasField("dim_value")
    ]


def validate_onnx(model_path: Path) -> None:
    model = onnx.load(str(model_path))
    onnx.checker.check_model(model)
    initializer_names = {initializer.name for initializer in model.graph.initializer}
    inputs = [
        value for value in model.graph.input if value.name not in initializer_names
    ]
    input_shapes = {value.name: concrete_dimensions(value) for value in inputs}
    output_shapes = {
        value.name: concrete_dimensions(value) for value in model.graph.output
    }
    if not any(OBSERVATIONS in shape for shape in input_shapes.values()):
        raise ValueError(
            f"ONNX has no {OBSERVATIONS}-observation input: {input_shapes}"
        )
    if not any(ACTIONS in shape for shape in output_shapes.values()):
        raise ValueError(f"ONNX has no {ACTIONS}-action output: {output_shapes}")


def load_evaluator(root: Path):
    script = root / "training" / "scripts" / "evaluate_dg5f_stable.py"
    spec = importlib.util.spec_from_file_location("evaluate_dg5f_stable", script)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load evaluator: {script}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stage(
    model_path: Path,
    evaluation_csv: Path,
    evaluation_approval: Path,
    unity_models_dir: Path,
    repository_root: Path,
) -> tuple[Path, Path]:
    validate_onnx(model_path)
    evaluator = load_evaluator(repository_root)
    success_rate, reach_rate = evaluator.validate(evaluation_csv)
    evaluation_gate = json.loads(evaluation_approval.read_text(encoding="utf-8"))
    expected_gate = {
        "specVersion": "3.0.0",
        "behaviorName": "DG5FStableGrasp",
        "episodes": 200,
        "modelSha256": sha256(model_path),
        "evaluationCsvSha256": sha256(evaluation_csv),
    }
    mismatches = {
        key: (expected, evaluation_gate.get(key))
        for key, expected in expected_gate.items()
        if evaluation_gate.get(key) != expected
    }
    if mismatches:
        raise ValueError(
            "model-bound evaluation approval mismatch: "
            + ", ".join(
                f"{key} expected={expected!r} actual={actual!r}"
                for key, (expected, actual) in mismatches.items()
            )
        )

    unity_models_dir.mkdir(parents=True, exist_ok=True)
    canonical_path = unity_models_dir / CANONICAL_NAME
    temporary_path = unity_models_dir / f".{CANONICAL_NAME}.tmp"
    shutil.copyfile(model_path, temporary_path)
    os.replace(temporary_path, canonical_path)
    model_hash = sha256(canonical_path)

    approval_path = unity_models_dir / "DG5FStableGrasp.approved.json"
    approval = {
        "specVersion": "3.0.0",
        "behaviorName": "DG5FStableGrasp",
        "observations": OBSERVATIONS,
        "actions": ACTIONS,
        "episodes": 200,
        "successRate": success_rate,
        "reachRate": reach_rate,
        "sha256": model_hash,
        "sourceModel": str(model_path.resolve()),
        "evaluationCsv": str(evaluation_csv.resolve()),
    }
    temporary_approval = approval_path.with_suffix(".json.tmp")
    temporary_approval.write_text(
        json.dumps(approval, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary_approval, approval_path)
    return canonical_path, approval_path


def main() -> int:
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("evaluation_csv", type=Path)
    parser.add_argument(
        "--evaluation-approval",
        type=Path,
        help="Model-bound approval emitted by run_dg5f_stable_evaluation.sh "
        "(default: <evaluation_csv>.approved.json)",
    )
    parser.add_argument(
        "--unity-models-dir",
        type=Path,
        default=root / "unity" / "Assets" / "MLAgents" / "Grasp" / "Models",
    )
    args = parser.parse_args()
    evaluation_approval = args.evaluation_approval or Path(
        str(args.evaluation_csv) + ".approved.json"
    )
    try:
        canonical, approval = stage(
            args.model,
            args.evaluation_csv,
            evaluation_approval,
            args.unity_models_dir,
            root,
        )
    except (
        OSError,
        ValueError,
        RuntimeError,
        json.JSONDecodeError,
        onnx.checker.ValidationError,
    ) as exc:
        print(f"[FAIL] stable model not promoted: {exc}", file=sys.stderr)
        return 1
    print(f"[PASS] staged approved model: {canonical}")
    print(f"[PASS] approval manifest: {approval}")
    print("[NEXT] Run Unity menu: Tools/ML-Agents/Assign Approved DG5F Stable Model")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
