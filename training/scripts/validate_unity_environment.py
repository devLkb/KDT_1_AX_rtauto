#!/usr/bin/env python3
"""Validate an externally packaged Unity player before ML-Agents starts."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml


class ValidationError(RuntimeError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _positive_int(manifest: dict[str, Any], key: str) -> int:
    value = manifest.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValidationError(f"manifest {key!r} must be a positive integer")
    return value


def validate(environment: Path, config: Path, manifest_path: Path) -> dict[str, Any]:
    if not environment.is_file():
        raise ValidationError(f"Unity player does not exist: {environment}")
    if not config.is_file():
        raise ValidationError(f"trainer config does not exist: {config}")
    if not manifest_path.is_file():
        raise ValidationError(f"environment manifest does not exist: {manifest_path}")

    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValidationError(f"invalid environment manifest: {exc}") from exc
    if manifest.get("format") != "dg5f-unity-environment":
        raise ValidationError("unsupported environment manifest format")

    spec_version = manifest.get("spec_version")
    behavior = manifest.get("behavior")
    curriculum = manifest.get("curriculum_parameter")
    if not isinstance(spec_version, str) or not spec_version:
        raise ValidationError("manifest 'spec_version' must be a non-empty string")
    if not isinstance(behavior, str) or not behavior:
        raise ValidationError("manifest 'behavior' must be a non-empty string")
    if not isinstance(curriculum, str) or not curriculum:
        raise ValidationError(
            "manifest 'curriculum_parameter' must be a non-empty string"
        )
    _positive_int(manifest, "observation_size")
    _positive_int(manifest, "continuous_actions")

    player_expected = manifest.get("player_sha256")
    if not isinstance(player_expected, str) or sha256(environment) != player_expected:
        raise ValidationError("Unity player SHA-256 does not match its manifest")

    dll_relative = manifest.get("grasp_dll")
    dll_expected = manifest.get("grasp_dll_sha256")
    if not isinstance(dll_relative, str) or not dll_relative:
        raise ValidationError("manifest 'grasp_dll' must be a relative path")
    dll = environment.parent / dll_relative
    if not dll.is_file():
        raise ValidationError(f"Grasp runtime DLL does not exist: {dll}")
    if not isinstance(dll_expected, str) or sha256(dll) != dll_expected:
        raise ValidationError("Grasp runtime DLL SHA-256 does not match its manifest")

    try:
        trainer = yaml.safe_load(config.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValidationError(f"invalid trainer config: {exc}") from exc
    if not isinstance(trainer, dict):
        raise ValidationError("trainer config must contain a mapping")
    behaviors = trainer.get("behaviors")
    if not isinstance(behaviors, dict) or set(behaviors) != {behavior}:
        actual = sorted(behaviors) if isinstance(behaviors, dict) else []
        raise ValidationError(
            f"behavior mismatch: package requires {behavior!r}, config has {actual}"
        )
    parameters = trainer.get("environment_parameters")
    if not isinstance(parameters, dict) or curriculum not in parameters:
        actual = sorted(parameters) if isinstance(parameters, dict) else []
        raise ValidationError(
            "curriculum parameter mismatch: "
            f"package requires {curriculum!r}, config has {actual}"
        )
    return manifest


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--env", required=True, type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    args = parser.parse_args()
    try:
        manifest = validate(args.env, args.config, args.manifest)
    except ValidationError as exc:
        parser.exit(2, f"[ERROR] Unity 환경 계약 검증 실패: {exc}\n")
    print(
        "[Unity] 외부 패키지 검증 완료: "
        f"spec={manifest['spec_version']}, behavior={manifest['behavior']}, "
        f"observations={manifest['observation_size']}, "
        f"actions={manifest['continuous_actions']}"
    )


if __name__ == "__main__":
    main()
