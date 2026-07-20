#!/usr/bin/env python3
"""Derive the exact 512-step smoke config from the canonical reach config."""

from __future__ import annotations

import argparse
import os
import re
from pathlib import Path


BEHAVIOR = "DG5FGraspPointReach"
CANONICAL_MAX_STEPS = 5_000_000
SMOKE_MAX_STEPS = 512


def generate(source: Path, destination: Path) -> None:
    text = source.read_text(encoding="utf-8")
    if f"  {BEHAVIOR}:\n" not in text:
        raise ValueError(f"source config does not define {BEHAVIOR}")
    if "environment_parameters:" in text or "curriculum:" in text:
        raise ValueError("source config must not contain curriculum parameters")

    pattern = re.compile(
        rf"^(?P<indent>\s+)max_steps:\s*{CANONICAL_MAX_STEPS}\s*$",
        re.MULTILINE,
    )
    generated, replacements = pattern.subn(
        rf"\g<indent>max_steps: {SMOKE_MAX_STEPS}", text
    )
    if replacements != 1:
        raise ValueError(
            f"expected one max_steps: {CANONICAL_MAX_STEPS}, found {replacements}"
        )

    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    temporary.write_text(generated, encoding="utf-8")
    os.replace(temporary, destination)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    try:
        generate(args.source, args.destination)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
