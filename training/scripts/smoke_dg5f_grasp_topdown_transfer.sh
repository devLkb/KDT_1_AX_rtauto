#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${VENV:-${VIRTUAL_ENV:-$ROOT/vision/.vision}}"
SOURCE_CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp_topdown_transfer.yaml}"
SMOKE_CONFIG="$(mktemp "${TMPDIR:-/tmp}/dg5f_topdown_smoke_512.XXXXXX.yaml")"
trap 'rm -f "$SMOKE_CONFIG"' EXIT

"$VENV/bin/python" "$ROOT/training/scripts/generate_dg5f_topdown_smoke_config.py" \
  "$SOURCE_CONFIG" "$SMOKE_CONFIG"

CONFIG="$SMOKE_CONFIG" \
RUN_ID="${RUN_ID:-dg5f-grasp-topdown-transfer-smoke-512}" \
TORCH_DEVICE="${TORCH_DEVICE:-cpu}" \
"$ROOT/training/scripts/train_dg5f_grasp_topdown_transfer.sh" start
