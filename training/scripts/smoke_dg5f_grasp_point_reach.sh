#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp_point_reach.yaml}"
PYTHON="${PYTHON:-${VENV:-${VIRTUAL_ENV:-$ROOT/vision/.vision}}/bin/python}"
if [[ -n "${DG5F_SMOKE_CONFIG:-}" ]]; then
  SMOKE_CONFIG="$DG5F_SMOKE_CONFIG"
else
  SMOKE_CONFIG="$(mktemp "${TMPDIR:-/tmp}/dg5f_grasp_point_reach_smoke_512.XXXXXX.yaml")"
  trap 'rm -f "$SMOKE_CONFIG"' EXIT
fi

[[ -x "$PYTHON" ]] || {
  echo "[ERROR] Python executable not found: $PYTHON" >&2
  exit 2
}

"$PYTHON" "$ROOT/training/scripts/generate_grasp_point_reach_smoke_config.py" \
  "$SOURCE_CONFIG" "$SMOKE_CONFIG"

CONFIG="$SMOKE_CONFIG" \
RUN_ID="${RUN_ID:-dg5f-grasp-ready-reach-smoke-512}" \
"$ROOT/training/scripts/train_dg5f_grasp_point_reach.sh" "$@"
