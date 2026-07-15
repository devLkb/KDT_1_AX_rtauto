#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${VENV:-$ROOT/vision/.vision}"
CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp.yaml}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
RUN_ID="${RUN_ID:-dg5f_grasp_v2}"
ENV_PATH="${ENV_PATH:-}"
NUM_ENVS="${NUM_ENVS:-2}"
TIME_SCALE="${TIME_SCALE:-10}"

args=("$CONFIG" --run-id "$RUN_ID" --time-scale "$TIME_SCALE" --results-dir "$RESULTS_DIR")
if [[ -n "$ENV_PATH" ]]; then
  args+=(--env "$ENV_PATH" --num-envs "$NUM_ENVS" --no-graphics)
fi

exec "$VENV/bin/mlagents-learn" "${args[@]}" "$@"
