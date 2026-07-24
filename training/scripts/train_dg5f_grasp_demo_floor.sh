#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SOURCE_RUN_ID="${DG5F_DEMO_SOURCE_RUN_ID:-dg5f_v1_gpu_fixed}"
RUN_ID="${RUN_ID:-dg5f_v1_floor_safe_demo}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp_demo_floor_finetune.yaml}"
ENV_PATH="${ENV_PATH:-$ROOT/training/builds/DG5FGrasp/DG5FGrasp.x86_64}"
MODE="${1:-start}"

case "$MODE" in
  start)
    [[ -f "$RESULTS_DIR/$SOURCE_RUN_ID/DG5FGrasp/checkpoint.pt" ]] || {
      echo "[ERROR] source checkpoint not found: $RESULTS_DIR/$SOURCE_RUN_ID/DG5FGrasp/checkpoint.pt" >&2
      echo "        extract dg5f_v1_gpu_fixed_526647.tar.gz into $RESULTS_DIR first" >&2
      exit 2
    }
    extra_args=(--initialize-from "$SOURCE_RUN_ID")
    ;;
  resume)
    [[ -f "$RESULTS_DIR/$RUN_ID/DG5FGrasp/checkpoint.pt" ]] || {
      echo "[ERROR] demo checkpoint not found: $RESULTS_DIR/$RUN_ID/DG5FGrasp/checkpoint.pt" >&2
      exit 2
    }
    extra_args=(--resume)
    ;;
  *)
    echo "usage: $0 [start|resume]" >&2
    exit 2
    ;;
esac

export CONFIG RESULTS_DIR RUN_ID ENV_PATH
exec "$ROOT/training/scripts/train_dg5f_grasp.sh" "${extra_args[@]}"
