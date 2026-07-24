#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${VENV:-${VIRTUAL_ENV:-$ROOT/vision/.vision}}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
SOURCE_RUN_ID="${DG5F_TOPDOWN_SOURCE_RUN_ID:-dg5f_topdown_transfer_source_599887}"
SOURCE_CHECKPOINT="${DG5F_TOPDOWN_SOURCE_CHECKPOINT:-$ROOT/training/results/dg5f_vdi_surface3cm_hold3s_curriculum_best_observed_600k_20260723/DG5FGrasp-599887.pt}"
RUN_ID="${RUN_ID:-dg5f_grasp_topdown_transfer_1m}"
CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp_topdown_transfer.yaml}"
ENV_PATH="${ENV_PATH:-$ROOT/training/builds/DG5FGraspTopDownTransfer/DG5FGrasp.x86_64}"
MODE="${1:-start}"

case "$MODE" in
  start)
    [[ ! -e "$RESULTS_DIR/$RUN_ID" ]] || {
      echo "[ERROR] new transfer run already exists; refusing to overwrite or resume it: $RUN_ID" >&2
      exit 2
    }
    "$VENV/bin/python" \
      "$ROOT/training/scripts/prepare_dg5f_topdown_transfer.py" \
      --source "$SOURCE_CHECKPOINT" \
      --results-dir "$RESULTS_DIR" \
      --source-run-id "$SOURCE_RUN_ID"
    extra_args=(--initialize-from "$SOURCE_RUN_ID")
    ;;
  resume)
    [[ "$RUN_ID" != "$SOURCE_RUN_ID" ]] || {
      echo "[ERROR] the immutable source run can never be resumed" >&2
      exit 2
    }
    [[ -f "$RESULTS_DIR/$RUN_ID/DG5FGrasp/checkpoint.pt" ]] || {
      echo "[ERROR] transfer run checkpoint not found: $RESULTS_DIR/$RUN_ID/DG5FGrasp/checkpoint.pt" >&2
      exit 2
    }
    extra_args=(--resume)
    ;;
  *)
    echo "usage: $0 [start|resume]" >&2
    exit 2
    ;;
esac

export CONFIG RESULTS_DIR RUN_ID ENV_PATH VENV
exec "$ROOT/training/scripts/train_dg5f_grasp.sh" "${extra_args[@]}"
