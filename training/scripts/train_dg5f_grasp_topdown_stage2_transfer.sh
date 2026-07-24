#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
SOURCE_RUN_ID="${DG5F_STAGE2_SOURCE_RUN_ID:-dg5f_grasp_topdown_curriculum_v2_cpu_1m_20260724}"
RUN_ID="${RUN_ID:-dg5f_grasp_topdown_stage2_cpu_1m}"
CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp_topdown_stage2_transfer.yaml}"
ENV_PATH="${ENV_PATH:-$ROOT/training/builds/DG5FGraspTopDownCurriculumV2/DG5FGrasp.x86_64}"
MODE="${1:-start}"

SOURCE_CHECKPOINT="$RESULTS_DIR/$SOURCE_RUN_ID/DG5FGrasp/checkpoint.pt"

case "$MODE" in
  start)
    [[ "$RUN_ID" != "$SOURCE_RUN_ID" ]] || {
      echo "[ERROR] source and target run IDs must differ" >&2
      exit 2
    }
    [[ -f "$SOURCE_CHECKPOINT" ]] || {
      echo "[ERROR] Stage 1 source checkpoint not found: $SOURCE_CHECKPOINT" >&2
      exit 2
    }
    [[ ! -e "$RESULTS_DIR/$RUN_ID" ]] || {
      echo "[ERROR] new Stage 2 run already exists; refusing to overwrite: $RUN_ID" >&2
      exit 2
    }
    extra_args=(--initialize-from "$SOURCE_RUN_ID")
    ;;
  resume)
    [[ -f "$RESULTS_DIR/$RUN_ID/DG5FGrasp/checkpoint.pt" ]] || {
      echo "[ERROR] Stage 2 run checkpoint not found: $RESULTS_DIR/$RUN_ID/DG5FGrasp/checkpoint.pt" >&2
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
export TORCH_DEVICE="${TORCH_DEVICE:-cpu}"
exec "$ROOT/training/scripts/train_dg5f_grasp.sh" "${extra_args[@]}"
