#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${DG5F_RUN_ID:-dg5f_v2_joint26_gpu_fixed}"
CONFIG="${DG5F_CONFIG:-$ROOT/training/config/dg5f_grasp_v2.yaml}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
EPISODES="${DG5F_EVAL_EPISODES:-200}"
BASE_SEED="${DG5F_EVAL_BASE_SEED:-200000}"
CSV_PATH="${DG5F_EVAL_CSV:-$RESULTS_DIR/$RUN_ID/evaluation_v2.csv}"
PLAYER="${ENV_PATH:-$ROOT/training/builds/DG5FGraspJoint26/DG5FGrasp.x86_64}"
TIMEOUT_SECONDS="${DG5F_EVAL_TIMEOUT_SECONDS:-600}"
TRAINER_PID_FILE="$RESULTS_DIR/$RUN_ID/run_logs/evaluation.trainer.pid"

[[ -f "$RESULTS_DIR/$RUN_ID/DG5FGraspJoint/checkpoint.pt" ]] || {
  echo "[ERROR] checkpoint가 없습니다: $RESULTS_DIR/$RUN_ID/DG5FGraspJoint/checkpoint.pt" >&2
  exit 2
}
[[ -x "$PLAYER" ]] || {
  echo "[ERROR] Unity player가 없습니다: $PLAYER" >&2
  exit 2
}
if [[ "$EPISODES" -ne 200 ]]; then
  echo "[WARN] 승인 평가는 200 episode가 필요합니다 (현재 $EPISODES)." >&2
fi

mkdir -p "$(dirname "$CSV_PATH")"
rm -f "$CSV_PATH"
rm -f "$TRAINER_PID_FILE"

VENV="${VENV:-/root/venvs/ax310}" \
CONFIG="$CONFIG" \
RESULTS_DIR="$RESULTS_DIR" \
RUN_ID="$RUN_ID" \
ENV_PATH="$PLAYER" \
NUM_ENVS=1 \
TIME_SCALE=1 \
DG5F_TRAINER_PID_FILE="$TRAINER_PID_FILE" \
"$ROOT/training/scripts/train_dg5f_grasp.sh" \
  --resume \
  --inference \
  --deterministic \
  --env-args \
  --dg5f-eval-episodes "$EPISODES" \
  --dg5f-eval-base-seed "$BASE_SEED" \
  --dg5f-eval-csv "$CSV_PATH" &
launcher_pid=$!

signal_trainer() {
  local signal="$1" pid=""
  if [[ -r "$TRAINER_PID_FILE" ]]; then
    read -r pid <"$TRAINER_PID_FILE" || true
  fi
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill "-$signal" "$pid" 2>/dev/null || true
  elif kill -0 "$launcher_pid" 2>/dev/null; then
    kill "-$signal" "$launcher_pid" 2>/dev/null || true
  fi
}

cleanup() {
  if kill -0 "$launcher_pid" 2>/dev/null; then
    signal_trainer INT
    wait "$launcher_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

deadline=$((SECONDS + TIMEOUT_SECONDS))
completed=0
while kill -0 "$launcher_pid" 2>/dev/null; do
  if [[ -f "$CSV_PATH" ]]; then
    row_count=$(($(wc -l <"$CSV_PATH") - 1))
    if ((row_count >= EPISODES)); then
      completed=1
      echo "[Eval] $row_count episode 기록 완료. inference trainer를 정상 종료합니다."
      signal_trainer INT
      break
    fi
  fi
  if ((SECONDS >= deadline)); then
    echo "[ERROR] 평가가 ${TIMEOUT_SECONDS}초 안에 완료되지 않았습니다." >&2
    break
  fi
  sleep 1
done

set +e
wait "$launcher_pid"
trainer_status=$?
set -e
trap - EXIT INT TERM

if ((completed == 0)); then
  echo "[ERROR] ${EPISODES}-episode ledger가 완성되기 전에 trainer가 종료되었습니다." >&2
  exit 1
fi
if ((trainer_status != 0 && trainer_status != 130)); then
  echo "[ERROR] inference trainer 종료 코드: $trainer_status" >&2
  exit "$trainer_status"
fi

"${VENV:-/root/venvs/ax310}/bin/python" \
  "$ROOT/training/scripts/evaluate_dg5f_v2.py" \
  "$CSV_PATH" \
  --expected-episodes "$EPISODES"
