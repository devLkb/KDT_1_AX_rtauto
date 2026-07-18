#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${DG5F_RUN_ID:-dg5f_stable_grasp_gpu}"
CONFIG="${DG5F_CONFIG:-$ROOT/training/config/dg5f_stable_grasp.yaml}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
EPISODES="${DG5F_EVAL_EPISODES:-200}"
BASE_SEED="${DG5F_EVAL_BASE_SEED:-300000}"
CSV_PATH="${DG5F_EVAL_CSV:-$RESULTS_DIR/$RUN_ID/evaluation_stable.csv}"
APPROVAL_PATH="${DG5F_EVAL_APPROVAL:-$CSV_PATH.approved.json}"
PLAYER="${ENV_PATH:-$ROOT/training/builds/DG5FStableGrasp/DG5FGrasp.x86_64}"
TIMEOUT_SECONDS="${DG5F_EVAL_TIMEOUT_SECONDS:-900}"
TRAINER_PID_FILE="$RESULTS_DIR/$RUN_ID/run_logs/evaluation.trainer.pid"
BEHAVIOR="DG5FStableGrasp"

[[ -f "$RESULTS_DIR/$RUN_ID/$BEHAVIOR/checkpoint.pt" ]] || {
  echo "[ERROR] checkpoint가 없습니다: $RESULTS_DIR/$RUN_ID/$BEHAVIOR/checkpoint.pt" >&2
  exit 2
}
[[ -x "$PLAYER" ]] || {
  echo "[ERROR] Unity player가 없습니다: $PLAYER" >&2
  exit 2
}
[[ "$EPISODES" -eq 200 ]] || {
  echo "[ERROR] 승인은 unseen-seed 200 episode가 필요합니다 (현재 $EPISODES)." >&2
  exit 2
}

mkdir -p "$(dirname "$CSV_PATH")"
rm -f "$CSV_PATH" "$TRAINER_PID_FILE"

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
  [[ -r "$TRAINER_PID_FILE" ]] && read -r pid <"$TRAINER_PID_FILE" || true
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

((completed == 1)) || {
  echo "[ERROR] ${EPISODES}-episode ledger가 완성되지 않았습니다." >&2
  exit 1
}
((trainer_status == 0 || trainer_status == 130)) || exit "$trainer_status"

PYTHON="${VENV:-/root/venvs/ax310}/bin/python"
"$PYTHON" \
  "$ROOT/training/scripts/evaluate_dg5f_stable.py" \
  "$CSV_PATH" \
  --expected-episodes "$EPISODES"

MODEL_PATH="$RESULTS_DIR/$RUN_ID/$BEHAVIOR.onnx"
[[ -f "$MODEL_PATH" ]] || {
  echo "[ERROR] 평가 checkpoint의 canonical ONNX가 없습니다: $MODEL_PATH" >&2
  exit 2
}

"$PYTHON" - "$MODEL_PATH" "$CSV_PATH" "$APPROVAL_PATH" "$RUN_ID" <<'PY'
import hashlib
import json
import os
import sys
from pathlib import Path

model, ledger, approval, run_id = map(Path, sys.argv[1:5])

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

payload = {
    "specVersion": "3.0.0",
    "behaviorName": "DG5FStableGrasp",
    "episodes": 200,
    "runId": str(run_id),
    "modelPath": str(model.resolve()),
    "modelSha256": sha256(model),
    "evaluationCsv": str(ledger.resolve()),
    "evaluationCsvSha256": sha256(ledger),
}
approval.parent.mkdir(parents=True, exist_ok=True)
temporary = approval.with_suffix(approval.suffix + ".tmp")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
os.replace(temporary, approval)
print(f"[PASS] model-bound evaluation approval: {approval}")
PY
