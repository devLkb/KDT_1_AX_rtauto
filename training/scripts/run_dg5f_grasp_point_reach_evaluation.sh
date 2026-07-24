#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUN_ID="${DG5F_RUN_ID:-dg5f-grasp-ready-reach}"
CONFIG="${DG5F_CONFIG:-$ROOT/training/config/dg5f_grasp_point_reach.yaml}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
EPISODES="${DG5F_EVAL_EPISODES:-500}"
BASE_SEED="${DG5F_EVAL_BASE_SEED:-500000}"
CSV_PATH="${DG5F_EVAL_CSV:-$RESULTS_DIR/$RUN_ID/evaluation.csv}"
APPROVAL_PATH="${DG5F_EVAL_APPROVAL:-$CSV_PATH.approved.json}"
PLAYER="${ENV_PATH:-$ROOT/training/builds/DG5FGraspReadyReach/DG5FGraspReadyReach.x86_64}"
TIMEOUT_SECONDS="${DG5F_EVAL_TIMEOUT_SECONDS:-1200}"
TRAINER_PID_FILE="$RESULTS_DIR/$RUN_ID/run_logs/evaluation.trainer.pid"
BEHAVIOR="DG5FGraspReadyReach"
VENV="${VENV:-${VIRTUAL_ENV:-$ROOT/vision/.vision}}"
PYTHON="$VENV/bin/python"

[[ "$EPISODES" =~ ^[0-9]+$ && "$EPISODES" -eq 500 ]] || {
  echo "[ERROR] model approval requires exactly 500 evaluation episodes" >&2
  exit 2
}
[[ "$BASE_SEED" =~ ^[0-9]+$ ]] || {
  echo "[ERROR] DG5F_EVAL_BASE_SEED must be a non-negative integer" >&2
  exit 2
}
[[ -f "$RESULTS_DIR/$RUN_ID/$BEHAVIOR/checkpoint.pt" ]] || {
  echo "[ERROR] checkpoint not found: $RESULTS_DIR/$RUN_ID/$BEHAVIOR/checkpoint.pt" >&2
  exit 2
}
[[ -x "$PLAYER" ]] || {
  echo "[ERROR] Unity evaluation player not found: $PLAYER" >&2
  exit 2
}
[[ -x "$PYTHON" ]] || {
  echo "[ERROR] Python executable not found: $PYTHON" >&2
  exit 2
}

mkdir -p "$(dirname "$CSV_PATH")"
rm -f "$CSV_PATH" "$TRAINER_PID_FILE"

VENV="$VENV" \
CONFIG="$CONFIG" \
RESULTS_DIR="$RESULTS_DIR" \
RUN_ID="$RUN_ID" \
ENV_PATH="$PLAYER" \
NUM_ENVS=1 \
TIME_SCALE=1 \
DG5F_TRAINER_PID_FILE="$TRAINER_PID_FILE" \
"$ROOT/training/scripts/train_dg5f_grasp_point_reach.sh" \
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
    echo "[ERROR] evaluation did not finish in ${TIMEOUT_SECONDS}s" >&2
    break
  fi
  sleep 1
done

set +e
wait "$launcher_pid"
trainer_status=$?
set -e
trap - EXIT INT TERM

if [[ -f "$CSV_PATH" ]]; then
  row_count=$(($(wc -l <"$CSV_PATH") - 1))
  ((row_count >= EPISODES)) && completed=1
fi
((completed == 1)) || {
  echo "[ERROR] the ${EPISODES}-episode evaluation ledger is incomplete" >&2
  exit 1
}
((trainer_status == 0 || trainer_status == 130)) || exit "$trainer_status"

"$PYTHON" "$ROOT/training/scripts/evaluate_dg5f_grasp_point_reach.py" \
  "$CSV_PATH" \
  --expected-episodes "$EPISODES" \
  --expected-base-seed "$BASE_SEED" \
  --minimum-success-rate 0.90

MODEL_PATH="$RESULTS_DIR/$RUN_ID/$BEHAVIOR.onnx"
[[ -f "$MODEL_PATH" ]] || {
  echo "[ERROR] canonical ONNX not found: $MODEL_PATH" >&2
  exit 2
}

"$PYTHON" - "$MODEL_PATH" "$CSV_PATH" "$APPROVAL_PATH" "$RUN_ID" "$BASE_SEED" <<'PY'
import hashlib
import json
import os
import sys
from pathlib import Path

model, ledger, approval = map(Path, sys.argv[1:4])
run_id = sys.argv[4]
base_seed = int(sys.argv[5])

def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()

payload = {
    "specVersion": "2.0.0",
    "behaviorName": "DG5FGraspReadyReach",
    "episodes": 500,
    "baseSeed": base_seed,
    "minimumSuccessRate": 0.90,
    "maximumDistanceMeters": 0.01,
    "maximumSpeedMetersPerSecond": 0.05,
    "minimumHoldSeconds": 0.25,
    "minimumPalmAlignment": 0.965925826,
    "minimumUpperConeAlignment": 0.707106781,
    "minimumTransitClearanceMeters": 0.10,
    "maximumEpisodeSeconds": 20.0,
    "runId": run_id,
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
