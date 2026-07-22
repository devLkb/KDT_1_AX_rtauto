#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
VENV="${VENV:-${VIRTUAL_ENV:-$ROOT/vision/.vision}}"
CONFIG="${CONFIG:-$ROOT/training/config/dg5f_grasp_point_reach.yaml}"
RESULTS_DIR="${RESULTS_DIR:-$ROOT/training/results}"
RUN_ID="${RUN_ID:-dg5f-grasp-ready-reach}"
ENV_PATH="${ENV_PATH:-}"
NUM_ENVS="${NUM_ENVS:-1}"
TIME_SCALE="${TIME_SCALE:-10}"
TORCH_DEVICE="${TORCH_DEVICE:-cuda}"
UNITY_DISPLAY_MODE="${UNITY_DISPLAY_MODE:-auto}"
XVFB_SCREEN="${XVFB_SCREEN:-640x480x24}"

for argument in "$@"; do
  if [[ "$argument" == "--initialize-from" || "$argument" == --initialize-from=* ]]; then
    echo "[ERROR] DG5FGraspReadyReach must not initialize from another run" >&2
    exit 2
  fi
done

if [[ ! -x "$VENV/bin/python" || ! -x "$VENV/bin/mlagents-learn" ]]; then
  echo "[ERROR] ML-Agents virtual environment not found: $VENV" >&2
  exit 2
fi
if [[ ! -f "$CONFIG" ]]; then
  echo "[ERROR] ML-Agents config not found: $CONFIG" >&2
  exit 2
fi
if ! grep -q '^  DG5FGraspReadyReach:$' "$CONFIG"; then
  echo "[ERROR] config does not define DG5FGraspReadyReach: $CONFIG" >&2
  exit 2
fi

if [[ "$TORCH_DEVICE" == cuda* ]]; then
  "$VENV/bin/python" - <<'PY'
import sys
import torch

if not torch.cuda.is_available():
    sys.exit("[ERROR] PyTorch cannot use CUDA in the selected environment")
print(
    f"[GPU] torch={torch.__version__}, cuda={torch.version.cuda}, "
    f"device={torch.cuda.get_device_name(0)}"
)
PY
fi

args=(
  "$CONFIG"
  --run-id "$RUN_ID"
  --time-scale "$TIME_SCALE"
  --results-dir "$RESULTS_DIR"
  --torch-device "$TORCH_DEVICE"
)
launcher=()
if [[ -n "$ENV_PATH" ]]; then
  if [[ ! -x "$ENV_PATH" ]]; then
    echo "[ERROR] Unity training player is not executable: $ENV_PATH" >&2
    exit 2
  fi
  args+=(--env "$ENV_PATH" --num-envs "$NUM_ENVS")
  case "$UNITY_DISPLAY_MODE" in
    auto)
      if [[ -z "${DISPLAY:-}" ]]; then
        command -v xvfb-run >/dev/null 2>&1 || {
          echo "[ERROR] xvfb-run is required when DISPLAY is unset" >&2
          exit 2
        }
        launcher=(xvfb-run -a -s "-screen 0 $XVFB_SCREEN")
      fi
      ;;
    xvfb)
      command -v xvfb-run >/dev/null 2>&1 || {
        echo "[ERROR] xvfb-run not found" >&2
        exit 2
      }
      launcher=(xvfb-run -a -s "-screen 0 $XVFB_SCREEN")
      ;;
    nographics)
      args+=(--no-graphics)
      ;;
    *)
      echo "[ERROR] UNITY_DISPLAY_MODE must be auto, xvfb, or nographics" >&2
      exit 2
      ;;
  esac
fi

echo "[Config] $CONFIG"
echo "[Run] $RUN_ID"
exec "${launcher[@]}" "$VENV/bin/python" \
  "$ROOT/training/scripts/mlagents_learn_compat.py" "${args[@]}" "$@"
