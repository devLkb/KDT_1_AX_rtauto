#!/usr/bin/env bash
set -euo pipefail

ROOT="${DG5F_ROOT:-/workspace/KDT_1_AX_rtauto}"
VENV="${DG5F_VENV:-/root/venvs/ax310}"
FROZEN_V1_RUN_ID="dg5f_v1_gpu_fixed"

# Preferred CLI: dg5f v2 status / dg5f v2 resume / dg5f v3 init.
# Keep the old environment-variable-only form compatible for existing jobs.
STAGE=""
if [[ "${1:-}" =~ ^v[1-4]$ ]]; then
  STAGE="$1"
  shift
fi

case "$STAGE" in
  v1)
    STAGE_RUN_ID="dg5f_v1_gpu_fixed"
    STAGE_CONFIG="$ROOT/training/config/dg5f_grasp.yaml"
    STAGE_PLAYER="$ROOT/training/builds/DG5FGrasp/DG5FGrasp.x86_64"
    STAGE_SESSION="dg5f_v1"
    PREVIOUS_RUN_ID=""
    ;;
  v2)
    STAGE_RUN_ID="dg5f_v2_gpu_fixed"
    STAGE_CONFIG="$ROOT/training/config/dg5f_grasp_v2.yaml"
    STAGE_PLAYER="$ROOT/training/builds/DG5FGraspV2/DG5FGrasp.x86_64"
    STAGE_SESSION="dg5f_v2"
    PREVIOUS_RUN_ID="dg5f_v1_gpu_fixed"
    ;;
  v3)
    STAGE_RUN_ID="dg5f_v3_gpu_fixed"
    STAGE_CONFIG="$ROOT/training/config/dg5f_grasp_v3.yaml"
    STAGE_PLAYER="$ROOT/training/builds/DG5FGraspV3/DG5FGrasp.x86_64"
    STAGE_SESSION="dg5f_v3"
    PREVIOUS_RUN_ID="dg5f_v2_gpu_fixed"
    ;;
  v4)
    STAGE_RUN_ID="dg5f_v4_gpu_fixed"
    STAGE_CONFIG="$ROOT/training/config/dg5f_grasp_v4.yaml"
    STAGE_PLAYER="$ROOT/training/builds/DG5FGraspV4/DG5FGrasp.x86_64"
    STAGE_SESSION="dg5f_v4"
    PREVIOUS_RUN_ID="dg5f_v3_gpu_fixed"
    ;;
  "")
    STAGE_RUN_ID="dg5f_v1_gpu_fixed"
    STAGE_CONFIG="$ROOT/training/config/dg5f_grasp.yaml"
    STAGE_PLAYER=""
    STAGE_SESSION="dg5f"
    PREVIOUS_RUN_ID=""
    ;;
esac

NUM_ENVS="${DG5F_NUM_ENVS:-4}"
TIME_SCALE="${DG5F_TIME_SCALE:-20}"
if [[ -n "$STAGE" ]]; then
  # An explicit stage is deterministic and cannot accidentally inherit a
  # stale exported DG5F_RUN_ID/DG5F_CONFIG from another stage.
  RUN_ID="$STAGE_RUN_ID"
  CONFIG="$STAGE_CONFIG"
  SESSION="$STAGE_SESSION"
else
  RUN_ID="${DG5F_RUN_ID:-$STAGE_RUN_ID}"
  CONFIG="${DG5F_CONFIG:-$STAGE_CONFIG}"
  SESSION="${DG5F_SESSION:-$STAGE_SESSION}"
fi
[[ "$CONFIG" == /* ]] || CONFIG="$ROOT/$CONFIG"
RESULTS_DIR="$ROOT/training/results"
RUN_DIR="$RESULTS_DIR/$RUN_ID"
DEFAULT_PLAYER="$ROOT/training/builds/DG5FGrasp/DG5FGrasp.x86_64"
if [[ -n "$STAGE_PLAYER" ]]; then
  DEFAULT_PLAYER="$STAGE_PLAYER"
elif [[ "$CONFIG" == *"/dg5f_grasp_v2.yaml" ]]; then
  DEFAULT_PLAYER="$ROOT/training/builds/DG5FGraspV2/DG5FGrasp.x86_64"
fi
if [[ -n "$STAGE" ]]; then
  PLAYER="$DEFAULT_PLAYER"
else
  PLAYER="${DG5F_PLAYER:-$DEFAULT_PLAYER}"
fi
[[ "$PLAYER" == /* ]] || PLAYER="$ROOT/$PLAYER"
TRAINER="$ROOT/training/scripts/train_dg5f_grasp.sh"
CONSOLE_LOG="$ROOT/training/logs/${RUN_ID}.console.log"
TRAINER_PID_FILE="$ROOT/training/logs/${RUN_ID}.trainer.pid"
STOP_TIMEOUT="${DG5F_STOP_TIMEOUT:-90}"
TERM_TIMEOUT="${DG5F_TERM_TIMEOUT:-10}"

usage() {
  cat <<EOF
사용법: dg5f <단계> <명령>

  단계     v1 | v2 | v3 | v4
  status   프로세스, GPU, 최신 학습 지표 한 번 확인
  watch    status를 2초마다 갱신 (종료: Ctrl+C, 학습은 계속됨)
  logs     현재 run의 콘솔/Unity 로그 추적
  resume   tmux에서 checkpoint 학습 재개
  start    tmux에서 새 학습 시작 (새 RUN_ID 권장)
  init     이전 단계 checkpoint로 새 run 전이학습 시작
  init ID  지정한 ID의 checkpoint로 새 run 전이학습 시작
  view     tmux 학습 콘솔에 읽기 전용 접속
  attach   tmux 학습 콘솔에 쓰기 가능 접속
  stop     tmux 학습에 Ctrl+C를 보내 checkpoint 저장 후 종료
  gpu      GPU 상태를 1초마다 확인
  help     이 도움말

현재 선택: STAGE=${STAGE:-legacy}, RUN_ID=$RUN_ID, SESSION=$SESSION
기본값: NUM_ENVS=$NUM_ENVS, TIME_SCALE=$TIME_SCALE
설정: CONFIG=$CONFIG

예:
  dg5f v2 status
  dg5f v2 watch
  dg5f v2 resume
  dg5f v3 init       # dg5f_v2_gpu_fixed에서 자동 전이
  dg5f v4 stop

커스텀 실험은 단계 접두사 없이 DG5F_RUN_ID, DG5F_CONFIG, DG5F_PLAYER,
DG5F_SESSION을 지정하는 기존 형식을 사용합니다.
EOF
}

require_files() {
  [[ -x "$VENV/bin/python" ]] || { echo "[ERROR] ax310 없음: $VENV" >&2; exit 2; }
  [[ -x "$PLAYER" ]] || { echo "[ERROR] Unity player 없음: $PLAYER" >&2; exit 2; }
  [[ -x "$TRAINER" ]] || { echo "[ERROR] launcher 없음: $TRAINER" >&2; exit 2; }
  [[ -f "$CONFIG" ]] || { echo "[ERROR] config 없음: $CONFIG" >&2; exit 2; }
}

# Read process arguments from /proc instead of matching a pgrep regular
# expression. This avoids partial RUN_ID matches and also lets stop distinguish
# the trainer from its forked ML-Agents environment workers (which inherit the
# same command line).
trainer_pids() {
  local proc pid i
  local -a argv=()
  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    argv=()
    mapfile -d '' -t argv 2>/dev/null <"$proc/cmdline" || continue
    # xvfb-run also contains the Python command in its arguments; argv[0:2]
    # must be the actual interpreter and compatibility script.
    [[ "${argv[0]:-}" == "$VENV/bin/python" ]] || continue
    [[ "${argv[1]:-}" == "$ROOT/training/scripts/mlagents_learn_compat.py" ]] || continue
    for ((i = 2; i < ${#argv[@]} - 1; i++)); do
      if [[ "${argv[i]}" == "--run-id" && "${argv[i + 1]}" == "$RUN_ID" ]]; then
        echo "$pid"
        break
      fi
    done
  done
}

trainer_main_pids() {
  local pid ppid all

  # dg5f launches write the authoritative main PID before ML-Agents starts.
  # Validate the file against the current run so a stale/reused PID is harmless.
  if [[ -r "$TRAINER_PID_FILE" ]]; then
    read -r pid <"$TRAINER_PID_FILE" || true
    if [[ "$pid" =~ ^[0-9]+$ ]] && grep -qx "$pid" < <(trainer_pids); then
      echo "$pid"
      return
    fi
  fi

  # Fallback for a trainer started before PID-file support. Forked environment
  # workers have another matching trainer process as their parent.
  all="$(trainer_pids)"
  for pid in $all; do
    ppid="$(awk '/^PPid:/{print $2}' "/proc/$pid/status" 2>/dev/null || true)"
    if ! grep -qw "$ppid" <<<"$all"; then
      echo "$pid"
    fi
  done
}

unity_pids() {
  local proc pid i log_path
  local -a argv=()
  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    argv=()
    mapfile -d '' -t argv 2>/dev/null <"$proc/cmdline" || continue
    [[ "${argv[0]:-}" == "$PLAYER" ]] || continue
    for ((i = 1; i < ${#argv[@]} - 1; i++)); do
      if [[ "${argv[i],,}" == "-logfile" ]]; then
        log_path="${argv[i + 1]}"
        if [[ "$log_path" == "$RUN_DIR/run_logs/Player-"*.log ]]; then
          echo "$pid"
          break
        fi
      fi
    done
  done
}

wait_for_exit() {
  local finder="$1" timeout="$2" started="$SECONDS"
  while [[ -n "$("$finder")" ]]; do
    ((SECONDS - started >= timeout)) && return 1
    sleep 1
  done
}

wait_for_tmux_exit() {
  local timeout="$1" started="$SECONDS"
  while tmux has-session -t "$SESSION" 2>/dev/null; do
    ((SECONDS - started >= timeout)) && return 1
    sleep 1
  done
}

signal_pids() {
  local signal="$1"
  shift
  (($# > 0)) || return 0
  kill "-$signal" "$@" 2>/dev/null || true
}

stop_training() {
  local pids
  pids="$(trainer_main_pids)"

  # During the short CUDA/Xvfb setup phase there is not a trainer PID yet.
  # Give it a chance to appear so we can signal only the main Python process,
  # rather than broadcasting Ctrl+C to all forked environment workers.
  if [[ -z "$pids" ]] && tmux has-session -t "$SESSION" 2>/dev/null; then
    local started="$SECONDS"
    while ((SECONDS - started < 5)); do
      pids="$(trainer_main_pids)"
      [[ -n "$pids" ]] && break
      tmux has-session -t "$SESSION" 2>/dev/null || break
      sleep 1
    done
  fi

  if [[ -n "$pids" ]]; then
    echo "trainer PID $(tr '\n' ' ' <<<"$pids")에 SIGINT 전송. checkpoint 저장을 기다립니다."
    # Signal only the trainer. Sending terminal Ctrl+C broadcasts SIGINT to
    # workers while UnityEnvironment() is still being constructed and can
    # orphan partially launched Unity players.
    # shellcheck disable=SC2086
    signal_pids INT $pids
  elif tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux send-keys -t "$SESSION" C-c
    echo "trainer 시작 전 launcher에 Ctrl+C를 전송했습니다."
  fi

  if ! wait_for_exit trainer_pids "$STOP_TIMEOUT"; then
    pids="$(trainer_pids)"
    echo "[WARN] ${STOP_TIMEOUT}초 안에 종료되지 않아 trainer/worker에 SIGTERM을 보냅니다." >&2
    # shellcheck disable=SC2086
    signal_pids TERM $pids
    if ! wait_for_exit trainer_pids "$TERM_TIMEOUT"; then
      pids="$(trainer_pids)"
      echo "[WARN] trainer/worker 강제 종료(SIGKILL)." >&2
      # shellcheck disable=SC2086
      signal_pids KILL $pids
      wait_for_exit trainer_pids 5 || true
    fi
  fi

  # Clean up players left by an interrupted startup (including leftovers from
  # older dg5f versions). Restrict the match to this RUN_ID's Unity log path.
  pids="$(unity_pids)"
  if [[ -n "$pids" ]]; then
    echo "남은 Unity player PID $(tr '\n' ' ' <<<"$pids") 종료 중..."
    # shellcheck disable=SC2086
    signal_pids TERM $pids
    if ! wait_for_exit unity_pids "$TERM_TIMEOUT"; then
      pids="$(unity_pids)"
      # shellcheck disable=SC2086
      signal_pids KILL $pids
      wait_for_exit unity_pids 5 || true
    fi
  fi

  # Let xvfb-run execute its EXIT trap before removing the tmux session.
  # Killing tmux as soon as Python disappears races that trap and leaves Xvfb
  # behind even though trainer and Unity are already gone.
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    if ! wait_for_tmux_exit "$TERM_TIMEOUT"; then
      echo "[WARN] tmux session 정리가 지연되어 강제 종료합니다." >&2
      tmux kill-session -t "$SESSION"
    fi
  fi
  rm -f "$TRAINER_PID_FILE"

  if [[ -z "$(trainer_pids)" && -z "$(unity_pids)" ]]; then
    echo "중지 완료: trainer와 Unity player가 모두 종료되었습니다."
  else
    echo "[ERROR] 일부 프로세스가 아직 실행 중입니다. dg5f status로 확인하세요." >&2
    return 1
  fi
}

status() {
  echo "=== DG5F: $RUN_ID ==="
  local pids
  pids="$(trainer_main_pids)"
  if [[ -n "$pids" ]]; then
    echo "상태: 실행 중 (trainer PID: $(tr '\n' ' ' <<<"$pids"))"
  else
    echo "상태: 중지됨"
  fi

  echo
  echo "--- process ---"
  ps -eo pid,stat,etime,%cpu,%mem,cmd \
    | grep -E "mlagents_learn_compat.py|DG5FGrasp.x86_64" \
    | grep -v grep \
    | sed -E 's/[[:space:]]+/ /g' || true

  echo
  echo "--- GPU process ---"
  nvidia-smi --query-compute-apps=pid,name,used_gpu_memory \
    --format=csv,noheader 2>/dev/null || true

  echo
  echo "--- metrics ---"
  "$VENV/bin/python" "$ROOT/training/scripts/dg5f_status.py" "$RUN_DIR"
}

launch() {
  local mode="$1"
  local source_run_id="${2:-}"
  if [[ "$mode" == init && -z "$source_run_id" ]]; then
    echo "[ERROR] 사용법: dg5f init <source_run_id>" >&2
    exit 2
  fi
  if [[ "$mode" == resume && "$RUN_ID" == "$FROZEN_V1_RUN_ID" ]]; then
    echo "[ERROR] $FROZEN_V1_RUN_ID 은 526647-step 전이 원본으로 동결되어 resume할 수 없습니다." >&2
    echo "        DG5F_RUN_ID를 v2 run으로 지정하세요." >&2
    exit 2
  fi
  require_files
  command -v tmux >/dev/null 2>&1 || {
    echo "[ERROR] tmux가 없습니다: apt-get install -y tmux" >&2
    exit 2
  }
  if [[ -n "$(trainer_pids)" ]]; then
    echo "[ERROR] $RUN_ID 학습이 이미 실행 중입니다." >&2
    exit 2
  fi
  if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[ERROR] tmux session '$SESSION'이 이미 있습니다." >&2
    exit 2
  fi
  if [[ "$mode" != resume && -d "$RUN_DIR" ]]; then
    echo "[ERROR] 결과가 이미 있습니다: $RUN_DIR" >&2
    echo "        dg5f resume 또는 새 DG5F_RUN_ID를 사용하세요." >&2
    exit 2
  fi
  if [[ "$mode" == init ]]; then
    [[ "$source_run_id" != "$RUN_ID" ]] || {
      echo "[ERROR] 전이 원본과 새 RUN_ID가 같을 수 없습니다: $RUN_ID" >&2
      exit 2
    }
    local source_checkpoint="$RESULTS_DIR/$source_run_id/DG5FGrasp/checkpoint.pt"
    [[ -f "$source_checkpoint" ]] || {
      echo "[ERROR] 전이 원본 checkpoint가 없습니다: $source_checkpoint" >&2
      exit 2
    }
  fi

  mkdir -p "$ROOT/training/logs"
  local -a train=(
    env
    "VENV=$VENV"
    "ENV_PATH=$PLAYER"
    "RESULTS_DIR=$RESULTS_DIR"
    "RUN_ID=$RUN_ID"
    "NUM_ENVS=$NUM_ENVS"
    "TIME_SCALE=$TIME_SCALE"
    "CONFIG=$CONFIG"
    "DG5F_TRAINER_PID_FILE=$TRAINER_PID_FILE"
    "$TRAINER"
  )
  [[ "$mode" == resume ]] && train+=(--resume)
  [[ "$mode" == init ]] && train+=(--initialize-from "$source_run_id")

  local train_cmd inner
  printf -v train_cmd '%q ' "${train[@]}"
  printf -v inner 'cd %q && set -o pipefail && %s 2>&1 | tee -a %q' \
    "$ROOT" "$train_cmd" "$CONSOLE_LOG"
  tmux new-session -d -s "$SESSION" "bash -lc $(printf '%q' "$inner")"
  echo "시작됨: run=$RUN_ID, mode=$mode, config=$CONFIG, envs=$NUM_ENVS, time_scale=$TIME_SCALE"
  [[ "$mode" == init ]] && echo "전이 원본: $source_run_id"
  if [[ -n "$STAGE" ]]; then
    echo "보기: dg5f $STAGE view    상태: dg5f $STAGE watch    중단: dg5f $STAGE stop"
  else
    echo "보기: dg5f view    상태: dg5f watch    중단: dg5f stop"
  fi
}

case "${1:-help}" in
  status) status ;;
  watch)
    if [[ -n "$STAGE" ]]; then
      watch -n 2 dg5f "$STAGE" status
    else
      watch -n 2 dg5f status
    fi
    ;;
  logs)
    if [[ -f "$CONSOLE_LOG" ]]; then
      tail -F "$CONSOLE_LOG"
    else
      tail -F "$RUN_DIR"/run_logs/Player-*.log
    fi
    ;;
  resume) launch resume ;;
  start) launch start ;;
  init) launch init "${2:-$PREVIOUS_RUN_ID}" ;;
  view) tmux attach-session -r -t "$SESSION" ;;
  attach) tmux attach-session -t "$SESSION" ;;
  stop) stop_training ;;
  gpu) watch -n 1 nvidia-smi ;;
  help|-h|--help) usage ;;
  *) echo "[ERROR] 알 수 없는 명령: $1" >&2; usage; exit 2 ;;
esac
