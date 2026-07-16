#!/usr/bin/env bash
set -euo pipefail

ROOT="${DG5F_ROOT:-/workspace/KDT_1_AX_rtauto}"
VENV="${DG5F_VENV:-/root/venvs/ax310}"
RUN_ID="${DG5F_RUN_ID:-dg5f_v1_gpu_fixed}"
NUM_ENVS="${DG5F_NUM_ENVS:-4}"
TIME_SCALE="${DG5F_TIME_SCALE:-20}"
SESSION="${DG5F_SESSION:-dg5f}"
RESULTS_DIR="$ROOT/training/results"
RUN_DIR="$RESULTS_DIR/$RUN_ID"
PLAYER="$ROOT/training/builds/DG5FGrasp/DG5FGrasp.x86_64"
TRAINER="$ROOT/training/scripts/train_dg5f_grasp.sh"
CONSOLE_LOG="$ROOT/training/logs/${RUN_ID}.console.log"

usage() {
  cat <<EOF
사용법: dg5f <명령>

  status   프로세스, GPU, 최신 학습 지표 한 번 확인
  watch    status를 2초마다 갱신 (종료: Ctrl+C, 학습은 계속됨)
  logs     현재 run의 콘솔/Unity 로그 추적
  resume   tmux에서 checkpoint 학습 재개
  start    tmux에서 새 학습 시작 (새 RUN_ID 권장)
  view     tmux 학습 콘솔에 읽기 전용 접속
  attach   tmux 학습 콘솔에 쓰기 가능 접속
  stop     tmux 학습에 Ctrl+C를 보내 checkpoint 저장 후 종료
  gpu      GPU 상태를 1초마다 확인
  help     이 도움말

기본값: RUN_ID=$RUN_ID, NUM_ENVS=$NUM_ENVS, TIME_SCALE=$TIME_SCALE
변경 예: DG5F_RUN_ID=test DG5F_NUM_ENVS=2 dg5f start
EOF
}

require_files() {
  [[ -x "$VENV/bin/python" ]] || { echo "[ERROR] ax310 없음: $VENV" >&2; exit 2; }
  [[ -x "$PLAYER" ]] || { echo "[ERROR] Unity player 없음: $PLAYER" >&2; exit 2; }
  [[ -x "$TRAINER" ]] || { echo "[ERROR] launcher 없음: $TRAINER" >&2; exit 2; }
}

trainer_pids() {
  pgrep -f "mlagents_learn_compat.py.*--run-id ${RUN_ID}([[:space:]]|$)" || true
}

status() {
  echo "=== DG5F: $RUN_ID ==="
  local pids
  pids="$(trainer_pids)"
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
  if [[ "$mode" == start && -d "$RUN_DIR" ]]; then
    echo "[ERROR] 결과가 이미 있습니다: $RUN_DIR" >&2
    echo "        dg5f resume 또는 새 DG5F_RUN_ID를 사용하세요." >&2
    exit 2
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
    "$TRAINER"
  )
  [[ "$mode" == resume ]] && train+=(--resume)

  local train_cmd inner
  printf -v train_cmd '%q ' "${train[@]}"
  printf -v inner 'cd %q && set -o pipefail && %s 2>&1 | tee -a %q' \
    "$ROOT" "$train_cmd" "$CONSOLE_LOG"
  tmux new-session -d -s "$SESSION" "bash -lc $(printf '%q' "$inner")"
  echo "시작됨: run=$RUN_ID, envs=$NUM_ENVS, time_scale=$TIME_SCALE"
  echo "보기: dg5f view    상태: dg5f watch    중단: dg5f stop"
}

case "${1:-help}" in
  status) status ;;
  watch) watch -n 2 dg5f status ;;
  logs)
    if [[ -f "$CONSOLE_LOG" ]]; then
      tail -F "$CONSOLE_LOG"
    else
      tail -F "$RUN_DIR"/run_logs/Player-*.log
    fi
    ;;
  resume) launch resume ;;
  start) launch start ;;
  view) tmux attach-session -r -t "$SESSION" ;;
  attach) tmux attach-session -t "$SESSION" ;;
  stop)
    if tmux has-session -t "$SESSION" 2>/dev/null; then
      tmux send-keys -t "$SESSION" C-c
      echo "Ctrl+C 전송됨. checkpoint 저장이 끝날 때까지 잠시 기다리세요."
    else
      echo "tmux session '$SESSION'이 없습니다. 기존 일반 터미널 학습은 그 터미널에서 Ctrl+C 하세요."
    fi
    ;;
  gpu) watch -n 1 nvidia-smi ;;
  help|-h|--help) usage ;;
  *) echo "[ERROR] 알 수 없는 명령: $1" >&2; usage; exit 2 ;;
esac
