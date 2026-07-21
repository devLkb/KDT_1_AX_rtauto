#!/usr/bin/env bash
set -u
TRAIN_SESSION="dg5f-v1-continued"
TRAIN_LOG="/workspace/KDT_1_AX_rtauto/training/logs/dg5f_v1_continued_gpu_20260721.log"
WATCH_LOG="/workspace/KDT_1_AX_rtauto/training/logs/dg5f_v1_continued_gpu_20260721.stop-at-1000000.log"
TARGET=1000000
printf '[%s] watcher started; target_step=%d\n' "$(date -u +%FT%TZ)" "$TARGET" >> "$WATCH_LOG"
while tmux has-session -t "$TRAIN_SESSION" 2>/dev/null; do
  step=$(grep -oE 'Step: [0-9]+' "$TRAIN_LOG" 2>/dev/null | tail -1 | awk '{print $2}')
  step=${step:-0}
  if (( step >= TARGET )); then
    printf '[%s] target reached; observed_step=%d; sending SIGINT via tmux\n' "$(date -u +%FT%TZ)" "$step" >> "$WATCH_LOG"
    tmux send-keys -t "$TRAIN_SESSION" C-c
    for _ in $(seq 1 180); do
      if ! tmux has-session -t "$TRAIN_SESSION" 2>/dev/null; then
        printf '[%s] training stopped cleanly\n' "$(date -u +%FT%TZ)" >> "$WATCH_LOG"
        exit 0
      fi
      sleep 1
    done
    printf '[%s] ERROR: training session still exists 180s after SIGINT\n' "$(date -u +%FT%TZ)" >> "$WATCH_LOG"
    exit 1
  fi
  sleep 2
done
printf '[%s] training ended before target\n' "$(date -u +%FT%TZ)" >> "$WATCH_LOG"
exit 2
