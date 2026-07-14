# -*- coding: utf-8 -*-
"""DG5F 비전 노드: 웹캠 → MediaPipe → 20관절 각도[deg] → One Euro → UDP.

svh/vision_node.py의 단일 카메라 경로를 DG5F 20채널용으로 개조.
  - 패킷(v3): float32 × 25, little-endian ('<25f')
    [0..19] DG5F 관절각[deg] / [20..22] 엄지끝 정규화좌표 / [23] 핀치 플래그
    / [24] 엄지-검지 끝거리 비율(연속) — Unity 핀치 연속 블렌딩용
  - 포트: 5006 (SVH 5005와 공존 — 동시 실행 가능)
  - 로그: 실행마다 새 CSV (vision_dg5f_YYYYMMDD_HHMM.csv) — 덮어쓰기 함정 방지
  - occlusion 시 마지막 유효값 hold (SVH와 동일 정책)

사용: python vision_node_dg5f.py [left|right]   (기본 right, 종료: 미리보기 창에서 q)
      왼손 모델 구동 시 'left' — 미러 채널 부호 반전 적용. 웹캠에도 왼손을 보여줄 것.
"""
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

from one_euro_filter import OneEuroFilter
from dg5f_angles import (compute_raw, map_to_dg5f, compute_thumb_tip,
                         CHANNEL_NAMES, PINCH_ON, PINCH_OFF)

# ------------------------- 설정 -------------------------
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
UNITY_IP = "127.0.0.1"
UNITY_PORT = 5006             # ⚠️ SVH(5005)와 다른 포트 — 공존용
SEND_HZ_CAP = 120
LOG_EVERY_SEC = 0.5
LOG_CSV = time.strftime("vision_dg5f_%Y%m%d_%H%M.csv")
# One Euro: 값 단위가 deg(0~115)라 SVH(rad) 대비 beta를 1/57 스케일로 낮춤.
# min_cutoff 1.0→0.6 (2026-07-13 라이브: 엄지 대향 지터 1.1°/프레임 → 저속 지터 억제 강화)
FILTER_FREQ, FILTER_MIN_CUTOFF, FILTER_BETA = 30.0, 0.6, 0.0005
# --------------------------------------------------------


def landmarks_to_xyz(hand_landmarks):
    pts = np.zeros((21, 3), dtype=np.float64)
    for i, lm in enumerate(hand_landmarks.landmark):
        pts[i] = (lm.x, lm.y, lm.z)
    return pts


def main():
    hand = sys.argv[1].lower() if len(sys.argv) > 1 else "right"
    if hand not in ("right", "left"):
        print(f"[오류] 인자는 left/right만 가능: {hand}")
        return
    hands = mp.solutions.hands.Hands(
        model_complexity=1, max_num_hands=1,
        min_detection_confidence=0.6, min_tracking_confidence=0.6)

    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)

    filters = {n: OneEuroFilter(freq=FILTER_FREQ, min_cutoff=FILTER_MIN_CUTOFF,
                                beta=FILTER_BETA) for n in CHANNEL_NAMES}
    tip_filters = [OneEuroFilter(freq=FILTER_FREQ, min_cutoff=FILTER_MIN_CUTOFF,
                                 beta=0.001) for _ in range(3)]
    pinch_filter = OneEuroFilter(freq=FILTER_FREQ, min_cutoff=FILTER_MIN_CUTOFF,
                                 beta=0.001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_send = last_log = 0.0
    last_valid = None
    pinch_on = False  # 핀치 히스테리시스 상태 (걸림 <PINCH_ON, 풀림 >PINCH_OFF)

    log_f = open(LOG_CSV, "w", encoding="utf-8")
    log_f.write(",".join(["t_unix", "detected"]
                         + [f"raw_{n}" for n in CHANNEL_NAMES]
                         + [f"filt_{n}" for n in CHANNEL_NAMES]) + "\n")

    print(f"[시작] DG5F vision (hand={hand}) → {UNITY_IP}:{UNITY_PORT} (종료: q)")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)  # 거울 모드(보기 편의, 각도 계산 무관)
        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        now = time.time()
        mapped = None
        if res.multi_hand_landmarks:
            mp.solutions.drawing_utils.draw_landmarks(
                frame, res.multi_hand_landmarks[0],
                mp.solutions.hands.HAND_CONNECTIONS)
            xyz = landmarks_to_xyz(res.multi_hand_landmarks[0])
            mapped = map_to_dg5f(compute_raw(xyz), hand)    # deg
            tip, pinch_d = compute_thumb_tip(xyz)           # v2: 엄지끝 위치(정규화)+끝거리
            if pinch_on:
                pinch_on = pinch_d < PINCH_OFF   # 풀림은 더 멀어져야 (히스테리시스)
            else:
                pinch_on = pinch_d < PINCH_ON
            tip_f = [f(v) for f, v in zip(tip_filters, tip)]
            vals = ([filters[n](v) for n, v in zip(CHANNEL_NAMES, mapped)]
                    + tip_f + [1.0 if pinch_on else 0.0]
                    + [pinch_filter(pinch_d)])
            last_valid = vals
        elif last_valid is not None:
            vals = last_valid                                # occlusion hold
        else:
            vals = None

        if vals is not None:
            r = mapped if mapped is not None else vals[:20]
            log_f.write(f"{now:.3f},{1 if mapped is not None else 0},"
                        + ",".join(f"{v:.3f}" for v in r) + ","
                        + ",".join(f"{v:.3f}" for v in vals[:20]) + "\n")
            if (now - last_send) >= (1.0 / SEND_HZ_CAP):
                sock.sendto(struct.pack("<25f", *vals), (UNITY_IP, UNITY_PORT))
                last_send = now
                if now - last_log >= LOG_EVERY_SEC:
                    print("[send]", " ".join(f"{v:5.1f}" for v in vals[:4]),
                          f"| tip ({vals[20]:.2f},{vals[21]:.2f},{vals[22]:.2f})"
                          f" pinch={vals[23]:.0f} d={vals[24]:.2f}")
                    last_log = now

        cv2.imshow("dg5f vision (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()
    log_f.close()
    print(f"[종료] 로그: {LOG_CSV}")


if __name__ == "__main__":
    main()
