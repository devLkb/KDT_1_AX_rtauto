# -*- coding: utf-8 -*-
"""DG5F 비전 노드: 웹캠 → MediaPipe → 20관절 각도[deg] → One Euro → UDP.

svh/vision_node.py의 단일 카메라 경로를 DG5F 20채널용으로 개조.
  - 패킷(v3): float32 × 25, little-endian ('<25f')
    [0..19] DG5F 관절각[deg] / [20..22] 엄지끝 정규화좌표 / [23] 핀치 플래그
    / [24] 엄지-검지 끝거리 비율(연속) — Unity 핀치 연속 블렌딩용
  - 포트: 5006 (SVH 5005와 공존 — 동시 실행 가능)
  - 로그: 실행마다 새 CSV (logs/vision_dg5f_YYYYMMDD_HHMM.csv) — 덮어쓰기 함정 방지
  - occlusion 시 마지막 유효값 hold (SVH와 동일 정책)

사용: python vision_node_dg5f.py [left|right]   (기본 right, 종료: 미리보기 창에서 q)
      왼손 모델 구동 시 'left' — 미러 채널 부호 반전 적용. 웹캠에도 왼손을 보여줄 것.
"""
import os
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp
import numpy as np

from one_euro_filter import OneEuroFilter
from dg5f_angles import (compute_raw, map_to_dg5f, compute_thumb_tip,
                         compute_finger_tips, CHANNEL_NAMES, PINCH_ON, PINCH_OFF,
                         PACKET_FMT, TIP_FINGERS)
from dg5f_paths import unique_log_path

# ------------------------- 설정 -------------------------
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
UNITY_IP = "127.0.0.1"
UNITY_PORT = 5006             # ⚠️ SVH(5005)와 다른 포트 — 공존용
SEND_HZ_CAP = 120
LOG_EVERY_SEC = 0.5
# 경로 규칙은 dg5f_paths가 소유 — 초 단위 + 중복 시 접미사라 덮어쓰기 불가
LOG_CSV = unique_log_path("vision_dg5f")
# One Euro: 값 단위가 deg(0~115)라 SVH(rad) 대비 beta를 1/57 스케일로 낮춤.
# min_cutoff 1.0→0.6 (2026-07-13 라이브: 엄지 대향 지터 1.1°/프레임 → 저속 지터 억제 강화)
FILTER_FREQ, FILTER_MIN_CUTOFF, FILTER_BETA = 30.0, 0.6, 0.0005
# 엄지 tip 위치(정규화 0~1) 전용 — §25-4(2026-07-15): 정지 중 노이즈가 초당 0.2유닛(2.6cm)
# 저속 드리프트라 min_cutoff 0.6Hz는 전부 통과(입력 2.6=필터후 2.6cm 실측), beta 0.001은
# 속도보상 무력. → min_cutoff 0.15(드리프트 차단) + beta 0.5(의도 동작 속도 1~3유닛/s에서
# 컷오프를 0.6~1.7Hz로 열어 지연 방지). 떨리면 min_cutoff↓, 굼뜨면 beta↑ 순으로 튜닝.
TIP_MIN_CUTOFF, TIP_BETA = 0.15, 0.5
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
    tip_filters = [OneEuroFilter(freq=FILTER_FREQ, min_cutoff=TIP_MIN_CUTOFF,
                                 beta=TIP_BETA) for _ in range(3)]
    # 손가락 리치벡터(v4 [25..36]) — 엄지 tip과 같은 노이즈 성격(정규화 위치)이라 §25-4
    # 튜닝값(TIP_MIN_CUTOFF/TIP_BETA)을 그대로 쓴다. 각도 채널 필터와 섞지 말 것(단위 다름).
    finger_tip_filters = [OneEuroFilter(freq=FILTER_FREQ, min_cutoff=TIP_MIN_CUTOFF,
                                        beta=TIP_BETA) for _ in range(3 * len(TIP_FINGERS))]
    pinch_filter = OneEuroFilter(freq=FILTER_FREQ, min_cutoff=FILTER_MIN_CUTOFF,
                                 beta=0.001)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_send = last_log = 0.0
    last_valid = None
    pinch_on = False  # 핀치 히스테리시스 상태 (걸림 <PINCH_ON, 풀림 >PINCH_OFF)

    # unique_log_path가 logs/ 생성 + 중복 회피까지 끝냄 (덮어쓰기 구조적 불가)
    log_f = open(LOG_CSV, "w", encoding="utf-8")
    # 리치벡터·핀치도 기록 — 없으면 Unity 쪽 thumbik/fingerik CSV가 사라지는 순간 그 세션의
    # 손끝 분석이 영영 불가능해진다(2026-07-16 실제로 라이브 로그를 잃고 재분석 못 함).
    # 컬럼은 **뒤에만 추가** — analyze_teleop이 이름으로 읽어(v[f"filt_{name}"]) 위치 무관하지만
    # 앞에 끼우면 사람이 옛 로그와 눈으로 비교할 때 헷갈린다.
    tip_cols = [f"tip_{a}" for a in "xyz"] + ["pinch_on", "pinch_d"]
    for name, _ in TIP_FINGERS:
        tip_cols += [f"{name}_{a}" for a in "xyz"]
    log_f.write(",".join(["t_unix", "detected"]
                         + [f"raw_{n}" for n in CHANNEL_NAMES]
                         + [f"filt_{n}" for n in CHANNEL_NAMES]
                         + tip_cols) + "\n")

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
            ftips = compute_finger_tips(xyz)                 # v4: 손가락 리치벡터 4×3
            tip_f = [f(v) for f, v in zip(tip_filters, tip)]
            ftips_f = [f(v) for f, v in zip(finger_tip_filters, ftips)]
            vals = ([filters[n](v) for n, v in zip(CHANNEL_NAMES, mapped)]
                    + tip_f + [1.0 if pinch_on else 0.0]
                    + [pinch_filter(pinch_d)]
                    + ftips_f)
            last_valid = vals
        elif last_valid is not None:
            vals = last_valid                                # occlusion hold
        else:
            vals = None

        if vals is not None:
            r = mapped if mapped is not None else vals[:20]
            # 각도는 deg라 .3f로 충분, 리치벡터는 0~1 정규화라 .4f (Unity CSV의 F4와 맞춤)
            log_f.write(f"{now:.3f},{1 if mapped is not None else 0},"
                        + ",".join(f"{v:.3f}" for v in r) + ","
                        + ",".join(f"{v:.3f}" for v in vals[:20]) + ","
                        + ",".join(f"{v:.4f}" for v in vals[20:]) + "\n")
            if (now - last_send) >= (1.0 / SEND_HZ_CAP):
                sock.sendto(struct.pack(PACKET_FMT, *vals), (UNITY_IP, UNITY_PORT))
                last_send = now
                if now - last_log >= LOG_EVERY_SEC:
                    # |thumb|·|idx| = 펴짐 비율 — 쭉 폈을 때 1.0 근처여야 한다(§26 판정선).
                    # 못 미치면 해당 직진도 상한(DEFAULT_THUMB/FINGER_STRAIGHT)을 낮출 것.
                    print("[send]", " ".join(f"{v:5.1f}" for v in vals[:4]),
                          f"| tip ({vals[20]:.2f},{vals[21]:.2f},{vals[22]:.2f})"
                          f" pinch={vals[23]:.0f} d={vals[24]:.2f}"
                          f" | |thumb|={np.linalg.norm(vals[20:23]):.2f}"
                          f" |idx|={np.linalg.norm(vals[25:28]):.2f}")
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
