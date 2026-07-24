# -*- coding: utf-8 -*-
"""MediaPipe 랜드마크 원본(21×3) 덤프 프로브 — 손바닥 접힘(5_1 프록시) 검증용.

왜 별도 스크립트인가 (2026-07-18):
  vision_dg5f 로그는 계산된 각도·벡터만 남기고 랜드마크 원본을 버린다. 그래서
  "MediaPipe가 손바닥 5점(0,5,9,13,17)의 접힘/함몰을 실제로 출력하는가"를 기존
  로그로는 검증할 수 없다. 라이브 노드를 건드리지 않고(포트·필터 무영향) 같은
  카메라·MediaPipe 설정으로 랜드마크만 통째로 기록한다.

카메라·모델 설정은 vision_node_dg5f.py와 **동일하게 유지**할 것 — 노이즈 특성이
같아야 여기서 잰 SNR이 라이브 파이프라인에 그대로 적용된다.

사용: python probe_landmarks.py <라벨>   (종료: 미리보기 창에서 q)
  권장 세션 3종(각 20초 이상):
    still     — 손바닥 펴고 카메라 향해 정지 (노이즈 바닥 측정)
    cup       — 평평한 손 ↔ 컵핑(새끼쪽 손바닥 접기) 반복 (신호 측정)
    pinkybend — 손바닥 평평하게 둔 채 새끼만 굽혔다 폈다 (crosstalk 측정)
  로그: logs/lmprobe_<라벨>_<YYYYMMDD_HHMMSS>.csv (t_unix, detected, lm0_x..lm20_z)
  분석: python analyze_lmprobe.py [파일들...]  (인자 없으면 최신 세트 자동)
"""
import sys
import time

import cv2
import mediapipe as mp

from dg5f_paths import unique_log_path

# vision_node_dg5f.py와 동일 설정 (변경 시 양쪽 함께)
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480


def main():
    label = sys.argv[1] if len(sys.argv) > 1 else "free"
    hands = mp.solutions.hands.Hands(
        model_complexity=1, max_num_hands=1,
        min_detection_confidence=0.6, min_tracking_confidence=0.6)

    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)

    log_path = unique_log_path(f"lmprobe_{label}")
    log_f = open(log_path, "w", encoding="utf-8")
    log_f.write(",".join(["t_unix", "detected"]
                         + [f"lm{i}_{a}" for i in range(21) for a in "xyz"]) + "\n")

    print(f"[시작] 랜드마크 프로브 (label={label}) → {log_path} (종료: q)")
    t0 = time.time()
    n_frames = n_det = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)  # vision_node와 동일 (거울 모드)
        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        now = time.time()
        n_frames += 1
        if res.multi_hand_landmarks:
            n_det += 1
            hl = res.multi_hand_landmarks[0]
            mp.solutions.drawing_utils.draw_landmarks(
                frame, hl, mp.solutions.hands.HAND_CONNECTIONS)
            coords = [f"{v:.6f}" for lm in hl.landmark for v in (lm.x, lm.y, lm.z)]
            log_f.write(f"{now:.3f},1," + ",".join(coords) + "\n")
        else:
            # 미검출 프레임도 남긴다 — 검출률 자체가 "얼마나 민감하게 잡는가"의 일부
            log_f.write(f"{now:.3f},0," + ",".join(["0"] * 63) + "\n")

        cv2.putText(frame, f"{label}  {now - t0:5.1f}s  det {n_det}/{n_frames}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("lmprobe (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    log_f.close()
    print(f"[종료] {n_det}/{n_frames} 프레임 검출, 로그: {log_path}")


if __name__ == "__main__":
    main()
