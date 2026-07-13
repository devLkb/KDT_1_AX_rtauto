"""
1단계 보정용 디버그 스크립트 (노트북 웹캠, 단일 카메라).

목적: 손을 폈다 굽혔다 할 때 9개 채널의 raw 각도(rad)가
      어디서 어디까지 움직이는지 눈으로 확인해서,
      svh_angles.py의 human_min / human_max를 본인 손에 맞게 보정.

사용법:
  python3 calibrate_raw.py
  - 손을 카메라 앞에 두고 완전히 폈다가 -> 주먹 꽉 쥐었다가 반복.
  - 엄지 대향(opposition), 손가락 벌림(spread)도 최대/최소까지 움직여 보기.
  - 화면에 각 채널의 [현재값 | 관측 최소 | 관측 최대]가 실시간 표시됩니다.
  - 충분히 움직인 뒤 q로 종료하면, 관측된 min/max를 정리해서 출력합니다.
  - 그 값을 svh_angles.py의 (human_min, human_max)에 넣으세요.

CAM_INDEX: 내장 웹캠이 0이 아니면 1,2로 바꿔가며 맞추세요.

로그:
  - 콘솔: LOG_EVERY_SEC 간격으로 9채널 값 한 줄 출력
  - 파일: angles_log.csv 에 매 프레임 기록 (timestamp + 9채널)
"""
import csv
import json
import time

import cv2
import numpy as np
import mediapipe as mp

from svh_angles import compute_svh_angles, CHANNEL_NAMES

CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
LOG_EVERY_SEC = 0.5          # 콘솔 로그 간격(초)
CSV_PATH = "angles_log.csv"  # 매 프레임 CSV 저장


def landmarks_to_xyz(hand_landmarks):
    pts = np.zeros((21, 3), dtype=np.float64)
    for i, lm in enumerate(hand_landmarks.landmark):
        pts[i] = (lm.x, lm.y, lm.z)
    return pts


def main():
    mp_hands = mp.solutions.hands
    mp_draw = mp.solutions.drawing_utils
    hands = mp_hands.Hands(model_complexity=1, max_num_hands=1,
                           min_detection_confidence=0.6,
                           min_tracking_confidence=0.6)

    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print(f"[오류] 카메라 {CAM_INDEX}를 열 수 없습니다. "
              f"CAM_INDEX를 1이나 2로 바꿔보세요.")
        return

    # 관측된 min/max 누적
    obs_min = {n: float("inf") for n in CHANNEL_NAMES}
    obs_max = {n: float("-inf") for n in CHANNEL_NAMES}

    # CSV 로그 준비
    csv_file = open(CSV_PATH, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp"] + CHANNEL_NAMES)
    last_console = 0.0

    print("[시작] 손을 폈다/굽혔다 반복하세요. 종료: q")
    print(f"[로그] 매 프레임 -> {CSV_PATH}, 콘솔 {LOG_EVERY_SEC}초 간격")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)  # 거울 모드(보기 편하게)
        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if res.multi_hand_landmarks:
            hlm = res.multi_hand_landmarks[0]
            mp_draw.draw_landmarks(frame, hlm, mp_hands.HAND_CONNECTIONS)
            xyz = landmarks_to_xyz(hlm)
            raw = compute_svh_angles(xyz)

            # 매 프레임 CSV 기록
            now = time.time()
            writer.writerow([f"{now:.3f}"] + [f"{raw[n]:.4f}" for n in CHANNEL_NAMES])

            # 주기적으로 콘솔에 한 줄 로그
            if now - last_console >= LOG_EVERY_SEC:
                vals = " ".join(f"{n.split('_')[0][:5]}={raw[n]:.2f}" for n in CHANNEL_NAMES)
                print(f"[{time.strftime('%H:%M:%S')}] {vals}")
                last_console = now

            y = 20
            for name in CHANNEL_NAMES:
                v = raw[name]
                obs_min[name] = min(obs_min[name], v)
                obs_max[name] = max(obs_max[name], v)
                txt = f"{name:20s} {v:5.2f} [min {obs_min[name]:4.2f} | max {obs_max[name]:4.2f}]"
                cv2.putText(frame, txt, (10, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
                y += 20

        cv2.imshow("raw angle calibration (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    csv_file.close()
    print(f"\n[저장 완료] 각도 로그: {CSV_PATH}")

    # ---- 보정 결과를 calibration.json으로 자동 저장 ----
    # svh_angles.py가 이 파일을 자동으로 읽어 human_min/max를 덮어씁니다.
    # 손으로 svh_angles.py를 고칠 필요가 없습니다.
    MARGIN = 0.05  # 관측 min/max에 주는 여유(rad)
    human_ranges = {}
    print("\n===== 관측 결과 (calibration.json에 자동 저장) =====")
    print(f"{'channel':22s} human_min  human_max  (마진 {MARGIN} 적용)")
    for name in CHANNEL_NAMES:
        lo = obs_min[name] if obs_min[name] != float("inf") else 0.0
        hi = obs_max[name] if obs_max[name] != float("-inf") else 0.0
        lo_m = round(lo + MARGIN, 3)   # min은 살짝 올리고
        hi_m = round(hi - MARGIN, 3)   # max는 살짝 내려 (안쪽으로 여유)
        if hi_m <= lo_m:               # 범위가 너무 좁으면 마진 없이
            lo_m, hi_m = round(lo, 3), round(hi, 3)
        human_ranges[name] = {"min": lo_m, "max": hi_m}
        print(f"{name:22s} {lo_m:8.2f}  {hi_m:8.2f}")
    print("========================================================")

    out = {
        "note": "calibrate_raw.py 자동 생성. svh_angles.py가 임포트 시 읽음.",
        "margin_rad": MARGIN,
        "human_ranges": human_ranges,
    }
    with open("calibration.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print("[저장 완료] 보정값: calibration.json")
    print("-> 이제 vision_node.py 등이 이 값을 자동으로 사용합니다. "
          "svh_angles.py를 손으로 고칠 필요 없음.")


if __name__ == "__main__":
    main()
