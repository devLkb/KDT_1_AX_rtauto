# -*- coding: utf-8 -*-
"""DG5F 보정 루틴 — 본인 손의 채널별 각도 범위(human_min/max)를 실측해서 저장.

svh/calibrate_raw.py의 DG5F 20채널 버전. 관절 매핑 전에 반드시 이걸 먼저 실행:
사람마다 손 크기·웹캠 각도가 달라서 raw 각도 범위가 다름 → 기본값이면 매핑이 어긋남.

사용법:
  python calibrate_dg5f.py
  1) 손을 카메라에 잘 보이게 두고, 다음 동작을 각각 3회 이상 천천히 반복:
     - 완전히 펴기 ↔ 주먹 꽉 쥐기 (굽힘 채널들)
     - 엄지를 손바닥 반대로 쫙 벌렸다 ↔ 새끼 쪽으로 최대 오므리기 (엄지 대향)
     - 엄지를 최대한 쭉 펴세요 (여러 방향으로, 각 2초 유지 — 엄지 직진도 상한
       thumb_straight_ratio 보정용. 생략해도 기본값 0.97로 동작하는 선택 항목)
     - 손가락 쫙 벌리기 ↔ 모으기 (벌림 채널 — 지금은 게이트지만 미리 보정)
  2) 화면에 [현재 | 관측min | 관측max] 실시간 표시. 충분히 움직였으면 q 종료.
  3) dg5f_calibration.json 자동 저장 → vision_node_dg5f.py가 자동으로 읽음.

⚠️ 반드시 이 스크립트로 보정할 것 — vision_node의 로그는 clamp된 값이라 역산 불가
   (SVH 때 보정을 vision_node로 시도했다 저장 안 되는 함정 있었음).
"""
import csv
import json
import os
import time

import cv2
import mediapipe as mp
import numpy as np

from dg5f_angles import compute_raw, CHANNEL_NAMES, WRIST, THUMB, MIDDLE

CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
LOG_EVERY_SEC = 0.5
# 로그는 스크립트 위치 기준 logs/ 하위에 저장 — 실행 CWD와 무관
LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
CSV_PATH = os.path.join(LOG_DIR, time.strftime("calib_log_%Y%m%d_%H%M.csv"))

# 극값 대신 백분위수 사용 — MediaPipe 스파이크가 min/max를 오염시키는 것 방지
# (2026-07-13 실측: 절대 극값 방식은 pip max가 2.6~2.9rad(물리 불가)로 오염됐었음)
PCT_LO, PCT_HI = 2.0, 98.0

# 채널별 물리 한계 상한(rad) — 백분위수로도 못 거른 잔여 오염 캡
HUMAN_CAP = {
    "mcp": 1.7, "pip": 1.9, "dip": 1.6,
    "thumb_cmc": 1.0, "thumb_opp": 1.3, "thumb_mcp": 1.3, "thumb_ip": 1.5,
    "pinky_cmc": 1.2,
}


def _cap_for(name):
    if name in HUMAN_CAP:
        return HUMAN_CAP[name]
    for suffix, cap in HUMAN_CAP.items():
        if name.endswith(suffix):
            return cap
    return float("inf")


def landmarks_to_xyz(hand_landmarks):
    pts = np.zeros((21, 3), dtype=np.float64)
    for i, lm in enumerate(hand_landmarks.landmark):
        pts[i] = (lm.x, lm.y, lm.z)
    return pts


def main():
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(model_complexity=1, max_num_hands=1,
                           min_detection_confidence=0.6,
                           min_tracking_confidence=0.6)
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    if not cap.isOpened():
        print(f"[오류] 카메라 {CAM_INDEX} 열기 실패 — CAM_INDEX를 1,2로 바꿔보세요.")
        return

    obs_min = {n: float("inf") for n in CHANNEL_NAMES}
    obs_max = {n: float("-inf") for n in CHANNEL_NAMES}
    samples = {n: [] for n in CHANNEL_NAMES}  # 백분위수 계산용 전체 샘플
    # 엄지 직진도 샘플 = |엄지끝−CMC| 직선 / 같은 프레임 엄지 마디합 (v3, §26).
    # v2의 직선/손길이는 MediaPipe z 압축의 방향 의존 오차를 그대로 받아 세션·방향에
    # 따라 최대치가 0.65~1.0으로 요동 — 같은 프레임 같은 랜드마크끼리의 비(직진도)는
    # 삼각부등식으로 항상 0~1이고 쭉 펴면 방향 불문 1 근처. 여기서는 그 상한(쭉 폈을 때
    # 실측치)을 p95로 저장 (max는 스파이크 취약, median은 굽힌 프레임 섞여 과소).
    straight_samples = []

    os.makedirs(LOG_DIR, exist_ok=True)
    csv_file = open(CSV_PATH, "w", newline="")
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp"] + CHANNEL_NAMES)
    last_console = 0.0

    print("[시작] 펴기↔주먹, 엄지 대향, 벌리기↔모으기를 각 3회+ 반복. 종료: q")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        frame = cv2.flip(frame, 1)
        res = hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

        if res.multi_hand_landmarks:
            hlm = res.multi_hand_landmarks[0]
            mp.solutions.drawing_utils.draw_landmarks(
                frame, hlm, mp_hands.HAND_CONNECTIONS)
            xyz = landmarks_to_xyz(hlm)
            raw = compute_raw(xyz)

            # 엄지 직진도 샘플 — compute_thumb_tip의 '펴짐 비율' 상한 보정(v3).
            chain = (np.linalg.norm(xyz[THUMB[1]] - xyz[THUMB[0]])
                     + np.linalg.norm(xyz[THUMB[2]] - xyz[THUMB[1]])
                     + np.linalg.norm(xyz[THUMB[3]] - xyz[THUMB[2]]))
            if chain > 1e-6:
                straight_samples.append(
                    float(np.linalg.norm(xyz[THUMB[3]] - xyz[THUMB[0]]) / chain))

            now = time.time()
            writer.writerow([f"{now:.3f}"] + [f"{v:.4f}" for v in raw])
            if now - last_console >= LOG_EVERY_SEC:
                print(f"[{time.strftime('%H:%M:%S')}] "
                      + " ".join(f"{v:5.2f}" for v in raw[:8]) + " ...")
                last_console = now

            y = 16
            for name, v in zip(CHANNEL_NAMES, raw):
                obs_min[name] = min(obs_min[name], v)
                obs_max[name] = max(obs_max[name], v)
                samples[name].append(v)
                txt = (f"{name:11s} {v:5.2f} "
                       f"[{obs_min[name]:5.2f}|{obs_max[name]:5.2f}]")
                cv2.putText(frame, txt, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.38, (0, 255, 0), 1)
                y += 15

        cv2.imshow("dg5f calibration (q to quit)", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    csv_file.close()

    human_ranges = {}
    print(f"\n===== 보정 결과 (백분위 {PCT_LO}/{PCT_HI}% + 물리캡, dg5f_calibration.json 저장) =====")
    for name in CHANNEL_NAMES:
        arr = samples[name]
        if len(arr) < 30:
            print(f"  {name:12s} 샘플 부족({len(arr)}) — 기본값 유지 권장, 저장 생략")
            continue
        lo = float(np.percentile(arr, PCT_LO))
        hi = float(np.percentile(arr, PCT_HI))
        cap = _cap_for(name)
        capped = hi > cap
        hi = min(hi, cap)
        human_ranges[name] = {"min": round(lo, 3), "max": round(hi, 3)}
        print(f"  {name:12s} {lo:7.2f} ~ {hi:7.2f}"
              + (f"  (물리캡 {cap} 적용)" if capped else "")
              + (f"  ⚠️범위폭 {hi-lo:.2f} 좁음 — 해당 동작 확인" if hi - lo < 0.3 and "abd" not in name else ""))

    out = {
        "note": "calibrate_dg5f.py 자동 생성 — dg5f_angles.py가 임포트 시 읽음. "
                "재보정하면 덮어써짐.",
        "version": 3,  # v3: thumb_straight_ratio(직진도). v2 thumb_reach_ratio(직선/손길이)는 폐기.
        "method": f"percentile {PCT_LO}/{PCT_HI} + human cap; thumb_straight p95",
        "created": time.strftime("%Y-%m-%d %H:%M"),
        "human_ranges": human_ranges,
    }
    if len(straight_samples) >= 30:
        out["thumb_straight_ratio"] = round(float(np.percentile(straight_samples, 95)), 4)
        print(f"  thumb_straight_ratio = {out['thumb_straight_ratio']:.3f} "
              f"(|엄지끝-CMC| 직선/마디합 p95, {len(straight_samples)}샘플)")
        if out["thumb_straight_ratio"] < 0.90:
            print("  ⚠️ thumb_straight_ratio가 0.90 미만 — 엄지를 충분히 안 편 것 같음. "
                  "'엄지 쭉 펴기' 동작을 포함해 재보정 권장.")
    else:
        print(f"  ⚠️ thumb_straight_ratio 샘플 부족 — 런타임 기본값({0.97})으로 동작")
    with open("dg5f_calibration.json", "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"[저장] dg5f_calibration.json + 원시 로그 {CSV_PATH}")
    print("→ vision_node_dg5f.py 다시 실행하면 자동 적용됩니다.")


if __name__ == "__main__":
    main()
