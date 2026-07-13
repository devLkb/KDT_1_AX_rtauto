"""
비전 프로세스 메인 루프.

카메라 2대 캡처 -> 각 프레임 MediaPipe -> 삼각측량으로 3D 복원
-> SVH 9개 관절 각도 계산 -> One Euro 필터 -> UDP로 유니티에 전송.

전송 데이터는 "무거운 영상"이 아니라 float 9개(관절 각도)뿐입니다.

======================= 사용 전 반드시 채워야 할 것 =======================
1) 스테레오 캘리브레이션 파일(stereo_calib.npz):
   cv2.calibrateCamera + cv2.stereoCalibrate로 미리 생성.
   포함 키: mtxL, distL, mtxR, distR, R, T
   -> P1, P2(투영행렬)을 만들어 삼각측량에 사용.
2) svh_angles.py의 SVH_CHANNELS 범위를 SVH URDF 실제 값으로 교체.
3) CAM_LEFT / CAM_RIGHT 인덱스를 본인 노트북의 카메라 번호로.
==========================================================================

단일 카메라만으로 뒷단(각도/필터/전송)을 먼저 검증하려면
STEREO = False 로 두고 CAM_LEFT 한 대만 사용하세요.
이때 z는 mediapipe 추정값이라 부정확하지만 파이프라인 검증에는 충분합니다.
"""
import socket
import struct
import time

import cv2
import numpy as np
import mediapipe as mp

from one_euro_filter import OneEuroFilter
from svh_angles import compute_svh_angles, map_to_svh, CHANNEL_NAMES

# ------------------------- 설정 -------------------------
STEREO = False                # 1단계: 단일 카메라(노트북 웹캠)로 뒷단 검증.
                              # 5단계에서 카메라 2대 붙일 때 True로 변경.
CAM_LEFT = 0                  # 노트북 내장 웹캠. 안 열리면 1,2로.
CAM_RIGHT = 1
FRAME_W, FRAME_H = 640, 480   # 손 트래킹엔 이 해상도면 충분, USB 대역폭 절약
CALIB_FILE = "stereo_calib.npz"

UNITY_IP = "127.0.0.1"        # 같은 노트북이면 localhost. 다른 기기면 그 IP.
UNITY_PORT = 5005
SEND_HZ_CAP = 120             # 전송 상한(과도한 패킷 방지)
LOG_EVERY_SEC = 0.5           # 콘솔에 전송값 찍는 간격
# raw/필터후 각도 기록(None이면 끔). 검증용.
# 실행 시각을 파일명에 넣어 실행마다 새 파일 생성(덮어쓰기 방지, 튜닝 전후 비교용).
LOG_CSV = time.strftime("vision_log_%Y%m%d_%H%M.csv")
# --------------------------------------------------------


def open_cam(idx):
    cap = cv2.VideoCapture(idx)
    # MJPG로 열어 USB 대역폭 절약 (웹캠 2대 동시 사용 시 중요)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    return cap


def load_projection_matrices():
    """캘리브레이션 파일에서 좌/우 투영행렬 P1,P2를 만든다."""
    data = np.load(CALIB_FILE)
    mtxL, distL = data["mtxL"], data["distL"]
    mtxR, distR = data["mtxR"], data["distR"]
    R, T = data["R"], data["T"]
    # 좌 카메라를 기준 좌표계로: P1 = K1 [I|0], P2 = K2 [R|T]
    P1 = mtxL @ np.hstack([np.eye(3), np.zeros((3, 1))])
    P2 = mtxR @ np.hstack([R, T.reshape(3, 1)])
    return P1, P2, (mtxL, distL, mtxR, distR)


def landmarks_to_pixels(hand_landmarks, w, h):
    """mediapipe 정규화 landmark -> 픽셀 좌표 (21,2)."""
    pts = np.zeros((21, 2), dtype=np.float64)
    for i, lm in enumerate(hand_landmarks.landmark):
        pts[i, 0] = lm.x * w
        pts[i, 1] = lm.y * h
    return pts


def landmarks_to_xyz(hand_landmarks):
    """단일 카메라 폴백용: mediapipe (x,y,z) 그대로 (21,3)."""
    pts = np.zeros((21, 3), dtype=np.float64)
    for i, lm in enumerate(hand_landmarks.landmark):
        pts[i] = (lm.x, lm.y, lm.z)
    return pts


def triangulate(ptsL, ptsR, P1, P2):
    """대응하는 좌/우 픽셀점(21,2) 2쌍 -> 3D 좌표 (21,3)."""
    ptsL_t = ptsL.T.astype(np.float64)   # (2,21)
    ptsR_t = ptsR.T.astype(np.float64)
    X = cv2.triangulatePoints(P1, P2, ptsL_t, ptsR_t)  # (4,21) 동차좌표
    X /= X[3]
    return X[:3].T   # (21,3)


def main():
    mp_hands = mp.solutions.hands
    # 카메라마다 검출기 하나씩(스레드 안전 위해 분리)
    handsL = mp_hands.Hands(model_complexity=1, max_num_hands=1,
                            min_detection_confidence=0.6,
                            min_tracking_confidence=0.6)
    handsR = mp_hands.Hands(model_complexity=1, max_num_hands=1,
                            min_detection_confidence=0.6,
                            min_tracking_confidence=0.6) if STEREO else None

    capL = open_cam(CAM_LEFT)
    capR = open_cam(CAM_RIGHT) if STEREO else None

    P1 = P2 = None
    if STEREO:
        try:
            P1, P2, _ = load_projection_matrices()
        except Exception as e:
            print(f"[경고] 캘리브레이션 로드 실패({e}). "
                  f"STEREO를 False로 두고 먼저 단일 카메라로 검증하세요.")
            return

    # 9채널 각각에 One Euro 필터
    filters = {name: OneEuroFilter(freq=30.0, min_cutoff=0.8, beta=0.01)
               for name in CHANNEL_NAMES}

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    last_send = 0.0
    last_log = 0.0
    last_valid = None   # occlusion 시 hold할 마지막 유효 각도

    # 검증용 CSV: t, detected(검출성공=1/hold=0), raw 9, filtered 9 (모두 rad)
    log_f = None
    if LOG_CSV:
        log_f = open(LOG_CSV, "w", encoding="utf-8")
        cols = (["t_unix", "detected"]
                + [f"raw_{n}" for n in CHANNEL_NAMES]
                + [f"filt_{n}" for n in CHANNEL_NAMES])
        log_f.write(",".join(cols) + "\n")

    print("[시작] 종료하려면 창에서 q. STEREO =", STEREO)
    while True:
        okL, frameL = capL.read()
        if not okL:
            continue

        xyz = None
        if STEREO:
            okR, frameR = capR.read()
            if not okR:
                continue
            resL = handsL.process(cv2.cvtColor(frameL, cv2.COLOR_BGR2RGB))
            resR = handsR.process(cv2.cvtColor(frameR, cv2.COLOR_BGR2RGB))
            if resL.multi_hand_landmarks and resR.multi_hand_landmarks:
                pL = landmarks_to_pixels(resL.multi_hand_landmarks[0], FRAME_W, FRAME_H)
                pR = landmarks_to_pixels(resR.multi_hand_landmarks[0], FRAME_W, FRAME_H)
                xyz = triangulate(pL, pR, P1, P2)
        else:
            # 단일 카메라: 거울 모드(보기 편하게). 각도 계산엔 영향 없음.
            # 주의: STEREO=True일 때는 flip 금지(삼각측량 좌표가 꼬임).
            frameL = cv2.flip(frameL, 1)
            resL = handsL.process(cv2.cvtColor(frameL, cv2.COLOR_BGR2RGB))
            if resL.multi_hand_landmarks:
                mp.solutions.drawing_utils.draw_landmarks(
                    frameL, resL.multi_hand_landmarks[0],
                    mp.solutions.hands.HAND_CONNECTIONS)
                xyz = landmarks_to_xyz(resL.multi_hand_landmarks[0])

        now = time.time()
        mapped = None
        if xyz is not None:
            raw = compute_svh_angles(xyz)
            mapped = map_to_svh(raw)   # 필터 전(raw) — 검증 로그용
            # 필터 적용
            svh = [filters[name](v) for name, v in zip(CHANNEL_NAMES, mapped)]
            last_valid = svh
        elif last_valid is not None:
            svh = last_valid          # occlusion: 마지막 값 hold
        else:
            svh = None

        if log_f is not None and svh is not None:
            r = mapped if mapped is not None else svh   # hold 프레임은 filtered로 채움
            log_f.write(f"{now:.3f},{1 if mapped is not None else 0},"
                        + ",".join(f"{v:.5f}" for v in r) + ","
                        + ",".join(f"{v:.5f}" for v in svh) + "\n")

        # UDP 전송: float32 x 9 (little-endian). 유니티에서 동일 포맷으로 파싱.
        if svh is not None and (now - last_send) >= (1.0 / SEND_HZ_CAP):
            packet = struct.pack("<9f", *svh)
            sock.sendto(packet, (UNITY_IP, UNITY_PORT))
            last_send = now
            # 주기적으로 전송값 콘솔 출력(검증용)
            if now - last_log >= LOG_EVERY_SEC:
                vals = " ".join(f"{v:.2f}" for v in svh)
                print(f"[send->{UNITY_IP}:{UNITY_PORT}] {vals}")
                last_log = now

        # 디버그용 미리보기 (원격/헤드리스면 이 블록 삭제)
        cv2.imshow("vision_node (q to quit)", frameL)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    capL.release()
    if capR:
        capR.release()
    cv2.destroyAllWindows()
    sock.close()
    if log_f is not None:
        log_f.close()
        print(f"[종료] vision_node — 로그 저장: {LOG_CSV}")
    else:
        print("[종료] vision_node")


if __name__ == "__main__":
    main()
