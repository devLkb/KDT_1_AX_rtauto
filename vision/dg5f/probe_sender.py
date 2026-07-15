# -*- coding: utf-8 -*-
"""웹캠 없이 DG5F 텔레옵 배선을 결정적으로 검증하는 프로브 송신기.

지정한 포즈의 20채널 패킷을 50Hz로 송신한다. Unity Play 중에 실행해
xDrive.target이 기대값으로 들어가는지 확인하는 용도 (SVH 프로브 패킷 검증 패턴).

사용:
  python probe_sender.py fist          # 주먹 (오른손 모델)
  python probe_sender.py open          # 펴기
  python probe_sender.py cycle         # 주먹↔펴기 2초 주기 반복
  python probe_sender.py oktip         # v2 핀치 스냅 검증 (엄지 IK → 검지 끝 1.2cm)
  python probe_sender.py tipfar        # v3 리치 복원 검증 (핀치 해제, 펴짐 75% 목표)
  python probe_sender.py tipmax        # 펴짐 100% → 목표가 정확히 robotThumbMaxReach
  python probe_sender.py tipover       # |n|=1.4 위반 패킷 → Unity 안전망 클램프 검증
  python probe_sender.py fist left     # 왼손 모델용 (미러 채널 부호 반전)
  (Ctrl+C 종료)
"""
import socket
import struct
import sys
import time

from dg5f_angles import CHANNEL_NAMES, LEFT_MIRROR_CHANNELS

UNITY_IP, UNITY_PORT = "127.0.0.1", 5006

#         엄지: cmc opp  mcp ip | 검지: abd mcp pip dip | 중지 | 약지 | 새끼: cmc lat mcp pip
# (오른손 관절공간 기준 — left는 mirror_left가 변환. 새끼 slot17=5_2 측면은 항상 0)
OPEN = [0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0]
FIST = [40.0, -80.0, 60.0, 60.0,   0.0, 100.0, 80.0, 70.0,
        0.0, 100.0, 80.0, 70.0,    0.0, 95.0, 80.0, 70.0,   0.0, 0.0, 80.0, 70.0]
OK = [0.0, -80.0, 40.0, 30.0,   0.0, 62.0, 52.0, 38.0,
      0.0, 0.0, 0.0, 0.0,       0.0, 0.0, 0.0, 0.0,   0.0, 0.0, 0.0, 0.0]


def mirror_left(vals):
    return [-v if n in LEFT_MIRROR_CHANNELS else v
            for v, n in zip(vals, CHANNEL_NAMES)]


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "cycle"
    hand = sys.argv[2].lower() if len(sys.argv) > 2 else "right"
    fist = mirror_left(FIST) if hand == "left" else FIST
    open_ = mirror_left(OPEN) if hand == "left" else OPEN
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print(f"[probe_sender] mode={mode} hand={hand} → {UNITY_IP}:{UNITY_PORT} (Ctrl+C 종료)")
    t0 = time.time()
    try:
        while True:
            if mode == "fist":
                vals = fist
            elif mode == "open":
                vals = open_
            elif mode == "ok":
                vals = mirror_left(OK) if hand == "left" else OK
            elif mode == "oktip":
                # v2 핀치 스냅 검증: 검지만 OK 컬, 핀치 플래그 1 → 엄지 IK가 검지 끝으로
                # tip 3개 = 체인 정규화 리치벡터(2026-07-14 의미 변경, 핀치 우선이라 무시됨)
                base = mirror_left(OK) if hand == "left" else OK
                vals = base + [0.25, 0.55, 0.45, 1.0]
            elif mode == "tipfar":
                # v3 리치 복원 검증(2026-07-15 기준 길이 재정의): 핀치 해제(끝거리비 0.8).
                # 리치 크기 |(0.25,0.55,0.45)|≈0.75 = "펴짐 비율 75%" →
                # 로봇 엄지 끝이 1_1에서 0.75×robotThumbMaxReach(=9.3cm) 지점에 수렴해야 함
                base = mirror_left(OPEN) if hand == "left" else OPEN
                vals = base + [0.25, 0.55, 0.45, 0.0, 0.8]
            elif mode == "tipmax":
                # 펴짐 100%: |(0.30,0.36,0.88)|≈1.00 → 목표가 정확히 robotThumbMaxReach
                # (=12.4cm) 구면 위에 찍혀야 함 (완료기준: 사람 100% 폄 = 로봇 도달 상한)
                base = mirror_left(OPEN) if hand == "left" else OPEN
                vals = base + [0.30, 0.36, 0.88, 0.0, 0.8]
            elif mode == "tipover":
                # 계약 위반 패킷(|n|=1.4>1, 구버전 송신기/오보정 상황 재현) —
                # Python은 송신 전 클램프하므로 정상 경로에선 안 나옴. Unity 쪽
                # robotThumbMaxReach 구면 안전망이 12.4cm로 깎는지 검증용.
                base = mirror_left(OPEN) if hand == "left" else OPEN
                vals = base + [0.42, 0.50, 1.21, 0.0, 0.8]
            else:  # cycle
                vals = fist if int((time.time() - t0) / 2.0) % 2 == 0 else open_
            fmt = "<%df" % len(vals)
            sock.sendto(struct.pack(fmt, *vals), (UNITY_IP, UNITY_PORT))
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()


if __name__ == "__main__":
    main()
