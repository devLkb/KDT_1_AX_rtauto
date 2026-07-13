# -*- coding: utf-8 -*-
"""웹캠 없이 DG5F 텔레옵 배선을 결정적으로 검증하는 프로브 송신기.

지정한 포즈의 20채널 패킷을 50Hz로 송신한다. Unity Play 중에 실행해
xDrive.target이 기대값으로 들어가는지 확인하는 용도 (SVH 프로브 패킷 검증 패턴).

사용:
  python probe_sender.py fist          # 주먹 (오른손 모델)
  python probe_sender.py open          # 펴기
  python probe_sender.py cycle         # 주먹↔펴기 2초 주기 반복
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
                base = mirror_left(OK) if hand == "left" else OK
                vals = base + [0.2, 0.3, 0.6, 1.0]  # tip(무시됨—핀치 우선), pinch=1
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
