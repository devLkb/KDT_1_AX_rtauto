"""
UDP 수신 테스트 (유니티 대용).

vision_node.py가 보내는 9개 관절 각도(float32 x 9, little-endian)를
받아서 콘솔에 출력합니다. 유니티를 아직 안 붙였을 때, 전송이
제대로 되는지 먼저 확인하는 용도입니다.

사용법:
  1) 터미널 A: python udp_test_receiver.py   (먼저 켜서 대기)
  2) 터미널 B: python vision_node.py          (STEREO=False 상태)
  3) 손을 웹캠에 비추면, 터미널 A에 9개 값이 실시간으로 찍힘.

vision_node의 UNITY_IP/UNITY_PORT와 아래 값이 같아야 합니다.
"""
import socket
import struct
import sys
import time

IP = "127.0.0.1"
PORT = 5005

# 컬럼 라벨: svh_angles.CHANNEL_NAMES(=SVH 드라이버 JTC/SVHChannel) 순서와 동일.
# 검지·중지는 Distal(원위)이 Proximal(근위)보다 먼저 온다.
CHANNEL_NAMES = [
    "thmFlx", "thmOpp", "idxDst", "idxPrx", "midDst",
    "midPrx", "ring", "pinky", "spread",
]

# python udp_test_receiver.py --auto-exit 2
#   -> 첫 패킷 수신 후, 패킷이 N초 끊기면 자동 종료(자동 검증용).
# 인자 없으면 기존처럼 Ctrl+C로 종료(단 settimeout으로 즉시 반응).
AUTO_EXIT = None
if "--auto-exit" in sys.argv:
    AUTO_EXIT = float(sys.argv[sys.argv.index("--auto-exit") + 1])


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, PORT))
    sock.settimeout(0.5)   # blocking recv에서 Ctrl+C 즉시 반응 (트러블슈팅 #4)
    print(f"[수신 대기] {IP}:{PORT} — vision_node.py를 실행하세요. 종료: Ctrl+C")
    print("  " + "  ".join(f"{n:>6s}" for n in CHANNEL_NAMES))
    count = 0
    last_rx = None
    try:
        while True:
            try:
                data, _ = sock.recvfrom(1024)
            except socket.timeout:
                if AUTO_EXIT and last_rx and (time.time() - last_rx) >= AUTO_EXIT:
                    print(f"[자동 종료] {AUTO_EXIT}s 동안 패킷 없음. 총 {count}개 수신.")
                    break
                continue
            if len(data) >= 36:                       # 9 * 4 bytes
                vals = struct.unpack("<9f", data[:36])
                line = "  ".join(f"{v:6.2f}" for v in vals)
                print("  " + line)
                count += 1
                last_rx = time.time()
    except KeyboardInterrupt:
        print(f"\n[종료] 수신기 (총 {count}개 수신)")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
