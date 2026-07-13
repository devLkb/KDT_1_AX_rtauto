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

IP = "127.0.0.1"
PORT = 5005

CHANNEL_NAMES = [
    "thumb_flex", "thumb_opp", "index_flex", "middle_flex", "ring_flex",
    "pinky_flex", "spread", "idx_mid_cpl", "ring_pky_cpl",
]


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((IP, PORT))
    print(f"[수신 대기] {IP}:{PORT} — vision_node.py를 실행하세요. 종료: Ctrl+C")
    print("  " + "  ".join(f"{n[:6]:>6s}" for n in CHANNEL_NAMES))
    try:
        while True:
            data, _ = sock.recvfrom(1024)
            if len(data) >= 36:                       # 9 * 4 bytes
                vals = struct.unpack("<9f", data[:36])
                line = "  ".join(f"{v:6.2f}" for v in vals)
                print("  " + line)
    except KeyboardInterrupt:
        print("\n[종료] 수신기")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
