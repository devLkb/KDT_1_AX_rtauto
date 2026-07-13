# -*- coding: utf-8 -*-
"""주먹<->펴기 프로브: 4사이클 x (주먹 2.5s + 펴기 2.5s), 60Hz UDP 송신."""
import socket, struct, time

FIST = [0.9704, 0.9879, 1.334, 0.79849, 1.334, 0.79849, 0.98175, 0.98175, 0.5829]
OPEN = [0.0] * 9
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_for(vals, dur):
    end = time.time() + dur
    pkt = struct.pack("<9f", *vals)
    while time.time() < end:
        sock.sendto(pkt, ("127.0.0.1", 5005))
        time.sleep(1/60)

time.sleep(1.0)  # play 안정화 대기
for c in range(4):
    print(f"cycle {c+1}: fist"); send_for(FIST, 2.5)
    print(f"cycle {c+1}: open"); send_for(OPEN, 2.5)
print("done")
sock.close()
