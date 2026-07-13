# -*- coding: utf-8 -*-
"""판별 프로브. mode=mid: 리밋 안쪽 목표(80%/15%) 스텝. mode=ramp: 0..max 풀레인지, 0.8s 선형 램프."""
import socket, struct, time, sys

MAX = [0.9704, 0.9879, 1.334, 0.79849, 1.334, 0.79849, 0.98175, 0.98175, 0.5829]
mode = sys.argv[1]
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send(vals):
    sock.sendto(struct.pack("<9f", *vals), ("127.0.0.1", 5005))

def hold(vals, dur):
    end = time.time() + dur
    while time.time() < end:
        send(vals); time.sleep(1/60)

def ramp(v0, v1, dur):
    t0 = time.time()
    while True:
        s = (time.time() - t0) / dur
        if s >= 1.0: break
        send([a + (b - a) * s for a, b in zip(v0, v1)]); time.sleep(1/60)

if mode == "mid":
    FIST = [0.80 * m for m in MAX]; OPEN = [0.15 * m for m in MAX]
    time.sleep(1.0)
    for c in range(4):
        print(f"cycle {c+1}"); hold(FIST, 2.5); hold(OPEN, 2.5)
else:  # ramp
    FIST = MAX[:]; OPEN = [0.0] * 9
    time.sleep(1.0)
    for c in range(4):
        print(f"cycle {c+1}"); ramp(OPEN, FIST, 0.8); hold(FIST, 1.7); ramp(FIST, OPEN, 0.8); hold(OPEN, 1.7)
print("done")
sock.close()
