import socket, struct, time
# packet order (SVHChannel): [0]ThumbFlex [1]ThumbOpp [2]IdxDist [3]IdxProx [4]MidDist [5]MidProx [6]Ring [7]Pinky [8]Spread
FIST = [0.9704, 0.9879, 1.334, 0.79849, 1.334, 0.79849, 0.98175, 0.98175, 0.5829]
OPEN = [0.0] * 9
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_for(vals, dur):
    end = time.time() + dur
    while time.time() < end:
        s.sendto(struct.pack('<9f', *vals), ('127.0.0.1', 5005))
        time.sleep(1/30)
for phase, dur in [(FIST, 2.0), (OPEN, 2.0), (FIST, 2.0), (OPEN, 2.0)]:
    send_for(phase, dur)
print("done")
