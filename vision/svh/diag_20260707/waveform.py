# -*- coding: utf-8 -*-
"""idxDist의 tgt/act 시계열 파형 확인 (after_solver.csv, 1사이클, 0.1s 간격 텍스트)."""
import numpy as np, csv
SP = "C:/Users/dltmd/AppData/Local/Temp/claude/C--Users-dltmd-Desktop-KDT/891a6cb3-e653-4a30-a19a-864a5724b4d1/scratchpad/"
with open(SP + "after_solver.csv", encoding="utf-8-sig") as f:
    r = csv.reader(f); hdr = next(r)
    A = np.array([row for row in r if len(row) == len(hdr)], dtype=float)
t = A[:,0] - A[0,0]
rx, tgt, act = A[:,3], A[:,12], A[:,21]   # idxDist
# 첫 fist 시작(rx가 처음 60 넘는 지점)부터 6초
i0 = np.argmax(rx > 60)
sel = (t >= t[i0]-0.3) & (t <= t[i0]+6.0)
ts, tg, ac = t[sel], tgt[sel], act[sel]
step = max(1, int(0.1/np.median(np.diff(ts))))
print("t(s)   tgt     act    (idxDist, deg)")
for i in range(0, len(ts), step):
    bar = int((ac[i]+50)/140*60)
    print(f"{ts[i]:6.2f} {tg[i]:6.1f} {ac[i]:7.1f}  |{'.'*max(bar,0)}*")
