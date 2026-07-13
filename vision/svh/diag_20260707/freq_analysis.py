# -*- coding: utf-8 -*-
"""주먹 홀드 정상상태 구간의 진동 주파수(영교차 기반) 분석.
25Hz(물리 나이퀴스트, 스텝마다 부호 교대)면 수치 채터링, 수 Hz면 기계적 공진."""
import numpy as np, csv, sys

CH = ["thmFlex","thmOpp","idxDist","idxProx","midDist","midProx","ring","pinky","spread"]
SP = "C:/Users/dltmd/AppData/Local/Temp/claude/C--Users-dltmd-Desktop-KDT/891a6cb3-e653-4a30-a19a-864a5724b4d1/scratchpad/"

def load(path):
    with open(path, encoding="utf-8-sig") as f:
        r = csv.reader(f); hdr = next(r)
        A = np.array([row for row in r if len(row) == len(hdr)], dtype=float)
    return A[:,0], A[:,1:10], A[:,10:19], A[:,19:28]

def fist_steady(t, rx, skip=1.0):
    mask = np.abs(rx[:,2] - 76.4) < 3.0
    out = []; s = None
    for i, m in enumerate(mask):
        if m and s is None: s = i
        elif not m and s is not None:
            if t[i-1]-t[s] >= 1.2: out.append((s, i-1))
            s = None
    if s is not None: out.append((s, len(mask)-1))
    segs = []
    for w in out:
        s2 = w[0]
        while s2 < w[1] and t[s2] < t[w[0]] + skip: s2 += 1
        if w[1]-s2 > 20: segs.append((s2, w[1]))
    return segs

for f in sys.argv[1:]:
    t, rx, tgt, act = load(SP + f)
    segs = fist_steady(t, rx)
    print(f"\n== {f} (주먹 정상상태 {len(segs)}구간) ==")
    print(f"{'ch':<8}{'p2p':>8}{'주파수Hz':>9}{'스텝간 부호교대율%':>18}")
    for i, n in enumerate(CH):
        zc = 0; alt = 0; N = 0; ptp = []
        for s, e in segs:
            x = act[s:e+1, i] - act[s:e+1, i].mean()
            d = np.diff(x)
            zc += ((x[:-1] * x[1:]) < 0).sum()
            alt += ((d[:-1] * d[1:]) < 0).sum()  # 매 스텝 방향 교대 = 나이퀴스트 채터
            N += len(x); ptp.append(x.ptp())
        dur = sum(t[e]-t[s] for s, e in segs)
        freq = zc / (2*dur) if dur > 0 else 0
        altr = alt / max(N-2*len(segs), 1) * 100
        print(f"{n:<8}{np.mean(ptp):8.1f}{freq:9.1f}{altr:18.0f}")
