# -*- coding: utf-8 -*-
"""판별 프로브 평가: 홀드 정상상태 act p2p / 한계밖 음수 / |act-tgt| (rx 홀드 레벨 자동)."""
import numpy as np, csv, sys
CH = ["thmFlex","thmOpp","idxDist","idxProx","midDist","midProx","ring","pinky","spread"]
SP = "C:/Users/dltmd/AppData/Local/Temp/claude/C--Users-dltmd-Desktop-KDT/891a6cb3-e653-4a30-a19a-864a5724b4d1/scratchpad/"

def load(path):
    with open(path, encoding="utf-8-sig") as f:
        r = csv.reader(f); hdr = next(r)
        A = np.array([row for row in r if len(row) == len(hdr)], dtype=float)
    return A[:,0], A[:,1:10], A[:,10:19], A[:,19:28]

def holds(t, rx, level, tol=2.0, min_dur=1.0, skip=0.8):
    mask = np.abs(rx[:,2] - level) < tol
    out = []; s = None
    for i, m in enumerate(mask):
        if m and s is None: s = i
        elif not m and s is not None:
            if t[i-1]-t[s] >= min_dur: out.append((s, i-1))
            s = None
    if s is not None and t[-1]-t[s] >= min_dur: out.append((s, len(mask)-1))
    segs = []
    for w in out:
        s2 = w[0]
        while s2 < w[1] and t[s2] < t[w[0]] + skip: s2 += 1
        if w[1]-s2 > 15: segs.append((s2, w[1]))
    return segs

for f, hi, lo in [("after_midlimit.csv", 61.1, 11.5), ("after_ramp.csv", 76.4, 0.0)]:
    t, rx, tgt, act = load(SP + f)
    hw = holds(t, rx, hi); lw = holds(t, rx, lo)
    print(f"\n== {f}: 상홀드 {len(hw)}개 / 하홀드 {len(lw)}개 ==")
    print(f"{'ch':<8}{'p2p상':>8}{'p2p하':>8}{'|err|상':>9}{'음수<-1° %':>11}")
    for i, n in enumerate(CH):
        p_h = np.mean([act[s:e+1,i].ptp() for s,e in hw]) if hw else np.nan
        p_l = np.mean([act[s:e+1,i].ptp() for s,e in lw]) if lw else np.nan
        er = np.mean([np.median(np.abs(act[s:e+1,i]-tgt[s:e+1,i])) for s,e in hw]) if hw else np.nan
        below = (act[:,i] < -1.0).mean()*100
        print(f"{n:<8}{p_h:8.2f}{p_l:8.2f}{er:9.2f}{below:10.1f}%")
