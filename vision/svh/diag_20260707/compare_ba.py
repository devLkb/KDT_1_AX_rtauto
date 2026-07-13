# -*- coding: utf-8 -*-
"""stiffness 10000/damping200 (before) vs 1000/임계감쇠 (after) 프로브 비교.
지표: (1) act p2p (정상상태 창)  (2) 한계 밖 음수 시간 비율  (3) 정상상태 |act-tgt|"""
import numpy as np, csv

CH = ["thmFlex","thmOpp","idxDist","idxProx","midDist","midProx","ring","pinky","spread"]
FIST_DEG = [55.6, 56.6, 76.4, 45.7, 76.4, 45.7, 56.3, 56.3, 33.4]
SP = "C:/Users/dltmd/AppData/Local/Temp/claude/C--Users-dltmd-Desktop-KDT/891a6cb3-e653-4a30-a19a-864a5724b4d1/scratchpad/"

def load(path):
    with open(path, encoding="utf-8-sig") as f:
        r = csv.reader(f); hdr = next(r)
        rows = [row for row in r if len(row) == len(hdr)]
    A = np.array(rows, dtype=float)
    return A[:,0], A[:,1:10], A[:,10:19], A[:,19:28]  # t, rx, tgt, act

def hold_windows(rx, t, level, tol=3.0, min_dur=1.2):
    """rx의 idxDist가 level 근처로 유지되는 구간들."""
    mask = np.abs(rx[:,2] - level) < tol
    out = []; s = None
    for i, m in enumerate(mask):
        if m and s is None: s = i
        elif not m and s is not None:
            if t[i-1]-t[s] >= min_dur: out.append((s, i-1))
            s = None
    if s is not None and t[-1]-t[s] >= min_dur: out.append((s, len(mask)-1))
    return out

def steady(w, t, skip=1.0):
    """hold 창에서 앞 skip초(과도응답) 제외한 정상상태 구간."""
    s, e = w
    while s < e and t[s] < t[w[0]] + skip: s += 1
    return (s, e) if e - s > 10 else None

def metrics(path):
    t, rx, tgt, act = load(path)
    fists = hold_windows(rx, t, FIST_DEG[2])
    opens = hold_windows(rx, t, 0.0)
    res = {}
    for i, n in enumerate(CH):
        p2p_f, p2p_o, err_f = [], [], []
        for w in fists:
            sw = steady(w, t)
            if sw:
                seg = act[sw[0]:sw[1]+1, i]
                p2p_f.append(seg.ptp())
                err_f.append(np.median(np.abs(seg - tgt[sw[0]:sw[1]+1, i])))
        for w in opens:
            sw = steady(w, t)
            if sw: p2p_o.append(act[sw[0]:sw[1]+1, i].ptp())
        below = (act[:,i] < -1.0).mean()*100      # 한계(0°) 1° 이상 침범
        res[n] = dict(p2p_fist=np.mean(p2p_f) if p2p_f else np.nan,
                      p2p_open=np.mean(p2p_o) if p2p_o else np.nan,
                      below=below,
                      err_fist=np.mean(err_f) if err_f else np.nan)
    return res, len(fists), len(opens)

import sys
after_file = sys.argv[1] if len(sys.argv) > 1 else "after.csv"
B, bf, bo = metrics(SP + "before.csv")
A, af, ao = metrics(SP + after_file)
print(f"before(10000/200, solver6/1): fist홀드 {bf}/open {bo}, after({after_file}): {af}/{ao} (홀드 앞 1초 과도응답 제외)")
print(f"\n{'ch':<8} | {'act p2p 주먹홀드':^21} | {'act p2p 펴기홀드':^21} | {'한계밖 음수 시간%':^19} | {'정상상태 |act-tgt| 주먹':^22}")
print(f"{'':<8} | {'before':>9}{'after':>10} | {'before':>9}{'after':>10} | {'before':>8}{'after':>9} | {'before':>10}{'after':>10}")
for n in CH:
    b, a = B[n], A[n]
    print(f"{n:<8} | {b['p2p_fist']:9.1f}{a['p2p_fist']:10.2f} | {b['p2p_open']:9.1f}{a['p2p_open']:10.2f} | "
          f"{b['below']:7.1f}%{a['below']:8.1f}% | {b['err_fist']:10.2f}{a['err_fist']:10.2f}")
