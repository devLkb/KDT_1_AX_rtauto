# -*- coding: utf-8 -*-
"""7/2 라이브 로그(vision_log_live1.csv)에서 손 정지 구간의 raw/filt 지터 측정 (deg)."""
import numpy as np, csv

def load(path):
    with open(path, encoding="utf-8-sig") as f:
        r = csv.reader(f); hdr = next(r)
        rows = [row for row in r if len(row) == len(hdr)]
    return hdr, np.array(rows, dtype=float)

names = ["thumb_flexion","thumb_opposition","index_distal","index_proximal",
         "middle_distal","middle_proximal","ring","pinky","spread"]
_, V = load("C:/Users/dltmd/Desktop/KDT/svh/vision_log_live1.csv")
t = V[:,0]; det = V[:,1]; raw = np.degrees(V[:,2:11]); filt = np.degrees(V[:,11:20])
print(f"rows={len(V)} dur={t[-1]-t[0]:.1f}s detected={det.mean()*100:.0f}%")

# 손 정지: filt 전 채널 rolling std가 가장 작은 3초 창 + 검출 연속
win = 90
best = None
for s in range(0, len(V)-win, 15):
    if det[s:s+win].min() < 1: continue
    m = filt[s:s+win].std(axis=0).max()
    if best is None or m < best[1]:
        best = (s, m)
if best is None:
    print("검출 연속 3초 구간 없음"); raise SystemExit
s = best[0]; e = s+win
print(f"\n가장 정지된 3초: t+{t[s]-t[0]:.1f}s~ (창 내 최대 filt std={best[1]:.2f} deg)")
print(f"{'ch':<18}{'raw_std':>8}{'raw_p2p':>9}{'filt_std':>9}{'filt_p2p':>9}")
for i, n in enumerate(names):
    print(f"{n:<18}{raw[s:e,i].std():8.2f}{raw[s:e,i].ptp():9.2f}{filt[s:e,i].std():9.2f}{filt[s:e,i].ptp():9.2f}")

# 전체 세션 프레임간 변화량(지터 속도감)
mask = det > 0
d_raw = np.abs(np.diff(raw[mask], axis=0))
d_filt = np.abs(np.diff(filt[mask], axis=0))
print(f"\n프레임간 |Δ| 중앙값 (deg/frame @~30fps):")
print(f"{'ch':<18}{'raw':>7}{'filt':>7}")
for i, n in enumerate(names):
    print(f"{n:<18}{np.median(d_raw[:,i]):7.3f}{np.median(d_filt[:,i]):7.3f}")
