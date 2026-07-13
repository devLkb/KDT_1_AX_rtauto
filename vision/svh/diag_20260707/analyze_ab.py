# -*- coding: utf-8 -*-
"""A/B 증상 진단: 7/6 로그 기반.
A: 주먹/펴기 순간의 9채널 값 + rx/tgt/act 대응 + 부호 뒤집힘 채널 탐지
B: 손 정지 구간에서 입력(rx) 지터 vs 물리(act) 진동 분리
"""
import numpy as np
import csv

SVH = "C:/Users/dltmd/Desktop/KDT/svh/"
CH = ["thmFlex","thmOpp","idxDist","idxProx","midDist","midProx","ring","pinky","spread"]
SVH_MAX_DEG = [55.60, 56.60, 76.43, 45.75, 76.43, 45.75, 56.25, 56.25, 33.40]

def load(path):
    with open(path, encoding="utf-8-sig") as f:
        r = csv.reader(f)
        hdr = next(r)
        rows = [row for row in r if len(row) == len(hdr)]
    return hdr, np.array(rows, dtype=float)

# ---------- vision_log: raw/filt (rad) ----------
vh, V = load(SVH + "vision_log.csv")
t_v = V[:,0]; det = V[:,1]
raw = V[:,2:11]; filt = V[:,11:20]
print(f"vision_log: {len(V)} rows, {t_v[-1]-t_v[0]:.1f}s, detected={det.mean()*100:.1f}%")

# grip level: 6개 굽힘채널(filt)을 svh_max로 정규화한 평균 (thumb/spread 제외한 flexion 위주)
flex_idx = [0,2,3,4,5,6,7]  # thmFlex, idx*, mid*, ring, pinky
svh_max_rad = np.array([0.9704,0.9879,1.334,0.79849,1.334,0.79849,0.98175,0.98175,0.5829])
grip = (filt[:,flex_idx] / svh_max_rad[flex_idx]).mean(axis=1)

def stable_windows(mask, t, min_dur=0.6):
    """mask가 연속으로 참인 구간 (시작,끝) 인덱스 리스트."""
    out = []; s = None
    for i, m in enumerate(mask):
        if m and s is None: s = i
        elif not m and s is not None:
            if t[i-1]-t[s] >= min_dur: out.append((s, i-1))
            s = None
    if s is not None and t[-1]-t[s] >= min_dur: out.append((s, len(mask)-1))
    return out

fist_w = stable_windows(grip > 0.75, t_v)
open_w = stable_windows(grip < 0.10, t_v)
print(f"fist windows(grip>0.75): {len(fist_w)}, open windows(grip<0.10): {len(open_w)}")

def rep(windows, arr):
    """가장 긴 구간의 채널별 중앙값."""
    if not windows: return None, None
    s, e = max(windows, key=lambda w: w[1]-w[0])
    return np.median(arr[s:e+1], axis=0), (s, e)

fist_filt, fist_span = rep(fist_w, filt)
open_filt, open_span = rep(open_w, filt)
fist_raw, _ = rep(fist_w, raw)
open_raw, _ = rep(open_w, raw)

names = ["thumb_flexion","thumb_opposition","index_distal","index_proximal",
         "middle_distal","middle_proximal","ring","pinky","spread"]
print("\n=== A-1. UDP 송신값 (filt, rad / deg) : 주먹 vs 펴기 (중앙값) ===")
print(f"{'채널':<18}{'주먹rad':>9}{'주먹deg':>9}{'펴기rad':>9}{'펴기deg':>9}{'max_deg':>9}{'주먹%':>7}")
for i, n in enumerate(names):
    fr = fist_filt[i] if fist_filt is not None else float('nan')
    op = open_filt[i] if open_filt is not None else float('nan')
    print(f"{n:<18}{fr:9.3f}{np.degrees(fr):9.1f}{op:9.3f}{np.degrees(op):9.1f}{SVH_MAX_DEG[i]:9.1f}{np.degrees(fr)/SVH_MAX_DEG[i]*100:7.0f}")

# ---------- unity_joint_log: rx/tgt/act (deg) ----------
uh, U = load(SVH + "unity_joint_log.csv")
t_u = U[:,0]; rx = U[:,1:10]; tgt = U[:,10:19]; act = U[:,19:28]
print(f"\nunity_joint_log: {len(U)} rows, {t_u[-1]-t_u[0]:.1f}s")
print(f"time overlap: unity[{t_u[0]:.0f}..{t_u[-1]:.0f}] vision[{t_v[0]:.0f}..{t_v[-1]:.0f}]")

# 채널별 rx->act 상관 (부호 뒤집힘 탐지). rx가 거의 안 움직인 채널은 판단 보류.
print("\n=== A-2. 채널별 rx vs tgt vs act 정합 (전체 세션) ===")
print(f"{'ch':<8}{'rx범위(deg)':>14}{'corr(rx,tgt)':>13}{'corr(rx,act)':>13}{'act-rx RMS':>11}{'판정':>12}")
for i, n in enumerate(CH):
    r = rx[:,i]; g = tgt[:,i]; a = act[:,i]
    rng = r.max()-r.min()
    if rng < 2.0:
        verdict = "입력변화없음"
        c1 = c2 = float('nan')
    else:
        c1 = np.corrcoef(r, g)[0,1]; c2 = np.corrcoef(r, a)[0,1]
        verdict = "OK" if c2 > 0.7 else ("반전의심!" if c2 < -0.3 else "불일치")
    rms = np.sqrt(np.mean((a-r)**2))
    print(f"{n:<8}{r.min():6.1f}~{r.max():6.1f}{c1:13.2f}{c2:13.2f}{rms:11.1f}{verdict:>12}")

# 주먹 순간(rx grip 높음)에서 rx/tgt/act 나란히
rx_grip = (rx[:,flex_idx] / np.array(SVH_MAX_DEG)[flex_idx]).mean(axis=1)
fw = stable_windows(rx_grip > 0.75, t_u)
ow = stable_windows(rx_grip < 0.10, t_u)
for label, wins in [("주먹", fw), ("펴기", ow)]:
    if not wins:
        print(f"\n(unity log에 {label} 구간 없음)")
        continue
    s, e = max(wins, key=lambda w: w[1]-w[0])
    print(f"\n=== A-3. {label} 구간(unity, {t_u[e]-t_u[s]:.1f}s) rx/tgt/act 중앙값 (deg) ===")
    print(f"{'ch':<8}{'rx':>8}{'tgt':>8}{'act':>8}{'act-tgt':>9}")
    for i, n in enumerate(CH):
        mrx = np.median(rx[s:e+1,i]); mtg = np.median(tgt[s:e+1,i]); mac = np.median(act[s:e+1,i])
        flag = "  <-- 반대!" if (mrx > 0.5*SVH_MAX_DEG[i] and mac < 0.25*SVH_MAX_DEG[i]) or \
                              (mrx < 0.15*SVH_MAX_DEG[i] and mac > 0.5*SVH_MAX_DEG[i]) else ""
        print(f"{n:<8}{mrx:8.1f}{mtg:8.1f}{mac:8.1f}{mac-mtg:9.1f}{flag}")

# ---------- B. 정지 구간: 입력 지터 vs 물리 진동 ----------
# rx(입력)가 안정된(rolling std 작음) 3초+ 구간을 찾아, 그 구간에서 rx/tgt/act의 std, p2p 비교
print("\n=== B. 정지 구간 분석 (rx 안정 구간에서 입력지터 vs 물리진동) ===")
win = 50  # ~2.5s at 20ms fixed step
best = None
for s in range(0, len(U)-win, 10):
    seg_rx = rx[s:s+win]
    if seg_rx.std(axis=0).max() < 0.8:  # 모든 채널 rx std < 0.8deg → 손 정지
        score = act[s:s+win].std(axis=0).max()
        if best is None or score > best[1]:
            best = (s, score)
if best is None:
    print("rx 안정 구간을 못 찾음 (손이 계속 움직였음)")
else:
    s = best[0]; e = s+win
    print(f"구간: t={t_u[s]:.1f}..{t_u[e]:.1f} ({t_u[e]-t_u[s]:.1f}s), 그때 grip={rx_grip[s:e].mean():.2f}")
    print(f"{'ch':<8}{'rx_std':>8}{'rx_p2p':>8}{'tgt_std':>8}{'act_std':>8}{'act_p2p':>9}{'판정':>14}")
    for i, n in enumerate(CH):
        rs = rx[s:e,i].std(); rp = rx[s:e,i].ptp()
        ts = tgt[s:e,i].std()
        as_ = act[s:e,i].std(); ap = act[s:e,i].ptp()
        if ap > 5 and rp < 2: v = "물리 진동!"
        elif rp > 5: v = "입력 지터"
        elif ap < 1: v = "안정"
        else: v = "경미"
        print(f"{n:<8}{rs:8.2f}{rp:8.2f}{ts:8.2f}{as_:8.2f}{ap:9.2f}{v:>14}")

# 전 구간에서 act가 tgt를 크게 벗어난 정도 (물리 진동 총량)
print("\n=== B-2. 전체 세션: act-tgt 편차 (물리 추종 오차) ===")
print(f"{'ch':<8}{'|act-tgt| mean':>15}{'max':>8}{'act<min?':>10}{'act>max?':>10}")
for i, n in enumerate(CH):
    d = np.abs(act[:,i]-tgt[:,i])
    below = (act[:,i] < -3).mean()*100
    above = (act[:,i] > SVH_MAX_DEG[i]+3).mean()*100
    print(f"{n:<8}{d.mean():15.2f}{d.max():8.1f}{below:9.1f}%{above:9.1f}%")

# vision 쪽 입력 지터(손 정지 시 필터 전/후) — vision_log에서 raw std 작은 구간
print("\n=== B-3. vision_log: 손 정지 구간 raw vs filt 지터 (deg) ===")
best_v = None
for s in range(0, len(V)-90, 30):  # 90 frames ~3s
    seg = np.degrees(raw[s:s+90])
    if det[s:s+90].min() > 0 and seg.std(axis=0).max() < 1.5:
        if best_v is None or seg.std(axis=0).max() > best_v[1]:
            best_v = (s, seg.std(axis=0).max())
if best_v:
    s = best_v[0]; e = s+90
    print(f"구간 t={t_v[s]:.1f}..{t_v[e]:.1f}")
    print(f"{'ch':<18}{'raw_std':>9}{'raw_p2p':>9}{'filt_std':>9}{'filt_p2p':>9}")
    for i, n in enumerate(names):
        rs = np.degrees(raw[s:e,i]).std(); rp = np.degrees(raw[s:e,i]).ptp()
        fs = np.degrees(filt[s:e,i]).std(); fp = np.degrees(filt[s:e,i]).ptp()
        print(f"{n:<18}{rs:9.2f}{rp:9.2f}{fs:9.2f}{fp:9.2f}")
else:
    print("정지 구간 못 찾음 — 완화 기준으로 재시도 필요")
