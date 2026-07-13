"""
Phase 3 추적 품질 진단.

vision_log.csv (Python: raw/filtered, rad) + unity_joint_log.csv (Unity: rx/tgt/act, deg)
를 unix time으로 정렬해 채널별로:

  1. 비전 노이즈: 정지 구간에서 raw 표준편차 (MediaPipe 랜드마크 떨림)
  2. 필터 효과: filtered 표준편차 (One Euro가 얼마나 잡는지)
  3. 스파이크: 프레임 간 raw 점프(>SPIKE_DEG) 횟수 — "펴고 있는데 접혔다 펴짐"의 원인
  4. 유니티 추종: rx(수신) vs act(실제 관절) RMSE·지연 — 물리/lerp 문제인지

사용: python analyze_tracking.py
출력: 콘솔 요약 표 + tracking_report.png (채널별 시계열)
"""
import csv
import math

import numpy as np

VISION_CSV = "vision_log.csv"
UNITY_CSV = "unity_joint_log.csv"
REPORT_PNG = "tracking_report.png"

CHANNELS = ["thmFlex", "thmOpp", "idxDist", "idxProx",
            "midDist", "midProx", "ring", "pinky", "spread"]
# vision_log 컬럼명은 Python CHANNEL_NAMES 기준이므로 순서만 같으면 됨(둘 다 패킷 순서)

SPIKE_DEG = 8.0        # 프레임 간 이만큼 점프하면 스파이크로 집계
STILL_WIN_SEC = 1.0    # 정지 구간 판정 윈도(필터값 변화가 작은 구간)
STILL_THRESH_DEG = 2.0 # 윈도 내 filtered 범위가 이보다 작으면 "정지"로 간주


def load_csv(path):
    with open(path, newline="", encoding="utf-8-sig") as f:  # Unity 로그는 BOM 포함
        rows = list(csv.reader(f))
    header, data = rows[0], rows[1:]
    arr = np.array([[float(v) for v in r] for r in data if len(r) == len(header)])
    return header, arr


def col(header, arr, name):
    return arr[:, header.index(name)]


def find_still_mask(t, filt_deg):
    """모든 채널 filtered가 STILL_WIN_SEC 동안 STILL_THRESH_DEG 이내면 정지 구간."""
    n = len(t)
    mask = np.zeros(n, dtype=bool)
    j0 = 0
    for i in range(n):
        while t[i] - t[j0] > STILL_WIN_SEC:
            j0 += 1
        w = filt_deg[j0:i + 1]
        if len(w) >= 5 and (w.max(axis=0) - w.min(axis=0)).max() < STILL_THRESH_DEG:
            mask[j0:i + 1] = True
    return mask


def lag_by_xcorr(t_a, a, t_b, b, max_lag=0.5):
    """b가 a를 얼마나 늦게 따라가는지(초). 공통 시간축 20ms로 리샘플 후 상호상관."""
    t0, t1 = max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1])
    if t1 - t0 < 3.0:
        return float("nan")
    ts = np.arange(t0, t1, 0.02)
    ra = np.interp(ts, t_a, a) - np.mean(a)
    rb = np.interp(ts, t_b, b) - np.mean(b)
    if np.std(ra) < 1e-6 or np.std(rb) < 1e-6:
        return float("nan")
    max_shift = int(max_lag / 0.02)
    best, best_s = -1e18, 0
    for s in range(0, max_shift + 1):
        c = np.dot(ra[:len(ra) - s if s else None], rb[s:])
        if c > best:
            best, best_s = c, s
    return best_s * 0.02


def main():
    vh, va = load_csv(VISION_CSV)
    uh, ua = load_csv(UNITY_CSV)

    tv = col(vh, va, "t_unix")
    tu = col(uh, ua, "t_unix")
    detected = col(vh, va, "detected")

    # vision은 rad -> deg
    raw = np.degrees(np.array([col(vh, va, c) for c in vh if c.startswith("raw_")]).T)
    filt = np.degrees(np.array([col(vh, va, c) for c in vh if c.startswith("filt_")]).T)
    rx = np.array([col(uh, ua, "rx_" + c) for c in CHANNELS]).T
    act = np.array([col(uh, ua, "act_" + c) for c in CHANNELS]).T

    # 공통 구간으로 자르기
    t0, t1 = max(tv[0], tu[0]), min(tv[-1], tu[-1])
    vm = (tv >= t0) & (tv <= t1)
    um = (tu >= t0) & (tu <= t1)
    tv, raw, filt, detected = tv[vm], raw[vm], filt[vm], detected[vm]
    tu, rx, act = tu[um], rx[um], act[um]
    print(f"공통 구간 {t1 - t0:.1f}s | vision {len(tv)}행({len(tv)/(t1-t0):.0f}Hz) "
          f"unity {len(tu)}행 | 검출율 {100*detected.mean():.1f}%")

    still = find_still_mask(tv, filt)
    print(f"정지 구간 비율: {100*still.mean():.1f}%  (스파이크/노이즈는 이 구간에서 측정)\n")

    hdr = f"{'채널':10s} {'raw σ':>7s} {'filt σ':>7s} {'스파이크/분':>9s} {'act-rx RMSE':>11s} {'지연(s)':>8s}"
    print(hdr)
    print("-" * len(hdr))
    dt_med = np.median(np.diff(tv))
    for i, name in enumerate(CHANNELS):
        raw_sd = raw[still, i].std() if still.any() else float("nan")
        filt_sd = filt[still, i].std() if still.any() else float("nan")
        # 스파이크: 검출 프레임에서의 raw 점프(hold 프레임 제외)
        dj = np.abs(np.diff(raw[:, i]))
        det2 = (detected[1:] > 0) & (detected[:-1] > 0)
        spikes = int(((dj > SPIKE_DEG) & det2).sum())
        spikes_per_min = spikes / ((tv[-1] - tv[0]) / 60.0)
        # unity 추종 오차·지연
        act_i = np.interp(tu, tu, act[:, i])
        rx_i = rx[:, i]
        rmse = math.sqrt(float(np.mean((act_i - rx_i) ** 2)))
        lag = lag_by_xcorr(tu, rx_i, tu, act_i)
        print(f"{name:10s} {raw_sd:7.2f} {filt_sd:7.2f} {spikes_per_min:9.1f} "
              f"{rmse:11.2f} {lag:8.2f}")

    print("\n해석 가이드:")
    print("  raw σ 큼 + filt σ 작음  -> 필터가 잘 잡는 중(비전 노이즈는 있음)")
    print("  filt σ 큼               -> 필터 튜닝 필요(min_cutoff 낮추기)")
    print("  스파이크/분 큼          -> 오검출. One Euro론 못 잡음 -> median/outlier 게이트 필요")
    print("  RMSE 큼 + 지연 큼       -> 유니티 쪽(lerp/stiffness) 문제")

    # 채널별 시계열 그림
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(9, 1, figsize=(14, 22), sharex=True)
        for i, (ax, name) in enumerate(zip(axes, CHANNELS)):
            ax.plot(tv - t0, raw[:, i], lw=0.5, alpha=0.5, label="raw(vision)")
            ax.plot(tv - t0, filt[:, i], lw=1.0, label="filtered(sent)")
            ax.plot(tu - t0, act[:, i], lw=1.0, label="actual(unity)")
            ax.set_ylabel(name + " (deg)")
            if i == 0:
                ax.legend(loc="upper right", fontsize=8)
        axes[-1].set_xlabel("t (s)")
        fig.tight_layout()
        fig.savefig(REPORT_PNG, dpi=100)
        print(f"\n그림 저장: {REPORT_PNG}")
    except ImportError:
        print("\n(matplotlib 없음 - 그림 생략. pip install matplotlib)")


if __name__ == "__main__":
    main()
