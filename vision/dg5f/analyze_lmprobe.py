# -*- coding: utf-8 -*-
"""lmprobe 랜드마크 로그 분석 — 손바닥 5점(0,5,9,13,17) 민감도 + 5_1 프록시 후보 SNR.

무엇을 판정하나:
  ① 랜드마크 안정성: still 세션에서 손바닥 5점의 축별 흔들림(std, 손길이 % 단위)
     — MediaPipe z가 xy 대비 얼마나 나쁜지 실측.
  ② 5_1(새끼 손바닥접기) 프록시 후보 4종의 세션별 분포:
       still 세션 std = 노이즈 바닥, cup 세션 p5~p95 폭 = 신호,
       pinkybend 세션 p5~p95 폭 = crosstalk(작아야 함).
     still+cup이 같이 주어지면 SNR(=신호/노이즈std)까지 계산.

후보 (2026-07-18 몬테카를로 사전분석 — 실데이터로 재검증하는 것이 이 스크립트의 목적):
  dihedral_user : (0→5)×(0→13) vs (0→13)×(0→17) 사이각 (원안)
  dihedral_cond : (9→5)×(0→9) vs (0→13)×(0→17) 사이각 (요측 기준을 0-5-9로 고정한 개선안)
  elev_0_17     : arcsin(unit(0→17)·요측법선)
  proxy_cur     : 새끼 MCP 굽힘 × 0.5 (현행 raw[16] — pinkybend에서 크게 새는 게 예상 결함)

사용:
  python analyze_lmprobe.py [lmprobe_*.csv ...]
  인자 없으면 logs/에서 라벨(still/cup/pinkybend)별 최신 파일을 자동 선택.
"""
import glob
import os
import sys

import numpy as np
import pandas as pd

from dg5f_paths import LOG_DIR

PALM = [0, 5, 9, 13, 17]
LABELS = ["still", "cup", "pinkybend"]


def load(path):
    df = pd.read_csv(path)
    df = df[df["detected"] == 1]
    lm = df[[f"lm{i}_{a}" for i in range(21) for a in "xyz"]].to_numpy()
    return lm.reshape(len(df), 21, 3)


def unit(v):
    return v / np.maximum(np.linalg.norm(v, axis=-1, keepdims=True), 1e-12)


def signed_dihedral(n1, n2, hinge):
    """hinge 축 기준 부호 있는 평면 사이각[deg] (프레임 배열 단위)."""
    x = np.sum(n1 * n2, axis=-1)
    y = np.sum(np.cross(n1, n2) * unit(hinge), axis=-1)
    return np.degrees(np.arctan2(y, x))


def estimators(lm):
    """lm: (N,21,3) → {이름: (N,) [deg]}. vision 파이프라인과 동일하게 원시 정규화 좌표 사용."""
    w, i5, m9, r13, p17 = (lm[:, 0], lm[:, 5], lm[:, 9], lm[:, 13], lm[:, 17])
    n_user_rad = unit(np.cross(i5 - w, r13 - w))
    n_cond_rad = unit(np.cross(i5 - m9, m9 - w))
    n_uln = unit(np.cross(r13 - w, p17 - w))
    hinge = r13 - w
    # 현행 프록시: _bend(0, 17, 18) × 0.5
    v1, v2 = lm[:, 0] - lm[:, 17], lm[:, 18] - lm[:, 17]
    bend = 180.0 - np.degrees(np.arccos(np.clip(
        np.sum(unit(v1) * unit(v2), axis=-1), -1.0, 1.0)))
    return {
        "dihedral_user": signed_dihedral(n_user_rad, n_uln, hinge),
        "dihedral_cond": signed_dihedral(n_cond_rad, n_uln, hinge),
        "elev_0_17": np.degrees(np.arcsin(np.clip(
            np.sum(unit(p17 - w) * n_cond_rad, axis=-1), -1.0, 1.0))),
        "proxy_cur": bend * 0.5,
    }


def pick_latest():
    out = {}
    for lab in LABELS:
        files = sorted(glob.glob(os.path.join(LOG_DIR, f"lmprobe_{lab}_*.csv")))
        if files:
            out[lab] = files[-1]
    return out


def label_of(path):
    base = os.path.basename(path)
    for lab in LABELS:
        if base.startswith(f"lmprobe_{lab}_"):
            return lab
    return base


def main():
    if len(sys.argv) > 1:
        files = {label_of(p): p for p in sys.argv[1:]}
    else:
        files = pick_latest()
    if not files:
        print(f"[오류] lmprobe CSV 없음 — probe_landmarks.py로 먼저 녹화 ({LOG_DIR})")
        return

    stats = {}
    for lab, path in files.items():
        lm = load(path)
        total = len(pd.read_csv(path))
        print(f"\n=== {lab}: {os.path.basename(path)} — 검출 {len(lm)}/{total} "
              f"({100.0 * len(lm) / max(total, 1):.0f}%) ===")
        if len(lm) < 30:
            print("  프레임 부족 — 세션 20초 이상 재녹화 필요")
            continue

        hand_len = float(np.linalg.norm(lm[:, 9] - lm[:, 0], axis=-1).mean())
        if lab == "still":
            print(f"  손바닥 5점 흔들림 std (손길이={hand_len:.3f} 기준 %):")
            for idx in PALM:
                sd = lm[:, idx].std(axis=0) / hand_len * 100
                print(f"    lm{idx:2d}: x {sd[0]:5.2f}  y {sd[1]:5.2f}  z {sd[2]:5.2f}"
                      f"   (z/xy배율 {sd[2] / max(sd[:2].mean(), 1e-9):4.1f}x)")

        ests = estimators(lm)
        stats[lab] = ests
        print("  5_1 프록시 후보 [deg]:  평균    std    p5     p95    폭")
        for name, v in ests.items():
            p5, p95 = np.percentile(v, [5, 95])
            print(f"    {name:14s}: {v.mean():7.2f} {v.std():6.2f} {p5:7.2f} {p95:7.2f}"
                  f" {p95 - p5:6.2f}")

    if "still" in stats and "cup" in stats:
        print("\n=== 판정: SNR = cup(p5~p95 폭) / still(std) — 필터 전 원시값 기준 ===")
        for name in stats["still"]:
            noise = stats["still"][name].std()
            p5, p95 = np.percentile(stats["cup"][name], [5, 95])
            sig = p95 - p5
            xt = ""
            if "pinkybend" in stats:
                # 폭에는 노이즈 바닥이 섞여 있으므로 still 폭 **대비 배율**로 본다
                # (순수 노이즈면 ~1.0, 새끼 굽힘이 새면 >1)
                q5, q95 = np.percentile(stats["pinkybend"][name], [5, 95])
                s5, s95 = np.percentile(stats["still"][name], [5, 95])
                xt = (f"  crosstalk폭 {q95 - q5:6.2f}°"
                      f" (still폭 대비 {(q95 - q5) / max(s95 - s5, 1e-9):4.2f}x)")
            print(f"  {name:14s}: 신호 {sig:6.2f}° / 노이즈 {noise:5.2f}° = SNR {sig / max(noise, 1e-9):4.1f}{xt}")
        print("  기준: SNR ≥ 5면 필터 후 실용, 2~5면 필터 튜닝 필요, <2면 신호가 노이즈에 묻힘.")
        print("  crosstalk 배율 ≤1.2x면 무시 가능, ≥1.5x면 새끼 굽힘이 5_1로 새는 후보 — 탈락.")


if __name__ == "__main__":
    main()
