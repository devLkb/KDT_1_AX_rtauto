#!/usr/bin/env python3
"""최종 발표용 GraspLift 그림 2장을 렌더링한다.

  1) dg5f_grasp_lift_slide_training.png  — 배포 정책 학습 곡선(누적 보상/성공률/에피소드 길이)
  2) dg5f_grasp_lift_slide_tradeoff.png  — top-down 계수 스윕의 자세 vs 신뢰성 트레이드오프

사용법: python tools/plot_grasp_lift_slides.py
"""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "training" / "results"
OUT = REPO / "docs" / "images"
BEHAVIOR = "DG5FGraspLift"

DEPLOYED = "dg5f_grasp_lift_h012_topdown"
SWEEP = [
    # 배포 런은 topdown_potential_max 를 설정하지 않으므로 spec 기본값 0.30 이다.
    (DEPLOYED, "배포\n(0.30)"),
    ("dg5f_grasp_lift_t1_topdown150", "T1\n(1.50)"),
    ("dg5f_grasp_lift_t3_topdown225", "T3\n(2.25)"),
    ("dg5f_grasp_lift_t2_topdown300", "T2\n(3.00)"),
]

INK = "#1b1b1f"
ACCENT = "#2563eb"
WARN = "#dc2626"
GRID = "#d4d4d8"


def scalars(run_id: str, tag: str):
    ea = EventAccumulator(str(RESULTS / run_id / BEHAVIOR), size_guidance={"scalars": 0})
    ea.Reload()
    if tag not in ea.Tags()["scalars"]:
        return [], []
    ev = ea.Scalars(tag)
    return [e.step for e in ev], [e.value for e in ev]


def tail_mean(run_id: str, tag: str, n: int = 10) -> float:
    _, values = scalars(run_id, tag)
    if not values:
        return float("nan")
    tail = values[-n:]
    return sum(tail) / len(tail)


def smooth(values, weight=0.7):
    out, last = [], None
    for v in values:
        last = v if last is None else last * weight + v * (1 - weight)
        out.append(last)
    return out


def style(ax):
    ax.grid(alpha=0.35, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=INK, labelsize=12)


def slide_training():
    panels = [
        ("Environment/Cumulative Reward", "누적 보상", None),
        ("GraspLift/Success", "성공률", (0.0, 1.05)),
        ("Environment/Episode Length", "에피소드 길이 (steps)", None),
    ]
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    for ax, (tag, title, ylim) in zip(axes, panels):
        steps, values = scalars(DEPLOYED, tag)
        xs = [s / 1e6 for s in steps]
        ax.plot(xs, values, color=ACCENT, alpha=0.20, linewidth=1)
        ax.plot(xs, smooth(values), color=ACCENT, linewidth=2.6)
        ax.set_title(title, fontsize=16, color=INK, pad=12)
        ax.set_xlabel("학습 스텝 (백만)", fontsize=12, color=INK)
        if ylim:
            ax.set_ylim(*ylim)
        style(ax)
        if values:
            # 마지막 1점은 요약 구간 노이즈가 그대로 실리므로 최근 10점 평균을 라벨로 쓴다.
            final = sum(values[-10:]) / len(values[-10:])
            ax.annotate(
                f"{final:.2f}" if tag != "GraspLift/Success" else f"{final:.3f}",
                xy=(xs[-1], final),
                xytext=(-6, 14),
                textcoords="offset points",
                ha="right",
                fontsize=13,
                fontweight="bold",
                color=ACCENT,
                bbox=dict(boxstyle="round,pad=0.18", facecolor="white", edgecolor="none", alpha=0.85),
            )
    fig.suptitle("DG5F Grasp + Lift 배포 정책 학습 곡선 (12 cm 블록, 1.5 M steps)", fontsize=18, color=INK)
    fig.tight_layout(rect=(0, 0.02, 1, 0.94))
    path = OUT / "dg5f_grasp_lift_slide_training.png"
    fig.savefig(path, dpi=160, facecolor="white")
    plt.close(fig)
    print(f"saved: {path}")


def slide_tradeoff():
    labels = [lab for _, lab in SWEEP]
    posture = [tail_mean(r, "GraspLift/GraspPostureAngleDegrees") for r, _ in SWEEP]
    success = [tail_mean(r, "GraspLift/Success") for r, _ in SWEEP]

    fig, ax = plt.subplots(figsize=(11, 6))
    xs = range(len(labels))
    bars = ax.bar(xs, posture, width=0.55, color=ACCENT, alpha=0.85, label="grasp posture 각도 (낮을수록 top-down)")
    ax.axhline(70, color="#6b7280", linestyle="--", linewidth=1.6)
    ax.text(len(labels) - 0.45, 71.0, "70° gate", color="#6b7280", fontsize=12, ha="right")
    ax.set_ylim(0, 90)
    ax.set_ylabel("grasp posture 각도 (°)", fontsize=13, color=INK)
    ax.set_xticks(list(xs))
    ax.set_xticklabels(labels, fontsize=12, color=INK)
    ax.set_xlabel("topdown_potential_max 설정", fontsize=13, color=INK)
    style(ax)
    for bar, v in zip(bars, posture):
        ax.text(bar.get_x() + bar.get_width() / 2, v + 1.2, f"{v:.1f}°", ha="center", fontsize=12, color=INK)

    ax2 = ax.twinx()
    ax2.plot(list(xs), success, color=WARN, marker="o", markersize=9, linewidth=2.6, label="성공률")
    ax2.set_ylim(0.90, 1.01)
    ax2.set_ylabel("성공률", fontsize=13, color=WARN)
    ax2.tick_params(colors=WARN, labelsize=12)
    for side in ("top", "left"):
        ax2.spines[side].set_visible(False)
    ax2.spines["right"].set_color(WARN)
    for x, v in zip(xs, success):
        ax2.annotate(f"{v:.3f}", xy=(x, v), xytext=(0, 14), textcoords="offset points", ha="center", fontsize=12, color=WARN)

    handles = [bars, ax2.lines[0]]
    ax.legend(handles, [h.get_label() for h in handles], fontsize=12, loc="lower left", frameon=False)
    ax.set_title("top-down 유도 계수를 키우면 자세는 좋아지고 신뢰성은 떨어진다", fontsize=17, color=INK, pad=16)
    fig.tight_layout()
    path = OUT / "dg5f_grasp_lift_slide_tradeoff.png"
    fig.savefig(path, dpi=160, facecolor="white")
    plt.close(fig)
    print(f"saved: {path}")


def main() -> int:
    plt.rcParams["font.family"] = ["NanumGothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    OUT.mkdir(parents=True, exist_ok=True)
    slide_training()
    slide_tradeoff()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
