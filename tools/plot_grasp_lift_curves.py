#!/usr/bin/env python3
"""GraspLift 학습 곡선(누적 보상 / 성공률 / 에피소드 길이)을 PNG로 렌더링한다.

사용법:
    python tools/plot_grasp_lift_curves.py [run_id ...] [-o out.png]
run_id 를 생략하면 최근 top-down 스윕(t1~t4)과 배포 모델 런을 그린다.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

REPO = Path(__file__).resolve().parents[1]
RESULTS = REPO / "training" / "results"
BEHAVIOR = "DG5FGraspLift"

DEFAULT_RUNS = [
    ("dg5f_grasp_lift_h012_topdown", "deployed (h012, topdown 0.75)"),
    ("dg5f_grasp_lift_t1_topdown150", "t1 topdown 1.50"),
    ("dg5f_grasp_lift_t2_topdown300", "t2 topdown 3.00"),
    ("dg5f_grasp_lift_t3_topdown225", "t3 topdown 2.25"),
    ("dg5f_grasp_lift_t4_topdown150_long", "t4 topdown 1.50 (long)"),
]

PANELS = [
    ("Environment/Cumulative Reward", "누적 보상 (Cumulative Reward)", "reward"),
    ("GraspLift/Success", "성공률 (Success rate)", "rate"),
    ("Environment/Episode Length", "에피소드 길이 (steps)", "steps"),
]


def load(run_dir: Path, tag: str):
    ea = EventAccumulator(str(run_dir), size_guidance={"scalars": 0})
    ea.Reload()
    if tag not in ea.Tags()["scalars"]:
        return [], []
    events = ea.Scalars(tag)
    return [e.step for e in events], [e.value for e in events]


def smooth(values, weight=0.6):
    out, last = [], None
    for v in values:
        last = v if last is None else last * weight + v * (1 - weight)
        out.append(last)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*", help="training/results 아래 run id")
    ap.add_argument("-o", "--out", default=str(REPO / "docs" / "images" / "dg5f_grasp_lift_curves.png"))
    ap.add_argument("--smooth", type=float, default=0.6)
    args = ap.parse_args()

    runs = [(r, r) for r in args.runs] if args.runs else DEFAULT_RUNS

    plt.rcParams["font.family"] = ["NanumGothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, axes = plt.subplots(1, len(PANELS), figsize=(16, 4.6))
    colors = plt.get_cmap("tab10").colors

    for idx, (run_id, label) in enumerate(runs):
        run_dir = RESULTS / run_id / BEHAVIOR
        if not run_dir.is_dir():
            print(f"[skip] {run_dir} 없음")
            continue
        color = colors[idx % len(colors)]
        for ax, (tag, title, kind) in zip(axes, PANELS):
            steps, values = load(run_dir, tag)
            if not steps:
                continue
            ax.plot(steps, values, color=color, alpha=0.18, linewidth=1)
            ax.plot(steps, smooth(values, args.smooth), color=color, linewidth=1.9, label=label)
            ax.set_title(title)
            ax.set_xlabel("steps")
            if kind == "rate":
                ax.set_ylim(-0.02, 1.02)
            ax.grid(alpha=0.25)

    axes[0].legend(fontsize=8, loc="lower right")
    fig.suptitle("DG5F GraspLift 학습 곡선", fontsize=13)
    fig.tight_layout()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140)
    print(f"saved: {out}")

    print("\n=== 최종값 (마지막 기록 지점) ===")
    for run_id, label in runs:
        run_dir = RESULTS / run_id / BEHAVIOR
        if not run_dir.is_dir():
            continue
        cells = []
        for tag, _, _ in PANELS:
            steps, values = load(run_dir, tag)
            cells.append(f"{tag.split('/')[-1]}={values[-1]:.3f}" if values else f"{tag}=n/a")
        last_step = load(run_dir, PANELS[0][0])[0]
        cells.append(f"steps={last_step[-1] if last_step else 0}")
        print(f"{label:32s} " + "  ".join(cells))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
