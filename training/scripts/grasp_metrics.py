#!/usr/bin/env python3
"""Show TensorBoard scalars and a compact diagnosis for the current grasp run."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import statistics
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator


@dataclass(frozen=True)
class Metric:
    label: str
    tag: str
    scale: float = 1.0
    unit: str = ""
    lower_is_better: bool | None = None


METRICS = (
    Metric("누적 보상", "Environment/Cumulative Reward", lower_is_better=False),
    Metric("에피소드 길이", "Environment/Episode Length", unit=" step", lower_is_better=True),
    Metric("도달 성공률", "Reach/Success", scale=100.0, unit="%", lower_is_better=False),
    Metric("위치 고정 성공률", "Reach/LockSuccess", scale=100.0, unit="%", lower_is_better=False),
    Metric("파지 성공률", "Grasp/Success", scale=100.0, unit="%", lower_is_better=False),
    Metric("완료 시간", "Reach/CompletionSeconds", unit=" s", lower_is_better=True),
    Metric("종료 거리", "Reach/FinalDistanceMeters", scale=100.0, unit=" cm", lower_is_better=True),
    Metric("표면 여유 거리", "Reach/FinalSurfaceClearanceMeters", scale=100.0, unit=" cm", lower_is_better=True),
    Metric("최소 거리", "Reach/BestDistanceMeters", scale=100.0, unit=" cm", lower_is_better=True),
    Metric("위치 유지 시간", "Reach/HoldSeconds", unit=" s", lower_is_better=False),
    Metric("위치 유지 재시도", "Reach/HoldResets", unit=" 회", lower_is_better=True),
    Metric("최소 이동 여유", "Reach/MinimumTransitClearanceMeters", scale=100.0, unit=" cm", lower_is_better=False),
    Metric("손바닥 정렬", "Reach/PalmAlignment", lower_is_better=False),
    Metric("상단 원뿔 정렬", "Reach/UpperConeAlignment", lower_is_better=False),
    # ReadyReach records this tag with Count aggregation. It can exceed 1 and
    # must not be presented as a percentage.
    Metric("조기 하강 실패 건수", "Failure/PrematureDescent", unit=" 건", lower_is_better=True),
    # Legacy environments only emit Failure/Timeout on a failure. Therefore
    # its scalar value is not an all-episode denominator-backed rate.
    Metric("시간 초과 기록값", "Failure/Timeout", lower_is_better=True),
    Metric("정책 엔트로피", "Policy/Entropy"),
    Metric("가치 추정", "Policy/Extrinsic Value Estimate"),
    Metric("정책 손실", "Losses/Policy Loss"),
    Metric("가치 손실", "Losses/Value Loss", lower_is_better=True),
    Metric("학습률", "Policy/Learning Rate"),
)


ScalarPoint = tuple[int, float, float]  # step, wall_time, value


def _process_run_ids(results_dir: Path) -> set[str]:
    """Return run IDs belonging to live mlagents-learn processes."""
    run_ids: set[str] = set()
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return run_ids
    results_arg = str(results_dir.resolve())
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            raw = (proc / "cmdline").read_bytes()
        except (OSError, PermissionError):
            continue
        if b"mlagents-learn" not in raw and b"mlagents_learn_compat.py" not in raw:
            continue
        argv = [item.decode(errors="replace") for item in raw.split(b"\0") if item]
        if not any(
            arg == f"--results-dir={results_arg}"
            or (arg == "--results-dir" and index + 1 < len(argv) and argv[index + 1] == results_arg)
            for index, arg in enumerate(argv)
        ):
            continue
        for index, arg in enumerate(argv):
            if arg.startswith("--run-id="):
                run_ids.add(arg.split("=", 1)[1])
            elif arg == "--run-id" and index + 1 < len(argv):
                run_ids.add(argv[index + 1])
    return run_ids


def event_files(run_dir: Path) -> list[Path]:
    return sorted(
        run_dir.glob("*/events.out.tfevents.*"), key=lambda path: path.stat().st_mtime
    )


def select_run(results_dir: Path, requested: str | None) -> tuple[Path, bool]:
    if requested:
        candidate = Path(requested).expanduser()
        run_dir = candidate if candidate.is_dir() else results_dir / requested
        if not run_dir.is_dir():
            raise FileNotFoundError(f"run을 찾을 수 없습니다: {run_dir}")
        return run_dir.resolve(), run_dir.name in _process_run_ids(results_dir)

    candidates = [path for path in results_dir.iterdir() if path.is_dir() and event_files(path)]
    if not candidates:
        raise FileNotFoundError(f"TensorBoard event가 없습니다: {results_dir}")
    active_ids = _process_run_ids(results_dir)
    active = [path for path in candidates if path.name in active_ids]
    pool = active or candidates
    selected = max(pool, key=lambda path: max(p.stat().st_mtime for p in event_files(path)))
    return selected.resolve(), selected.name in active_ids


def load_scalars(
    run_dir: Path, *, latest_session_only: bool = False
) -> tuple[dict[str, list[ScalarPoint]], list[str]]:
    """Load scalars, optionally limiting them to the latest resumed session."""
    merged: dict[str, dict[int, tuple[float, float]]] = {}
    errors: list[str] = []
    files = event_files(run_dir)
    if latest_session_only and files:
        files = files[-1:]
    for path in files:
        accumulator = EventAccumulator(str(path), size_guidance={"scalars": 0})
        try:
            accumulator.Reload()
        except Exception as exc:  # A partially-written live file must not kill the CLI.
            errors.append(f"{path.name}: {exc}")
            continue
        for tag in accumulator.Tags().get("scalars", []):
            points = merged.setdefault(tag, {})
            for event in accumulator.Scalars(tag):
                points[event.step] = (event.wall_time, event.value)
    return (
        {
            tag: [(step, wall_time, value) for step, (wall_time, value) in sorted(points.items())]
            for tag, points in merged.items()
        },
        errors,
    )


def rolling(points: list[ScalarPoint], window: int, previous: bool = False) -> float | None:
    finite = [value for _, _, value in points if math.isfinite(value)]
    if previous:
        finite = finite[-2 * window : -window]
    else:
        finite = finite[-window:]
    return statistics.fmean(finite) if finite else None


def target_steps(run_dir: Path) -> int | None:
    manifest = run_dir / "training_manifest.json"
    if manifest.is_file():
        try:
            value = json.loads(manifest.read_text(encoding="utf-8")).get("target_steps")
            if value is not None:
                return int(value)
        except (OSError, ValueError, TypeError):
            pass

    configuration = run_dir / "configuration.yaml"
    if configuration.is_file():
        # Avoid a PyYAML dependency just to read one integer.
        match = re.search(r"(?m)^\s*max_steps:\s*([0-9_]+)\s*$", configuration.read_text(errors="ignore"))
        if match:
            return int(match.group(1).replace("_", ""))
    return None


def console_issue(root: Path, run_id: str) -> str | None:
    """Return the last fatal-looking console exception for a run, if present."""
    log_dir = root / "training" / "logs"
    candidates = [
        path
        for path in (log_dir / f"{run_id}.log", log_dir / f"{run_id}.console.log")
        if path.is_file()
    ]
    if not candidates:
        return None
    path = max(candidates, key=lambda item: item.stat().st_mtime)
    try:
        with path.open("rb") as stream:
            stream.seek(max(0, path.stat().st_size - 200_000))
            text = stream.read().decode(errors="replace")
    except OSError:
        return None
    # A resumed run appends to the same console log. Historical fatal errors
    # before the most recent successful resume must not be reported as current.
    resume_marker = "Resuming training from step "
    if resume_marker in text:
        text = text[text.rfind(resume_marker) :]
    if "Traceback (most recent call last):" not in text:
        return None
    exceptions = re.findall(
        r"(?m)^([A-Za-z_][A-Za-z0-9_.]*(?:Error|Exception): .+)$", text
    )
    return f"{path.name}: {exceptions[-1]}" if exceptions else f"{path.name}: Python traceback 발견"


def step_rate(data: dict[str, list[ScalarPoint]]) -> float | None:
    preferred = data.get("Environment/Cumulative Reward")
    points = preferred or max(data.values(), key=len, default=[])
    points = points[-100:]
    if len(points) < 2:
        return None
    elapsed = points[-1][1] - points[0][1]
    advanced = points[-1][0] - points[0][0]
    return advanced / elapsed if elapsed > 0 and advanced > 0 else None


def ready_reach_curriculum_status(
    run_dir: Path, data: dict[str, list[ScalarPoint]]
) -> str | None:
    """Describe the active 5 cm -> 3 cm ReadyReach lesson."""
    tag = "Environment/Lesson Number/reach_stage"
    points = data.get(tag, [])
    if not points or "ready_reach_curriculum" not in run_dir.name:
        return None
    lesson = max(0, int(round(points[-1][2])))
    stage = min(3, lesson + 1)
    contracts = {
        1: (5.0, "완화", 0.02),
        2: (3.0, "15.0 cm/s", 0.10),
        3: (1.0, "5.0 cm/s", 0.25),
    }
    distance_cm, speed, hold_seconds = contracts[stage]
    suffix = " (기본 학습의 최종 단계)" if stage == 2 else ""
    return (
        f"stage {stage} | 목표 거리 ≤ {distance_cm:.1f} cm | "
        f"속도 {speed} | 유지 {hold_seconds:.2f} s{suffix}"
    )


def format_number(value: float | None, metric: Metric) -> str:
    if value is None:
        return "-"
    scaled = value * metric.scale
    if metric.unit == "%":
        return f"{scaled:.2f}%"
    if metric.unit in {" cm", " s"}:
        return f"{scaled:.3f}{metric.unit}"
    if metric.unit == " step":
        return f"{scaled:.1f}{metric.unit}"
    if abs(scaled) < 0.001 and scaled != 0:
        return f"{scaled:.3e}"
    return f"{scaled:.4f}{metric.unit}"


def trend_text(current: float | None, previous: float | None, metric: Metric) -> str:
    if current is None or previous is None:
        return "-"
    delta = (current - previous) * metric.scale
    tolerance = max(1e-9, abs(previous * metric.scale) * 0.01)
    if abs(delta) <= tolerance:
        marker = "→"
    elif metric.lower_is_better is None:
        marker = "↑" if delta > 0 else "↓"
    else:
        improved = delta < 0 if metric.lower_is_better else delta > 0
        marker = "좋아짐" if improved else "나빠짐"
    suffix = "pp" if metric.unit == "%" else metric.unit.strip()
    return f"{marker} {delta:+.3g}{suffix}"


def get_mean(data: dict[str, list[ScalarPoint]], tag: str, window: int) -> float | None:
    return rolling(data.get(tag, []), window)


def analysis_lines(data: dict[str, list[ScalarPoint]], window: int, current_step: int) -> list[str]:
    lines: list[str] = []
    success_tag = next(
        (tag for tag in ("Reach/LockSuccess", "Reach/Success", "Grasp/Success") if tag in data),
        None,
    )
    if success_tag:
        success = get_mean(data, success_tag, window)
        if success is not None:
            if success < 0.01:
                lines.append(f"성공 신호가 최근 구간에서 {success * 100:.2f}%로 아직 관측되지 않았습니다.")
            elif success < 0.5:
                lines.append(f"성공률은 최근 {success * 100:.2f}%로 초기 학습 수준입니다.")
            else:
                lines.append(f"성공률은 최근 {success * 100:.2f}%입니다.")

    completion = get_mean(data, "Reach/CompletionSeconds", window)
    if completion is not None and completion >= 19.0:
        lines.append(f"평균 완료 시간이 {completion:.2f}s로 20초 episode 제한에 붙어 있어 대부분 시간 초과로 보입니다.")

    final_distance = get_mean(data, "Reach/FinalDistanceMeters", window)
    best_distance = get_mean(data, "Reach/BestDistanceMeters", window)
    if final_distance is not None and best_distance is not None:
        gap = final_distance - best_distance
        if gap > 0.02:
            lines.append(
                f"최소 거리 {best_distance * 100:.1f}cm까지 접근하지만 종료 시 {final_distance * 100:.1f}cm로 "
                f"{gap * 100:.1f}cm 다시 멀어집니다. 접근 후 위치 유지가 병목입니다."
            )

    failure_points = data.get("Failure/PrematureDescent", [])
    if failure_points:
        failure = rolling(failure_points, window)
        last_failure_step = failure_points[-1][0]
        if failure is not None:
            lines.append(
                f"조기 하강 실패가 {last_failure_step:,} step에도 기록됐으며, 발생한 summary당 "
                f"최근 평균은 {failure:.2f}건입니다(이 tag는 실패율이 아니라 Count입니다)."
            )

    reward_points = data.get("Environment/Cumulative Reward", [])
    reward_now = rolling(reward_points, window)
    reward_before = rolling(reward_points, window, previous=True)
    if reward_now is not None and reward_before is not None:
        delta = reward_now - reward_before
        if abs(delta) <= max(0.02, abs(reward_before) * 0.02):
            lines.append(f"보상은 앞 구간 대비 {delta:+.3f}로 정체입니다.")
        else:
            direction = "개선" if delta > 0 else "악화"
            lines.append(f"보상은 앞 구간 대비 {delta:+.3f}로 {direction} 중입니다.")

    non_finite = sum(
        1 for points in data.values() for _, _, value in points[-window:] if not math.isfinite(value)
    )
    if non_finite:
        lines.append(f"주의: 최근 scalar에 NaN/Inf가 {non_finite}개 있습니다.")
    else:
        lines.append("최근 TensorBoard scalar에는 NaN/Inf가 없습니다.")

    if current_step < 100_000:
        lines.append("아직 100k step 이전이므로 수렴 판정보다는 학습 신호 형성 여부만 보는 단계입니다.")
    return lines


def print_all_tags(data: dict[str, list[ScalarPoint]]) -> None:
    print("\n--- 전체 최신 scalar ---")
    for tag in sorted(data):
        if not data[tag]:
            continue
        step, _, value = data[tag][-1]
        print(f"{tag:<46} step={step:>9,}  value={value:.8g}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", help="run ID 또는 run 디렉터리 (기본: 실행 중인 최신 run)")
    parser.add_argument("--root", type=Path, default=Path(os.environ.get("DG5F_ROOT", Path(__file__).resolve().parents[2])))
    parser.add_argument("--window", type=int, default=100, help="최근/이전 평균에 쓸 summary 개수")
    parser.add_argument("--all", action="store_true", help="모든 최신 scalar도 출력")
    args = parser.parse_args()
    if args.window < 1:
        parser.error("--window는 1 이상이어야 합니다")

    results_dir = args.root.resolve() / "training" / "results"
    try:
        run_dir, active = select_run(results_dir, args.run)
        # Restarting Unity re-synchronizes episode completions and creates a
        # new event file. Mixing its sparse early summaries with the previous
        # process makes rolling deltas look much larger than they are.
        session_files = event_files(run_dir)
        latest_session_only = active and len(session_files) > 1
        data, errors = load_scalars(run_dir, latest_session_only=latest_session_only)
    except (FileNotFoundError, OSError) as exc:
        print(f"[ERROR] {exc}")
        return 2
    if not data:
        print(f"[ERROR] 읽을 수 있는 scalar가 없습니다: {run_dir}")
        return 2

    step = max(point[-1][0] for point in data.values() if point)
    target = target_steps(run_dir)
    rate = step_rate(data)
    latest_event = max(event_files(run_dir), key=lambda path: path.stat().st_mtime)
    updated = datetime.fromtimestamp(latest_event.stat().st_mtime, timezone.utc)

    print(f"Grasp TensorBoard log  {datetime.now(timezone.utc):%Y-%m-%d %H:%M:%S UTC}")
    print("=" * 88)
    print(f"run    : {run_dir.name} ({'학습 중' if active else '중지됨'})")
    progress = f" / {target:,} ({step / target * 100:.2f}%)" if target else ""
    rate_text = f" | {rate:,.1f} step/s" if rate else ""
    eta_text = ""
    if target and rate and step < target:
        eta_text = f" | ETA {(target - step) / rate / 60:.1f}분"
    print(f"step   : {step:,}{progress}{rate_text}{eta_text}")
    print(f"event  : {latest_event.name} | 갱신 {updated:%Y-%m-%d %H:%M:%S UTC}")
    curriculum = ready_reach_curriculum_status(run_dir, data)
    if curriculum:
        print(f"policy : {curriculum}")
    if latest_session_only:
        print(f"scope  : 최근 재개 세션만 사용 (이전 event {len(session_files) - 1}개와 혼합하지 않음)")
    print(f"window : 최근 {args.window}개 summary와 그 직전 {args.window}개 비교")
    print()
    print(f"{'지표':<22} {'최신':>15} {'최근 평균':>15} {'이전 대비':>22}")
    print("-" * 80)
    shown = 0
    for metric in METRICS:
        points = data.get(metric.tag, [])
        if not points:
            continue
        latest = points[-1][2]
        now = rolling(points, args.window)
        before = rolling(points, args.window, previous=True)
        print(
            f"{metric.label:<22} {format_number(latest, metric):>15} "
            f"{format_number(now, metric):>15} {trend_text(now, before, metric):>22}"
        )
        shown += 1
    if not shown:
        print("표시할 표준 DG5F scalar가 없습니다. --all로 전체 tag를 확인하세요.")

    print("\n--- 자동 분석 ---")
    diagnoses = analysis_lines(data, args.window, step)
    issue = console_issue(args.root.resolve(), run_dir.name)
    if issue:
        diagnoses.append(f"중단 오류 발견: {issue}")
    elif not active and target and step < target:
        diagnoses.append("목표 step 전에 학습 프로세스가 중지되어 있습니다. console log 확인이 필요합니다.")
    for index, line in enumerate(diagnoses, start=1):
        print(f"{index}. {line}")
    if errors:
        print(f"\n[WARN] {len(errors)}개 event 파일을 읽지 못했습니다. 학습 중 부분 기록이면 일시적일 수 있습니다.")
    if args.all:
        print_all_tags(data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
