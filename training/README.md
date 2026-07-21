# DG5FGraspPointReach training

현재 training 계약은 하나다. 빨간 목표 좌표로 UR5e 팔 6축을 제어해 palm의
`GraspPoint`를 빠르고 정확하게 배치한다. DG5F 손가락 20관절은 policy I/O에서 제외되며
도달 후 `vision/dg5f` 텔레옵이 제어한다.

- v1 PPO config: `config/dg5f_grasp.yaml`
- v2 hand-first transfer config:
  `config/dg5f_grasp_v2_handfirst_lr5e5.yaml`
- launcher: `scripts/train_dg5f_grasp.sh`
- generated builds/results are ignored by Git
- each Unity environment contains 20 independent training-area prefab instances
- default trainer execution is CUDA with one Unity environment, for 20 agents total
- the implementation gate includes a 512-step built-player communication smoke; it is not a convergence claim
- the launcher uses the active virtualenv first, so run `vision` to activate `ax310` before training
- Unity 6000.4.0f1 crashes with `-nographics` on this Linux server; the launcher automatically uses Xvfb when `DISPLAY` is absent
- `VENV`, `TORCH_DEVICE`, `UNITY_DISPLAY_MODE`, `CONFIG`, `RESULTS_DIR`, `RUN_ID`, `ENV_PATH`, `NUM_ENVS`, and `TIME_SCALE` are overridable

- Behavior `DG5FGraspPointReach`, spec `1.0.0`
- 26 observations / 6 continuous arm actions
- target radius `0.20..0.85 m`, azimuth `0..360°`, each sampled uniformly
- success: distance `<= 0.01 m` and speed `<= 0.05 m/s` held for `0.25 s`
- episode limit: 20 simulation seconds
- fresh PPO only; no legacy checkpoint initialization or curriculum

정확한 순서와 reward는 [`docs/AGENT_SPEC.md`](../docs/AGENT_SPEC.md), 전체 계획은
[`docs/train_plan.md`](../docs/train_plan.md), 환경 생성부터 평가는
[`docs/ML_AGENTS_TRAINING_GUIDE.md`](../docs/ML_AGENTS_TRAINING_GUIDE.md)를 따른다.

## Active files

- PPO config: `config/dg5f_grasp_point_reach.yaml`
- trainer launcher: `scripts/train_dg5f_grasp_point_reach.sh`
- exact smoke: `scripts/smoke_dg5f_grasp_point_reach.sh`
- smoke config generator: `scripts/generate_grasp_point_reach_smoke_config.py`
- deterministic evaluation: `scripts/run_dg5f_grasp_point_reach_evaluation.sh`
- evaluation validator: `scripts/evaluate_dg5f_grasp_point_reach.py`
- Linux player: `builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64`

`builds/`, `results/`, `logs/`의 예전 DG5FGrasp/StableGrasp/Joint26 파일은 추적되지
않는 역사적 산출물일 수 있다. 새 Behavior와 호환되는 checkpoint로 취급하지 않는다.

## Setup

```bash
cd /home/lkb/workspace/KDT_1_AX_rtauto
source vision/.vision/bin/activate
pip check
```

Unity에서 **Tools > ML-Agents > Build DG5F GraspPoint Reach Linux Player**를 실행한 뒤
아래 player가 존재하고 실행 가능한지 확인한다.

```bash
test -x training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64
```

Each stage selects its standard run ID, config, player, and tmux session. `init`
automatically selects the previous stage (`v2 <- v1`, `v3 <- v2`, `v4 <- v3`).
An explicit source remains possible, for example `dg5f v3 init custom_v2_run`.
Stage-prefixed commands deliberately ignore stale exported stage variables. Custom
experiments use the legacy no-stage form with `DG5F_RUN_ID`, `DG5F_CONFIG`,
`DG5F_PLAYER`, and `DG5F_SESSION`.
Unity physics and environment stepping remain CPU workloads. With the default
`buffer_size: 10240`, CUDA utilization is bursty because PPO updates begin only after
enough trajectories have been collected; allocated CUDA memory already confirms that
the policy is resident on the GPU. Progress is printed every 200 agent steps.

`threaded: false` is intentional for ax310: its PyTorch default-device setting is
thread-local, while this ML-Agents version otherwise creates CPU trajectory tensors in
the background update thread and fails against CUDA model weights. The compatibility
launcher also initializes the selected device inside new Python threads, protecting old
smoke configs that still contain `threaded: true`. It selects PyTorch's legacy ONNX
exporter because ML-Agents pins ONNX 1.15.

Current code implements joint26 v2: thumb contact (finger index 0) and any
opposing finger (indices 1..4) must remain simultaneous for 0.5 simulation
seconds. V2's `DG5FGraspJoint` has 116 observations and 26 continuous actions
(arm 6 + hand 20). The packaged V3 behavior is renamed to
`DG5FStableGrasp`, but keeps the same tensor shapes.

Start v2 from the frozen 526647-step v1 checkpoint:

```bash
ENV_PATH="$PWD/training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64" \
training/scripts/smoke_dg5f_grasp_point_reach.sh
```

`init` first creates and verifies the read-only
`dg5f_v1_joint26_bootstrap`. It copies the V1 arm/task encoder columns, critic,
and six arm-action outputs, initializes the new hand inputs/outputs, drops
optimizer moments, and starts the new V2 global step at zero. The retired
closure run is preserved as `dg5f_v2_closure_failed_343k` and is rejected by
`resume`. The joint26 pilots at learning rates `1e-4` and `5e-5` are preserved
as `dg5f_v2_joint26_gpu_fixed` and
`dg5f_v2_joint26_lr5e5_gpu_fixed`; neither can be resumed or selected for V3.
The former hand-first run
`dg5f_v2_joint26_handfirst3_lr5e5_gpu_fixed` stopped at step 839959 while the
result/player directories were being replaced and is now read-only. Operational
`dg5f v2` points to `dg5f_v2_stablegrasp_v3_lr5e5_gpu_fixed`: it starts at
step zero from the same verified joint26 bootstrap, without optimizer state,
and uses the StableGrasp 3.0.0 player at `5e-5`.

The hand-first curriculum locks all six arm targets during stage 1, leaves the
20 hand actions independent with at most 1 degree per decision, disables only
the approach potential, and uses a 5-second stage timeout. Stages 2 and 3 keep
the 20-second episode timeout, plus a distinct 5-second post-reach grasp
timeout. Curriculum transitions use the unsmoothed mean reward of the most
recent 200 episodes (`2.2` for stage 1 and `1.8` for stage 2).
Its rebuilt player is intentionally isolated at
`builds/DG5FGraspJoint26HandFirst`; the launcher will reject startup until that
new player has been built or uploaded, so it cannot silently use the retired
joint26 binary.

## Packaged V3 environment

`DG5FStableGrasp-v3.0.0-linux-x86_64-20260718.tar.xz` is installed separately
at `builds/DG5FGraspV3`; it must not replace the V2
`DG5FGraspJoint26HandFirst` directory. The package uses behavior
`DG5FStableGrasp`, curriculum parameter `stable_grasp_stage`, 116 observations,
and 26 continuous actions. `scripts/train_dg5f_grasp.sh` validates the installed
binary hashes and these trainer names using
`manifests/DG5FStableGrasp-v3.0.0.json` (or the identical manifest beside the
installed player).

The packaged environment is the current operational `v2` target:

```bash
dg5f v2 watch
```

Because ML-Agents resolves `--initialize-from` by behavior name, the launcher
creates an immutable local bridge from the bootstrap's
`DG5FGraspJoint/checkpoint.pt` to the package's
`DG5FStableGrasp/checkpoint.pt`. See
[`AGENT_SPEC_V3.md`](../docs/AGENT_SPEC_V3.md).

## 5M training

```bash
RUN_ID=dg5f-grasp-point-reach-5m \
ENV_PATH="$PWD/training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64" \
TORCH_DEVICE=cuda \
training/scripts/train_dg5f_grasp_point_reach.sh
```

`VENV`, `CONFIG`, `TORCH_DEVICE`, `UNITY_DISPLAY_MODE`, `XVFB_SCREEN`, `RESULTS_DIR`,
`RUN_ID`, `ENV_PATH`, `NUM_ENVS`, `TIME_SCALE`을 override할 수 있다. DISPLAY가 없는 Linux에서는 Xvfb를
사용한다. `--resume`은 이 새 run 자체를 중단 후 재개할 때만 사용한다.

## Approval evaluation

최신 Reach player를 다시 빌드한 뒤 run ID가 가리키는 checkpoint를 500개 미학습
seed에서 deterministic inference로 평가한다.

```bash
DG5F_RUN_ID=dg5f_v2_joint26_handfirst_lr5e5_gpu_fixed \
training/scripts/run_dg5f_v2_evaluation.sh
```

승인에는 성공률 90% 이상, 모든 성공의 1 cm/0.05 m/s/0.25초 조건 충족, 고유 seed
500개, 비유한 물리 및 workspace 안전 실패 0건이 모두 필요하다. 통과 모델은 평균 최종
오차가 작은 순, 동률이면 median/p95 완료 시간이 짧은 순으로 선택한다. 평가 wrapper는
CSV와 canonical ONNX의 hash를 `DG5F_EVAL_APPROVAL` JSON에 함께 기록한다.
