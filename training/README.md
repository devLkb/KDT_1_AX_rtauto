# DG5FGraspPointReach training

현재 training 계약은 하나다. 빨간 목표 좌표로 UR5e 팔 6축을 제어해 palm의
`GraspPoint`를 빠르고 정확하게 배치한다. DG5F 손가락 20관절은 policy I/O에서 제외되며
도달 후 `vision/dg5f` 텔레옵이 제어한다.

## Contract

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

## 512-step smoke

```bash
ENV_PATH="$PWD/training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64" \
training/scripts/smoke_dg5f_grasp_point_reach.sh
```

이는 communicator와 26/6 shape를 확인하는 gate이며 수렴 실험이 아니다.

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
DG5F_RUN_ID=dg5f-grasp-point-reach-5m \
DG5F_EVAL_EPISODES=500 \
DG5F_EVAL_BASE_SEED=500000 \
training/scripts/run_dg5f_grasp_point_reach_evaluation.sh
```

승인에는 성공률 90% 이상, 모든 성공의 1 cm/0.05 m/s/0.25초 조건 충족, 고유 seed
500개, 비유한 물리 및 workspace 안전 실패 0건이 모두 필요하다. 통과 모델은 평균 최종
오차가 작은 순, 동률이면 median/p95 완료 시간이 짧은 순으로 선택한다. 평가 wrapper는
CSV와 canonical ONNX의 hash를 `DG5F_EVAL_APPROVAL` JSON에 함께 기록한다.
