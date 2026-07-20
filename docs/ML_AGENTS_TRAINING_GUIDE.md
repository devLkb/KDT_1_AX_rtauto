# DG5FGraspPointReach 학습 실행 가이드

이 가이드는 단일 `DG5FGraspPointReach` 정책을 checkpoint 없이 학습하고 500개 seed로
승인 평가하는 절차다. 정확한 정책 상수는 [`AGENT_SPEC.md`](AGENT_SPEC.md)를 우선한다.

## 1. 사전 확인

```bash
cd /home/lkb/workspace/KDT_1_AX_rtauto
source vision/.vision/bin/activate
python --version
pip check
mlagents-learn --help >/dev/null
```

기준 환경은 Python 3.10.12, Unity 6000.4.0f1,
`mlagents/mlagents_envs==1.2.0.dev0`이다. CUDA 학습이면 다음도 확인한다.

```bash
python -c 'import torch; print(torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

## 2. 씬과 Linux player 생성

Unity에서 `unity/`를 연 뒤 다음 메뉴를 실행한다.

1. **Tools > ML-Agents > Build DG5F GraspPoint Reach Scene**
2. **Tools > ML-Agents > Build DG5F GraspPoint Reach Linux Player**

생성 기준:

- Scene: `Assets/MLAgents/Reach/DG5F_GraspPointReachTraining.unity`
- Prefab: `Assets/MLAgents/Reach/TrainingArea.prefab`
- Player: `training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64`
- 20개 독립 영역, Behavior `DG5FGraspPointReach`, observation/action `26/6`

CI나 CLI에서는 Unity의
`KDT.ReachTraining.Editor.ArmReachTrainingBuild.BuildLinux` 메서드를 실행해 같은
산출물을 만든다. 빌드 전에 Unity Test Runner의 Reach EditMode/PlayMode 테스트를 통과시킨다.

## 3. 512-step communicator smoke

smoke는 별도 run ID와 결과 폴더를 사용하고 trainer의 `max_steps`를 정확히 512로
설정한다. 한 player 안의 20개 Agent가 묶음으로 step을 전달하므로 마지막 checkpoint
파일명은 512보다 큰 집계 step을 표시할 수 있다.

```bash
ENV_PATH="$PWD/training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64" \
training/scripts/smoke_dg5f_grasp_point_reach.sh
```

확인 사항:

- Unity와 trainer가 `DG5FGraspPointReach`로 연결됨
- observation 26 / continuous action 6 shape 오류 없음
- 로그에 `max_steps: 512`가 출력되고 checkpoint/ONNX export 후 정상 종료
- NaN/Infinity, workspace 실패, 중복 환경 seed 없음

smoke 결과의 reward나 성공률은 수렴 근거가 아니다.

## 4. 5M fresh PPO 학습

활성 config는 `training/config/dg5f_grasp_point_reach.yaml` 하나다.

```bash
RUN_ID=dg5f-grasp-point-reach-5m \
ENV_PATH="$PWD/training/builds/DG5FGraspPointReach/DG5FGraspPointReach.x86_64" \
TORCH_DEVICE=cuda \
TIME_SCALE=10 \
training/scripts/train_dg5f_grasp_point_reach.sh
```

- 새 run ID로 시작하고 `--resume`, `--initialize-from`, 이전 checkpoint를 사용하지 않는다.
- 기본 max steps는 5,000,000이다.
- Linux Unity 6000.4 player가 `-nographics`에서 종료될 수 있으므로 DISPLAY가 없으면
  launcher가 Xvfb를 사용한다.
- PPO update 사이에는 CUDA 사용률이 낮을 수 있다. 시작 로그의 `[GPU]`와
  `nvidia-smi`의 trainer process로 실제 device를 확인한다.

TensorBoard:

```bash
tensorboard --logdir training/results --port 6006
```

episode reward와 length뿐 아니라 성공률, 최종 오차, 완료 시간, timeout과 안전 실패를
함께 본다. 중단 후 같은 실험을 재개해야 할 때만 동일 run ID와 `--resume`을 사용한다.
폐기된 파지 모델에서 새 run을 초기화하지 않는다.

## 5. 500-seed 결정론 평가

최신 Reach player를 다시 빌드한 뒤 run ID로 평가할 checkpoint를 선택한다. 평가 wrapper는
trainer를 deterministic inference 모드로 연결하고 완료 후 같은 run의 canonical ONNX와
CSV hash를 승인 JSON에 묶는다.

```bash
DG5F_RUN_ID=dg5f-grasp-point-reach-5m \
DG5F_EVAL_EPISODES=500 \
DG5F_EVAL_BASE_SEED=500000 \
training/scripts/run_dg5f_grasp_point_reach_evaluation.sh
```

평가기와 CSV validator는 다음을 모두 확인해야 한다.

- 정확히 500개 고유 seed와 episode 행
- 성공률 90% 이상
- 모든 성공 행의 거리 `<= 0.01 m`, 속도 `<= 0.05 m/s`, hold `>= 0.25 s`
- 비유한 물리, workspace 안전 실패, 중복 seed 0건

여러 checkpoint가 통과하면 평균 최종 오차, median 완료 시간, p95 완료 시간 순으로
모델을 선택한다. CSV와 선택한 ONNX는 같은 run의 산출물로 함께 보관한다.

평가 interface의 기본값은 500 episode, base seed 500000, timeout 1200초다.
`DG5F_EVAL_CSV`, `DG5F_EVAL_APPROVAL`, `DG5F_EVAL_TIMEOUT_SECONDS`, `RESULTS_DIR`,
`ENV_PATH`, `VENV`로 출력과 실행 환경을 바꿀 수 있지만 승인 episode 수는 500으로
고정한다.

## 6. Editor 연결 디버깅

player 없이 확인할 때:

```bash
RUN_ID=dg5f-grasp-point-reach-editor \
training/scripts/train_dg5f_grasp_point_reach.sh
```

trainer가 `Start training by pressing the Play button in the Unity Editor`를 출력하면
Reach training scene을 열고 Play한다. Editor에서 여러 scene이나 예전 Grasp Behavior를
동시에 실행하지 않는다.

## 7. 문제 해결

- **Behavior 또는 tensor shape 불일치**: scene/prefab을 Reach builder로 재생성하고
  `DG5FGraspPointReach`, 26, 6을 확인한다.
- **player를 찾지 못함**: `ENV_PATH`가 실행 가능한
  `DG5FGraspPointReach.x86_64`인지 확인한다.
- **기존 RUN_ID 오류**: 새 실험이면 새 ID를 사용한다. 재개가 목적일 때만 `--resume`한다.
- **trainer가 연결을 기다림**: player process, port 충돌, Xvfb와 Unity log를 확인한다.
- **GPU tensor device 오류**: 공용 Python 3.10 venv와 compatibility launcher를 사용하고
  `TORCH_DEVICE`를 확인한다.
- **학습은 되지만 성공이 없음**: 먼저 reset target 반경, GraspPoint 위치, arm drive
  clamp와 20초 simulation-time 계산을 PlayMode 테스트로 확인한다. 폐기 모델을
  bootstrap하는 방식으로 우회하지 않는다.
