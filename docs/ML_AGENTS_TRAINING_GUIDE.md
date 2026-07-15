# DG5F 단계형 공 파지·상승·유지 학습 실행 가이드

이 문서는 `DG5FGraspV4` v4.0 환경을 빌드하고 통신 smoke와 본학습을 실행하는 절차를 설명한다.
환경과 PPO가 동작하는 전체 과정은 [`ML_AGENTS_LEARNING_FLOW.md`](ML_AGENTS_LEARNING_FLOW.md)를 참고한다.
모든 명령은 저장소 루트에서 실행하고 Python 명령은 `vision/.vision` 가상환경을 사용한다.

## 1. 사전 확인

- Unity: `6000.4.0f1`
- Unity package: `com.unity.ml-agents` 4.0.0
- Python: 3.10.x, `vision/.vision`
- contract: [`AGENT_SPEC.md`](AGENT_SPEC.md) v4.1.0

```bash
vision/.vision/bin/pip check
vision/.vision/bin/mlagents-learn --help
```

## 2. 씬과 prefab 재생성

Agent reference나 workspace 설정을 바꾼 경우 생성기를 먼저 실행한다.

```bash
UNITY_EDITOR=/home/lkb/Unity/Hub/Editor/6000.4.0f1/Editor/Unity

"$UNITY_EDITOR" \
  -batchmode -nographics -quit \
  -projectPath "$PWD/unity" \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingSceneBuilder.Build
```

생성 결과:

- `unity/Assets/MLAgents/Grasp/TrainingArea.prefab`
- `unity/Assets/MLAgents/Grasp/DG5F_GraspTraining.unity`

`TrainingArea.prefab`은 공·받침대·UR5e/DG5F Agent를 하나로 묶은 재사용 단위다.
씬에는 이 prefab을 X축 4열 x Y축 5행으로 20개 배치한다. 모든 Agent는 같은 `DG5FGraspV4`
policy를 공유하지만 공, 받침대, 접촉 센서, episode 상태는 서로 독립이다.

생성된 Agent는 57 observations, 7 continuous actions, 10Hz decision, `MaxStep=0`을 사용한다.

## 3. Linux 학습 환경 빌드

Unity Editor가 같은 프로젝트를 열고 있으면 종료한 뒤 실행한다.

```bash
"$UNITY_EDITOR" \
  -batchmode -nographics -quit \
  -projectPath "$PWD/unity" \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingBuild.BuildLinuxHeadless
```

```bash
test -x training/builds/DG5FGrasp/DG5FGrasp.x86_64 && echo "build ready"
```

## 4. 512-step communicator smoke

본학습 전에 built player가 Python trainer와 512 step을 정상 교환하는지 확인한다.
기본 YAML을 임시 복사해 기존 결과와 설정을 변경하지 않는다.

```bash
cp training/config/dg5f_grasp.yaml /tmp/dg5f_grasp_512.yaml
sed -i 's/max_steps: 5000000/max_steps: 512/' /tmp/dg5f_grasp_512.yaml

CONFIG=/tmp/dg5f_grasp_512.yaml \
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RESULTS_DIR=/tmp/dg5f_grasp_results \
RUN_ID=dg5f_grasp_v4_comm_512 \
NUM_ENVS=1 TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh --force
```

합격 조건:

1. `DG5FGraspV4?team=0` 연결
2. observation/action shape 오류 없음
3. 512 step 이상 도달(20 Agent batch 처리로 최종 step은 초과 가능)
4. Unity crash, `NaN`, communicator timeout 없음

## 5. 50k stability smoke

```bash
cp training/config/dg5f_grasp.yaml /tmp/dg5f_grasp_50k.yaml
sed -i 's/max_steps: 5000000/max_steps: 50000/' /tmp/dg5f_grasp_50k.yaml

CONFIG=/tmp/dg5f_grasp_50k.yaml \
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RUN_ID=dg5f_grasp_v4_smoke_50k \
NUM_ENVS=1 TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh
```

확인할 항목:

- 즉시 실패 reset 반복, 무한 episode 또는 20초 timeout 누락 없음
- 공 이탈·낙하·비유한 좌표 발생 시 원인별 실패 벌점(Timeout `-0.1`, GripLost/Dropped `-0.5`,
  안전 위반 `-1.0`)과 원인을 기록한 뒤 reset
- 물리 관통 reset이 임계 깊이/시간에서만 발생하고 단계 보너스가 반복 지급되지 않음
- checkpoint와 ONNX 생성
- cumulative reward, episode length, 상승·유지·이동량 통계가 유한

## 6. TensorBoard

```bash
vision/.vision/bin/tensorboard --logdir training/results --port 6006
```

주요 지표:

- `Environment/Cumulative Reward`
- episode length
- policy entropy
- `Grasp/Success`
- `Grasp/CompletionSeconds`
- `Curriculum/Stage`
- `Grasp/MaxLiftHeightMeters`, `Grasp/HoldSeconds`
- `Motion/NormalizedArmTravel`
- `Failure/*` 원인별 rate

stock ML-Agents는 `Grasp/Success`를 curriculum criterion으로 사용할 수 없으므로 기본 config는
lesson 0으로 고정된다. 최소 200회 고정 정책 평가에서 성공률 80% 이상을 확인한 뒤 다음 config를
생성한다. 조건 미달이면 스크립트가 non-zero로 종료한다.

```bash
vision/.vision/bin/python training/scripts/promote_dg5f_lesson.py \
  --current-stage 0 --episodes 200 --successes 160 \
  --base-config training/config/dg5f_grasp.yaml \
  --output /tmp/dg5f_grasp_stage1.yaml
```

## 7. 5M 본학습

launcher 기본 RUN_ID가 `dg5f_grasp_v4`이므로 다음 명령에서 생략할 수 있다.

```bash
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
NUM_ENVS=1 TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh
```

이전 결과와 checkpoint는 삭제하거나 `--force`로 덮어쓰지 않는다. 이전 checkpoint는 V4 계약에 재사용하지 않는다.

### 중단 후 V4 재개

```bash
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RUN_ID=dg5f_grasp_v4 NUM_ENVS=1 TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh --resume
```

## 8. Editor 연결 디버깅

1. Unity에서 `Assets/MLAgents/Grasp/DG5F_GraspTraining.unity`를 연다.
2. trainer를 실행해 연결을 기다린다.
3. Unity Play를 누른다.

```bash
RUN_ID=dg5f_grasp_v4_editor_debug \
NUM_ENVS=1 TIME_SCALE=1 \
training/scripts/train_dg5f_grasp.sh
```

## 9. Launcher 환경변수

| 변수 | 기본값 | 용도 |
|---|---|---|
| `VENV` | `vision/.vision` | Python 가상환경 |
| `CONFIG` | `training/config/dg5f_grasp.yaml` | ML-Agents YAML |
| `RESULTS_DIR` | `training/results` | checkpoint/TensorBoard 결과 |
| `RUN_ID` | `dg5f_grasp_v4` | 학습 실행 식별자 |
| `ENV_PATH` | 빈 값 | Linux player 경로; 빈 값이면 Editor 연결 |
| `NUM_ENVS` | `1` | 병렬 Unity 프로세스 수. 프로세스마다 Agent 20개 |
| `TIME_SCALE` | `10` | Unity simulation 배속 |

## 10. 문제 해결

### `project is already open`

같은 Unity 프로젝트를 연 Editor를 종료하고 batchmode 명령을 다시 실행한다.

### Trainer가 Unity 연결을 기다림

- `ENV_PATH`와 실행 권한 확인
- 기존 Unity player/trainer 프로세스 확인
- 필요하면 `--base-port 5006` 추가

### 기존 RUN_ID 오류

- 같은 V4 실험 재개: `--resume`
- 새 실험: 새 RUN_ID
- `--force`: 해당 결과를 명시적으로 폐기할 때만 사용

### Episode가 예상보다 길어짐

V4는 `MaxStep=0`을 유지하지만 자체 상태기계가 Unity 시뮬레이션 시간 정확히 20초에서 timeout 실패를 기록한다. 벽시계가 아니라 physics time을 사용한다.

### `Failed to open plugin: ...libassimp.so`

URDF Importer의 선택적 native plugin 경고다. 이미 생성된 prefab을 쓰는 player가 communicator와 step을 정상 진행하면 현재 학습 경로에는 영향이 없다.
