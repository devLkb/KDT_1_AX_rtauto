# DG5FGraspV4 Agent Spec v4.1.0

## Goal and success

UR5e가 로봇 주변 도달공간에 놓인 지름 4cm, 질량 0.05kg 공에 접근해 DG5F로 파지하고,
공 중심을 episode 시작 높이보다 10cm 들어 올린 뒤 5초 동안 안정적으로 유지한다.

최종 성공은 다음 조건을 모두 만족할 때만 성립한다.

1. 엄지와 나머지 네 손가락 중 하나 이상이 공에 0.25초 연속 접촉해 안정 파지를 만든다.
2. 공 중심이 episode 시작 높이보다 10cm 이상 상승한다.
3. 파지 접촉 유지, 상승 높이 9cm 이상, 공 선속도 0.05m/s 이하를 Unity 시뮬레이션
   시간으로 5초 연속 만족한다.

학습 씬은 `Assets/MLAgents/Grasp/DG5F_GraspTraining.unity`다.
기존 `DG5F_Import.unity` 텔레오퍼레이션 씬은 변경하지 않는다.

## Interface contract

- Behavior: `DG5FGraspV4`
- Spec: `4.1.0`
- Policy: PPO
- Decision frequency: 10Hz (`DecisionPeriod=5`, physics 50Hz)
- Continuous actions: 7
- Vector observations: 57
- Episode timeout: Unity 시뮬레이션 시간 20초

V4는 observation, reward, 성공·실패 계약이 V3과 호환되지 않는다. V3 checkpoint나
ONNX를 초기화 또는 재개 입력으로 사용하지 않는다.

### Actions

| Index | Meaning | Scale |
|---:|---|---|
| 0..5 | UR5e joint xDrive target delta | `[-1,1] * 2 deg/decision` |
| 6 | DG5F closure delta | `[-1,1] * 0.04/decision`, accumulated in `[0,1]` |

팔 target은 `Dg5fGraspSpec`의 학습 안전범위로 제한한 뒤 URDF drive limit으로 다시 제한한다.
closure는 검증된 왼손 `OPEN -> FIST` 20관절 프로파일을 선형 보간한다. 손가락별 독립 action은 없다.

### Observations

순서와 크기는 V4 계약 안에서 고정한다. 관절 위치와 target은 URDF 전체 limit이 아니라
학습 안전범위로 정규화한다.

| Range | Count | Value |
|---:|---:|---|
| 0..5 | 6 | 안전범위로 정규화한 팔 관절 위치 |
| 6..11 | 6 | 정규화한 팔 관절 속도 |
| 12 | 1 | grip closure mapped to `[-1,1]` |
| 13..15 | 3 | GraspPoint 기준 공 위치, robot-base axes |
| 16..18 | 3 | 공 선속도, robot-base axes |
| 19..21 | 3 | 공 각속도, robot-base axes |
| 22 | 1 | episode 시작 위치 대비 공의 수직 변위 |
| 23..37 | 15 | 공 기준 다섯 손끝 위치, palm axes |
| 38..42 | 5 | 엄지·검지·중지·약지·소지 접촉 flag |
| 43..48 | 6 | 안전범위로 정규화한 팔 xDrive target |
| 49..52 | 4 | 현재 단계 `Reach/Grasp/Lift/Hold` one-hot |
| 53 | 1 | 안정 파지 진행도 `[0,1]` |
| 54 | 1 | 현재 목표 대비 유효 유지 진행도 `[0,1]` |
| 55 | 1 | 현재 curriculum 상승 목표, 10cm 기준 정규화 |
| 56 | 1 | 현재 curriculum 유지 목표, 5초 기준 정규화 |

## Episode state machine

1. `Reach`: GraspPoint가 공에 접근한다. 엄지와 opposing finger 접촉이 시작되면 `Grasp`로 간다.
2. `Grasp`: 올바른 접촉을 0.25초 연속 유지하면 안정 파지를 한 번만 인정하고 `Lift`로 간다.
   접촉이 그 전에 끊기면 파지 타이머를 0으로 만들고 `Reach`로 돌아간다.
3. `Lift`: 안정 파지 후 공이 현재 curriculum 상승 목표에 도달하면 `Hold`로 간다.
4. `Hold`: 접촉 유지, 현재 lesson 상승 목표보다 1cm 낮은 높이 이상, 공 속도 0.05m/s 이하를
   현재 curriculum 유지 시간 동안 연속 만족하면 성공한다. 최종 lesson의 높이 하한은 9cm다.
   높이 또는 속도 조건이 깨지면 유지 타이머를 0으로 만들고 조건을 다시 만족할 때까지 기다린다.

안정 파지 이후 올바른 접촉이 한 physics step이라도 끊기면 `GripLost`로 즉시 실패한다.
안정 파지 전 `Grasp` 단계의 접촉 중단은 파지 타이머만 0으로 만들며 재파지를 허용한다.
상승 목표에 한 번 도달한 뒤 높이가 2cm 이하로 다시 내려가면 유지 타이머 초기화보다
`Dropped` 실패 판정을 우선한다. 목표 자체가 2cm인 첫 lesson은 이 낙하 경계를 적용하지 않는다.

## Workspace and curriculum

받침대는 robot-base 좌표계에 고정하고 매 episode 공만 panel 상단에 재배치한다. 반경 표본은
각 lesson의 범위 안에서 면적 균등하게 생성하며, 방위는 episode 시작 GraspPoint 방향을 기준으로 한다.

| Lesson | Direction | Horizontal radius | Lift target | Hold target | Movement `lambda` |
|---:|---:|---:|---:|---:|---:|
| 0 | `+/-15 deg` | 0.25..0.35m | 0.02m | 0.5s | 0 |
| 1 | `+/-30 deg` | 0.25..0.45m | 0.05m | 1s | 0 |
| 2 | `+/-60 deg` | 0.25..0.55m | 0.10m | 2s | 0.01 |
| 3 | `+/-120 deg` | 0.25..0.65m | 0.10m | 3s | 0.01 |
| 4 | full 360 deg | 0.25..0.70m | 0.10m | 5s | 0.02 |

공 표면은 panel 경계를 넘지 않아야 하며 전체 로봇의 활성 non-trigger collider와 최소 0.05m
간격을 둔다. 공 중심은 robot base로부터 0.80m 이내에서 생성한다.

stock ML-Agents curriculum은 custom `Grasp/Success`를 승급 criterion으로 읽지 못한다.
따라서 기본 YAML은 lesson 0을 constant로 고정하고 shaped reward에 의한 자동 승급을 금지한다.
별도 고정 정책 평가가 최소 200 episode와 성공률 80%를 만족한 뒤에만 다음 명령으로 다음
lesson config를 생성한다.

```bash
vision/.vision/bin/python training/scripts/promote_dg5f_lesson.py \
  --current-stage 0 --episodes 200 --successes 160 \
  --base-config training/config/dg5f_grasp.yaml \
  --output /tmp/dg5f_grasp_stage1.yaml
```

생성된 config를 다음 학습/재개에 사용한다. 스크립트는 200회 미만, 성공률 80% 미만,
현재 config와 `--current-stage` 불일치를 거부한다.

Trainer가 연결되면 `Academy.Instance.EnvironmentParameters`의 `lesson` 값 `0..4`가 현재
단계를 선택한다. Trainer 없이 Editor 또는 inference로 실행할 때의 기본값은 최종 lesson 4다.

## Reward

매 policy decision의 기본 보상 계약은 다음과 같다. 목표 순서는
`접근 → 파지 → 상승 → 5초 유지`이며 각 단계가 dense한 학습 신호를 갖는다.

| Component | Reward |
|---|---:|
| time cost | `-0.001` |
| GraspPoint 거리 감소 potential | episode 최대 누적 `+1.0` |
| 접촉 potential | 엄지 접촉 `+0.25`, 반대 손가락 접촉 `+0.25` |
| 최초 안정 파지 | `+0.5`, episode당 한 번 |
| 상승 진행 potential | lesson 목표 높이까지 최대 누적 `+1.0` |
| 최초 목표 높이 도달 | `+1.0`, episode당 한 번 |
| 유지 진행 potential | lesson 목표 시간까지 최대 누적 `+1.0` |
| 팔 관절 이동량 | `-lambda * sum(abs(delta q) / jointRange)` |
| 성공 종료 | `+3.0` |
| Timeout 종료 | `-0.1` |
| GripLost/Dropped 종료 | `-0.5` |
| WorkspaceExit/Penetration/NonFinitePhysics 종료 | `-1.0` |

거리, 접촉, 상승, 유지는 모두 이전 상태와 현재 상태의 potential 차이만 지급한다. 접근 후
후퇴, 접촉 후 상실, 상승 후 하강, 유지 타이머 리셋은 반대 부호로 차감하므로 반복 왕복이나
접촉 유지만으로 reward를 누적할 수 없다. 안정 파지와 목표 높이 보너스는 episode당 각각
한 번만 지급한다. 움직임 비용은 팔 6축만 포함하며 closure 이동은 포함하지 않는다.

terminal 보상은 실패 원인에 따라 차등을 둔다. Timeout은 과제를 끝내지 못했을 뿐이므로
`-0.1`만 지급해 초기 탐색 단계의 shaping gradient를 보존하고, 잡은 공을 잃는
GripLost/Dropped는 `-0.5`, 안전 위반(작업공간 이탈·관통·비유한 물리)은 `-1.0`을 지급한다.

## Failure and reset

다음 조건은 위 표의 실패 terminal 보상을 지급하고 즉시 episode를 종료한다.

- 안정 파지 이후 올바른 접촉을 잃음
- 상승 목표에 도달한 뒤 공 상승 높이가 2cm 이하로 다시 내려감(lesson 1..4)
- 공이 허용 작업공간을 이탈함
- hand/palm과 panel의 허용치를 넘는 관통
- 공 또는 articulation의 position/velocity/target에 NaN 또는 Infinity 발생
- Unity 시뮬레이션 시간 20초 초과

모든 실패는 누락 없는 단일 failure reason을 기록한다. reset은 closure를 0으로 만든 뒤 팔 6개와
손 20개 관절의 xDrive target, position, velocity를 동기화하고 공 pose와 선속도·각속도,
gravity, 접촉 센서, 단계, 보너스 flag, 파지·유지 타이머, 이동량 누계를 초기화한다.

## TensorBoard statistics

각 episode는 최소 다음 custom statistic을 기록한다.

- `Curriculum/Stage`
- `Grasp/Success`
- `Grasp/MaxLiftHeightMeters`
- `Grasp/HoldSeconds`
- `Motion/NormalizedArmTravel`
- failure reason별 one-hot rate: `Failure/GripLost`, `Failure/Dropped`,
  `Failure/WorkspaceExit`, `Failure/Penetration`, `Failure/NonFinitePhysics`, `Failure/Timeout`

Failure 통계는 발생한 reason만 1로 보내지 않고 매 episode 모든 reason에 0 또는 1을 보내
TensorBoard average가 실제 발생률이 되게 한다. TensorBoard 값은 구간 집계이므로 개별 성공
episode가 10cm/5초를 만족했는지 증명하는 최종 평가 원장은 별도로 보존한다.

## Parallel training topology

- `TrainingArea.prefab`: 공, panel, UR5e/DG5F Agent를 포함하는 독립 학습 단위
- 씬당 X축 4열 x Y축 5행, 3m 간격인 총 20개 prefab instance
- 20개 Agent 모두 `DG5FGraspV4` policy 공유
- 총 Agent 수: `20 x NUM_ENVS` (기본 `NUM_ENVS=1`)

## Training and acceptance

```bash
# Rebuild scene and prefab after the contract constants change
UNITY_EDITOR=/home/lkb/Unity/Hub/Editor/6000.4.0f1/Editor/Unity
"$UNITY_EDITOR" -batchmode -nographics -quit -projectPath unity \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingSceneBuilder.Build

# Build and train; launcher RUN_ID defaults to dg5f_grasp_v4
"$UNITY_EDITOR" -batchmode -nographics -quit -projectPath unity \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingBuild.BuildLinuxHeadless
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
NUM_ENVS=1 TIME_SCALE=10 training/scripts/train_dg5f_grasp.sh
```

최종 모델은 학습에 쓰지 않은 고정 seed 500개, 1배속 물리 평가에서 성공률 90% 이상이어야 한다.
모든 성공 episode는 실제 10cm 도달과 5초 유지를 기록해야 하며 failure reason 누락은 허용하지 않는다.
성공률 90% 이상 checkpoint만 후보로 삼고 평균 정규화 팔 관절 이동량이 가장 작은 모델을 채택한다.
`lambda=0` 기준 모델보다 이동량이 10% 이상 감소하고 성공률 하락이 2%p 이내인지 A/B 평가한다.

## Versioning rule

관측 순서/크기, action 의미/스케일, closure 프로파일, reward, curriculum, 성공 또는 실패 계약이
바뀌면 spec과 behavior 이름을 함께 올린다. V4 checkpoint는 같은 V4 계약과 RUN_ID에서만
`--resume`하며 기존 V3 결과 디렉터리는 보존한다.
