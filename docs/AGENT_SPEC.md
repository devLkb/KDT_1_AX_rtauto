# DG5FGrasp Agent Spec v3.1.0

## Goal and success

UR5e가 로봇 주변 3D 도달공간에 놓인 4cm 빨간 공에 접근해 DG5F로 파지한다.
성공은 엄지가 공에 접촉하고 검지·중지·약지·소지 중 하나 이상이 공에 접촉한 상태를
Unity 시뮬레이션 시간으로 1초 연속 유지하는 것이다. 성공 종료에는 추가 reward가 없다.

학습 씬은 `Assets/MLAgents/Grasp/DG5F_GraspTraining.unity`다.
기존 `DG5F_Import.unity` 텔레오퍼레이션 씬은 변경하지 않는다.

## Interface contract

- Behavior: `DG5FGrasp`
- Policy: PPO
- Decision frequency: 10Hz (`DecisionPeriod=5`, physics 50Hz)
- Continuous actions: 7
- Vector observations: 43
- `MaxStep=0`: 고정 시간 제한 없음

### Actions

| Index | Meaning | Scale |
|---:|---|---|
| 0..5 | UR5e joint xDrive target delta | `[-1,1] * 2 deg/decision` |
| 6 | DG5F closure delta | `[-1,1] * 0.04/decision`, accumulated in `[0,1]` |

팔 target은 `Dg5fGraspSpec`의 학습 안전범위로 제한한 뒤 URDF drive limit으로 다시 제한한다.
shoulder pan 안전범위는 `[-180°, 180°]`이며 나머지 다섯 관절 범위는 v1과 같다.
closure는 검증된 왼손 `OPEN -> FIST` 20관절 프로파일을 선형 보간한다. 손가락별 독립 action은 없다.

### Observations

순서와 크기는 v3 계약 안에서 고정한다. 관절 위치는 URDF 전체 limit이 아니라 학습 안전범위로 정규화한다.

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

## Workspace sampling

받침대는 robot-base 좌표계에 고정하고 매 episode 공만 새 위치로 옮긴다.

- 방위: `[0°, 360°)` 균등분포
- 수평 반경: `[0.25m, 0.70m]`, 면적 균등분포
- 로봇 베이스 기준 받침대 상단 높이: `0m` 고정
- 공 중심과 robot base 거리: `<= 0.80m`인 표본만 허용
- 공 표면과 전체 로봇의 활성 non-trigger collider 사이 거리: `>= 0.05m`
- 받침대: 학습 영역 기준 바닥 `y=0`, 상단 `y=0.25m`, 크기 `1.8m x 0.25m x 1.8m`인 solid panel
- 로봇 베이스는 panel 상단 `y=0.25m`에 배치
- 받침대 collider: 정적 `BoxCollider`
- 빨간 공은 panel 상단에 접하도록 배치하며 공 표면이 panel 경계를 넘지 않는다

면적 균등 반경은 다음 식으로 생성한다.

```text
r = sqrt(lerp(0.25^2, 0.70^2, U[0,1]))
```

## Reward

매 policy decision의 유일한 reward는 GraspPoint와 공 중심 사이 거리 비용이다.

```text
reward = -0.01 * clamp(distance(GraspPoint, ball) / 0.85m, 0, 1)
```

경계값은 `0m -> 0`, `0.425m -> -0.005`, `0.85m 이상 -> -0.01`이다.
접촉, 성공, 실패, 시간 경과에는 별도 reward가 없다.

## Episode termination and reset

성공 또는 다음 안전/정체 조건에서 즉시 `EndEpisode()`를 호출한다.

### Success

1. 엄지가 공에 접촉한다.
2. 나머지 네 손가락 중 하나 이상이 공에 접촉한다.
3. 두 조건을 시뮬레이션 시간 1초 동안 연속 유지한다.

접촉이 한 physics step이라도 끊기면 성공 유지 타이머는 0으로 돌아간다.

### Reset-only conditions

- 공 중심이 robot base에서 `0.85m`보다 멀어짐
- 공 중심이 현재 받침대 상단 높이 아래로 내려감
- 공 좌표에 `NaN` 또는 Infinity가 발생
- palm/finger collider와 받침대 collider의 관통 깊이가 `1cm` 이상인 상태가 `0.2초` 연속 지속
- 기록한 최저 거리에서 의미 있는 `2cm` 추가 접근이 `60초` 연속 발생하지 않음

의미 있는 `2cm` 접근이 생기면 정체 타이머를 0으로 재시작한다. 위 조건에는 실패 reward가 없다.

### Reset state

매 reset은 다음 순서로 episode 상태를 정리한다.

1. closure를 0으로 만들고 팔 6개와 손 20개 관절 target/position/velocity 복원
2. 고정된 panel과 로봇 베이스는 변경하지 않고 공만 panel 상단에 재배치
3. 공의 선속도·각속도 0, gravity on 복원
4. 모든 fingertip 접촉 기록 초기화
5. 성공·관통·정체 타이머 초기화

## Parallel training topology

- `TrainingArea.prefab`: 공, 받침대, UR5e/DG5F Agent를 포함하는 독립 학습 단위
- 씬당 X축 4열 x Y축 5행, 3m 간격, Z축 좌표 0인 총 20개 prefab instance
- 20개 Agent 모두 `DG5FGrasp` policy 공유
- 총 Agent 수: `20 x NUM_ENVS` (기본 `NUM_ENVS=1`, 총 20개)

## Commands

```bash
# Python environment check
vision/.vision/bin/pip check
vision/.vision/bin/mlagents-learn --help

# Rebuild scene and prefab
UNITY_EDITOR=/home/lkb/Unity/Hub/Editor/6000.4.0f1/Editor/Unity
"$UNITY_EDITOR" -batchmode -nographics -quit -projectPath unity \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingSceneBuilder.Build

# Build Linux player
"$UNITY_EDITOR" -batchmode -nographics -quit -projectPath unity \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingBuild.BuildLinuxHeadless

# Headless training (RUN_ID defaults to dg5f_grasp_panel_v3)
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
NUM_ENVS=1 TIME_SCALE=10 training/scripts/train_dg5f_grasp.sh
```

## Versioning rule

관측 순서/크기, action 의미/스케일, closure 프로파일, reward, 성공 또는 reset 계약이 바뀌면
spec 버전을 올린다. 이전 checkpoint는 v3에서 재사용하지 않으며 기존 결과 디렉터리는 보존한다.
