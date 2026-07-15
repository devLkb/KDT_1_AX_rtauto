> **폐기된 v4 복합 보상 문서:** 현재 단계별 전이학습 설계는 [`../train_plan.md`](../train_plan.md)를 따른다.

# DG5F 단계형 공 파지·상승·유지 강화학습 설계

## 1. 문서 관계

이 환경은 UR5e + DG5F가 4cm, 0.05kg 공을 `접근 → 파지 → 상승 → 유지` 순서로
다루도록 학습한다.

- 고정 인터페이스 계약: [`AGENT_SPEC.md`](AGENT_SPEC.md)
- 전체 학습 흐름: [`ML_AGENTS_LEARNING_FLOW.md`](ML_AGENTS_LEARNING_FLOW.md)
- 빌드·학습 실행: [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)
- PPO 설정: `training/config/dg5f_grasp.yaml`
- Agent: `unity/Assets/MLAgents/Grasp/Runtime/Dg5fGraspAgent.cs`

현재 계약은 `DG5FGraspV4 Agent Spec v4.1.0`이다. behavior 이름도 `DG5FGraspV4`로
분리해 V3 checkpoint가 실수로 로드되지 않게 한다.

## 2. 목표와 상태기계

최종 성공은 공을 초기 높이보다 10cm 올리고 다음 조건을 5초 연속 만족하는 것이다.

- 엄지와 나머지 손가락 하나 이상의 접촉
- 상승 높이 9cm 이상
- 공 속도 0.05m/s 이하

`Reach`, `Grasp`, `Lift`, `Hold` 네 단계를 observation one-hot으로 제공한다. 올바른 접촉을
0.25초 연속 유지해야 안정 파지가 되고, Hold 조건이 깨지면 유지 타이머는 즉시 0이 된다.
안정 파지 뒤 접촉을 잃거나 목표 도달 후 공이 2cm 이하로 다시 떨어지면 실패한다.
작업공간 이탈, 관통, 비유한 물리 상태, 20초 timeout도 명시적 실패 원인을 남긴다.

## 3. Action과 hand profile

정책 출력은 기존처럼 7개 continuous action이다.

- 팔 6축: 현재 xDrive target에 `[-1,1] * 2deg` delta 누적
- 손 1축: closure에 `[-1,1] * 0.04` delta 누적

closure는 검증된 왼손 `OPEN → FIST` 20관절 프로파일을 보간한다. Agent가 학습 scene의
유일한 xDrive writer이며 teleoperation/IK driver는 비활성화한다.

## 4. Observation

57개 observation은 기존 43개에 다음 14개를 append한다.

- 팔 xDrive target 6개
- 단계 one-hot 4개
- 안정 파지와 유효 유지 진행도 2개
- 현재 lesson의 상승·유지 목표 2개

기존 43개의 순서나 action 의미를 암묵적으로 바꾸지 않고 behavior/spec 버전을 함께 올렸다.
상세 index는 [`AGENT_SPEC.md`](AGENT_SPEC.md)에 고정한다.

## 5. Reward 설계

매 decision은 시간 비용 `-0.001`로 시작한다. 접근, 접촉, 상승, 유지는 모두 상태 potential의
차이만 지급해 왕복 동작이나 접촉 유지 파밍의 양의 보상을 반대 방향 변화가 상쇄하도록 한다.
`접근 → 파지 → 상승 → 유지` 각 단계가 dense한 신호를 갖게 설계한다.

- 접근 potential: 최대 `+1.0` (GraspPoint-공 거리 비례)
- 접촉 potential: 엄지 `+0.25`, 반대 손가락 `+0.25` (상실 시 반대 부호 차감)
- 최초 안정 파지: `+0.5`, episode당 한 번
- 상승 potential: lesson 목표 높이까지 최대 `+1.0`
- 최초 목표 높이 도달: `+1.0`, episode당 한 번
- 유지 potential: lesson 목표 시간까지 최대 `+1.0` (타이머 리셋 시 차감)
- 성공 terminal: `+3.0`
- 실패 terminal: Timeout `-0.1`, GripLost/Dropped `-0.5`,
  WorkspaceExit/Penetration/NonFinitePhysics `-1.0`
- 팔 이동 비용: `-lambda * Σ(|delta q| / jointRange)`

실패 terminal은 원인별로 차등을 둔다. Timeout은 과제 미완일 뿐이므로 약한 벌점만 주어
초기 탐색에서 shaping gradient가 큰 음수 terminal에 묻히지 않게 하고, 공을 잃는 실패와
안전 위반 실패는 더 큰 벌점을 유지한다.

이동량은 팔 실제 관절각의 decision 간 변화만 합산하고 closure는 제외한다. lesson 0~1은
`lambda=0`, lesson 2~3은 `0.01`, 최종 lesson은 `0.02`로 설정해 정지 정책을 먼저 강화하는
문제를 피한다.

## 6. Workspace와 curriculum

panel과 로봇은 고정하고 공만 reset한다. 공은 episode 시작 GraspPoint 방향을 중심으로 면적
균등 반경과 lesson별 방위 범위에서 표본화한다.

| Lesson | 방위 | 반경 | 상승 | 유지 | lambda |
|---:|---:|---:|---:|---:|---:|
| 0 | ±15° | 0.25~0.35m | 2cm | 0.5s | 0 |
| 1 | ±30° | 0.25~0.45m | 5cm | 1s | 0 |
| 2 | ±60° | 0.25~0.55m | 10cm | 2s | 0.01 |
| 3 | ±120° | 0.25~0.65m | 10cm | 3s | 0.01 |
| 4 | 360° | 0.25~0.70m | 10cm | 5s | 0.02 |

각 승급은 최소 200회 평가와 성공률 80%가 필요하다. stock ML-Agents는 custom
`Grasp/Success`를 curriculum criterion으로 사용할 수 없으므로 기본 YAML은 lesson 0을 constant로
고정한다. `training/scripts/promote_dg5f_lesson.py`가 평가 횟수와 성공률을 검증한 뒤에만 다음
lesson config를 생성하며, shaped reward에 의한 조기 자동 승급은 사용하지 않는다.

## 7. Reset과 병렬화

reset은 closure, 팔·손 관절 position/velocity/xDrive target, 공 pose/velocity/gravity,
접촉 집합, 상태기계 타이머, potential, 보너스 flag, 이동량 누계를 동기화한다. 공 배치 중에만
kinematic/gravity-off를 사용하고 곧바로 dynamic/gravity-on으로 복원한다.

scene은 20개의 독립 `TrainingArea.prefab`을 4x5로 배치한다. 모든 Agent가 하나의
`DG5FGraspV4` policy를 공유하지만 물리 상태와 episode는 서로 독립이다.

## 8. 기록과 검증

TensorBoard에는 stage, 성공, 실패 원인별 one-hot rate, 최대 상승, 최대 연속 유지,
정규화 팔 이동량을 기록한다. EditMode는 상태 경계를, PlayMode는 직렬화 계약과 100회 reset,
낙하·재파지·10cm·5초 시나리오를 검증한다.

50k smoke는 NaN, 무한 episode, 보너스 반복 수령, 즉시 실패 exploit을 확인한다. 최종 모델은
미학습 고정 seed 500회, 1배속에서 성공률 90% 이상이어야 한다. 그 후보 중 평균 팔 이동량이
가장 작은 checkpoint를 고르고 `lambda=0` 대비 이동량 10% 감소, 성공률 하락 2%p 이내인지
A/B 평가한다. BC/GAIL과 손 4-synergy 확장은 최종 lesson 정체 시 별도 실험으로 남긴다.
