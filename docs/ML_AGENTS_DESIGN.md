# DG5F 빨간 공 파지 강화학습 설계

## 1. 문서 목적

이 문서는 UR5e + DG5F가 빨간 공에 접근하고, 파지하고, 들어 올리는 강화학습 환경의 구조와 설계 이유를 설명한다.

- 변경 불가 인터페이스 계약: [`AGENT_SPEC.md`](AGENT_SPEC.md)
- 빌드 및 학습 실행 절차: [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)
- PPO 설정: `training/config/dg5f_grasp.yaml`
- Agent 구현: `unity/Assets/MLAgents/Grasp/Runtime/Dg5fGraspAgent.cs`

현재 계약 버전은 `DG5FGrasp Agent Spec v1.0.0`이다. 관측 순서, action 의미, 그리퍼 프로파일, 성공 조건을 바꾸면 spec 버전을 올리고 기존 checkpoint를 폐기하거나 별도로 관리한다.

## 2. 학습 목표

정책이 다음 동작을 순서대로 학습하는 것이 목표다.

1. UR5e를 빨간 공 근처로 이동한다.
2. DG5F의 엄지와 하나 이상의 반대편 손가락으로 공을 접촉한다.
3. 공을 초기 위치보다 5cm 이상 들어 올린다.
4. 안정적인 파지 상태를 1초 동안 유지한다.

최종 성공에는 다음 조건이 동시에 필요하다.

- 공 상승 높이 5cm 이상
- 엄지 접촉
- 검지·중지·약지·소지 중 하나 이상 접촉
- 공과 `GraspPoint` 거리 10cm 이하
- 위 상태를 1초 동안 연속 유지

최종 평가 목표는 고정 seed 100 episode 중 80회 이상 성공하고, 성공 episode의 중앙 완료 시간이 10초 이하인 것이다.

## 3. 전체 구조

현재 환경은 이미지가 아닌 Unity 물리 상태를 입력으로 사용하는 vector 기반 PPO다.

```text
43 vector observations
        ↓
MLP policy: 256 × 3 layers
        ↓
7 continuous actions
        ↓
UR5e xDrive target 6개 + DG5F closure 1개
```

- Behavior name: `DG5FGrasp`
- Algorithm: PPO
- Physics frequency: 50Hz
- Policy decision frequency: 10Hz
- Episode 제한: 750 physics steps = 150 decisions = 15초
- Reward signal: extrinsic only
- 기본 학습 장치: CPU

## 4. Action 설계

### 4.1 UR5e action 6개

`action[0..5]`는 UR5e 6개 관절의 절대 각도가 아니라 현재 xDrive target에 더하는 증분값이다.

```text
target_delta = clamp(action, -1, 1) × 2 degrees / decision
```

각 관절 target은 `Dg5fGraspSpec.ArmSafeMinDeg`와 `ArmSafeMaxDeg` 범위로 제한하고, 마지막으로 URDF drive limit에도 clamp한다.

증분 action을 사용한 이유:

- 절대 각도 출력보다 policy 출력 변화가 부드럽다.
- 한 decision의 최대 움직임을 제한할 수 있다.
- 초기 random policy가 큰 자세 점프를 만드는 위험을 줄인다.

### 4.2 DG5F action 1개

`action[6]`은 그리퍼 전체 closure의 증분값이다.

```text
closure_delta = clamp(action, -1, 1) × 0.04 / decision
closure = clamp(closure + closure_delta, 0, 1)
```

- `closure=0`: OPEN
- `closure=1`: 검증된 FIST 관절 프로파일

DG5F의 20개 회전 관절은 `OPEN → FIST` 프로파일을 closure 값으로 선형 보간한다. 실제 drive target은 각 관절의 URDF limit로 다시 제한한다.

20개 손 관절을 모두 독립 action으로 사용하지 않은 이유:

- 팔 6개와 합치면 action 차원이 26개로 커진다.
- 순수 PPO의 초기 접촉 탐색 난도가 크게 증가한다.
- 현재 목표는 다양한 손 모양 생성보다 안정적인 공 파지다.

따라서 v1은 `팔 6 + 손 1 = 연속 action 7개`로 제한한다.

## 5. Observation 설계

관측은 총 43개이며 spec v1 안에서 순서가 고정된다.

| 범위 | 개수 | 내용 |
|---:|---:|---|
| 0..5 | 6 | 정규화된 팔 관절 위치 |
| 6..11 | 6 | 정규화된 팔 관절 속도 |
| 12 | 1 | `[-1,1]`로 변환한 closure |
| 13..15 | 3 | `GraspPoint` 기준 공 위치, robot-base 좌표계 |
| 16..18 | 3 | 공 선속도, robot-base 좌표계 |
| 19..21 | 3 | 공 각속도, robot-base 좌표계 |
| 22 | 1 | episode 시작 높이 대비 공 상승량 |
| 23..37 | 15 | 공 기준 다섯 손끝 위치, palm 좌표계 |
| 38..42 | 5 | 엄지·검지·중지·약지·소지 접촉 여부 |

설계 의도:

- 관절 위치·속도: 현재 로봇 상태와 움직임 방향 제공
- 공 상대 위치: 팔이 어디로 이동해야 하는지 제공
- 공 속도: 공이 튕기거나 떨어지는 상태 구분
- 손끝 상대 위치: 손가락이 공을 감싸는 정도 제공
- 접촉 flag: 시각적으로 구분하기 어려운 실제 물리 접촉 제공
- lift 값: 들어 올리기 단계의 진행 상태 제공

현재 관측은 Unity가 직접 제공하는 privileged state다. 카메라나 실제 센서 입력이 아니므로 sim-to-real 단계에서는 별도 관측 설계가 필요하다.

## 6. Reward 설계

Reward는 sparse success만 사용하는 대신 접근, 감싸기, 접촉, 상승을 단계적으로 제공한다.

| 항목 | Reward |
|---|---:|
| 매 decision 시간 비용 | `-0.001` |
| 공 접근 진행량 | 최대 `±0.15` |
| 손끝 감싸기 진행량 | 최대 `±0.10` |
| 엄지 + 반대 손가락 첫 접촉 | `+0.15`, episode당 1회 |
| 공 상승 진행량 | 최대 `±0.25` |
| 성공 | `+1.0` |
| 실패 | `-0.5` |

접근, 감싸기, 상승 reward는 상태의 절대값이 아니라 이전 decision 대비 변화량으로 계산한다.

```text
approach reward ∝ previous distance - current distance
enclosure reward ∝ previous mean tip distance - current mean tip distance
lift reward ∝ current lift - previous lift
```

이 방식은 좋은 위치에 가만히 있는 행동이 매 step 동일한 보상을 반복해서 받는 문제를 줄인다. 목표에서 멀어지거나 공을 낮추면 음의 reward를 받을 수 있다.

실패 종료 조건:

- dynamic lesson에서 공이 테이블 아래로 떨어짐
- 공이 설정된 workspace 반경 밖으로 이동
- 공 위치가 `NaN` 또는 무한대가 되는 물리 오류

## 7. Curriculum 설계

전체 파지 동작을 처음부터 요구하지 않고 세 lesson으로 나눈다.

### Lesson 0: Reach

- 공: kinematic
- spawn half-width: 2cm
- 성공: `GraspPoint`와 공의 거리가 5cm 이하인 상태를 0.25초 유지

목적: 손가락 접촉과 공 낙하 없이 팔 접근부터 학습한다.

### Lesson 1: Grasp

- 공: dynamic, gravity 적용
- spawn half-width: 4cm
- 성공: 엄지와 반대편 손가락 접촉을 0.5초 유지

목적: 공과 실제 충돌하면서 closure 타이밍과 접촉을 학습한다.

### Lesson 2: LiftAndHold

- 공: dynamic, gravity 적용
- spawn half-width: 6cm
- 성공: 최종 성공 조건을 1초 유지

목적: 접근·파지·들기를 하나의 episode에서 수행한다.

Reach와 Grasp lesson은 최소 200 episode를 수행하고 smoothed reward가 0.8 이상일 때 다음 lesson으로 전환된다.

## 8. Episode reset

매 episode 시작 시 다음 상태를 복원한다.

1. 팔 6개와 손 20개 회전 관절을 prefab의 시작 target으로 teleport
2. 모든 관절 속도를 0으로 설정
3. closure를 0으로 초기화하고 손을 OPEN target으로 설정
4. 손끝 접촉 기록 초기화
5. 공 선속도와 각속도 초기화
6. lesson별 범위 안에서 공 x/z 위치와 yaw 랜덤화

리셋 직후 이전 episode의 닫힌 손 target이나 공 속도가 남지 않도록 reset 순서를 고정했다. PlayMode에서 100회 연속 리셋, 관절 유한값, 공 spawn 범위를 검증한다.

## 9. PPO 설정

기본 설정:

```yaml
trainer_type: ppo
batch_size: 1024
buffer_size: 10240
learning_rate: 0.0003
beta: 0.005
epsilon: 0.2
lambd: 0.95
num_epoch: 3
hidden_units: 256
num_layers: 3
gamma: 0.99
time_horizon: 128
max_steps: 5000000
checkpoint_interval: 500000
```

네트워크는 recurrent memory가 없는 3-layer MLP다. reward signal은 extrinsic만 사용한다. BC, GAIL, demonstration 데이터는 v1에 포함하지 않았다.

## 10. 학습 중 해석

초기 policy는 random continuous action을 출력하므로 로봇팔이 목적 없이 흔들리거나 여러 방향으로 움직일 수 있다. 이는 학습 시작 직후 정상 현상이다.

Trainer가 curriculum을 적용하면 처음에는 Reach lesson부터 시작한다. 따라서 초기에는 공을 집거나 들어 올리는 행동이 아니라 공 근처로 이동하는 행동을 먼저 학습한다. Reach와 Grasp 기준을 통과한 뒤에만 최종 LiftAndHold lesson으로 이동한다.

확인할 지표:

- `Environment/Cumulative Reward`
- episode length
- policy entropy
- curriculum lesson
- `Grasp/Success`
- `Grasp/CompletionSeconds`

## 11. 현재 한계와 후속 개선

### 현재 한계

1. 그리퍼가 closure 한 축만 사용하므로 손가락별 독립 전략을 만들 수 없다.
2. 카메라가 아닌 Unity 내부 공 위치·속도를 직접 관측한다.
3. 공 위치와 yaw 외 질량·마찰·관절 오차 domain randomization이 없다.
4. 순수 PPO이며 demonstration 기반 BC/GAIL이 없다.
5. 기본 config가 CPU 학습으로 고정되어 있다.
6. 512-step communicator smoke는 통과했지만 50k/5M 수렴과 최종 성공률은 아직 검증 대상이다.

### 후속 순서

1. 50k smoke에서 reward, episode length, 물리 안정성 확인
2. 1x/10x/20x 접촉 안정성 비교
3. 5M 본학습
4. 고정 seed 100 episode 평가
5. 수렴 실패 원인을 접근·접촉·상승 단계로 분리
6. 필요할 때만 독립 손가락 action, domain randomization, demonstration 학습 추가

한 번에 여러 계약을 변경하지 않는다. observation/action 변경은 기존 checkpoint와 호환되지 않으므로 spec 버전과 RUN_ID를 함께 변경한다.
