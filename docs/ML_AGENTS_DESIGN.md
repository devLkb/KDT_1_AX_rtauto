# DG5F 거리 기반 공 파지 강화학습 설계

## 1. 목적과 계약

이 환경은 UR5e + DG5F가 로봇 주변 3D workspace의 빨간 공에 접근해 접촉 파지를 완성하도록 학습한다.

- 고정 인터페이스 계약: [`AGENT_SPEC.md`](AGENT_SPEC.md)
- 빌드·학습 실행: [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)
- PPO 설정: `training/config/dg5f_grasp.yaml`
- Agent: `unity/Assets/MLAgents/Grasp/Runtime/Dg5fGraspAgent.cs`

현재 계약은 `DG5FGrasp Agent Spec v2.0.0`이다. v1 checkpoint는 재사용하지 않고 결과만 보존한다.

## 2. 단일 과제

정책은 별도 단계 없이 한 episode에서 다음 과제를 수행한다.

1. 임의 방위·반경·높이에 놓인 공으로 GraspPoint를 이동한다.
2. DG5F closure를 조절한다.
3. 엄지와 하나 이상의 다른 손가락이 공에 접촉한 상태를 1 시뮬레이션초 유지한다.

접촉 유지가 성공의 전부이며 공의 추가 이동이나 높이 변화는 성공 조건이 아니다.

## 3. 환경 구조

```text
43 vector observations
        ↓
MLP policy: 256 × 3 layers
        ↓
7 continuous actions
        ↓
UR5e xDrive targets 6 + DG5F closure 1
```

- Behavior: `DG5FGrasp`
- Algorithm: PPO
- Physics: 50Hz
- Policy decision: 10Hz
- Fixed episode cutoff: 없음 (`MaxStep=0`)
- Reward signal: extrinsic distance cost only
- 기본 학습 장치: CPU

## 4. Action

### 4.1 Arm

`action[0..5]`는 현재 xDrive target에 더하는 증분이다.

```text
target_delta = clamp(action, -1, 1) * 2 degrees / decision
```

shoulder pan은 전체 고유 방위를 다룰 수 있도록 `[-180°, 180°]`를 사용한다.
나머지 다섯 관절의 안전범위는 기존 값을 유지한다. 모든 target은 안전범위와 URDF drive limit에 차례로 clamp한다.

### 4.2 Hand

`action[6]`은 하나의 누적 closure다.

```text
closure_delta = clamp(action, -1, 1) * 0.04 / decision
closure = clamp(closure + closure_delta, 0, 1)
```

20개 손 관절은 검증된 `OPEN -> FIST` 프로파일을 closure로 선형 보간한다.
독립 손가락 action을 추가하지 않아 연속 action 크기를 7로 유지한다.

## 5. Observation

관측은 총 43개다. 상세 순서는 [`AGENT_SPEC.md`](AGENT_SPEC.md)에 고정한다.

- 팔 관절 위치 6 + 속도 6
- closure 1
- 공 상대 위치·선속도·각속도 9
- 공 수직 변위 1
- 공 기준 손끝 상대 위치 15
- 손가락 접촉 flag 5

관절 위치는 각 관절의 학습 안전범위를 `[-1,1]`로 변환한다. 따라서 shoulder pan의 관측도 새 `[-180°,180°]` 범위를 정확히 반영한다.

## 6. Workspace randomization

받침대 중심의 방위와 상단 높이, 수평 반경을 매 reset 새로 뽑는다.

- 방위: 360° 균등
- 반경: 0.25~0.70m, 면적 균등
- 상단 높이: 0.25~0.65m 균등
- 공 중심의 robot-base 3D 거리: 최대 0.80m

반경은 제곱 반경을 균등하게 뽑아 특정 반경대에 표본이 몰리지 않게 한다. 0.80m 경계를 넘는 조합은 rejection sampling한다.
받침대는 지름 0.30m의 원형 collider로, 바닥에서 표본화된 상단 높이까지 이어진다. 공은 원형 상단 중앙에 접하도록 놓인다.

## 7. Distance-only reward

reward는 policy가 직접 줄일 수 있는 GraspPoint-공 거리 하나만 사용한다.

```text
reward = -0.01 * clamp(grasp_distance / 0.85, 0, 1)
```

접근할수록 decision당 비용의 절댓값이 작아지고, 공에서 0.85m 이상 떨어지면 `-0.01`로 포화한다.
접촉과 episode 종료는 학습 신호에 별도 상수를 더하지 않는다.

## 8. Success and reset safety

성공 판정과 모든 안전 판정은 50Hz `FixedUpdate`에서 시뮬레이션 시간으로 누적한다.

- 성공: 엄지 + 다른 손가락 하나 이상의 접촉을 1초 연속 유지
- 이탈: 공 중심의 robot-base 거리 > 0.85m
- 낙하: 공 중심이 받침대 상단보다 낮음
- 물리 오류: 공 좌표가 비유한 값
- 관통: hand collider와 받침대가 1cm 이상 겹친 상태를 0.2초 유지
- 정체: 최저 거리 기준 2cm 의미 있는 개선 없이 60초 경과

2cm 개선 시 정체 기준 거리와 타이머를 함께 갱신한다. 접촉이나 관통이 임계시간 전에 끊기면 해당 연속 타이머는 0이 된다.

## 9. Reset completeness

reset은 관절 position/velocity와 xDrive target, closure, 접촉 집합, 공 pose/velocity, 받침대 pose, 모든 연속 타이머를 초기화한다.
공을 옮길 때는 잠시 kinematic/gravity-off 상태로 만들고 받침대 상단에 배치한 다음 dynamic/gravity-on 상태를 복원한다.

## 10. PPO and evaluation

PPO network와 기본 하이퍼파라미터는 기존 256×3 MLP 구성을 유지한다. 설정에는 별도 환경 파라미터가 없다.
본학습 전 다음 순서로 검증한다.

1. EditMode 계약·경계 테스트
2. PlayMode 씬·100회 reset·관절 범위 테스트
3. Linux player 빌드
4. 512-step Python↔Unity communicator smoke
5. 50k 안정성 smoke 후 `dg5f_grasp_v2` 본학습

현재 범위에는 BC/GAIL, domain randomization, 독립 손가락 제어, 새 reward 항목이 포함되지 않는다.
