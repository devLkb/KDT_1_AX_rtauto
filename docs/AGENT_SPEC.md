# DG5FGraspPointReach Agent Spec 1.0.0

이 문서는 학습 플레이어, PPO 설정, 평가기의 단일 정책 계약이다. 현재 정책은 손가락을
제어하지 않고 UR5e 팔을 움직여 `GraspPoint`를 목표 좌표에 도달시킨다.

## Policy

- Behavior: `DG5FGraspJoint`
- vector observations: 116, stack 1
- continuous actions: 26
- decision period: 5 physics steps
- stage 1 timeout: 5 simulation seconds
- stage 2/3 timeout: 20 simulation seconds, with a distinct 5-second
  post-reach grasp timeout (`MaxStep=0`)

## Observation order

모든 위치와 방향은 robot-base 좌표계, 길이는 미터, 시간은 초 기준이다. 환경에서
명시적으로 정규화하므로 trainer의 observation normalization은 사용하지 않는다.

| 인덱스 | 길이 | 값 |
|---:|---:|---|
| `0..5` | 6 | 안전 관절 범위로 정규화한 팔 관절각 |
| `6..11` | 6 | 정규화한 팔 관절속도 |
| `12..17` | 6 | 정규화한 xDrive 명령 target |
| `18..20` | 3 | `(target - GraspPoint) / 1.05 m`, 각 축 `[-1,1]` clamp |
| `21` | 1 | 중심 거리 `/ 1.05 m`, `[0,1]` clamp |
| `22..24` | 3 | 회전 오프셋을 포함한 GraspPoint 선속도 `/ 1 m/s`, 각 축 `[-1,1]` clamp |
| `25` | 1 | 성공 조건 연속 유지 진행도 `0..1` |

NaN 또는 Infinity는 관측에 내보내지 않고 즉시 물리 실패로 종료한다.

## Action order

`0..5`는 UR5e 6관절 순서의 연속 행동이다. 각 행동은 현재 명령 target에 더하는
관절 증분이며 `[-1, 1]`을 decision당 `[-4°, +4°]`에 대응시킨다. 적용 결과는 URDF
물리 한계와 프로젝트 안전 한계의 교집합으로 clamp한다. 손가락 ArticulationBody의
xDrive에는 정책이 쓰지 않는다.

## Target and reset

- 각 episode에서 빨간 목표 중심을 패널 평면의 환형 영역에 새로 생성한다.
- robot-base 평면 반경은 `0.20..0.85 m`, 방위각은 `0..360°`이며 각각 균등
  표본화한다.
- 목표 구 반지름은 `0.02 m`, 중심 높이는 패널 상면으로부터 `0.02 m`다.
- 패널 경계 밖, 로봇 collider와 겹치는 위치, GraspPoint 초기 거리 `0.10 m` 미만은
  최대 256회 재표본화한다.
- 각 병렬 영역은 독립 seed를 사용한다. reset은 팔 위치/속도/drive target, 목표,
  성공 hold timer, 이전 거리와 누적 상태를 모두 초기화한다.

목표는 static trigger이므로 팔이 목표를 밀어 성공 조건을 바꿀 수 없다.

## Reward and termination

decision 보상:

- `-0.001` per decision
- stage 1: no approach-potential reward
- stage 2/3: `0.25 * delta(approach potential)`
- thumb-only contact potential `0.25`
- opposing-only contact potential `0`
- dual-contact potential `0.5`
- hold potential `0.5 * clamp(hold / 0.5 seconds)`
- dual contact held for 0.5 seconds: `+2.0`, success termination
- ball out of bounds or non-finite physics: `-1.0`, failure termination
- ordinary timeout or post-reach timeout: failure termination
- `Failure/Timeout` counts both timeout kinds;
  `Failure/PostReachTimeout` marks the post-reach subset

성공은 아래 두 조건을 `0.25 s` 연속 만족할 때다.

- 중심 거리 `<= 0.01 m`
- GraspPoint 선속도 `<= 0.05 m/s`

성공 terminal reward:

1. ball within 4 cm of grasp point, fixed 35% pre-grasp, all arm targets locked
   to their initial pose, independent hand deltas capped at 1 degree
2. V1 0.35–0.70 m spawn distribution, normal arm, fixed pre-grasp
3. V1 spawn distribution, fully open hand, no control assistance

Lessons 1 and 2 advance on the unsmoothed mean reward of the most recent 200
episodes, at thresholds `2.2` and `1.8` respectively.

Deterministic approval evaluation always uses stage 3.

- 20 simulation seconds timeout: `-1`
- workspace 이탈 또는 비유한 물리 상태: `-2`

접근과 후퇴의 progress는 서로 상쇄된다. 실패 시 이미 받은 progress를 별도로 되돌리는
terminal settlement는 사용하지 않는다.

## Evaluation gate

학습과 겹치지 않는 결정론적 seed 500개를 평가한다.

- 성공률 `>= 90%`
- 모든 성공 행이 거리/속도/hold 조건 충족
- 중복 seed, 비유한 물리, workspace 안전 실패 0건
- 통과 모델은 평균 최종 오차가 작은 순, 동률이면 median 및 p95 완료 시간이 짧은
  순으로 선택

카메라 좌표 수신은 이 spec의 입력 경계 바깥이다. 후속 adapter는 보정한 robot-base
미터 좌표를 target Transform에 주입해야 하며 policy shape를 바꾸지 않는다.
