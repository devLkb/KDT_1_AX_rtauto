# DG5F Grasp + Lift (behavior `DG5FGraspLift`)

`Grasp` 브랜치의 파지·들어올리기 학습 구현. 요구사항은
[`docs/GraspClaude.md`](GraspClaude.md), 참고 설계는 Isaac Lab
[`VAlikV/IsaacLab_delto_envs`](https://github.com/VAlikV/IsaacLab_delto_envs)
(`envs/tesolo_delto_UR_env/delto_env.py`).

Isaac Lab 구현에서 가져온 것은 **설계 의도뿐**이다. 코드는 Unity 물리 /
ML-Agents 구조에 맞게 전부 새로 작성했다.

| Isaac Lab | 우리 구현 | 비고 |
|---|---|---|
| `grasp_contact_min = 3` | `GraspContactMinimum = 3` | 동일한 근거(3점 이상이어야 케이지) |
| `_force_closure_score` (쌍별 각도) | `MaximumOppositionAngleDegrees` + `IsForceClosureLike` | O(n²) 각도 계산은 동일, sigmoid score 대신 임계 판정 |
| `grasp_max_dist` (COM ↔ 파지중심) | `GraspCenterMaxDistance` | 동일 개념 |
| `grasp_hold_steps = 20` | `GraspConfirmSeconds = 0.3 s` | step이 아니라 시뮬레이션 시간 기준 |
| 스크립트 lift (`lift_delta_deg` 보간) | **없음** — 정책이 직접 팔을 올린다 | 스크립트 상승은 "파지 후 들어올리기"를 학습시키지 못한다 |
| `success_height = 0.12` | `LiftTargetHeight = 0.10` (커리큘럼 0.05→0.10) | |
| `slip_grace = 10 steps` | `SlipGraceSeconds = 0.20 s` | |
| `tilt_fail_deg` 종료 | 사용 안 함 | 대신 `ObjectPushedAway` / `Dropped` 로 커버 |
| 실린더 Ø0.06 × 0.15 m | Cube 0.055 × 0.10 × 0.055 m | 아래 "학습 Object" 참고 |

---

## 1. 왜 새 Behavior인가

기존 `DG5FGrasp`(= `Assets/MLAgents/Grasp`)는 이름과 달리 **파지 전 top-down
접근(pre-grasp reach)만** 학습한다. 손가락 20관절은 정책이 건드리지 않고
prefab의 열린 자세로 고정되며(`enablePolicyClosure = false`), 7번째 action은
호환용으로 무시된다. 성공 조건도 "물체 표면 3 cm 안에서 top-down 자세를
0.3초 유지"까지다.

그 구조 위에 파지를 얹을 수 없는 결정적인 이유가 두 가지 있다.

1. **하강 자체가 실패로 처리된다.** `IsUnsafeLowClearanceMotion` /
   `IsMisalignedDescent`가 패널 10 cm 이내로 내려오는 동작을 중단시키고,
   패널에 닿는 모든 로봇 콜라이더가 `SafetyPenalty(-2)`로 에피소드를 끝낸다.
   파지는 손가락이 테이블 근처까지 내려가야 가능하므로 정반대 계약이다.
2. **손이 정책 제어 대상이 아니다.**

그래서 `Assets/MLAgents/GraspLift/`에 새 Behavior를 만들었다. 기존
`Grasp`/`Reach` 코드는 파이프라인 데모·텔레옵 씬이 계속 쓰므로 **손대지
않았다**.

다만 관측/행동 shape은 **의도적으로 기존과 동일한 57/7**로 맞췄다. 이미
학습된 pre-grasp 정책(25.3° / 69%,
[`DG5F_PREGRASP_ANGLE_RESULT.md`](DG5F_PREGRASP_ANGLE_RESULT.md))을
`--initialize-from`으로 그대로 물려받기 위해서다. CPU 학습 환경에서 접근
단계를 처음부터 다시 배우는 것은 비용이 너무 크다.

---

## 2. 학습 Object — Cube

기존 공(반지름 2 cm)은 DG5F(손가락 길이 ~13 cm)에 비해 너무 작아 안정적인
파지가 불가능하다. `GraspClaude.md`가 허용한 대로 Cube로 교체했다.

> **크기는 추측이 아니라 실측으로 정했다.** 최초 구현은 URDF 산술로 0.055 m를
> 골랐는데, 시뮬레이터에서 닫힌 손을 실제로 재보니 대향 가능 범위 밖이라
> 물리적으로 파지가 불가능했다. 아래 "닫힌 손 실측" 절 참고.

| 항목 | 값 | 근거 |
|---|---|---|
| 크기 | 0.04 × 0.09 × 0.04 m (기본값) | 아래. 폭은 `block_width` 환경 파라미터로 런타임 변경 가능(0.025–0.06) |
| Mass | 폭²×높이×400 kg/m³ (기본 0.058 kg) | 부피에 비례 — 큰 블록이 자동으로 무거워져 난이도가 실제 물체처럼 스케일된다 |
| Collider | BoxCollider (primitive Cube 기본) | |
| Physics material | staticFriction 1.5 / dynamicFriction 1.2, combine **Maximum** | 참고 구현의 object `static_friction = 2.0`과 같은 의도. 패널은 0.8/Average로 남겨서 "밀면 미끄러지고, 잡으면 안 미끄러지게" 분리 |
| Rigidbody | mass 0.12, useGravity, **ContinuousDynamic** | 손가락이 물리 스텝보다 빠르게 닫히면 discrete 검출은 5 cm 블록을 뚫는다 |
| 초기화 범위 | robot-base local 반경 0.35–0.55 m 환형(면적 균등), Y축 랜덤 yaw, 패널 위 | reach 과제(0.35–0.70)보다 좁힘 — 파지는 난도가 훨씬 높다 |

**단면 0.055 m**: DG5F URDF의 검지/중지/약지 knuckle이 y = −0.027 /
−0.0025 / +0.022 (`lj_dg_2_1`..`lj_dg_4_1`)로 약 0.049 m를 차지한다. 5.5 cm
면은 세 손가락 span과 거의 같아서, 손가락이 실제로 감싸야만 잡히고 감싸면
잡히는 크기다.

**높이 0.10 m**: 손가락이 knuckle부터 ~0.13 m라 납작한 큐브를 테이블 위에
두면 파지 시 손가락이 테이블을 뚫어야 한다. 참고 구현이 굳이 0.15 m 높이
실린더를 쓴 것과 같은 이유다. 세로로 세운 블록이면 손이 **윗부분만 공중에서**
감쌀 수 있다.

### 닫힌 손 실측 (`GraspLiftHandGeometryProbe`)

씬을 로드하고 에이전트를 끈 뒤 손만 닫으면서 palm-local 좌표를 측정한 결과:

| closure | fingertip centroid (palm-local) | thumb–index | thumb–middle |
|---|---|---|---|
| 0.00 | (0.017, 0.153, 0.014) | 0.208 m | 0.224 m |
| 0.50 | (0.004, 0.104, 0.090) | 0.045 m | 0.067 m |
| **0.75** | (−0.006, 0.066, 0.075) | **0.036 m** | **0.031 m** |
| 1.00 | (−0.012, 0.049, 0.041) | 0.063 m | 0.046 m |

두 가지가 확정됐다.

1. **대향 가능 폭은 약 3.1–3.6 cm다.** 5.5 cm 블록은 손가락이 마주 볼 수조차 없다.
   따라서 기본 폭을 4 cm로 낮추고, `block_width` 파라미터로 실측 비교가 가능하게 했다.
2. **closure 1.0은 0.75보다 나쁜 그립이다.** 완전히 쥐면 손가락이 서로 지나쳐
   간격이 다시 벌어진다(0.036 → 0.063 m). 그래서 폐합 보상을 closure에 단순 비례로
   주면 안 되고, 최적 그립 closure에서 포화시켜야 한다
   (`EffectiveGripClosure`, 아래 Reward 절).

또한 closure 1.0의 centroid (−0.012, 0.049, 0.041)는 설정된 GraspPoint
(0, 0.05, 0.04)와 1.2 cm 차이로, **GraspPoint 위치 자체는 옳았음**이 확인됐다.

### 높이

높이를 처음에 0.12 m로 잡았다가 0.10 m, 다시 0.09 m로 낮췄다. 0.055:0.10 비율은 약 29°에서
전도되는데, 0.05:0.12는 약 23°라 smoke run에서 초기 정책이 스치기만 해도
계속 넘어졌다(실측: `FinalLiftHeight ≈ −0.03`, 즉 무게중심이 내려앉음).

---

## 3. Observation (57차원)

슬롯 0..48은 reach 체크포인트와 **의미·순서·정규화가 동일**하다(전이 목적).
49..56만 grasp/lift용으로 재정의했다.

| 슬롯 | 개수 | 내용 | 정규화 |
|---|---|---|---|
| 0–5 | 6 | 팔 6관절 각도 | `ArmSafeMin/MaxDeg` → [−1,1] |
| 6–11 | 6 | 팔 6관절 각속도 | /π, clamp |
| 12 | 1 | 손 closure | ×2−1 |
| 13–15 | 3 | (파지 목표점 − graspPoint), robot-base 좌표 | /1.0 m |
| 16–18 | 3 | 블록 선속도, robot-base | /2 m·s⁻¹ |
| 19–21 | 3 | 블록 각속도, robot-base | /10 rad·s⁻¹ |
| 22 | 1 | 블록 높이 − 스폰 높이 | /0.2 m |
| 23–37 | 15 | 손가락 5개 끝 − 블록 중심, **palm 좌표** | /0.2 m |
| 38–42 | 5 | 손가락 끝 접촉 플래그 | 0/1 |
| 43–48 | 6 | 팔 xDrive 지령값 | `ArmSafeMin/Max` → [−1,1] |
| 49 | 1 | **손바닥 접촉 플래그** | 0/1 |
| 50 | 1 | **접촉점 수 / 6** | 0..1 |
| 51 | 1 | **grasp dwell 진행률** | 0..1 |
| 52 | 1 | **grasp 확정 여부** | 0/1 |
| 53 | 1 | **lift 진행률** (높이/목표높이) | 0..1 |
| 54 | 1 | **lift hold 진행률** | 0..1 |
| 55 | 1 | graspPoint↔목표점 거리 | /0.85 m |
| 56 | 1 | 경과 시간 / 20 s | 0..1 |

"파지 목표점"은 블록 중심이 아니라 **중심 + 수직 2.5 cm**(윗면 2.5 cm 아래)다.
top-down power grasp에서 손바닥은 윗면 위에 있고 손가락이 윗부분을 감싸므로,
palm grasp volume 중심을 기하학적 중심에 맞추면 손바닥이 블록 안으로 들어간다.

---

## 4. Action (연속 7차원)

| 인덱스 | 의미 | 적용 |
|---|---|---|
| 0–5 | 팔 6관절 목표각 **증분** | `±2°/decision`, 물체 8 cm 이내에서는 ×0.35, `ArmSafeMin/MaxDeg` clamp |
| 6 | 손 closure **증분** | `±0.08/decision`, closure∈[0,1] |

closure 하나가 DG5F 20관절 전부를 prefab의 열린 자세 ↔ 검증된 주먹 자세
(`LeftFistDeg`)로 보간한다. 20차원 개별 제어 대신 1차원으로 둔 이유:

* 이 자세는 이미 실기에서 검증된 파지 자세다.
* CPU 학습에서 20차원 손 탐색은 현실적으로 수렴하지 않는다.
* 7차원 유지 = reach 정책 전이 가능.

decision period 5, fixed step 0.02 s → 0.1 s마다 결정, 20 s 에피소드 = 200 결정.

---

## 5. Reward

모든 항은 `Dg5fGraspLiftSpec`에 상수로 있고 코드에 사유 주석이 있다.

### 접근 (grasp 확정 전에만 계산)

| 항 | 식 | 이유 |
|---|---|---|
| 시간 비용 | `-0.001` / decision | 제자리 진동이 공짜가 되지 않게 |
| 접근 potential | `Δφ`, `φ = 1·(1 − clamp01(d/0.85))`, 손바닥이 물체를 향할 때만 | potential-based shaping이라 최적 정책을 바꾸지 않으면서 접근을 학습 가능하게 만든다. 손바닥 반대편에서 다가가는 것은 파지로 이어지지 않으므로 게이트 |
| top-down potential | `Δmax φ`, `φ = 0.3·p²`, `p = (70° − angle)/(70° − 35°)`, 물체 15→20 cm 이내 & 물체보다 위일 때만 | 손바닥이 아래를 향해야 파지가 가능. reach 과제에선 주 목표였지만 여기선 보조라 가중치를 0.5 → 0.3으로 낮췄다. 신기록 갱신분만 지급해 왕복 farming 차단 |
| 근접 제어 페널티 | `-0.002 · Σa²/6` (물체 8 cm 이내) | 물체 옆에서 팔을 거칠게 쓰면 블록을 쳐낸다 |

### 파지

| 항 | 식 | 이유 |
|---|---|---|
| 폐합 potential | `Δmax φ`, `φ = 0.5 × closure` (물체 10 cm 이내일 때만) | **"손가락을 닫는 행동 자체"에 보상을 주지 않기 위한 핵심 장치.** 물체 근처에서 닫을 때만 지급. **신기록분만** 지급하므로 에피소드당 총액이 0.5로 묶인다 — 여는 동작이 공짜이므로 평범한 per-step delta였다면 손을 폈다 쥐었다 반복하는 무한 farming loop가 된다(실제로 초기 구현의 버그였다. 왕복 1회당 +0.5, 20초에 약 +4로 실제 파지보다 쉽게 벌린다) |
| 폐합 원거리 페널티 | `Δclosure⁺ × (−0.25)` (물체 10 cm 밖) | 공중에서 주먹을 쥐는 전형적인 degenerate 행동. 페널티는 왕복으로 farming할 수 없으므로 이쪽은 per-step delta로 둔다. 여는 동작은 항상 0 |
| 접촉 potential | `Δmax φ`, `φ = 0.4·clamp01(n/3)` | 손가락을 실제로 블록에 올리는 것에 dense credit. 신기록분만 지급 → 툭툭 치기 farming 불가 |
| grasp 진행 potential | `Δmax (1.0 × dwell/0.3 s)` | 유효 파지를 유지하는 것에 부분 점수. 신기록분만 지급하므로 확정까지 총 지급액이 정확히 `GraspConfirmReward = 1.0` |

### 들어올리기

| 항 | 식 | 이유 |
|---|---|---|
| lift potential | `Δφ` (신기록 아님), `φ = 2.0 · clamp01(h/목표높이)` | 이 단계의 주 보상. **신기록이 아니라 매 스텝 델타**라서, 블록이 다시 내려가면 받았던 shaping을 그대로 토해낸다 — 떨어뜨림에 정확히 필요한 gradient |
| 성공 | `+5.0` (에피소드 종료) | 목표 높이·안정 유지 달성 |

### 잘못된 행동

| 항 | 값 | 조건 |
|---|---|---|
| `UnsafeSurfaceContact` | −2.0, 종료 | 움직이는 **팔 링크**가 패널에 충돌 |
| `Dropped` | −1.0, 종료 | 파지 확정 후 0.2 s 넘게 접촉 상실 **그리고** 최고 높이의 30% 아래로 낙하 |
| `ObjectPushedAway` | −1.0, 종료 | 파지 전 블록이 스폰 위치에서 수평 15 cm 이상 밀림 |
| `ObjectOutOfBounds` | −1.0, 종료 | 블록이 패널 밖으로 떨어지거나 작업공간(0.85 m) 이탈 |
| 닫힌 빈손 상승 | −0.004 / decision | `!grasp && closure ≥ 0.3 && graspPoint가 패널 위 15 cm 초과`. **"Grasp 없이 팔만 상승"** 대책. 절벽형 페널티 대신 지속적인 소액이라 가치함수를 불안정하게 만들지 않는다 |
| `Timeout`, `NonFinitePhysics` | 0 | shaping이 이미 도달 정도를 반영한다. 여기에 절벽을 더하면 "아예 시도하지 않는" 정책이 최적이 되어버린다 |

---

## 6. Grasp 성공 판정

**손가락을 닫았다는 것만으로는 절대 성공이 아니다.** 아래 4개를 **모두**
연속 0.3초 동안 만족해야 파지 확정이다(한 프레임이라도 깨지면 dwell 0으로 리셋).

1. **접촉점 ≥ 3** — 손가락 끝 5개 + 손바닥 중 3개 이상이 블록에 접촉.
2. **대향(force-closure proxy)** — 블록 중심에서 각 접촉점으로의 방향 중
   가장 벌어진 쌍의 각도 ≥ 90°. 이게 없으면 세 손가락으로 한쪽 면만 찌르는
   것도 파지로 오판된다.
3. **기하 구속** — 블록 중심과 접촉점 centroid 거리 ≤ 7 cm. 케이지 가장자리에
   걸친 것이 아니라 블록이 케이지 **안**에 있어야 한다.
4. **closure ≥ 0.3** — 손가락이 실제로 굽혀져 있어야 한다.

접촉 센서는 **콜라이더 GameObject에 붙인다**. URDF 임포터가 콜라이더를 링크의
`Collisions` 자식 아래에 두기 때문에, 링크에만 붙이면 `OnCollision*`이 오지
않는다(기존 `GraspContactSensor`의 실제 버그 — 아래 8절).

## 7. Lift 성공 판정 / Episode

**성공**: 파지 확정 이후
`블록 높이 − 스폰 높이 ≥ 목표높이`(커리큘럼 0.05 → 0.08 → 0.10 m) **그리고**
`|블록 선속도| ≤ 0.5 m/s`인 상태를 `LiftHoldSeconds`(0.25 → 0.35 → 0.50 s)
연속 유지. 던져 올린 블록은 "들고 있는" 것이 아니므로 속도 조건을 넣었다.

**실패**: `UnsafeSurfaceContact` / `Dropped` / `ObjectPushedAway` /
`ObjectOutOfBounds` / `NonFinitePhysics` / `Timeout`(20 s).

**Reset** (`OnEpisodeBegin`):

1. `RefreshGraspStage()` — 커리큘럼 lesson 반영.
2. 에피소드 상태 전부 초기화 (closure, dwell, 확정 플래그, hold, slip, 최고 높이, 모든 potential, 종료 사유).
3. 로봇: 팔 6 + 손 20관절 전부 `xDrive.target` / `jointPosition` / `jointVelocity`를 prefab 초기값으로 되돌림. 그 뒤 팔 지령 재적용 + closure 0으로 손 개방.
4. 블록: 유효 스폰을 최대 32회 재추첨 → `isKinematic = true`, `useGravity = false`, 선/각속도 0, 위치·**Y축 yaw만** 랜덤 회전(항상 똑바로 서서 시작), `Physics.SyncTransforms()`.
5. 접촉·안전 센서 전부 `ResetContacts()`.
6. 2 physics step 뒤 `ReleaseObject()` — articulation 콜라이더 transform이 직접 쓴 `jointPosition`보다 한 스텝 늦기 때문. 이때 스폰 높이와 potential 기준선을 **다시 latch**해서 정착 프레임이 보상을 새게 하지 않는다.

---

## 8. 구현 중 발견한 기존 코드 문제

1. **`GraspContactSensor`가 동작하지 않는다.** `GraspTrainingSceneBuilder`가
   센서를 `ll_dg_N_tip` GameObject에 붙이는데, URDF 임포터는 콜라이더를
   `ll_dg_N_tip/Collisions/...` 아래에 만든다. Unity는 `OnCollision*`을
   콜라이더의 GameObject로 보내므로 이 센서는 한 번도 발화하지 않는다.
   → 기존 observation 슬롯 38–42(손가락 접촉)는 **항상 0**이었다. reach
   과제는 접촉을 쓰지 않아 드러나지 않았다. 새 구현은 링크와 각 콜라이더
   양쪽에 센서를 붙이고 `contactIndex`로 묶어 OR 집계한다.
2. **패널 안전 센서가 손 링크까지 덮는다.** `ConfigureSafetySensors`가 root가
   아닌 모든 콜라이더를 계측하므로 손가락이 테이블에 닿기만 해도 −2점 종료다.
   파지 학습에서는 성립할 수 없어, 새 빌더는 `_dg_`/`ll_dg_mount` 하위를
   제외하고 **팔 링크만** 계측한다.
3. **손 xDrive forceLimit 7.5 N**은 열린 자세 유지용이라 파지력이 없다. 새
   씬은 손 관절을 stiffness 1500 / damping 120 / forceLimit 20으로 올린다.
4. **`GraspCenterMaxDistance`가 너무 느슨했다(내 구현 버그).** 7 cm였을 때, 손가락이
   테이블에 서 있는 블록에 그냥 닿아 있기만 해도 파지 계약이 충족됐다. 그래서
   `GraspConfirmed`가 57%인데 `BestLiftHeight`는 0.005 m인 모순된 상태가 나왔다.
   5 cm로 조였다.
5. **첫 블록 크기(5.5 cm)가 손의 대향 가능 범위보다 넓었다.** `GraspLiftHandGeometryProbe`로
   닫힌 손을 실측한 결과 thumb–index 최소 간격이 약 3.1–3.6 cm(closure 0.75)였다.
   5.5 cm 블록은 애초에 손가락이 마주 볼 수 없어 물리적으로 파지가 불가능했다.
   → 블록 폭을 `block_width` 환경 파라미터로 빼서 **재빌드 없이** 실측 비교할 수 있게 했다.
6. **전이 체크포인트의 탐색이 붕괴해 있다.** reach 정책은 거의 결정적
   (entropy ≈ −0.16, log_sigma ≈ −2)이고, 7번째 grip action은 학습 중 무시돼서
   출력 head가 임의값이다. 그대로 로드하면 grip 축을 사실상 탐색하지 않는다.
   → `prepare_dg5f_grasp_lift_transfer.py`가 grip mean head를 0으로 만들고
   log_sigma를 팔 −0.7 / grip 0.0으로 재설정한다. 원본 파일은 수정하지 않는다.

---

## 9. 실행

```bash
cd /home/lkb/workspace/KDT_1_AX_rtauto
source vision/.vision/bin/activate
```

### 씬 + Linux player 빌드

Unity 메뉴 **Tools > ML-Agents > Build DG5F Grasp Lift Linux Player**,
또는 배치 모드:

```bash
~/Unity/Hub/Editor/6000.4.0f1/Editor/Unity -batchmode -nographics -quit \
  -projectPath "$PWD/unity" \
  -executeMethod KDT.GraspLiftTraining.Editor.GraspLiftTrainingBuild.BuildLinuxPlayer \
  -logFile "$PWD/training/logs/grasplift_build.log"
```

### EditMode 테스트

```bash
~/Unity/Hub/Editor/6000.4.0f1/Editor/Unity -batchmode -runTests \
  -testPlatform EditMode -projectPath "$PWD/unity" \
  -testFilter "KDT.GraspLiftTraining.Tests" \
  -testResults "$PWD/training/test-results/grasplift-editmode.xml" \
  -logFile "$PWD/training/logs/grasplift_editmode.log"
```

### 학습

```bash
# pre-grasp reach 정책에서 전이 (권장)
RUN_ID=dg5f_grasp_lift_5m TIME_SCALE=20 TORCH_DEVICE=cpu \
  training/scripts/train_dg5f_grasp_lift.sh start --transfer

# 처음부터
RUN_ID=dg5f_grasp_lift_scratch training/scripts/train_dg5f_grasp_lift.sh start

# 중단된 run 재개
RUN_ID=dg5f_grasp_lift_5m training/scripts/train_dg5f_grasp_lift.sh resume
```

config: `training/config/dg5f_grasp_lift.yaml`
player: `training/builds/DG5FGraspLift/DG5FGraspLift.x86_64`

### 관찰할 지표

`GraspLift/GraspConfirmed` → `GraspLift/BestLiftHeight` → `GraspLift/Success`
순서로 올라와야 한다. `Failure/ObjectPushedAway`가 줄지 않으면 접근이
거칠다는 뜻이고, `GraspLift/FinalClosure`가 0에 머물면 grip 탐색이 다시
붕괴한 것이다.
