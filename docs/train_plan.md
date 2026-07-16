# DG5F 단계별 강화학습 계획

## 1. 목표

하나의 복합 보상으로 접근·파지·상승·유지를 동시에 학습하지 않는다. 각 단계에서 한 가지 능력을 먼저 수렴시키고, 검증된 checkpoint를 다음 단계의 초기 가중치로 사용한다.

| 단계 | 학습 목표 | 초기 가중치 |
|---|---|---|
| v1 | GraspPoint를 공 가까이 이동 | 무작위 초기화 |
| v2 | 엄지와 반대편 손가락으로 공을 동시에 접촉 | v1 |
| v3 | 파지를 유지하며 공을 들어 올림 | v2 |
| v4 | 목표 높이에서 공을 안정적으로 유지 | v3 |

## 2. 전이학습 불변 계약

v1부터 v4까지 아래 항목은 바꾸지 않는다.

- Behavior Name: `DG5FGrasp`
- observation 크기와 순서: 57개
- continuous action 크기와 순서: 7개
- PPO network 구조
- action 0..5: UR5e 6축 target 증분
- action 6: DG5F closure 증분

보상과 episode 성공 조건만 단계별로 확장한다. observation/action shape를 변경하면 이전 checkpoint를 그대로 초기화할 수 없으므로 별도 실험으로 취급한다.

## 3. 공통 실행 절차

학습 중 Unity 코드를 수정하지 않는다. 코드 변경은 domain reload를 일으켜 Python trainer 연결을 끊는다.

1. 현재 단계 학습 종료 및 checkpoint 생성
2. 고정 seed 평가
3. 승급 기준 확인
4. 다음 단계 보상·성공 조건 코드 추가
5. Unity 컴파일, EditMode/PlayMode 테스트
6. 512-step 통신 smoke
7. 새 `RUN_ID`로 이전 checkpoint 초기화

새 단계는 `--resume`이 아니라 `--initialize-from`을 사용한다.

```bash
RUN_ID=dg5f_v2 TIME_SCALE=5 \
training/scripts/train_dg5f_grasp.sh --initialize-from dg5f_v1
```

## 4. v1 — 접근

### 목표

`GraspPoint`와 공 중심 사이 거리를 5cm 이하로 만든다.

### 보상

거리 자체를 매 step 보상하지 않고 potential 차이만 사용한다.

```text
approach_potential(d) = 1 - clamp(d / maximum_distance, 0, 1)
progress_reward = current_potential - previous_potential
decision_time_penalty = -0.001
success_bonus = +1.0
```

- 가까워지면 양의 보상
- 멀어지면 같은 크기의 음의 보상
- 같은 위치에 머물러 potential 보상을 반복 획득할 수 없음
- 접촉, closure, lift, hold 보상 없음

### episode

- 성공: 거리 `<= 0.05m`
- 종료: 성공, 20초 timeout, 공 workspace 이탈, non-finite physics
- timeout과 안전 종료에는 추가 shaped reward를 넣지 않는다.
- 시작 공 위치: robot-base 기준 전 방위 `0..360°`, 면적 균등 반경 `0.35..0.70m`

### v1 승급 기준

- 최소 200회 고정 정책 평가
- unseen seed 성공률 `>= 80%`
- NaN/Inf, communicator timeout, reset 예외 0회
- 성공 시간과 최종 거리가 seed 편향 없이 안정적

## 5. v2 — 엄지 + 반대편 손가락 파지

v1 접근 보상을 작은 비중으로 유지한다. 아래 조건을 추가한다.

```text
grasp_contact = thumb_contact && any(opposing_finger_contact)
```

- 양쪽 접촉이 연속 0.5초 유지되면 성공
- 단일 손가락 접촉에는 성공 보상 없음
- 양쪽 동시 접촉에만 큰 보상
- 공을 밀어내는 정책 방지를 위해 접근 potential 유지

승급: 최소 200회 unseen seed 평가, 성공률 `>= 80%`, 접촉 유지 0.5초 충족.

## 6. v3 — 들어 올리기

v2 파지 조건을 유지하고 상승 보상을 추가한다.

```text
lift_reward = delta(ball_height), only while grasp_contact
success = grasp_contact && lift_height >= target_height
```

- 접촉 없이 공을 치거나 튕기는 행동에는 상승 보상 없음
- 목표 높이: 초기에는 2cm, 검증 후 5cm와 10cm로 확대
- 파지 상실 또는 공 낙하는 episode 종료

승급: 최소 200회 unseen seed 평가, 10cm 상승 성공률 `>= 80%`.

## 7. v4 — 안정 유지

v3 조건에 아래 조건을 추가한다.

- 상승 높이 `>= 0.10m`
- 공 속도 `<= 0.05m/s`
- 연속 유지 시간 5초
- 떨어뜨리거나 접촉을 잃으면 실패

최종 선택: 최소 500회 unseen seed 평가, 성공률 `>= 90%`.

## 8. 실행 명령

v1 새 학습:

```bash
RUN_ID=dg5f_v1 TIME_SCALE=5 \
training/scripts/train_dg5f_grasp.sh
```

Unity Editor에서 관찰할 때는 `ENV_PATH`를 지정하지 않고 trainer 실행 후 Play를 누른다.

다음 단계 초기화:

```bash
RUN_ID=dg5f_v2 TIME_SCALE=5 \
training/scripts/train_dg5f_grasp.sh --initialize-from dg5f_v1
```

## 9. 알려진 선행 조건

기존 구현은 episode reset에서 현재 로봇 collider pose를 기준으로 공 위치를 재검사했다. articulation reset 직후 collider가 이전 pose를 유지하는 프레임에서 모든 후보가 거부되어 Agent가 멈출 수 있었다. v1 reset은 초기 자세에 맞춰 검증한 제한된 spawn 영역을 직접 사용하고, stale collider 기반 무한 재표본화를 사용하지 않아야 한다.
