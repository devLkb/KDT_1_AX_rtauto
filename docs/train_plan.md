# DG5F joint26 단계별 학습 계획

## 고정 정책 계약

V2부터 V4까지 `SpecVersion=2.1.0`, Behavior Name
`DG5FGraspJoint`, observation 116개, continuous action 26개를 유지한다.

### Observation

| 인덱스 | 내용 |
|---|---|
| 0..5 | 팔 6축 각도 |
| 6..11 | 팔 6축 속도 |
| 12..31 | 손 20관절 각도 |
| 32..51 | 손 20관절 속도 |
| 52..71 | 손 20관절 명령 target |
| 72..80 | 공 위치, 선속도, 각속도 |
| 81 | 공 수직 변위 |
| 82..96 | 손끝 5개의 공 상대 위치 |
| 97..101 | 손가락별 접촉 |
| 102..107 | 팔 6축 명령 target |
| 108..111 | V1..V4 목표 one-hot |
| 112..115 | 접근, 최적 거리, 임계값, 경과 시간 |

### Action

- `0..5`: 팔 관절 target 각도 증분
- `6..25`: 엄지부터 새끼손가락까지 `5 x 4` 손 관절 target 각도 증분
- 손 target은 decision마다 최대 `±4°`로 누적하고 각 관절 xDrive limit로
  clamp한다.
- closure action과 `OPEN -> FIST` 보간은 사용하지 않는다.

reset은 모든 팔/손 관절의 position, velocity, xDrive target, Agent 내부
target 배열을 같은 값으로 동기화한다.

## V1 bootstrap

`dg5f_v1_gpu_fixed`의 526647-step checkpoint는 동결한다.
`training/scripts/bootstrap_v1_to_joint26.py`가 다음과 같이 별도
`dg5f_v1_joint26_bootstrap`을 만든다.

- V1 observation `0..11` -> joint26 `0..11`
- V1 observation `13..56` -> joint26 `72..115`
- V1 closure observation/action은 폐기
- hidden encoder와 critic/value head는 정확히 복사
- arm action 6개와 log-sigma는 정확히 복사
- 새 hand observation/action은 작은 난수와 보수적 log-sigma로 초기화
- optimizer moment는 폐기하고 global step은 0

변환기는 tensor 이름과 shape 및 복사 결과를 모두 검증하고 불일치 시
checkpoint를 만들지 않는다.

## V2 reward와 curriculum

- decision 비용 `-0.001`
- 접근 potential delta `0.25배`
- 엄지 단독 접촉 potential `0.25`
- opposing-only `0`
- 엄지+opposing 동시 접촉 potential `0.5`
- 연속 접촉 유지 potential `0.5 * clamp(t / 0.5초)`
- 0.5초 성공 종료 `+2.0`
- 공 이탈/비유한 물리 `-1.0`
- 실패 종료 시 남은 접근/접촉/유지 potential을 0으로 정산
- 접촉 손실 시 hold timer만 즉시 0으로 만들고 episode는 계속

`joint26_stage` ML-Agents curriculum:

1. grasp point 근처 공, 35% 고정 pre-grasp reset, 제한된 팔 범위와 작은
   손 증분
2. V1 `0.35..0.70m` 공 배치와 정상 팔 제어, 35% pre-grasp reset
3. 완전히 편 손 reset과 20관절 독립 제어

승인 평가는 stage 3에서만 수행한다.

## 실행과 승인

```bash
dg5f v2 init
dg5f v2 status
dg5f v2 resume
dg5f v3 init
```

- V2 표준 run: `dg5f_v2_joint26_gpu_fixed`
- 실패 closure 보존 run: `dg5f_v2_closure_failed_343k`
- 100k pilot: `Reach/Success >= 80%`, 접촉률 증가, 오류 0
- pilot 실패 시 V1 bootstrap에서 learning rate `5e-5`의 새 run 시작
- 최종 unseen-seed 200회: `Grasp/Success >= 80%`,
  `Reach/Success >= 80%`, 모든 성공 hold `>= 0.5초`, seed/물리/reset/통신
  오류 0
