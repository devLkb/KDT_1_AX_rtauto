# DG5FGraspPointReach 단일 학습 계획

## 고정 정책 계약

V2는 `SpecVersion=2.1.0`, Behavior Name `DG5FGraspJoint`를 사용한다.
외부 패키지 V3부터 Behavior Name은 `DG5FStableGrasp`로 바뀐다.
두 환경 모두 observation 116개, continuous action 26개로 policy tensor
shape는 유지한다. Behavior 이름 변경은
[`AGENT_SPEC_V3.md`](AGENT_SPEC_V3.md)의 checkpoint bridge로 처리한다.

세부 observation/action 순서와 reward 수식은
[`AGENT_SPEC.md`](AGENT_SPEC.md)를 바꾸지 않고 따른다.

## 학습 순서

1. EditMode/PlayMode 계약 테스트와 100회 reset 안정성 검증
2. Linux built player로 정확히 512 agent-step communicator smoke
3. checkpoint 없이 fresh PPO run 시작
4. 최대 5M steps 학습하며 TensorBoard와 안전 실패 감시
5. 학습과 겹치지 않는 500개 고정 seed 결정론 평가

기본 PPO:

| 항목 | 값 |
|---|---:|
| batch size | 256 |
| buffer size | 2048 |
| learning rate | `3e-4` |
| hidden units / layers | 256 / 3 |
| gamma | `0.99` |
| max steps | 5,000,000 |
| trainer normalization | false |

## 승인

- 평가 성공률 `>= 90%`
- 성공 행 전부 1 cm / 0.05 m/s / 0.25초 조건 충족
- 중복 seed, 비유한 물리, workspace 안전 실패 0건
- 통과 모델 중 평균 최종 오차 최소, 동률이면 median/p95 완료 시간 최소

512-step smoke나 training reward 상승은 승인 근거가 아니다.

## 폐기된 경로

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

1. grasp point 반경 4cm 공, 35% 고정 pre-grasp reset, 팔 target 고정,
   최대 1도 손 증분, 접근 보상 0, 5초 timeout
2. V1 `0.35..0.70m` 공 배치와 정상 팔 제어, 35% pre-grasp reset,
   20초 timeout 및 도달 후 5초 파지 timeout
3. 완전히 편 손 reset과 20관절 독립 제어, 20초 timeout 및 도달 후
   5초 파지 timeout

stage 1은 최근 200 episode 평균 reward가 `2.2`를 넘으면 stage 2로,
stage 2는 `1.8`을 넘으면 stage 3으로 전환한다. reward signal smoothing은
끄며 lesson 전환 시 optimizer는 재시작하지 않는다.

승인 평가는 stage 3에서만 수행한다.

## V3 stable whole-hand 환경

V3 패키지는 V2 player를 덮어쓰지 않고
`training/builds/DG5FGraspV3`에 설치한다.

- Behavior: `DG5FStableGrasp`
- curriculum parameter: `stable_grasp_stage`
- stage 1: 엄지 + 비엄지 2개 이상의 접촉을 0.5초 유지
- stage 2: 안정 파지 후 2 cm 들어 0.5초 유지
- stage 3: 5 cm 들어 최소 4 cm 높이에서 1초 유지

현재 운영 명령 `dg5f v2 init`은 V1 joint26 bootstrap의
`DG5FGraspJoint` checkpoint를 같은 tensor shape의 `DG5FStableGrasp`
초기화 checkpoint로 복사한 read-only bridge를 만든다. 이후
`dg5f v3 init`은 이 StableGrasp run을 같은 behavior로 이어받는다.
세부 계약은 [`AGENT_SPEC_V3.md`](AGENT_SPEC_V3.md)를 따른다.

## 실행과 승인

```bash
dg5f v2 init
dg5f v2 status
dg5f v2 resume
dg5f v3 init
```

- 현재 `dg5f v2` 표준 run:
  `dg5f_v2_stablegrasp_v3_lr5e5_gpu_fixed` (`learning_rate: 5e-5`).
  StableGrasp 3.0.0 player와 V1 joint26 bootstrap을 사용해 step 0부터
  학습한다.
- 중단된 이전 839,959-step run:
  `dg5f_v2_joint26_handfirst3_lr5e5_gpu_fixed` (조회/보존 전용)
- 실패 joint26 5e-5 pilot 보존 run:
  `dg5f_v2_joint26_lr5e5_gpu_fixed`
- 실패 joint26 1e-4 pilot 보존 run: `dg5f_v2_joint26_gpu_fixed`
- 실패 closure 보존 run: `dg5f_v2_closure_failed_343k`
- 100k pilot: stage 2 전환 완료, 전환 직전 20k에서
  `Reach >= 80%`, `Grasp >= 80%`, `DualContact >= 85%`,
  `Timeout <= 15%`, 오류 0. 100k까지 stage 1이면 즉시 중단한다.
- stage 2: 500k 전에 stage 3 전환, 전환 직전
  `Reach >= 80%`, `Grasp >= 70%`, `DualContact >= 75%`,
  `Timeout <= 25%`
- 최초 1e-4 pilot은 step 253740에서 중단했고, 같은 구조에서 학습률만
  내린 5e-5 pilot도 step 215202에서 중단했다. hand-first run은 V1
  bootstrap에서 optimizer 없이 step 0부터 시작한다.
- 실패 원인과 지표 근거:
  [`V2_TRAINING_FAILURE_ANALYSIS_20260717.md`](V2_TRAINING_FAILURE_ANALYSIS_20260717.md)
- 최종 unseen-seed 200회: `Grasp/Success >= 80%`,
  `Reach/Success >= 80%`, 모든 성공 hold `>= 0.5초`, seed/물리/reset/통신
  오류 0
