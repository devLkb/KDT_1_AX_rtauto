# DG5FGraspPointReach 단일 학습 계획

## 고정 정책 계약

- Behavior `DG5FGraspPointReach`, spec `1.0.0`
- observation 26개, continuous action 6개
- UR5e 팔만 제어하고 DG5F 손가락 20관절은 정책에서 제외
- `GraspPoint` 위치 도달만 학습하며 파지, 상승, 자세 제어는 제외
- 빨간 목표의 패널 평면 반경 0.20~0.85 m와 전 방향 360°를 각각 균등 생성
- 성공은 1 cm 이내와 0.05 m/s 이하를 0.25초 연속 유지
- episode는 최대 20 simulation seconds

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

move→grasp→lift 단계, curriculum, v1/v2/v3/v4 승격, arm encoder나 손 20 action의
checkpoint 전이는 사용하지 않는다. 기존 결과물은 역사적 실험 산출물일 뿐 새 Behavior와
호환되는 초기 모델로 취급하지 않는다.
