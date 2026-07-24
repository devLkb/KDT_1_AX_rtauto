# 문서 인덱스

## DG5FGraspReadyReach 강화학습

- **처음 읽을 문서** — Unity·강화학습 용어부터 전체 흐름까지:
  [`ML_AGENTS_LEARNING_FLOW.md`](ML_AGENTS_LEARNING_FLOW.md)
- 활성 Agent 2.0.0 계약: [`AGENT_SPEC.md`](AGENT_SPEC.md)
- 설계와 근거: [`ML_AGENTS_DESIGN.md`](ML_AGENTS_DESIGN.md)
- 빌드·학습 실행: [`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)
- 현재 구현 상태와 남은 작업:
  [`DG5F_GRASP_READY_REACH_HANDOFF.md`](DG5F_GRASP_READY_REACH_HANDOFF.md)

읽는 순서:

1. [강화학습 입문과 전체 흐름](ML_AGENTS_LEARNING_FLOW.md)
2. [Agent 2.0.0 계약](AGENT_SPEC.md)
3. [환경·보상 설계](ML_AGENTS_DESIGN.md)
4. [구현 및 검증 로드맵](ML_AGENTS_ROADMAP.md)
5. [빌드·학습·평가 실행 가이드](ML_AGENTS_TRAINING_GUIDE.md)
6. [고정 학습 계획](train_plan.md)

제품 파이프라인은 `목표 좌표 -> RL 열린 손 배치 및 팔 잠금 -> MediaPipe 손 파지`다.
MediaPipe와 카메라 수신은 후속 integration이며 현재 RL 환경에는 포함하지 않는다.

## DG5F 비전 텔레옵

- 시작점: [`../vision/dg5f/README.md`](../vision/dg5f/README.md)
- 보정: [`../vision/dg5f/CALIBRATION_GUIDE.md`](../vision/dg5f/CALIBRATION_GUIDE.md)
- 역할: 강화학습이 목표에 도달한 뒤 DG5F 손 20관절을 조작

텔레옵의 20관절 프로토콜은 유지되지만 강화학습 observation/action과는 독립이다.

## 이력과 진단

- [`WORKLOG.md`](WORKLOG.md): 프로젝트의 누적 작업 기록과 의사결정
- [`DEBUG_OSCILLATION_20260707.md`](DEBUG_OSCILLATION_20260707.md): 초기 진동 원인 분석
- [`DEBUG_OSCILLATION_20260708.md`](DEBUG_OSCILLATION_20260708.md): 관성 수정과 최종 검증

WORKLOG의 예전 단계형 파지 기록은 당시 이력이며 현재 실행 지침이 아니다. 활성 정책
계약은 항상 `AGENT_SPEC.md`를 우선한다.
