# DG5F GraspPoint 팔 도달 강화학습 설계

## 1. 목표와 시스템 경계

확정된 제품 파이프라인은 다음과 같다.

```text
3D 카메라가 목표 좌표 계산
  -> DG5FGraspPointReach 정책이 UR5e 팔 이동
  -> 사용자가 원격조작으로 DG5F 손 조작
```

이 저장소의 현재 강화학습 범위는 가운데 단계다. 카메라 수신·좌표 보정은 후속 입력
adapter의 책임이고, 손의 20관절 조작은 기존 `vision/dg5f` 텔레옵의 책임이다.
정책은 파지나 상승을 학습하지 않는다.

## 2. 단일 말단점과 제어

기존 20개 손가락 관절 상태/행동 대신 palm에 고정된 `GraspPoint` 하나를 작업점으로
사용한다. policy는 UR5e 6관절의 xDrive 명령만 증분 제어하며, 위치 목표에 빠르게
도달한 뒤 낮은 속도로 안정화하는 문제를 푼다. 자세 목표는 현재 범위가 아니다.

정확한 관측 26개, 행동 6개, 좌표계와 인덱스는
[`AGENT_SPEC.md`](AGENT_SPEC.md)가 유일한 계약이다.

## 3. 목표 분포

빨간 목표는 패널 상면에 닿는 static trigger다. robot-base 기준 반경
`0.20..0.85 m`, 전 방위 `0..360°`를 각각 균등 표본화해 가까운 전방 목표에
편중되지 않게 한다. 패널 밖, 로봇과 겹침, 초기 GraspPoint 거리 10 cm 미만인 표본은
버린다. 병렬 환경마다 seed를 분리해 동일 episode가 복제되지 않게 한다.

이 분포는 현재 카메라 대신 사용하는 학습 입력이다. 카메라 adapter가 추가되어도
robot-base 좌표의 target Transform이라는 환경 경계는 유지한다.

## 4. 보상 설계

- 거리 변화량은 실제 접근만 보상하고 후퇴에는 같은 비율로 벌점을 준다.
- decision당 작은 시간 비용으로 더 빠른 경로를 선호한다.
- 1 cm 이내에서 속도 0.05 m/s 이하를 0.25초 유지해야 terminal 성공을 준다.
- 성공 보너스는 남은 시간과 최종 오차를 함께 반영한다.
- timeout과 안전/비유한 물리 실패는 명시적으로 벌점 처리한다.

절대 거리의 매 step 음수 보상이나 실패 시 진행 보상 회수는 사용하지 않는다.
전자는 긴 trajectory의 scale을 불안정하게 만들고, 후자는 유효한 접근 경험까지
없애기 때문이다. 접근/후퇴 반복은 progress가 상쇄되고 시간 비용만 남는다.

## 5. 환경과 손 모델

20개 독립 학습 영역을 유지한다. 시각적으로는 DG5F 손을 남기지만 학습 환경의 손가락
ArticulationBody, collider, 텔레옵 수신기는 비활성화한다. 따라서 손 접촉이나 관절
상태가 팔 policy에 영향을 주지 않는다. 실제 원격조작용 prefab과 코드는 삭제하지 않는다.

episode reset은 팔의 실제 관절 상태와 명령 target을 일치시키고 모든 속도·reward
기억·hold timer를 초기화해야 한다. 20 simulation seconds 안에 성공하지 못하거나
workspace를 벗어나면 종료한다.

## 6. 학습과 승인

PPO는 checkpoint 없이 처음부터 단일 단계로 5M steps 학습한다. curriculum,
behavior cloning, 이전 move/grasp/lift 모델의 encoder/action head 전이는 사용하지
않는다. 512-step smoke는 communicator와 tensor shape 검증일 뿐 성능 판정이 아니다.

최종 승인은 미학습 500 seed 결정론 평가에서 성공률 90% 이상, 모든 성공의 정밀 조건
충족, 안전/비유한 실패 0건으로 한다. 자세한 실행 절차는
[`ML_AGENTS_TRAINING_GUIDE.md`](ML_AGENTS_TRAINING_GUIDE.md)를 따른다.
