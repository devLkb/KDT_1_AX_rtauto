# DG5F 빨간 공 파지 학습 실행 가이드

이 문서는 `mlagent` 브랜치의 `DG5FGrasp` 환경을 빌드하고, smoke 학습을 거쳐 본학습을 실행하는 절차를 설명한다.
모든 명령은 저장소 루트에서 실행한다.

## 1. 사전 확인

- Unity: `6000.4.0f1`
- Unity 패키지: `com.unity.ml-agents` 4.0.0
- Python: 3.10.x 가상환경 `vision/.vision`
- 학습 장치: 기본값 CPU

```bash
cd /home/lkb/workspace/KDT_1_AX_rtauto

vision/.vision/bin/pip check
vision/.vision/bin/mlagents-learn --help
```

가상환경이 없으면 README의 환경 설정 절차에 따라 먼저 설치한다.

## 2. Linux 학습 환경 빌드

### Unity Editor에서 빌드

1. `unity/` 프로젝트를 연다.
2. 메뉴 **Tools → ML-Agents → Build Linux Headless Training Environment**를 선택한다.
3. Console에서 `[GraspTrainingBuild] Built ...DG5FGrasp.x86_64`를 확인한다.

### 명령줄에서 빌드

같은 프로젝트를 Unity Editor가 열고 있으면 batchmode 빌드가 거부된다. Editor를 종료한 뒤 실행한다.

```bash
UNITY_EDITOR=/home/lkb/Unity/Hub/Editor/6000.4.0f1/Editor/Unity

"$UNITY_EDITOR" \
  -batchmode -nographics -quit \
  -projectPath "$PWD/unity" \
  -executeMethod KDT.GraspTraining.Editor.GraspTrainingBuild.BuildLinuxHeadless
```

빌드 결과:

```text
training/builds/DG5FGrasp/DG5FGrasp.x86_64
```

다음 명령으로 실행 파일 생성을 확인한다.

```bash
test -x training/builds/DG5FGrasp/DG5FGrasp.x86_64 && echo "build ready"
```

## 3. 50k smoke 학습

바로 5M 본학습을 시작하지 않는다. 임시 설정으로 50k step을 먼저 실행한다.

```bash
cp training/config/dg5f_grasp.yaml /tmp/dg5f_grasp_50k.yaml
sed -i 's/max_steps: 5000000/max_steps: 50000/' \
  /tmp/dg5f_grasp_50k.yaml

CONFIG=/tmp/dg5f_grasp_50k.yaml \
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RUN_ID=dg5f_grasp_smoke_50k \
NUM_ENVS=1 \
TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh
```

정상 연결 시 다음 로그가 출력된다.

```text
Connected to Unity environment with package version 4.0.0
Connected new brain: DG5FGrasp?team=0
DG5FGrasp. Step: ...
```

결과 위치:

```text
training/results/dg5f_grasp_smoke_50k/
```

Smoke 합격 조건:

1. observation/action 크기 오류 없이 Unity 연결
2. 50k step까지 비정상 종료 없음
3. `NaN`, physics 폭주, 반복적인 즉시 episode 종료 없음
4. ONNX와 checkpoint 생성
5. TensorBoard reward가 완전히 발산하지 않음

## 4. TensorBoard 확인

학습 터미널은 유지하고 새 터미널에서 실행한다.

```bash
vision/.vision/bin/tensorboard \
  --logdir training/results \
  --port 6006
```

브라우저에서 `http://localhost:6006`을 연다. 우선 확인할 값:

- `Environment/Cumulative Reward`
- episode length
- policy entropy
- curriculum lesson 진행 상태
- `Grasp/Success`
- `Grasp/CompletionSeconds`

## 5. 5M 본학습

50k smoke가 안정적이면 기본 PPO 설정으로 실행한다.

```bash
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RUN_ID=dg5f_grasp_v1 \
NUM_ENVS=2 \
TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh
```

처음에는 `NUM_ENVS=1`로 확인하고, CPU와 메모리 사용량이 안정적이면 `2`로 올린다. 현재 config의 `torch_settings.device`는 `cpu`다.

### 중단 후 재개

같은 `RUN_ID`와 결과 디렉터리를 사용한다.

```bash
ENV_PATH=training/builds/DG5FGrasp/DG5FGrasp.x86_64 \
RUN_ID=dg5f_grasp_v1 \
NUM_ENVS=2 \
TIME_SCALE=10 \
training/scripts/train_dg5f_grasp.sh --resume
```

`--force`는 같은 RUN_ID의 기존 결과를 덮어쓴다. 기존 학습을 보존해야 하면 사용하지 않는다.

## 6. Editor 연결 디버깅

빌드 없이 동작을 관찰할 때만 사용한다.

1. Unity에서 `Assets/MLAgents/Grasp/DG5F_GraspTraining.unity`를 연다.
2. 아래 trainer를 실행하고 대기한다.
3. Unity에서 Play를 누른다.

```bash
RUN_ID=dg5f_grasp_editor_debug \
NUM_ENVS=1 \
TIME_SCALE=1 \
training/scripts/train_dg5f_grasp.sh
```

장기 학습은 Editor가 아닌 Linux 빌드를 사용한다.

## 7. Launcher 환경변수

| 변수 | 기본값 | 용도 |
|---|---|---|
| `VENV` | `vision/.vision` | Python 가상환경 |
| `CONFIG` | `training/config/dg5f_grasp.yaml` | ML-Agents YAML |
| `RESULTS_DIR` | `training/results` | checkpoint/TensorBoard 결과 |
| `RUN_ID` | `dg5f_grasp_v1` | 학습 실행 식별자 |
| `ENV_PATH` | 비어 있음 | Linux player 경로; 비어 있으면 Editor 연결 |
| `NUM_ENVS` | `2` | 병렬 Unity 환경 수 |
| `TIME_SCALE` | `10` | Unity simulation 배속 |

추가 `mlagents-learn` 옵션은 launcher 명령 끝에 그대로 붙일 수 있다.

## 8. 자주 발생하는 문제

### `project is already open`

같은 Unity 프로젝트가 Editor에서 열려 있다. Editor 메뉴로 빌드하거나 Editor 종료 후 batchmode 빌드를 다시 실행한다.

### Trainer가 Unity 연결을 기다림

- `ENV_PATH`가 실제 실행 파일을 가리키는지 확인한다.
- 실행 권한을 확인한다: `chmod +x training/builds/DG5FGrasp/DG5FGrasp.x86_64`
- 다른 trainer가 같은 포트를 쓰는지 확인한다.
- 새 포트가 필요하면 명령 끝에 `--base-port 5006`을 추가한다.

### 기존 RUN_ID 오류

- 이어서 학습: `--resume`
- 새 실험: 다른 `RUN_ID`
- 기존 결과 폐기 후 재시작: `--force` — 결과 삭제 의도가 확실할 때만 사용

### `Failed to open plugin: ...libassimp.so`

URDF Importer의 선택적 native plugin 경고다. 현재 학습 씬은 이미 생성된 prefab을 사용하므로 communicator 연결과 step 진행이 정상이라면 파지 학습에는 영향이 없다. 다른 native plugin 오류나 Unity 종료가 함께 발생하면 별도로 조사한다.

### 학습이 지나치게 느림

1. `NUM_ENVS=1`, `TIME_SCALE=10`으로 기준 속도를 확인한다.
2. 시스템이 안정적이면 `NUM_ENVS=2`로 올린다.
3. 화면 관찰이 필요 없으면 반드시 built player와 `--no-graphics` 경로를 사용한다.

## 9. 본학습 완료 후 판정

checkpoint 생성만으로 완료 판정하지 않는다. `docs/AGENT_SPEC.md` 기준으로 고정 seed 100 episode를 평가한다.

- 성공: 80/100 이상
- median completion time: 10초 이하
- 추가 확인: 1x/10x/20x 접촉 안정성
