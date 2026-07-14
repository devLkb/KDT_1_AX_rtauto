# KDT_1_AX_rtauto — UR5e + Tesollo DG5F 디지털 트윈 / 웹캠 텔레옵

웹캠 MediaPipe 핸드트래킹으로 Unity 안의 Tesollo DG5F 로봇핸드(+UR5e 팔)를 실시간 조작하는
디지털 트윈 프로젝트. 이 리포만 클론하면 새 PC에서 동일하게 이어서 작업할 수 있도록
필요한 것만 담았다 (개발 이력·의사결정은 `docs/WORKLOG.md`).

## 리포 구조

| 폴더 | 내용 |
|---|---|
| `unity/` | Unity 프로젝트 (Assets + Packages + ProjectSettings — Library는 열 때 자동 생성) |
| `vision/dg5f/` | **DG5F 텔레옵 파이프라인**: 보정→웹캠 트래킹→UDP 송신 + 검증/분석 도구 |
| `tools/urdf_hand_import/` | URDF→Unity 임포트/물리검증/구동준비/프로브 범용 스크립트 |
| `urdf/dg5f/` | Tesollo DG5F URDF+메시 원본 4변형 (검증 스크립트의 대조 기준) |
| `urdf/ur5e_svh_build/` | UR5e xacro 변환 + 핸드 결합 스크립트 (SVH용, DG5F 결합 시 개조) |
| `docs/` | WORKLOG(작업 이력 전체), ML-Agents 로드맵, 진동 디버깅 기록 |

## 새 환경 셋업

### 1. Unity
- **Unity 6000.4.0f1** (다른 버전은 ArticulationBody 물리 재검증 필요)
- Unity Hub → Open → `unity/` 폴더 선택. 첫 오픈 시 Library 생성으로 수 분 소요.
- 렌더 파이프라인: Built-in (URP 아님 — 머티리얼 마젠타면 확인)
- 씬: `Assets/Scenes/DG5F_Import.unity` (메인 — DG5F 왼손 인스턴스 배치됨)
  ※ SVH 관련 코드·씬(SampleScene, unity_pkg)은 DG5F 전환에 따라 제거됨(2026-07-13).
  UR5e 팔 결합 시 `urdf/ur5e_svh_build/`의 변환 스크립트와 `Assets/Scripts/ArmTargetIK.cs`
  (팔 IK, WORKLOG §15·§18)를 재사용해 새로 구성.
- 프리팹: `Assets/Robots/Prefabs/dg5f_*.prefab` 4변형 — 구동 준비(게인/중력off/자기충돌무시/
  수신기/IK/로거) 완료 상태. 씬에 끌어놓으면 됨. 변형 교체는 메뉴 **Tools/DG5F**.

### 2. Python — **3.10.12 권장, 비전+ML-Agents 공용 가상환경 1개**

버전 선택 근거(2026-07-14 `vision/.vision`에서 검증):
- **ML-Agents(mlagents)는 Python 3.10.x 전용** → 3.10.12 기준으로 통일
- 기존 판단은 `mediapipe 0.10.14`(protobuf 4.x 계열) 때문에 ML-Agents(protobuf 3.x)와
  가상환경을 분리해야 한다는 것이었으나, **`mediapipe==0.10.11`로 낮추면 `protobuf==3.20.3`에서
  동작**해 ML-Agents와 같은 venv에 공존 가능하다.
- 현재 검증된 핵심 버전: `mediapipe==0.10.11`, `protobuf==3.20.3`, `numpy==1.23.5`,
  `opencv-contrib-python==4.8.1.78`, `torch==2.1.1+cpu`, `mlagents/mlagents_envs==1.2.0.dev0`.

```bash
# 비전(텔레옵) + ML-Agents 공용 venv (리포 현재 검증 경로)
python3.10 -m venv vision/.vision
source vision/.vision/bin/activate      # Windows: vision\.vision\Scripts\activate
pip install -r requirements-mlagents.txt
pip install -r requirements-vision.txt

# 전체 고정 버전 재현/감사가 필요할 때
pip install -r vision/requirements-vision-mlagents.resolved.txt
```

⚠️ `mlagents/mlagents_envs==1.2.0.dev0`은 현재 로컬 `release23` 소스 설치본이다. 새 PC에서는 Unity
`com.unity.ml-agents` 패키지와 Python `mlagents` 소스/휠 버전 짝을 맞춘 뒤 `mlagents-learn --help`를
반드시 확인한다(`docs/ML_AGENTS_ROADMAP.md` §3-1). 텔레옵 스크립트(`vision/`)도 같은 공용 venv에서 실행한다.

### 3. (선택) unity-cli — 에디터를 CLI로 제어 (임포트/프로브 자동화에 사용)
- https://github.com/akiojin/unity-cli 설치 후 Unity 프로젝트에 커넥터 패키지 추가.
- 없어도 텔레옵 자체는 동작 (Play는 에디터에서 직접).

### 4. 경로 상수 (새 PC에서 1회 수정)
스크립트들의 기본 경로가 개발 PC 기준이라 아래 중 하나로 맞출 것:
- `tools/urdf_hand_import/import_hand.py` 상단 `DEFAULT_PROJECT`(Unity 프로젝트 경로),
  `DEFAULT_CLI`(unity-cli 경로) 수정 — probe_test/phys_compare/setup_drive가 이걸 씀
- `vision/dg5f/analyze_teleop.py`는 환경변수 `DG5F_UNITY_LOGS`, `DG5F_URDF_DIR` 또는
  `--logs-dir/--urdf-dir` 인자로 대체 가능

## 텔레옵 실행 (왼손 모델 기준)

```bash
cd vision/dg5f
python calibrate_dg5f.py          # 최초 1회 보정 — CALIBRATION_GUIDE.md의 동작 수행
# Unity에서 Play ▶ (씬에 dg5f_left 인스턴스)
python vision_node_dg5f.py left   # 오른손 모델이면 인자 생략
```
- 프로토콜/채널 순서/좌표계 계약은 `vision/dg5f/README.md` 참고 (v2: 관절각 20 + 엄지끝 위치 + 핀치)
- 웹캠 없이 배선 검증: `python probe_sender.py fist left` / `ok left` / `oktip left`
- 추종 정량 분석: `python analyze_teleop.py latest latest --hand left`
  (Unity 쪽은 Dg5fJointLogger가 Play마다 자동 기록)

## 새 핸드 URDF 임포트 (범용 파이프라인)

```bash
cd tools/urdf_hand_import
python import_hand.py <hand.urdf> --prefab --verify   # 복사→패치→임포트→물리 전수대조→프리팹
python setup_drive.py <이름>                           # 구동 준비 일괄 (Controller 제거/게인/중력 등)
python probe_test.py <이름> --urdf <hand.urdf>         # 전 관절 사각파 구동 검증
```
자세한 절차·함정 목록은 `tools/urdf_hand_import/README.md`.

## 현재 상태 / 알려진 이슈 (2026-07-13)

- ✅ DG5F 4변형 임포트·물리검증·구동검증 완료, 굽힘 텔레옵 전 채널 PASS(상관 1.00)
- ✅ 엄지 손끝 위치 리타게팅 v2 + 핀치 스냅 (OK 사인 접촉 프로브 검증 완료)
- ⚠️ **엄지 라이브 움직임이 부드럽지 않음** — 진행 중. 후보: 데드밴드 동결/재가동 경계,
  CCD 스텝 제한, 비전 깊이 노이즈. `docs/WORKLOG.md` §20-3 미해결 항목 참고.
- ⬜ 벌림(n_1)·새끼접기(5_1) 채널 게이트 해제, UR5e+DG5F 결합, ML-Agents (로드맵 문서)
