# dg5f_teleop.exe — 빌드와 배포

`dg5f_teleop_gui.py`를 단일 exe로 묶어, 파이썬·mediapipe·SDK 설치 없이 창 하나로
보정 → Unity 시뮬 → 실물 DG-5F 구동까지 하게 만든 것.

---

## 1. 받는 사람 (테스트만 할 때)

### 다운로드

exe는 **저장소에 없다.** onefile로 132MB라 GitHub의 100MB 파일 제한에 걸려 push가
거부된다. 배포는 **Releases**로 한다:

```
https://github.com/devLkb/KDT_1_AX_rtauto/releases
```

→ 최신 릴리스의 `dg5f_teleop.exe` 하나만 받으면 된다. 설치 과정은 없다(더블클릭 실행).

### 실행 환경

| 항목 | 요구 |
|---|---|
| OS | **Windows 10/11 x64** (DGSDK.dll이 Windows 전용) |
| 웹캠 | 아무거나. 640x480@30 이상 |
| 파이썬 | **불필요** — 인터프리터·mediapipe·OpenCV 전부 exe 안에 있다 |
| Tesollo SDK | **불필요** — DGSDK.dll을 exe에 동봉했다 |

첫 실행은 **30초쯤** 걸린다. onefile이라 132MB를 임시폴더에 푸는 시간이다(두 번째부터도
같다 — 매번 푼다). 백신이 처음 한 번 검사하느라 더 걸릴 수 있다.

### 사용자 데이터가 쌓이는 곳

```
%LOCALAPPDATA%\dg5f\
    dg5f_calibration.json     ⑥ 내 손 보정
    dg5f_joint_map.json       ⑨ 관절 대응표
    dg5f_gui_preset.json      프리셋
    logs\                     ⑤ CSV 로그
```

exe를 지워도 남는다. 완전히 지우려면 이 폴더를 지우면 된다.
(보정 파일이 없으면 exe에 동봉된 기본 보정값으로 돈다 — 남의 손 기준이니 ⑥으로 새로 잴 것.)

### 테스트 순서

1. **보정** 모드 → `● 보정 녹화 시작` → 안내대로 ①~④ 동작 반복(30초~1분) → `완료·저장`
2. **시뮬** 모드 → ① Sim 체크(기본 127.0.0.1:5006) → Unity 씬 실행 → 손 따라 움직이는지
   확인. 다른 PC의 Unity로 쏘려면 그 PC의 LAN IP를 넣고 방화벽에서 UDP를 열 것.
3. **관절맵** 모드 → 실물 첫 연결이면 **여기부터**. ⑦에서 IP 넣고 [연결] →
   Arm → `🔍 관절 탐색 마법사`로 슬롯 20개 확인 → [적용] → [저장]
4. **실물** 모드 → 손 추종 구동

⚠️ 3번을 건너뛰고 실물을 돌리지 말 것. 관절 대응표 기본값은 **추정치**(항등 매핑, 부호
전부 +1)라 엉뚱한 관절이 움직일 수 있다.

### 안전

- **Esc = 비상정지** (단, 이 창이 활성일 때만 — Unity를 보고 있으면 안 먹는다).
  툴바의 `■ 정지` 버튼은 어느 모드에서나 보인다.
- 연결과 구동이 분리돼 있다. [연결]만으로는 명령이 한 개도 안 나간다. Arm을 켜야 움직인다.
- 연결 시점에 실물이 **rest(손 벌린 자세)** 여야 한다. 슬루 리밋의 기준점이 전 관절 0°다.
- 모듈 에러·연결 끊김이 이어지면 자동으로 Arm이 풀린다(⑦ 체크박스).

---

## 2. 만드는 사람 (빌드)

```bash
# vision venv 에서
python -m pip install pyinstaller
cd vision/dg5f
python -m PyInstaller dg5f_teleop.spec --noconfirm
# → dist/dg5f_teleop.exe
```

`dg5f_teleop.spec`이 동봉하는 것:

| 자산 | 왜 |
|---|---|
| `DGSDK.dll` | 실물 구동. 동반 DLL 없이 단독 로드되는 것을 확인하고 넣었다(2026-07-31) |
| `dg5f_calibration.json` | 기본 보정값. 사용자가 ⑥으로 재기 전까지 쓰는 폴백 |
| mediapipe 데이터 일체 | `collect_all`. `hand_landmark_full.tflite` 등은 PyInstaller가 **자동으로 못 찾는다** — 빠뜨리면 창은 뜨고 카메라도 열리는데 손만 안 잡히는, 원인 찾기 어려운 형태로 실패한다 |

⚠️ spec은 **CWD 기준**으로 `../태슬로sdk/...`에서 DLL을 찾는다. 반드시 `vision/dg5f`에서
실행할 것. DLL이 없으면 경고만 내고 계속 빌드되며, 그렇게 나온 exe는 ⑦에서 [찾기]로
DLL 경로를 직접 잡아 줘야 한다.

### 빌드 검증 (권장)

얼린 상태에서만 드러나는 문제가 있다. `_frozen_selftest.py`를 같은 방식으로 한 번 더
빌드해 돌리면 7개 항목을 점검한다 — 쓰기 폴더 분리, 동봉 자산, 보정 폴백, DLL 로드,
mediapipe 그래프 생성, 관절 대응표 저장/재읽기, 로그 경로.

```bash
sed -e 's/"dg5f_teleop_gui.py"/"_frozen_selftest.py"/' \
    -e 's/name="dg5f_teleop"/name="dg5f_selftest"/' \
    -e 's/console=False/console=True/' dg5f_teleop.spec > _selftest.spec
python -m PyInstaller _selftest.spec --noconfirm
dist/dg5f_selftest.exe
```

실제로 이 자가진단이 잡아낸 것: 한글 윈도우 콘솔(cp949)에서 임포트 단계 `UnicodeEncodeError`
크래시. exe만의 문제가 아니라 cmd.exe에서 소스를 돌려도 같았다(지금은 `dg5f_paths`가
stdout을 UTF-8로 고정해 막는다).

### 배포 (Releases)

```bash
gh release create v0.1.0-dg5f dist/dg5f_teleop.exe \
   --repo devLkb/KDT_1_AX_rtauto --target rhand \
   --title "DG5F 텔레옵 GUI" --notes "..."
```

Releases는 파일당 2GB까지 허용된다. **exe를 `git add` 하지 말 것** — 100MB 제한에 걸려
push가 통째로 거부되고, 한 번 커밋하면 히스토리에서 지우기 번거롭다(`.gitignore`에
`dist/`, `build/`가 들어 있다).

### 크기·기동 시간을 줄이려면

onefile 132MB / 기동 30초는 매번 압축을 푸는 탓이다. `EXE(...)`를 `COLLECT(...)`로 바꿔
**onedir**로 만들면 기동이 몇 초로 줄어든다. 대신 폴더째 배포해야 한다(zip으로 묶어
Releases에 올리면 된다).

---

## 3. 아직 실물 미검증인 것

전부 헤더 대조 + 가짜 그리퍼로만 확인했다. 첫 실물 연결 때 볼 것:

- **관절 대응표** — 기본값이 추정치다. ⑨ 탐색 마법사로 확정할 것 (이게 제일 중요)
- 끊김 콜백(`CallbackForOnDisconnected`)이 실제로 불리는가
  — 안 불릴 때를 대비해 "상태 수신 10회 연속 실패" 백업 감지를 같이 두었다
- 자가진단 결과 필드의 의미 (SDK 문서에 없어 값만 그대로 표시 중)
- 피드백 4종(JOINT/CURRENT/TEMPERATURE/MODULE_ERROR) 요청이 50Hz 서보 주기에 주는 영향
  — ⑦의 `제어주기`와 실측 Hz를 보고 조정
- 오른손. 매핑·보정은 **왼손 기준**으로 맞춰 왔고 오른손은 일부만 확인했다
