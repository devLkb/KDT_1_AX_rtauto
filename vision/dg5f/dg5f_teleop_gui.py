# -*- coding: utf-8 -*-
"""DG5F 텔레오퍼레이션 GUI — 웹캠+MediaPipe로 손을 20관절 각도[deg]로 만들어 UDP로 쏘되,
송신 대상(IP/포트)·모드·보정값(사람 범위)·관절 제한(로봇 범위)·관절 각도(수동)를
**실행 중에 UI로 바꿔** 볼 수 있는 컨트롤 패널.  (headless 버전=vision_node_dg5f.py)

이 파일이 exe 타깃(1단계). 계산 로직은 dg5f_angles를 그대로 재사용 —
  raw = compute_raw(landmarks)            # 사람 관절 프록시(rad)
  mapped = map_to_dg5f(raw, hand, mode)   # 로봇 관절각(deg)  ← UDP로 나가는 값
UI에서 바꾼 값은 dg5f_angles의 모듈 전역(DG5F_CHANNELS / RATIO_LIMIT / 엄지 상수)에
**라이브로 반영**된다(map_to_dg5f가 호출 시점에 그 전역을 읽으므로 재시작 불필요).

패킷은 vision_node와 동일한 v6 <72f>:
  [0..19] 관절각[deg] / [20..22] 엄지 tip / [23] 핀치 / [24] 끝거리비
  / [25..36] 손가락 리치 / [37..51] 손목→끝 / [52..71] 라디안 원값(디버그)
→ Unity Dg5fReceiver(sim)와 dg5f_sdk_bridge(real) 둘 다 그대로 받는다.

────────────────── 창 하나로 실물까지 (2026-07-31, ⑥⑦⑧ 추가) ──────────────────
예전엔 실물을 만지려면 프로세스가 셋이었다: calibrate_dg5f.py(보정) → 이 GUI(UDP) →
dg5f_sdk_bridge.py(:5007 → DGSDK.dll → 실물). 다른 사람에게 넘겼을 때 "내 손에 맞춘
보정값"과 "따로 띄워야 하는 브리지"가 그대로 걸림돌이라 셋을 창 안으로 들여왔다:
  ⑥ 보정 녹화  — calibrate_dg5f.build_calibration/save_calibration을 그대로 호출.
                 산출물은 스크립트로 보정한 것과 같은 파일·같은 규칙(dg5f_calibration.json).
  ⑦ 실물 직결  — dg5f_sdk_bridge의 Dg5fSdk/to_sdk_frame을 임포트해 전용 서보 스레드에서
                 구동. 연결/Arm 분리 · 슬루 리밋 · 클램프 · 비상정지.
  ⑧ 관절 검증  — 브리지의 --pose 대체. 전 채널 rest + 선택 1관절만.
⚠️ 세 기능 모두 **로직을 복사하지 않았다.** 계산·스키마·관절 대응표의 소유자는 여전히
   calibrate_dg5f / dg5f_angles / dg5f_sdk_bridge다. 사본을 만들면 "GUI로 한 검증"과
   "스크립트로 한 검증"이 갈라진다(07-27 MP_MODEL_COMPLEXITY 사고와 같은 구조).
   기존 경로(① Real 체크 → UDP :5007 → 별도 브리지 프로세스)도 그대로 살아 있다 —
   리눅스/맥은 DGSDK.dll을 못 쓰므로 그쪽을 써야 한다.

────────────────────────── 스레드 구조 (2026-07-27 성능 개편) ──────────────────────────
예전엔 tick() 하나가 Tk 메인 스레드에서 cap.read()(33ms 블로킹) + MediaPipe(10ms) +
PhotoImage 생성(5~13ms)을 다 하고 그 위에 after(20)을 더 얹었다. 그동안 Tk 이벤트 루프가
멈춰서 슬라이더를 드래그하면 콜백이 프레임 뒤에 줄줄이 큐잉됐다(체감 19.5fps).
지금은 3분할 — 각 단계는 '최신 1개만 유지하는' 슬롯으로 연결되고, 밀린 프레임은 버린다:

  [캡처 스레드]  cap.read() 반복        → frame_slot  ─┐
  [처리 스레드]  MediaPipe·매핑·필터·UDP송신          ─┤ 둘 다 UI를 막지 않음
  [메인(Tk)]     결과를 PhotoImage.paste + 판독 갱신  ←┘  프레임당 ~2ms

핵심 규칙 3개:
  1. tk 변수(StringVar 등)는 **메인 스레드만** 만진다(Tcl 인터프리터가 스레드 안전하지 않음).
     워커는 _sync_settings()가 만들어 원자적으로 갈아끼우는 불변 _Settings 스냅샷만 읽는다.
  2. cv2 / mediapipe 임포트는 워커 스레드에서 한다(합쳐 ~4.5초. 최상단에서 하면
     그만큼 창이 안 뜬다). 준비되면 모듈 전역 cv2 / mp 에 채워진다.
  3. cap.set() 은 **쓰지 않는다** — 실측 근거는 _capture_loop 주석 참조.
  4. 송신 경로(sendto)에는 **점4자리 IP만** 넘긴다. 검증은 _sync_settings에서 inet_pton으로
     끝내둔다 — 그러지 않으면 IP를 타이핑하는 중간 문자열('1','19','192','192.')이 전부
     DNS 조회로 들어가 한 번 입력에 10.8초를 멈춘다(실측 근거는 _sync_settings 주석).

────────────────────── 레이아웃/모드 (2026-07-30, 07-31 툴바) ──────────────────────
  [─── 툴바(고정): 할 일 [보정|시뮬|실물|전체] · 표시등 · 비상정지 ───]
  [영상(고정)] [그 모드에 필요한 패널만 (Canvas 안, 세로·가로 스크롤)] [수직바]
                                   [수평바]
  [────────────────── 상태바(고정) ──────────────────]
패널을 다 펼치면 세로 1560px라 어떤 노트북에도 안 들어간다 → 툴바에서 할 일을 고르면
그 일에 필요한 패널만 남고 창 크기도 거기 맞춰 다시 잡힌다(MODES / _apply_mode).
⚠️ **숨김은 끄기가 아니다.** 숨은 패널의 설정(송신 대상·Arm·검증 모드)은 계속 살아 있다.
   그래서 '지금 무엇이 나가고 있는가'는 항상 보이는 툴바 표시등과 상태바가 책임진다.
   비상정지도 툴바에 있다 — 실물이 붙어 있는 한 어느 모드에서나 누를 수 있어야 한다.
스크롤은 **오른쪽 패널에만** 건다. 전체를 한 Canvas에 넣으면 아래 패널을 보러 내려간 순간
미리보기가 화면 밖으로 나가서 '손을 보며 값을 맞추는' 일 자체가 불가능해진다.
영상 열은 VIDEO_COL_W로 자리를 예약한다 — 창 크기를 잡는 시점엔 영상 라벨이 아직
안내문 크기(≈107px)라, 예약이 없으면 첫 프레임에 라벨이 496px로 커지면서 패널을
창 밖으로 밀어낸다(그 상태가 "오른쪽 UI가 잘린다" = 07-30에 실제로 고친 증상).

────────────────────────── 로그 (2026-07-28 추가) ──────────────────────────
⑤ 체크박스를 켜면 logs/teleop_<초단위>.csv 에 **한 프레임 = 한 행**으로 파이프라인 4개 층을
전부 남긴다(랜드마크 → 사람각 → 로봇각 → UDP 송신값). 상세는 _TeleopLogger 참조.

실행:  <vision venv python> dg5f_teleop_gui.py
"""
import os
import sys

# ── 리눅스(X11) 스레드 락: 어떤 X 연결보다 **먼저** 켜야 한다 ────────────────────────
# X11 Xlib은 여러 스레드가 같은 Display를 쓰면 시퀀스 번호가 깨진다. 이 GUI는 메인 스레드가
# Tk를, 워커가 cv2/mediapipe를 돌리는데 그 라이브러리들이 X/Qt를 물고 오면 충돌한다.
# XInitThreads()는 libX11의 내부 락을 켜는데, **Tk()나 cv2 초기화가 Display를 연 뒤에
# 부르면 아무 효과가 없다** — 그래서 모든 import보다 위에 둔다(tkinter/cv2 임포트보다 먼저).
# 리눅스가 아니면 libX11이 없으니 조용히 넘어간다(no-op).
if sys.platform.startswith("linux"):
    try:
        import ctypes
        # ⚠️ "libX11.so"(확장자 없는 심볼릭 링크)는 libx11-dev에만 있다 → .so.6을 쓸 것.
        ctypes.CDLL("libX11.so.6").XInitThreads()
    except (OSError, AttributeError):
        pass            # Wayland 전용·헤드리스 등 X를 안 쓰는 환경 — 필요도 없다

# ⚠️ 이 두 줄은 **어떤 import보다 먼저** 와야 한다(matplotlib은 임포트 시점에 백엔드를 정한다).
# mediapipe.solutions.drawing_utils가 모듈 레벨에서 `import matplotlib.pyplot`을 하는데,
# 리눅스 matplotlib의 기본 백엔드는 **TkAgg**다. 우리는 mediapipe를 **워커 스레드**에서
# 임포트하므로(_process_loop) 그 순간 tkinter/backend_tkagg가 워커에서 로드되며 X 연결을
# 건드리고, 메인 스레드의 Tk와 부딪혀 프로세스가 통째로 죽는다(리눅스 실측):
#     [xcb] Unknown sequence number while appending request
#     [xcb] You called XInitThreads, this is not your fault
#     python: xcb_io.c:157: append_pending_request:
#             Assertion `!xcb_xlib_unknown_seq_number' failed.  → Aborted (core dumped)
# Agg(비GUI)로 고정하면 워커에서 tkinter가 **아예 임포트되지 않는다**(실측: TkAgg일 때
# `tkinter in sys.modules`=True → Agg에선 False). 이 GUI는 pyplot을 쓰지 않으니 손실 없음.
# 윈도우에서는 TkAgg여도 죽지 않아 07-30까지 드러나지 않았다.
os.environ.setdefault("MPLBACKEND", "Agg")   # 필요하면 셸에서 MPLBACKEND로 덮어쓸 수 있다

import collections
import ctypes
import json
import socket
import struct
import threading
import time                       # os·sys는 위 X11/백엔드 처리에서 이미 임포트했다

import numpy as np
from PIL import Image, ImageTk

import tkinter as tk
from tkinter import ttk, filedialog, font as tkfont, messagebox

from one_euro_filter import OneEuroFilter
from dg5f_paths import unique_log_path, CALIB_PATH, PRESET_PATH, DATA_DIR, BUNDLE_DIR, read_path
import dg5f_angles as A
# 실물 DG-5F: SDK 래퍼(Dg5fSdk)·프레임 변환(to_sdk_frame)·관절 대응표를 **임포트해서** 쓴다.
# 여기에 사본을 만들지 말 것 — 브리지 프로세스와 GUI가 서로 다른 관절 대응을 갖게 되는
# 순간, 한쪽에서 한 실물 검증이 다른 쪽에서 무의미해진다(_RealHand 주석 참조).
# 임포트 부작용 없음(argparse/ctypes/socket/struct뿐, main()은 __main__ 가드 안).
import dg5f_sdk_bridge as SDKB

# cv2(0.6s) + mediapipe(3.9s) = 창이 뜨기까지의 대기시간. 워커 스레드가 임포트해서
# 여기에 채운다 → 창은 ~1.2초에 뜨고, 그 뒤 백그라운드로 모델이 준비된다.
# ⚠️ 모듈 최상단으로 되돌리지 말 것(그러면 B/C 개편 효과가 통째로 사라진다).
cv2 = None
mp = None

# ------------------------- 리눅스(X11) 스레드 안전 -------------------------
# X11 Xlib은 여러 스레드가 같은 Display를 쓰면 깨진다. 리눅스에서 cv2/mediapipe를 **워커
# 스레드에서 임포트**하면 그 임포트 부작용(cv2 highgui의 GTK 초기화, mediapipe→matplotlib)이
# X 연결을 건드려 메인 스레드의 Tk와 충돌하고, 프로세스가 통째로 죽는다:
#     [xcb] Unknown sequence number while appending request
#     python: xcb_io.c:157: append_pending_request:
#             Assertion `!xcb_xlib_unknown_seq_number' failed.  → Aborted (core dumped)
# MPLBACKEND=Agg(위)는 matplotlib 경로만 막는다.
#
# ⚠️ **"cv2·mediapipe를 메인 스레드에서 미리 임포트하면 된다"는 처방은 실패했다**
#    (2026-07-30 팀원 리눅스 실측: 둘 다 메인 스레드에서 임포트한 뒤 GUI를 띄워도 동일하게
#    abort). 즉 X를 건드리는 건 **임포트 부작용이 아니라 워커의 런타임 호출**(cv2 카메라
#    오픈 또는 mediapipe 그래프 초기화)이다. 같은 처방을 다시 시도하지 말 것.
#    → 프리로드는 진단용 레버로만 남긴다(기본 OFF, 어느 OS에서도 동작 변화 없음).
#      DG5F_PRELOAD=1 로 켤 수 있다. 리눅스 근본 대책은 미해결(스택 트레이스 확보 대기).
PRELOAD_HEAVY = os.environ.get("DG5F_PRELOAD", "").strip().lower() not in ("", "0", "false", "no")


def _preload_heavy():
    """cv2·mediapipe를 **메인 스레드에서** 미리 임포트한다. tk.Tk() 보다 먼저 부를 것 —
    X 연결을 여는 순서까지 메인 스레드로 몰아두는 편이 안전하다. 반환값=소요 초."""
    global cv2, mp
    t0 = time.perf_counter()
    import cv2 as _cv2
    cv2 = _cv2
    import mediapipe as _mp
    mp = _mp
    _ = _mp.solutions.hands      # drawing_utils(→matplotlib)까지 지금 로드해 둔다
    return time.perf_counter() - t0

# ------------------------- 기본 설정 (vision_node와 동일 값) -------------------------
CAM_INDEX = 0
CAM_BACKEND = None          # None=OpenCV 기본(Windows=MSMF, 실측 640x480@30 그대로 나옴).
                            # ⚠️ cv2.CAP_DSHOW는 open이 1.2초로 빠르지만 이 웹캠에서
                            #    read()가 504ms(2fps)로 붕괴한다 — 바꾸려면 반드시 재측정.
DEF_SIM_IP, DEF_SIM_PORT = "127.0.0.1", 5006      # Unity 트윈
DEF_REAL_IP, DEF_REAL_PORT = "127.0.0.1", 5007    # 실물 SDK 브리지
SEND_HZ_CAP = 60
FILTER_FREQ, FILTER_MIN_CUTOFF, FILTER_BETA = 30.0, 0.6, 0.0005
TIP_MIN_CUTOFF, TIP_BETA = 0.15, 0.5

DISPLAY_W = 480             # 미리보기 표시 폭(카메라 원본 640 → 세로는 비율 유지).
                            # 640으로 그리면 Tk 전송이 프레임당 7.7ms, 480이면 1.7ms.
                            # 각도 계산은 항상 원본 프레임으로 하므로 전송값과 무관.
DISPLAY_H = DISPLAY_W * 3 // 4   # 4:3 웹캠 기준 표시 높이 — 영상 자리 예약용(초기 창 크기 계산).
VIDEO_PAD = 6               # 미리보기 라벨 내부 여백. ⚠️ 크게 주지 말 것 — 라벨 폭은
                            # 이미지폭+2*VIDEO_PAD+4(테두리)라서, 여백을 키우면 프레임이
                            # 도착한 순간 영상 열이 예약폭(VIDEO_COL_W)을 넘어 컨트롤 패널을
                            # 창 밖으로 밀어낸다(2026-07-30에 padding=40으로 실제 발생:
                            # 라벨 175→564px, 우측 패널이 58px 잘려 가로 스크롤 없이는 안 보였음).
VIDEO_COL_W = DISPLAY_W + 2 * VIDEO_PAD + 4       # 영상 열에 예약하는 폭(=라벨 최종 폭)
# 창 높이는 **시작할 때 한 번만** 정한다(_set_default_geometry). 모드마다 패널이 3~10개로
# 달라지는데 그때마다 창을 다시 잡으면 버튼 하나 누를 때마다 창이 커졌다 작아져 쓰기 어렵다.
# 760 = 영상(376) + 컨트롤이 넉넉히 보이는 높이. 화면이 작으면 92%로 깎인다.
DEFAULT_H = 760
UI_HZ = 30                  # UI 리프레시 상한 (카메라 30fps보다 높일 이유 없음)
READOUT_HZ = 10             # ④ 판독 + 상태바 갱신 주기(문자열 포맷 아끼기)
UI_PERIOD_MS = 1000.0 / UI_HZ

N = 20
CH = A.CHANNEL_NAMES                       # 20 채널 이름
JOINT_ID = [f"{i // 4 + 1}_{i % 4 + 1}" for i in range(N)]   # 1_1 .. 5_4

# MediaPipe Hands 모델 정확도(0=경량/빠름, 1=정확/느림).
# ★2026-07-27: 0 → 1. **반드시 probe_landmarks.py·calibrate_dg5f.py·vision_node_dg5f.py와 같아야 한다.**
#   여기만 0이었던 탓에 보정·프로브 녹화(전부 complexity=1)로 뽑은 상수가 라이브와 안 맞았다.
#   실측 차이(같은 사람·같은 자세, 엄지끝↔소지MCP 거리):
#     펴짐    complexity=1: 1.019/1.058  vs  complexity=0: 1.054/1.012   (거의 동일)
#     완전대향 complexity=1: 0.227/0.247  vs  complexity=0: 0.512/0.476   (**2배 차이**)
#   → 경량 모델은 엄지를 손바닥 안쪽까지 깊게 넣지 못한다. 그 결과 THUMB_OPP_D_FULL=0.25가
#     라이브에서 도달 불가가 되어 대향 풀스케일 도달률 0.0%, 명령이 64~68°에서 멈췄다.
#     오른손이 특히 심했던 건 거리 상단이 max 2.006까지 튀어(왼손 1.312) 사각지대가 넓었기 때문.
#   ⚠️ 속도가 문제되면 0으로 내려도 되지만, **그 경우 그 모델로 전부 재보정**해야 한다
#     (보정 파일과 THUMB_OPP_D_* 둘 다). 모델만 바꾸고 상수를 두면 오늘 같은 증상이 재발한다.
MP_MODEL_COMPLEXITY = 1


# 프리셋 경로도 dg5f_paths가 소유한다(2026-07-31). 예전엔 여기서 _base_dir()로 따로
# 계산했는데, 그 바람에 **프리셋만 exe 폴더, 보정·관절표·로그는 번들 임시폴더**로
# 갈라져 있었다. exe에서 쓰기 가능한 곳은 DATA_DIR(%LOCALAPPDATA%\dg5f) 하나뿐이다.


# landmarks_to_xyz는 dg5f_angles가 소유한다(2026-07-28 통합) — 종횡비 등방 보정 포함.
#   ⚠️ 여기에 사본을 되살리지 말 것. 보정(calibrate)과 라이브가 다른 좌표계를 쓰게 된다
#   (한 곳만 값이 달라 어긋났던 07-27 MP_MODEL_COMPLEXITY 사고와 같은 구조).


# ------------------------- dg5f_angles 전역에 라이브 반영하는 헬퍼 -------------------------
# 이 함수들은 메인(UI) 스레드에서만 호출된다. 처리 스레드는 같은 전역을 읽지만,
# 갱신 단위가 '리스트 원소 하나에 튜플 하나를 대입'(GIL 하에서 원자적)이라 찢어진 값이
# 나올 수는 없다 — 최악의 경우 슬라이더를 놓은 그 한 프레임이 옛 값으로 나갈 뿐이다.
def _ch_idx(ch):
    return CH.index(ch)


def get_human_range(ch):
    _n, hmn, hmx, _dmn, _dmx, _g = A.DG5F_CHANNELS[_ch_idx(ch)]
    return hmn, hmx


def set_human_range(ch, lo, hi):
    i = _ch_idx(ch)
    n, _hmn, _hmx, dmn, dmx, g = A.DG5F_CHANNELS[i]
    A.DG5F_CHANNELS[i] = (n, lo, hi, dmn, dmx, g)   # map_to_dg5f가 이 리스트를 라이브로 읽음


def get_robot_range(ch):
    """현재 매핑이 참고하는 로봇 [lo,hi](deg). ratio 우선순위(RATIO_LIMIT)를 먼저 보여준다."""
    if ch in A.RATIO_LIMIT:
        return A.RATIO_LIMIT[ch]
    _n, _hmn, _hmx, dmn, dmx, _g = A.DG5F_CHANNELS[_ch_idx(ch)]
    return dmn, dmx


def set_robot_range(ch, lo, hi):
    """로봇 범위를 direct(DG5F_CHANNELS dmin/dmax)·ratio(RATIO_LIMIT) 양쪽에 함께 기록 →
    모드 전환해도 일관. 특수 엄지 채널(cmc/opp)은 전용 상수까지 갱신."""
    i = _ch_idx(ch)
    n, hmn, hmx, _dmn, _dmx, g = A.DG5F_CHANNELS[i]
    A.DG5F_CHANNELS[i] = (n, hmn, hmx, lo, hi, g)   # direct clamp + ratio 폴백
    A.RATIO_LIMIT[ch] = (lo, hi)                    # ratio 최우선
    if ch == "thumb_cmc":                           # |abd|→[fold,spread] 선형(direct/ratio 공통)
        A.THUMB_CMC_FOLD_DEG = lo
        A.THUMB_CMC_SPREAD_DEG = hi
    elif ch == "thumb_opp":                         # 단방향 대향 최대각(ratio 전용 상수)
        A.THUMB_OPP_RATIO_MAX_DEG = hi


# ------------------------- 스레드 간 핸드오프 -------------------------
class _LatestSlot:
    """최신 1개만 유지하는 스레드 간 슬롯. 소비자가 느리면 **오래된 것을 버린다** —
    텔레오퍼레이션에서 밀린 프레임은 가치가 없고(지연만 늘고), 큐로 쌓으면 지연이 무한정
    자란다. 생산자는 절대 블로킹되지 않는다."""

    def __init__(self):
        self._cv = threading.Condition()
        self._item = None
        self._seq = 0

    def put(self, item):
        with self._cv:
            self._item = item
            self._seq += 1
            self._cv.notify_all()

    def wait_new(self, seq, timeout=None):
        """seq 이후의 새 아이템을 기다려 (새 seq, item)을 준다. 타임아웃이면 (seq, None)."""
        with self._cv:
            if self._seq == seq:
                self._cv.wait(timeout)
            if self._seq == seq:
                return seq, None
            return self._seq, self._item

    def peek(self):
        """대기 없이 현재 (seq, item). UI 스레드용 — 절대 블로킹되면 안 된다."""
        with self._cv:
            return self._seq, self._item


class _Settings:
    """처리 스레드가 읽는 설정 스냅샷(불변). tk 변수를 워커에서 직접 .get() 하면
    Tcl 인터프리터를 메인 스레드 밖에서 건드리게 되므로, 평범한 파이썬 값으로 복사해 넘긴다.
    교체는 참조 하나를 대입하는 것으로만 한다(원자적 → 락 불필요)."""
    __slots__ = ("hand", "mapmode", "overrides", "targets", "verify")

    def __init__(self, hand, mapmode, overrides, targets, verify=None):
        self.hand = hand
        self.mapmode = mapmode
        self.overrides = overrides   # {ch_idx: deg} — 워커는 읽기만
        self.targets = targets       # ((ip, port), ...) 파싱 완료 → 송신 경로에서 int() 안 함
        self.verify = verify         # (ch_idx, deg) 관절 검증 모드 / None — 손·오버라이드보다 우선


class _Result:
    """처리 스레드 → UI 스레드로 넘기는 한 프레임분 결과."""
    __slots__ = ("disp", "detected", "raw", "mapped", "sent")

    def __init__(self, disp, detected, raw, mapped, sent):
        self.disp = disp             # numpy RGB (표시용 축소 프레임, 랜드마크 그려진 상태)
        self.detected = detected
        self.raw = raw               # 20ch 사람 프록시(rad)
        self.mapped = mapped         # 20ch 로봇 각(deg, 필터 전)
        self.sent = sent             # 20ch 필터 후(= 실제 UDP 전송값) / None


class _TeleopLogger:
    """텔레옵 파이프라인 전 구간을 한 CSV에 남기는 로거 (2026-07-28 신설).

    왜 만들었나:
      이 GUI에는 신설(07-23) 이래 로깅이 **아예 없었다**. 그래서 라이브 세션을 사후 분석하려면
      Unity가 받은 rx/act(unity_dg5f_*.csv)만 봐야 했는데, 거기서 이상이 보여도 원인이
      ①랜드마크 노이즈인지 ②프록시 수식인지 ③매핑 상수인지 가릴 수가 없다. 실제로 07-28
      벌림 crosstalk 분석(5_2가 굽힘과 r=0.95)에서 여기서 막혀 결론을 못 냈다.
      → 같은 프레임의 네 층을 한 행에 남겨 층간 책임을 가른다.

    열 구성 (한 행 = 처리 스레드 한 프레임):
      t_unix,detected,hand,mapmode,tx        메타. t_unix는 UTC 초 — Unity 로거와 같은 시계라
                                             unity_dg5f_*.csv / rad_dg5f_*.csv 와 그대로 조인된다.
      lm0_x..lm20_z   (63)  ① MediaPipe 랜드마크 원값(이미지 정규화 좌표, 미검출이면 공란)
      raw_<채널>      (20)  ② 사람 관절 프록시[rad] — A.compute_raw 출력
      mapped_<채널>   (20)  ③ 로봇 관절각[deg] — A.map_to_dg5f 출력(오버라이드·필터 **전**)
      sent_<채널>     (20)  ④ 실제 패킷에 실린 값 — 오버라이드 적용 + OneEuro 필터 **후**
      ※ tx=1은 이 프레임에 sendto가 실제로 나갔다는 뜻. SEND_HZ_CAP(60Hz) 때문에 값은
        준비됐어도 송신은 걸러질 수 있어 sent_*와 별도 열로 둔다.
      ※ 미검출 프레임의 raw_*는 occlusion hold로 **실제 사용된** 직전 값이다(공란 아님).
        그 프레임이 hold인지는 detected=0으로 구분한다.

    스레드 규칙:
      처리 스레드는 deque.append만 한다(포맷·디스크 접촉 없음). 포맷과 flush는 쓰기 전담
      스레드가 맡는다 — 07-27 성능 개편의 "워커는 UI/디스크에 막히지 않는다"를 깨지 않기 위함.
      백로그가 MAX_BACKLOG를 넘으면 **오래된 행부터 버린다**. 로그 때문에 텔레옵이 느려지는
      것보다 로그에 구멍이 나는 편이 낫다(_LatestSlot과 같은 철학). 버린 수는 UI에 표시한다.
      두 스레드가 같은 deque에 popleft를 하지만 append/popleft는 GIL 하에서 원자적이라
      깨지지 않는다 — 경합해도 '어느 행이 버려지는가'만 달라진다.

    파일 경로는 dg5f_paths.unique_log_path가 소유(초 단위 + 중복 시 접미사, 덮어쓰기 불가).
    켜고 끌 때마다 새 파일이 열린다 — 껐다 켜서 앞 세션을 덮는 사고를 원천 차단.

    용량: 30fps × 123열 ≈ 1.2KB/행 → 분당 약 2MB. 길게 켜두고 돌릴 때 참고.
    """

    MAX_BACKLOG = 600      # ≈20초분(30fps). 넘으면 오래된 행부터 폐기.
    FLUSH_EVERY = 100

    def __init__(self):
        self.active = False
        self.path = None
        self.count = 0
        self.dropped = 0
        self._q = collections.deque()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._th = None

    def start(self):
        """새 CSV를 열고 쓰기 스레드를 띄운다. 실패는 OSError로 올린다(호출자가 UI에 알림)."""
        if self.active:
            return self.path
        path = unique_log_path("teleop")        # 디렉터리 생성·중복 회피·자리 선점까지 끝냄
        f = open(path, "w", encoding="utf-8", newline="")
        f.write(",".join(
            ["t_unix", "detected", "hand", "mapmode", "tx"]
            + [f"lm{i}_{a}" for i in range(21) for a in "xyz"]
            + [f"raw_{n}" for n in CH]
            + [f"mapped_{n}" for n in CH]
            + [f"sent_{n}" for n in CH]) + "\n")
        self.path, self.count, self.dropped = path, 0, 0
        self._q.clear()
        self._stop.clear()
        self._wake.clear()
        self._th = threading.Thread(target=self._writer, args=(f,), name="dg5f-log", daemon=True)
        self.active = True                      # log()가 큐에 넣기 시작하는 시점 = 스레드 뜬 뒤
        self._th.start()
        return path

    def log(self, t, detected, hand, mapmode, tx, xyz, raw, mapped, sent):
        """처리 스레드 전용. 절대 블로킹하지 않는다."""
        if not self.active:
            return
        q = self._q
        if len(q) >= self.MAX_BACKLOG:
            try:
                q.popleft()
                self.dropped += 1
            except IndexError:                  # 쓰기 스레드가 먼저 비웠다 — 버릴 게 없으니 그냥 넣는다
                pass
        q.append((t, detected, hand, mapmode, tx, xyz, raw, mapped, sent))
        self._wake.set()

    def stop(self):
        if not self.active:
            return
        self.active = False                     # 새 행 유입 차단 → 남은 큐만 비우면 끝
        self._stop.set()
        self._wake.set()
        if self._th is not None:
            self._th.join(timeout=2.0)          # 파일 close는 쓰기 스레드가 finally에서 한다
            self._th = None

    def _writer(self, f):
        try:
            while True:
                if not self._q:
                    if self._stop.is_set():
                        break
                    self._wake.wait(0.2)
                    self._wake.clear()
                    continue
                try:
                    rec = self._q.popleft()
                except IndexError:              # 처리 스레드의 오버플로 폐기와 경합 — 무해
                    continue
                f.write(self._fmt(rec))
                self.count += 1
                if self.count % self.FLUSH_EVERY == 0:
                    f.flush()
        finally:
            try:
                f.flush()
            finally:
                f.close()

    @staticmethod
    def _fmt(rec):
        t, detected, hand, mapmode, tx, xyz, raw, mapped, sent = rec
        cols = [f"{t:.3f}", "1" if detected else "0", hand, mapmode, "1" if tx else "0"]
        # 미검출 프레임엔 랜드마크가 없다 → 공란(분석 시 NaN). 0으로 채우면 원점에 손이
        # 있었던 것처럼 보여 통계가 오염된다.
        cols += [""] * 63 if xyz is None else [f"{v:.5f}" for v in xyz.reshape(-1)]
        cols += [""] * N if raw is None else [f"{v:.5f}" for v in raw]
        cols += [""] * N if mapped is None else [f"{v:.3f}" for v in mapped]
        cols += [""] * N if sent is None else [f"{v:.3f}" for v in sent]
        return ",".join(cols) + "\n"


class _Tooltip:
    """마우스를 올리면 뜨는 설명 풍선 (2026-07-31, 상시 표시 ※ 라벨 대체).

    설명문을 패널에 그대로 깔면 세로를 3~4할 잡아먹어서, 모드별로 골라 보여줘도 한 화면에
    안 들어갔다. 필요할 때만 뜨게 하면 자리를 0으로 만들면서 내용은 그대로 남는다.

    Tk 기본 위젯만 쓴다(ttk에는 툴팁이 없다). Toplevel + overrideredirect = 테두리·제목
    표시줄 없는 순수 풍선. wait 뒤에 뜨게 하는 건, 마우스가 지나가기만 해도 튀어나오면
    슬라이더를 조작하는 내내 화면이 깜빡이기 때문이다.
    """

    DELAY_MS = 350

    def __init__(self, widget, text, wrap=430, enabled=None):
        self.widget, self.text, self.wrap = widget, text, wrap
        self.enabled = enabled          # ()->bool. 없으면 항상 켬 (보기▸도움말 풍선)
        self._after = None
        self._tip = None
        widget.bind("<Enter>", self._schedule, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _schedule(self, _e=None):
        self._cancel()
        self._after = self.widget.after(self.DELAY_MS, self._show)

    def _cancel(self):
        if self._after is not None:
            self.widget.after_cancel(self._after)
            self._after = None

    def _show(self):
        if self._tip is not None or (self.enabled is not None and not self.enabled()):
            return
        # 위젯 **바로 아래**에 붙인다. 마우스 좌표를 쓰면 풍선이 커서를 덮어 Leave가
        # 곧바로 발생하며 깜빡인다.
        x = self.widget.winfo_rootx() + 12
        y = self.widget.winfo_rooty() + self.widget.winfo_height() + 4
        self._tip = tw = tk.Toplevel(self.widget)
        tw.wm_overrideredirect(True)
        tw.wm_geometry(f"+{x}+{y}")
        tk.Label(tw, text=self.text, justify="left", wraplength=self.wrap,
                 background="#ffffe0", foreground="#222", relief="solid", borderwidth=1,
                 padx=6, pady=4).pack()

    def _hide(self, _e=None):
        self._cancel()
        if self._tip is not None:
            self._tip.destroy()
            self._tip = None


class _RealHand:
    """실물 DG-5F를 **이 프로세스에서 직접** 구동한다 (2026-07-31, dg5f_sdk_bridge 흡수).

    예전 경로(지금도 그대로 살아 있다 — ① 패널 Real 체크):
        GUI ──UDP :5007──> dg5f_sdk_bridge.py ──DGSDK.dll──> 실물
    여기는 그 브리지를 창 안으로 들여온 것뿐이다. **SDK 래퍼(Dg5fSdk)·프레임 변환
    (to_sdk_frame)·관절 대응표(JOINT_ORDER/SIGN/OFFSET/CLAMP)는 dg5f_sdk_bridge에서
    임포트해서 쓴다 — 사본 금지.** 관절 대응은 아직 실물 미검증이라 앞으로 고쳐야 하는데,
    두 벌이 있으면 한쪽에서 확정한 값이 다른 쪽에 반영되지 않아 검증 자체가 무의미해진다.

    스레드: 전용 서보 스레드 하나가 연결·송신·해제를 전부 맡는다.
      · ConnectToGripper/SystemStart는 수 초, MoveServoJoint는 Modbus TCP 왕복이라
        수~수십 ms 블로킹이다. 어느 쪽도 UI·캡처·처리 스레드에 있으면 안 된다
        (07-27 성능 개편의 "워커는 무엇에도 막히지 않는다"를 실물 경로에도 적용).
      · 처리 스레드는 submit()으로 최신 목표 20개를 **대입만** 한다(_LatestSlot과 같은 철학:
        밀린 목표는 버린다 — 실물에 보낼 값은 언제나 '지금 가장 최신' 하나뿐이다).

    안전 3원칙 (관절 대응 미검증 상태를 전제로 보수적으로 잡는다):
      1. **연결과 구동을 분리한다.** 연결만으로는 명령이 한 개도 나가지 않는다(state=대기).
         Arm을 켜야 servo가 시작된다.
      2. **항상 슬루 리밋을 통과한다.** 연결 직후 기준점은 rest(전 관절 0°)이므로, 손을
         이미 굽힌 채 Arm을 켜도 틱당 max_step[deg]씩만 접근한다(첫 프레임 점프 없음).
         ⚠️ 그래서 연결 시점에 실물이 rest(손 벌린 자세)여야 한다 — 아니면 그 차이만큼
         반대로 슬루하며 움직인다. 브리지 docstring의 '손 벌린 rest에서 시작'과 같은 전제다.
      3. **비상정지 = 즉시 disarm.** SDK에 전원을 끊는 수단은 없으므로 '더 이상 새 명령을
         보내지 않고 현재 자세에서 멈춘다'가 이 계층에서 가능한 최대치다. 재개하려면
         사람이 Arm을 다시 켜야 한다(자동 복귀 없음).
    """

    REST = [0.0] * SDKB.N_JOINTS
    STALE_SEC = 1.0          # 이보다 오래 새 목표가 없으면 '입력 끊김' 표시(마지막 자세 유지)

    def __init__(self):
        # 아래 값들은 서로 다른 스레드가 읽고 쓰지만 전부 '스칼라/참조 하나 대입'이라
        # GIL 하에서 찢어지지 않는다(_Settings와 같은 취급). 락은 요청 슬롯에만 건다.
        self.state = "미연결"
        self.connected = False
        self.armed = False       # 메인 스레드가 켜고, 오류 시 서보 스레드가 끈다
        self.tick = 0            # servo 호출 성공 횟수
        self.rate = 0.0          # **실측** servo 주기[Hz] — 설정값과 다를 수 있어 따로 보여준다
        self.hz = 50.0
        self.max_step = 2.0
        self.unmirror = False
        self.fb = None           # 그리퍼가 올려 보낸 최신 상태(read_state) / None
        # **명령을 한 번도 보내기 전**의 관절각(슬롯별). 영점의 기준은 이것뿐이다 —
        # Arm을 켜는 순간 손은 rest(0)로 끌려가므로, 그 뒤에 읽으면 우리가 방금 명령한
        # 값을 되읽는 것이지 하드웨어의 자연 자세가 아니다. connect 때 비우고, 아직
        # armed가 아닐 때 들어온 첫 상태로 채운다.
        self.rest_fb = None
        self.fb_t = 0.0
        self.fb_err = 0          # 연속 모듈에러 횟수(디바운스)
        self.fb_stale = 0        # 연속 read_state 실패 횟수(콜백을 못 믿을 때의 2차 감지)
        self.autostop = True     # 모듈에러·연결끊김이면 스스로 disarm
        self.diag = None         # 마지막 자가진단 결과
        self.last_op = None      # (동작 이름, DG_RESULT) — 토크제한·영점 같은 1회성 명령
        # 슬롯 직접 구동 — ⑨ 탐색 마법사 전용. **관절 대응표를 통과하지 않고** SDK 배열에
        # 그대로 쓴다. 표를 확정하려면 표에 의존하지 않는 명령 경로가 하나 있어야 한다.
        self.probe = None        # {슬롯: deg} / None (None이면 평소대로 손 추종)
        self._sdk = None
        self._target = None      # 최신 목표 20ch[deg] — 처리 스레드가 대입
        self._target_t = 0.0
        self._last_cmd = None    # 마지막으로 실제 내보낸 프레임(슬루 리밋 기준점)
        self._reqs = []          # 서보 스레드가 순서대로 처리할 요청 큐
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = None

    # ---- 메인(UI) 스레드 ----
    def _request(self, req):
        """서보 스레드에 시킬 일을 **순서대로** 쌓는다.
        ⚠️ 슬롯 하나로 두면 안 된다 — 연결 직후 토크 제한처럼 연달아 넣는 요청이
           앞의 것을 덮어써서 연결 자체가 사라진다(2026-07-31에 실제로 그렇게 짰다가 고침)."""
        with self._lock:
            self._reqs.append(req)
        if self._th is None:
            self._stop.clear()
            self._th = threading.Thread(target=self._loop, name="dg5f-servo", daemon=True)
            self._th.start()

    def connect(self, ip, port, model, dll, lpf):
        self.state = f"연결 중… {ip}:{port}"
        self._request(("connect", ip, port, model, dll, lpf))

    def disconnect(self):
        self.armed = False
        self._request(("disconnect",))

    def set_torque_limit(self, on):
        self._request(("torque", bool(on)))

    def diagnose(self):
        self.diag = None
        self._request(("diagnose",))

    def encoder_zero(self):
        self._request(("encoder_zero",))

    def go_rest(self):
        """전 관절을 rest(0°)로 되돌린다. 서보 경로에 슬루를 걸어 내리므로 모드 문제가 없다
        (MoveJointAll을 안 쓰는 이유는 Dg5fSdk의 바인딩 삭제 주석 참조)."""
        self.probe = None
        self.submit(list(self.REST))

    # ---- ⑨ 탐색 마법사 지원 (메인 스레드가 부른다) ----
    def probe_slot(self, slot, deg):
        """SDK 슬롯 하나만 deg로, 나머지는 0으로 보낸다. **관절 대응표를 통과하지 않는다.**
        slot=None이면 탐색을 끝내고 평소 경로(손 추종)로 돌아간다."""
        self.probe = None if slot is None else {int(slot): float(deg)}

    def wait_settle(self, timeout=2.5, tol=0.5):
        """명령이 실물에 도달해 멈출 때까지 기다린다. (도달?, 마지막 상태).

        슬루 리밋 때문에 명령을 넣어도 실제로는 여러 틱에 걸쳐 움직인다 — 바로 읽으면
        '움직이는 중'을 찍는다. 피드백의 joint[]가 tol 안에서 안정될 때까지 본다.
        moving/targetArrived 플래그를 쓰지 않는 건 그 의미가 실물 미검증이기 때문이다."""
        t0 = time.time()
        prev, still = None, 0
        while time.time() - t0 < timeout:
            time.sleep(0.12)
            st = self.fb
            if st is None:
                continue
            cur = st["joint"]
            if prev is not None and max(abs(a - b) for a, b in zip(cur, prev)) < tol:
                still += 1
                if still >= 3:                   # 연속 3회 정지 → 도달로 본다
                    return True, st
            else:
                still = 0
            prev = cur
        return False, self.fb

    def apply_map(self, d):
        """⑨ 관절 대응표를 반영한다. 이미 서보가 돌고 있으면 **그 스레드에 시킨다** —
        틱 사이에 바뀌어야 '새 슬롯 + 옛 부호'가 섞인 한 프레임이 안 나간다.
        반환값 = 어디서 처리했는지(상태 표시용)."""
        if self._th is None:                 # 아직 연결한 적 없음 → 경쟁 상대가 없다
            SDKB.apply_joint_map(d)
            return "즉시"
        self._request(("jointmap", d))
        return "서보 스레드"

    def shutdown(self):
        """창을 닫을 때. SystemStop + Disconnect는 서보 스레드의 finally가 한다."""
        self.armed = False
        self._stop.set()
        if self._th is not None:
            self._th.join(timeout=3.0)
            self._th = None

    # ---- 처리 스레드 ----
    def submit(self, deg20):
        """최신 목표만 남긴다. Arm 여부와 무관하게 항상 받아둔다 — Arm을 켜는 순간
        '지금 손 자세'로 (슬루 리밋을 밟으며) 따라가기 시작하는 게 자연스럽다."""
        self._target = list(deg20)
        self._target_t = time.time()

    # ---- 서보 스레드 ----
    def _loop(self):
        t_last = None
        try:
            while not self._stop.is_set():
                with self._lock:
                    pending, self._reqs = self._reqs, []
                for req in pending:              # 들어온 순서대로 (연결 → 토크제한 …)
                    self._handle(req)
                if self._sdk is None:
                    self._stop.wait(0.05)
                    t_last = None
                    continue
                if not self.armed:
                    # 구동은 안 해도 **상태는 읽는다.** Arm 전에 실제 관절각을 볼 수 있어야
                    # rest 자세인지 확인하고 켤 수 있고, ⑨ 영점도 여기서 딴다.
                    self._poll(0.2)
                    self._stop.wait(0.05)        # 실물엔 아무 명령도 안 나간다
                    t_last = None                # 멈춘 구간이 실측 Hz를 오염시키지 않게
                    continue

                t0 = time.perf_counter()
                if t_last is not None and t0 > t_last:
                    self.rate += 0.1 * (1.0 / (t0 - t_last) - self.rate)
                t_last = t0
                probe, tgt = self.probe, self._target
                stale = (time.time() - self._target_t) > self.STALE_SEC
                if probe is not None:
                    # ⑨ 탐색: 관절 대응표를 **통과하지 않고** 슬롯에 직접 쓴다(unmirror도 무시).
                    # 표를 확정하려는 중이므로 표에 의존하면 순환이 된다.
                    frame = [float(probe.get(s, 0.0)) for s in range(SDKB.N_JOINTS)]
                    stale = False
                else:
                    frame = SDKB.to_sdk_frame(self.REST if tgt is None else tgt, self.unmirror)
                step = self.max_step
                if self._last_cmd is not None and step > 0:
                    frame = [p + min(step, max(-step, t - p))
                             for p, t in zip(self._last_cmd, frame)]
                self._last_cmd = frame
                try:
                    res = self._sdk.servo(frame)
                except Exception as e:           # ctypes/드라이버 레벨 오류 — 즉시 멈춘다
                    self.armed = False
                    self.state = f"송신 오류 — {e} (Arm 해제됨)"
                    continue
                self.tick += 1
                self._poll(0.1)                  # 상태 읽기는 서보보다 낮은 주기로(통신량)
                if not self.armed:
                    # _poll이 방금 멈춰 세웠다(모듈에러·연결끊김). 아래 ARM 문구로 덮으면
                    # 사용자는 왜 멈췄는지 못 본다 — 그 메시지를 그대로 남기고 빠진다.
                    continue
                self.state = ("ARM {:.0f}Hz(설정 {:.0f}) #{}".format(self.rate, self.hz, self.tick)
                              + (" · 탐색 중" if probe is not None else "")
                              + (" · 입력 끊김(자세 유지)" if stale else "")
                              + (f" · ⚠DG_RESULT={res}" if res != SDKB.DG_RESULT_NONE else "")
                              + (f" · ⚠모듈에러 {self.fb['err']}"
                                 if self.fb and self.fb["err"] else ""))
                # ⚠️ 실측 Hz는 설정값보다 낮게 나온다 — 윈도우 타이머 분해능이 ~15ms라
                #    wait(0.02)가 실제로는 ~32ms를 잔다(50 설정 → 실측 31Hz, 드라이 실측).
                #    비전이 30fps라 실물 30Hz면 충분하지만, 슬루 리밋(°/틱)의 실제 속도는
                #    max_step × **실측** Hz다 — 상태 표시에 실측값을 같이 내는 이유.
                left = 1.0 / max(1.0, self.hz) - (time.perf_counter() - t0)
                if left > 0:
                    self._stop.wait(left)
        finally:
            self._close()

    def _poll(self, period):
        """그리퍼 상태를 period 초마다 읽는다. 서보 스레드 전용(DLL을 만지는 건 이 스레드뿐).

        서보와 같은 주기로 읽지 않는 이유: GetReceivedGripperData도 Modbus 왕복이라
        50Hz 서보에 얹으면 실제 명령 주기가 반으로 준다. 사람이 보는 값이라 10Hz면 충분하다.

        모듈에러는 **연속 3회**부터만 반응한다 — 필드 의미가 아직 실물 미검증이라, 한 번
        튄 값으로 구동을 끊으면 데모 중에 엉뚱하게 멈춘다. 반대로 진짜 고장이면 계속 뜬다."""
        now = time.time()
        if now - self.fb_t < period:
            return
        self.fb_t = now
        # ① SDK 콜백으로 온 끊김 — 가장 빠른 신호.
        if SDKB.LINK["down"] and self.armed and self.autostop:
            self.armed = False
            self.state = "■ 연결 끊김(SDK 알림) — 자동 정지. ⑦에서 다시 연결하세요"
            return
        try:
            st = self._sdk.read_state()
        except Exception:                        # ctypes 레벨 오류 — 다음 주기에 다시
            st = None
        if st is None:
            # ② 콜백을 못 받는 경우의 2차 감지. 상태를 연달아 못 읽으면 링크가 죽은 것으로
            #    본다 — 콜백 동작이 실물 미검증이라 이 백업을 같이 둔다.
            self.fb_stale += 1
            if self.fb_stale >= 10 and self.armed and self.autostop:
                self.armed = False
                self.state = "■ 상태 수신 끊김 — 자동 정지(연속 10회 실패)"
            return
        self.fb_stale = 0
        self.fb = st
        if self.rest_fb is None and not self.armed:
            self.rest_fb = list(st["joint"])     # 첫 명령 이전의 자연 자세 — 영점의 기준
        if st["err"]:
            self.fb_err += 1
            if self.autostop and self.fb_err >= 3 and self.armed:
                self.armed = False
                self.state = f"■ 모듈에러 {st['err']} — 자동 정지(연속 3회). ⑦에서 확인 후 재개"
        else:
            self.fb_err = 0

    def _handle(self, req):
        if req[0] == "jointmap":
            SDKB.apply_joint_map(req[1])     # 검사는 GUI가 이미 했다(validate_joint_map)
            return
        if req[0] == "torque":               # 하드웨어 토크 제한 on/off
            if self._sdk is not None:
                self.last_op = ("토크 제한 " + ("ON" if req[1] else "OFF"),
                                self._sdk.set_torque_limit(req[1]))
            return
        if req[0] == "diagnose":
            if self._sdk is not None:
                self.armed = False           # 진단 중에는 서보를 멈춘다(SDK도 거부한다)
                self.diag = self._sdk.diagnose()
            return
        if req[0] == "encoder_zero":
            if self._sdk is not None:
                self.armed = False
                self.last_op = ("하드웨어 영점", self._sdk.encoder_zero())
                self.rest_fb = None          # 영점이 바뀌었으니 기준 자세를 다시 잡는다
            return
        if req[0] == "disconnect":
            self._close()
            self.state = "미연결"
            return
        _, ip, port, model, dll, lpf = req
        self._close()
        try:
            sdk = SDKB.Dg5fSdk(dll)              # CDLL 로드 실패 → OSError
            sdk.connect(ip, port, SDKB.MODELS[model])
            if lpf > 0:
                # SDK 내장 저역필터. 서보 루프 밖(연결 시)에만 건드린다 — DLL 호출을
                # servo와 경쟁시키지 않기 위함. 바꾸려면 재연결.
                sdk.dll.SetLowPassFilterAlpha(1, ctypes.c_float(lpf))
        except Exception as e:                   # OSError(DLL/소켓) · RuntimeError(DG_RESULT) · KeyError(모델)
            self._sdk = None
            self.connected = False
            self.state = f"연결 실패 — {e}"
            return
        self._sdk = sdk
        self.connected = True
        self.tick = 0
        self.fb, self.fb_t, self.fb_err, self.rest_fb = None, 0.0, 0, None
        # 슬루 리밋 기준점 = rest. None으로 두면 첫 프레임이 리밋 없이 통과해 실물이 튄다
        # (브리지의 '첫 패킷은 그대로 통과'가 실물에선 위험해서 여기선 채택하지 않는다).
        self._last_cmd = list(self.REST)
        self.state = f"연결됨(대기) {ip}:{port} {model}"

    def _close(self):
        self.armed = False
        self.connected = False
        if self._sdk is not None:
            try:
                self._sdk.close()                # SystemStop → DisconnectToGripper
            except Exception:                    # 종료 경로 — 여기서 예외를 올려봐야 할 수 있는 게 없다
                pass
            self._sdk = None


class _JointMapWizard:
    """⑨ 관절 대응표 자동 채우기 마법사 (2026-07-31).

    예전 절차: dg5f_sdk_bridge.py의 JOINT_ORDER/SIGN 상수를 손으로 고치고, --pose로 한
    관절씩 쏴 보고, 안 맞으면 다시 소스를 고치고 재실행. 20관절이면 하루 일이다.

    지금: 슬롯 0→19를 순서대로 살짝(기본 15°) 움직인다. 사람은 **"방금 뭐가 움직였나"**만
    고르면 되고, 표 기입·되돌리기·도달 확인은 여기가 한다.

    ⚠️ 왜 완전 자동이 아닌가 — 피드백(ReceivedGripperData.joint[])도 **SDK 슬롯 인덱스**라
       '슬롯 7이 지금 20도'는 알려 주지만 '슬롯 7이 검지 PIP'라는 해부학적 대응은 알려 줄
       수 없다. 그건 실물 손을 보는 사람만 안다. 대신 피드백으로 이만큼은 자동이다:
         · 명령이 실제로 먹었는가(Δ가 났는가) — 안 먹으면 '움직임 없음'으로 표시
         · 얼마나 움직였는가 — 사람이 착각한 경우(다른 손가락을 봤다) 걸러 낼 근거
         · 도달 판정 — 슬루 리밋 때문에 몇 틱 걸리는 걸 기다려 준다
         · 영점 — 시작 자세를 기준으로 offset을 잡는다
    """

    PROBE_DEG = 15.0          # 탐색 진폭. 눈에 보이되 뭘 부딪히긴 어려운 크기
    MAX_STEP = 1.0            # 탐색 중 슬루(°/틱) — 평소보다 느리게, 보면서 판단하라고

    def __init__(self, gui):
        self.g = gui
        self.slot = 0
        self.answers = {}     # {슬롯: (채널 인덱스 or None, 부호)}
        # 시작 자세(=rest)를 **슬롯별로** 잡아 둔다. 영점은 슬롯의 성질인데 표에서는 채널
        # 행에 적히므로, 대응이 정해지는 마지막에 옳은 행으로 옮겨 넣어야 한다.
        # (이걸 안 하면 '영점 자동'을 마법사 **전에** 눌렀을 때 영점만 옛 대응에 남아
        #  엉뚱한 채널 행에 붙는다 — 드라이 테스트에서 실제로 재현된 함정이다.)
        self.rest = list(gui.real.rest_fb or [0.0] * N)
        self._saved_step = gui.real.max_step
        gui.real.max_step = self.MAX_STEP

        self.win = w = tk.Toplevel(gui.root)
        w.title("관절 탐색 마법사")
        w.transient(gui.root)
        w.protocol("WM_DELETE_WINDOW", self._cancel)
        f = ttk.Frame(w, padding=10)
        f.pack(fill="both", expand=True)

        self.lbl_head = ttk.Label(f, text="", font=("", 11, "bold"))
        self.lbl_head.pack(anchor="w")
        ttk.Label(f, text=f"[움직여 보기]를 누르면 그 슬롯만 {self.PROBE_DEG:g}° 움직입니다. "
                          "실물을 보고 어느 관절이 움직였는지 고르세요.",
                  wraplength=430, foreground="#666").pack(anchor="w", pady=(2, 6))
        self.lbl_fb = ttk.Label(f, text="", foreground="#37a", wraplength=430)
        self.lbl_fb.pack(anchor="w")

        row = ttk.Frame(f)
        row.pack(anchor="w", pady=(6, 0))
        ttk.Button(row, text="▶ 움직여 보기", command=self._move).pack(side="left")
        ttk.Button(row, text="↩ 0°로", command=self._home).pack(side="left", padx=4)

        row = ttk.Frame(f)
        row.pack(anchor="w", pady=(8, 0))
        ttk.Label(row, text="움직인 관절:").pack(side="left")
        self.cb = ttk.Combobox(row, width=24, state="readonly",
                               values=["(움직이지 않음 / 모르겠음)"]
                                      + [f"{JOINT_ID[i]}  {CH[i]}" for i in range(N)])
        self.cb.current(0)
        self.cb.pack(side="left", padx=4)
        self.sign = tk.StringVar(value="+1")
        ttk.Label(row, text="방향:").pack(side="left", padx=(8, 0))
        ttk.Radiobutton(row, text="맞음(+)", value="+1", variable=self.sign).pack(side="left")
        ttk.Radiobutton(row, text="반대(−)", value="-1", variable=self.sign).pack(side="left")
        self.g._help(f, "방향 '반대(−)'는 우리 채널의 + 방향(예: 굽힘)과 실물이 움직인 방향이 "
                        "반대일 때 고른다. 표의 부호 열에 −1로 들어간다.\n\n"
                        f"진폭 {self.PROBE_DEG:g}°는 눈에 보이면서 뭘 부딪히긴 어려운 크기로 "
                        f"잡았고, 탐색 중에는 슬루도 {self.MAX_STEP:g}°/틱으로 늦춰 둔다"
                        "(끝나면 원래 값으로 되돌린다)."
                     ).pack(anchor="w", pady=(4, 0))

        row = ttk.Frame(f)
        row.pack(anchor="w", pady=(10, 0))
        ttk.Button(row, text="다음 슬롯 ▶", command=self._next).pack(side="left")
        ttk.Button(row, text="건너뛰기", command=lambda: self._next(skip=True)).pack(side="left", padx=4)
        ttk.Button(row, text="여기까지 표에 반영하고 끝", command=self._finish).pack(side="left", padx=4)
        ttk.Button(row, text="취소", command=self._cancel).pack(side="left")

        self._refresh()
        w.grab_set()

    # ---- 진행 ----
    def _refresh(self):
        self.lbl_head.configure(text=f"SDK 슬롯 {self.slot} / {N - 1}"
                                     f"   (기록됨 {len(self.answers)}개)")
        self.cb.current(0)
        self.sign.set("+1")
        self.lbl_fb.configure(text="")

    def _move(self):
        r = self.g.real
        base = (r.fb or {}).get("joint")
        r.probe_slot(self.slot, self.PROBE_DEG)
        ok, st = r.wait_settle()
        if st is None:
            self.lbl_fb.configure(text="상태를 못 읽었다 — 연결 확인", foreground="#a33")
            return
        # 피드백으로 '진짜 움직였나'를 확인한다. 사람 눈만 믿으면 명령이 안 먹은 슬롯을
        # 못 움직였다고 착각하거나, 옆 손가락을 보고 잘못 고를 수 있다.
        if base is None:
            self.lbl_fb.configure(text=f"슬롯 {self.slot} 실측 {st['joint'][self.slot]:+.1f}°"
                                       + ("" if ok else "  (도달 대기 시간 초과)"),
                                  foreground="#37a")
            return
        d = [a - b for a, b in zip(st["joint"], base)]
        mx = max(range(N), key=lambda i: abs(d[i]))
        if abs(d[mx]) < 1.0:
            self.lbl_fb.configure(
                text=f"⚠ 어느 슬롯도 1° 넘게 안 움직였다 (최대 슬롯{mx} {d[mx]:+.1f}°). "
                     "이 슬롯은 명령이 안 먹거나 물리적으로 막혀 있을 수 있다.",
                foreground="#a33")
            return
        other = [f"슬롯{i} {d[i]:+.1f}°" for i in range(N) if i != self.slot and abs(d[i]) > 1.0]
        self.lbl_fb.configure(
            text=f"슬롯 {self.slot} Δ{d[self.slot]:+.1f}° (실측 {st['joint'][self.slot]:+.1f}°)"
                 + (f"   ⚠ 같이 움직인 것: {', '.join(other)}" if other else "")
                 + ("" if ok else "   (도달 대기 시간 초과)"),
            foreground="#a33" if other else "#060")

    def _home(self):
        self.g.real.probe_slot(self.slot, 0.0)
        self.g.real.wait_settle()

    def _record(self):
        i = self.cb.current()
        if i > 0:                                # 0 = 움직이지 않음/모르겠음
            self.answers[self.slot] = (i - 1, self.sign.get())

    def _next(self, skip=False):
        if not skip:
            self._record()
        self._home()
        if self.slot >= N - 1:
            self._finish()
            return
        self.slot += 1
        self._refresh()

    # ---- 마무리 ----
    def _apply_answers(self):
        """{슬롯: (채널, 부호)} → ⑨ 표. 답하지 않은 슬롯은 **건드리지 않는다**(기존 값 유지).

        ⚠️ 슬롯은 채널당 하나뿐이라, 같은 채널을 두 슬롯에 답했으면 나중 것이 이긴다.
           표에 다 넣은 뒤 검사(validate_joint_map)에서 중복이 잡히면 적용이 거부된다."""
        for slot, (ch_i, sign) in self.answers.items():
            self.g.jm_vars[ch_i][0].set(str(slot))
            self.g.jm_vars[ch_i][1].set(sign)
            # 영점도 여기서 같이 넣는다 — 슬롯별로 잰 값을 **대응이 정해진 지금** 옳은
            # 채널 행에 붙인다(위 self.rest 주석 참조).
            self.g.jm_vars[ch_i][2].set(f"{self.rest[slot]:.1f}")

    def _finish(self):
        self._record()
        self._restore()
        self._apply_answers()
        n = len(self.answers)
        self.g.lbl_jmap.configure(
            text=f"탐색 결과 {n}개 슬롯을 표에 넣었다 — 확인하고 [적용] → [저장]",
            foreground="#060" if n else "#a33")
        self.win.destroy()
        if n:
            messagebox.showinfo("관절 탐색", f"{n}개 슬롯을 표에 반영했습니다.\n\n"
                                             "표를 확인한 뒤 [적용]으로 실물에 걸고, "
                                             "문제 없으면 [저장]하세요.")

    def _cancel(self):
        self._restore()
        self.win.destroy()

    def _restore(self):
        """탐색을 끝내고 평소 경로로 되돌린다. **반드시 불러야 한다** — probe가 남아 있으면
        손을 추종하지 않고 마지막 탐색 자세를 계속 유지한다."""
        self._home()
        self.g.real.probe_slot(None, 0)
        self.g.real.max_step = self._saved_step


class _CalibRec:
    """⑥ 보정 녹화 — calibrate_dg5f.py를 **창 안에서** 돌리는 것 (2026-07-31).

    왜 필요한가: ③ 슬라이더는 채널을 하나씩 손으로 맞추는 도구라 '내 손'에는 맞춰뒀어도
    **다른 사람이 이 프로그램만 받아서 쓰면** 20채널 human_ranges가 남의 손 값이다.
    게다가 GUI 프리셋(dg5f_gui_preset.json)은 dg5f_angles가 읽지 않아 vision_node 등
    다른 도구와 값이 갈린다. 보정은 dg5f_calibration.json(CALIB_PATH) 한 곳에 써야 한다.

    ⚠️ 백분위·물리캡·스키마는 calibrate_dg5f.build_calibration이 소유한다(여기서 재구현
       금지). 그래서 산출물은 스크립트로 보정한 것과 **바이트 단위로 같은 규칙**이다.
    수집만 여기서 한다 — 처리 스레드가 이미 만든 raw를 재활용하므로 카메라를 또 열지 않는다.
    """

    def __init__(self):
        self.active = False
        self.n = 0
        self.samples = {n: [] for n in CH}
        self.straight = []

    def start(self):
        self.samples = {n: [] for n in CH}
        self.straight = []
        self.n = 0
        self.active = True

    def add(self, raw, xyz):
        """처리 스레드 전용. 검출된 프레임에서만 부른다."""
        if not self.active:
            return
        for name, v in zip(CH, raw):
            self.samples[name].append(float(v))
        sr = A.thumb_straight_ratio(xyz)         # 수식은 dg5f_angles 소유(사본 금지)
        if sr is not None:
            self.straight.append(sr)
        self.n += 1

    def stop(self):
        self.active = False


class TeleopGUI:
    def __init__(self, root):
        self.root = root
        root.title("DG5F 텔레오퍼레이션 컨트롤")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ---- 상태 ----
        self.hand = tk.StringVar(value="right")
        self.mapmode = tk.StringVar(value="ratio")
        self.cam_index = tk.IntVar(value=CAM_INDEX)
        self.sel_ch = tk.StringVar(value=CH[0])
        self.overrides = {}          # {ch_idx: deg}  수동 오버라이드 활성 채널
        self.ov_enabled = tk.BooleanVar(value=False)
        self._estopped = False       # 비상정지로 꺼진 상태(상태바 표시용 — Arm을 켜면 해제)
        self._loading = False        # 슬라이더 프로그램 세팅 중 콜백 억제
        self._ov_rev = 0             # overrides 변경 감지용(스냅샷 재생성 트리거)

        # ---- 처리 스레드 소유 상태 (UI 스레드에서 만지지 말 것) ----
        self.last_vals = None        # occlusion hold (52ch)
        self.last_raw = [0.0] * N
        self.last_mapped = [0.0] * N
        self.pinch_on = False
        self._filter_freq = FILTER_FREQ
        self._tx_ok = False          # 이 프레임에 sendto가 실제로 나갔나(SEND_HZ_CAP에 걸리면 False)

        # ---- 로거 (메인이 켜고 끄고, 처리 스레드가 넣고, 전용 스레드가 쓴다) ----
        self.logger = _TeleopLogger()
        # ---- 실물 로봇·보정 녹화 (같은 소유 규칙: 메인이 켜고 끄고, 처리 스레드가 먹인다) ----
        self.real = _RealHand()
        self.calib = _CalibRec()

        # ---- 스레드 공용(원자적 대입만) ----
        self.pkt_count = 0
        self.cam_status = "카메라 준비 중…"
        self.cam_fps = 0.0
        self.proc_fps = 0.0
        self.ui_fps = 0.0

        # ---- 필터 (처리 스레드 전용) ----
        self.filters = {n: OneEuroFilter(FILTER_FREQ, FILTER_MIN_CUTOFF, FILTER_BETA) for n in CH}
        self.tip_filters = [OneEuroFilter(FILTER_FREQ, TIP_MIN_CUTOFF, TIP_BETA) for _ in range(3)]
        self.ftip_filters = [OneEuroFilter(FILTER_FREQ, TIP_MIN_CUTOFF, TIP_BETA)
                             for _ in range(3 * len(A.TIP_FINGERS))]
        self.wtip_filters = [OneEuroFilter(FILTER_FREQ, TIP_MIN_CUTOFF, TIP_BETA)
                             for _ in range(3 * len(A.WRIST_TIP_FINGERS))]
        self.pinch_filter = OneEuroFilter(FILTER_FREQ, FILTER_MIN_CUTOFF, 0.001)

        # ---- 네트워크 ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_send = 0.0

        # ---- 스레드 배관 ----
        self._stop = threading.Event()
        self._ev_cv2 = threading.Event()     # cv2 임포트 완료 신호(캡처→처리)
        self.frame_slot = _LatestSlot()      # 캡처 → 처리
        self.result_slot = _LatestSlot()     # 처리 → UI
        self._settings = _Settings("right", "ratio", {}, ())
        self._settings_key = None
        self._bad_targets = []               # 입력 중/무효인 송신 대상(상태바 경고용)
        self._cam_req = 1                    # 재연결 요청 카운터(메인 스레드가 증가)
        self._cam_req_index = CAM_INDEX
        self._shown_seq = 0
        self._photo = None
        self._last_result = None
        self._last_new_t = time.perf_counter()   # 마지막 새 프레임 시각(정지 감지용)
        self._slow_t = 0.0
        self._ui_t, self._ui_n = time.perf_counter(), 0

        # ① 창을 먼저 띄운다 — 무거운 임포트/카메라 오픈은 전부 워커 뒤로.
        self._build_ui()
        self._load_channel_into_sliders(CH[0])
        self._sync_settings()

        self._th_cap = threading.Thread(target=self._capture_loop, name="dg5f-capture", daemon=True)
        self._th_proc = threading.Thread(target=self._process_loop, name="dg5f-process", daemon=True)
        self._th_cap.start()
        self._th_proc.start()
        self.root.after(1, self._ui_tick)

    # ============================ 모드(툴바) ============================
    # 어떤 일을 하러 왔는지에 따라 보여줄 패널만 고른다. 값은 (설명, 패널키 순서).
    # ⚠️ 패널의 동그라미 번호(①~⑧)는 **표시 순서가 아니라 고정 ID**다 — 코드 주석과
    #    docstring이 전부 그 번호로 서로를 가리키므로, 모드마다 순서가 바뀌어도 번호는 둔다.
    #    (모드에서는 그 일의 주 도구를 툴바 바로 밑에 올린다.)
    MODES = {
        "보정": ("내 손 가동범위를 재서 저장한다.\n처음 쓰는 사람은 여기부터.",
                 ("hand", "calib", "read")),
        "시뮬": ("Unity 트윈으로 보낸다.\n매핑·범위를 눈으로 맞추는 단계.",
                 ("send", "hand", "chan", "read", "verify", "log", "preset")),
        # 실물에는 ③(채널 파라미터)을 넣지 않는다 — 슬라이더 5개가 300px를 먹는데
        # 실물 데모 중에 사람 범위를 만질 일은 없다. 필요하면 '전체'로 넘어가면 된다.
        "실물": ("실물 DG-5F를 구동한다.\n관절 대응이 확정된 뒤 쓰는 모드.",
                 ("real", "verify", "send", "hand", "read", "log", "preset")),
        "관절맵": ("처음 실물에 붙일 때 한 번.\n⑧로 한 관절씩 돌려 보며 ⑨ 표를 확정한다.",
                   ("real", "verify", "jmap")),
        "전체": ("모든 패널. 세로로 길어 스크롤이 필요하다.",
                 ("send", "hand", "chan", "read", "log", "calib",
                  "real", "verify", "jmap", "preset")),
    }

    def _build_menubar(self):
        """윈도우 앱다운 메뉴 막대. 툴바에 다 늘어놓지 않고 자주 안 쓰는 동작은 여기로 뺀다."""
        mb = tk.Menu(self.root)

        m = tk.Menu(mb, tearoff=0)
        m.add_command(label="프리셋 저장…", command=self.save_preset)
        m.add_command(label="프리셋 불러오기…", command=self.load_preset)
        m.add_separator()
        m.add_command(label="종료", command=self.on_close)
        mb.add_cascade(label="파일", menu=m)

        m = tk.Menu(mb, tearoff=0)
        for name in self.MODES:
            m.add_radiobutton(label=name, value=name, variable=self.mode,
                              command=self._apply_mode)
        m.add_separator()
        m.add_checkbutton(label="도움말 풍선(툴팁)", variable=self.tips_on)
        mb.add_cascade(label="보기", menu=m)

        m = tk.Menu(mb, tearoff=0)
        m.add_command(label="카메라 재연결", command=self._request_camera)
        m.add_separator()
        m.add_command(label="보정 녹화 시작/완료", command=self._on_calib_toggle)
        m.add_command(label="현재 보정값 보기…", command=self._show_calib)
        m.add_separator()
        m.add_command(label="실물 연결…", command=self._rb_connect)
        m.add_command(label="실물 해제", command=self._rb_disconnect)
        m.add_command(label="비상정지", command=self._estop, accelerator="Esc")
        m.add_separator()
        m.add_command(label="관절 대응표 저장", command=self._jm_save)
        m.add_command(label="관절 대응표 불러오기", command=self._jm_load)
        mb.add_cascade(label="도구", menu=m)

        m = tk.Menu(mb, tearoff=0)
        m.add_command(label="정보", command=self._about)
        mb.add_cascade(label="도움말", menu=m)
        self.root.configure(menu=mb)
        # Esc = 비상정지. 실물이 붙어 있으면 어느 패널에 포커스가 있든 먹어야 한다.
        self.root.bind_all("<Escape>", lambda _e: self._estop())

    def _build_toolbar(self):
        """메뉴 아래 한 줄짜리 툴바. 모드 전환(세그먼트) + 자주 쓰는 동작 + 오른쪽 상태등.

        표시등과 비상정지를 여기 두는 이유: 패널을 숨겨도 그 기능은 계속 동작한다
        (숨김 ≠ 끄기). Arm이 켜진 채 보정 모드로 넘어가도 실물은 계속 손을 따라간다 —
        그때 **정지 수단이 화면 밖에 있으면 안 된다.**"""
        bar = ttk.Frame(self.root, padding=(6, 3))
        bar.grid(row=0, column=0, columnspan=3, sticky="ew")
        # Toolbutton = 라디오지만 납작한 버튼으로 그려지는 기본 ttk 스타일(툴바 세그먼트).
        for name, (desc, _keys) in self.MODES.items():
            rb = ttk.Radiobutton(bar, text=name, value=name, variable=self.mode,
                                 style="Toolbutton", width=6, command=self._apply_mode)
            rb.pack(side="left", padx=1)
            _Tooltip(rb, desc)
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=8)
        for text, cmd, tip in (
                ("카메라", self._request_camera, "카메라를 다시 연다 (② cam# 값으로)"),
                ("보정", self._on_calib_toggle, "⑥ 보정 녹화 시작 / 완료·저장"),
                ("연결", self._rb_connect, "⑦ 실물 그리퍼에 접속 (구동은 Arm을 따로 켜야 시작)")):
            b = ttk.Button(bar, text=text, style="Toolbutton", command=cmd)
            b.pack(side="left", padx=1)
            _Tooltip(b, tip)
        # 오른쪽 끝: 살아 있는 것들. tk.Label/Button(ttk 아님) — 테마가 배경색을 무시한다.
        self.btn_estop_bar = tk.Button(bar, text="■ 정지", command=self._estop,
                                       bg="#c62828", fg="white", activebackground="#8e0000",
                                       activeforeground="white", state="disabled")
        self.btn_estop_bar.pack(side="right", padx=(6, 0))
        _Tooltip(self.btn_estop_bar,
                 "비상정지 — 실물 송신을 즉시 끊고 현재 자세에서 멈춘다.\n"
                 "재개하려면 ⑦에서 Arm을 다시 켜야 한다(자동 복귀 없음).\n\n"
                 "단축키 Esc도 같은 동작이지만 **이 창이 활성일 때만** 먹는다 — Unity 쪽을 "
                 "보고 있을 땐 안 통하므로, 실물을 돌릴 땐 이 버튼이 보이는 상태로 둘 것.\n"
                 "SDK에 전원을 끊는 수단은 없다. '더 이상 새 명령을 보내지 않는다'가 "
                 "이 계층에서 가능한 최대치다.")
        self.lamp = tk.Label(bar, text="", padx=6)
        self.lamp.pack(side="right")
        _Tooltip(self.lamp, "지금 살아 있는 것 — 패널을 숨겨도 기능은 계속 돌아가므로,\n"
                            "실제로 무엇이 나가는지는 여기와 맨 아래 상태바로 확인한다.")

    def _help(self, parent, text, **kw):
        """설명문을 'ⓘ' 한 글자로 접고 내용은 툴팁으로 넘긴다.
        예전엔 ※ 라벨을 그대로 깔았는데 그것만으로 패널 세로의 3~4할이었다."""
        lb = ttk.Label(parent, text="ⓘ 도움말", foreground="#37a", cursor="question_arrow", **kw)
        _Tooltip(lb, text, enabled=self.tips_on.get)
        return lb

    def _apply_mode(self):
        """현재 모드의 패널만 배치한다. 숨김은 grid_remove — 위젯과 tk 변수는 그대로 살아
        있으므로(파괴하지 않는다) 모드를 오가도 입력값·연결 상태가 유지된다.
        ⚠️ 창 크기는 **건드리지 않는다** — 모드를 누를 때마다 창이 커졌다 작아지면
           위치까지 흔들려서 쓰기 불편하다(넘치는 만큼은 오른쪽 패널만 스크롤한다)."""
        _desc, keys = self.MODES[self.mode.get()]
        for f in self._panels.values():
            f.grid_remove()
        for row, key in enumerate(keys):
            self._panels[key].grid(row=row, column=0, sticky="ew", pady=3)
        self._canvas.yview_moveto(0.0)           # 모드를 바꾸면 항상 맨 위부터 본다
        self._canvas.xview_moveto(0.0)

    def _set_default_geometry(self):
        """창 크기는 시작할 때 **한 번만** 정하고 그 뒤로는 사용자 몫이다.
        폭은 영상(예약폭) + 패널 실측폭이라 잘리지 않고, 높이는 DEFAULT_H 고정 —
        모드마다 패널이 3개~10개로 달라져도 창은 그대로 있고 안쪽만 스크롤된다.
        ⚠️ '영상 자리'는 예약값(VIDEO_COL_W/DISPLAY_H)으로 잡는다 — 이 시점의 영상 라벨은
           아직 안내문 크기(≈175px)뿐이라, 실측으로 잡으면 첫 프레임에 라벨이 커지면서
           컨트롤 패널을 창 밖으로 밀어낸다(2026-07-30에 실제로 고친 증상)."""
        root = self.root
        # 폭은 **가장 넓은 패널** 기준으로 잡는다 — 지금 모드만 재면, ⑨ 표(가장 넓다)가
        # 있는 모드로 넘어갔을 때 가로 스크롤이 생긴다. 창 폭은 고정이므로 처음 한 번
        # 전체를 배치해 재고 되돌린다.
        cur = self.mode.get()
        self.mode.set("전체")
        self._apply_mode()
        root.update_idletasks()
        need_w = (VIDEO_COL_W + 14 + self._main.winfo_reqwidth()
                  + self._vbar.winfo_reqwidth() + 2)
        self.mode.set(cur)
        self._apply_mode()
        root.update_idletasks()
        w = min(need_w, int(root.winfo_screenwidth() * 0.95))
        h = min(DEFAULT_H, int(root.winfo_screenheight() * 0.92))
        root.geometry(f"{w}x{h}")
        root.minsize(VIDEO_COL_W + 220, 420)

    def _about(self):
        messagebox.showinfo(
            "DG5F 텔레오퍼레이션 컨트롤",
            "웹캠 손 추적 → DG-5F 20관절.\n\n"
            "보정: 내 손 가동범위 측정 → dg5f_calibration.json\n"
            "시뮬: Unity Dg5fReceiver로 UDP v6(72f) 송신\n"
            "실물: DGSDK.dll 직결 (연결/Arm 분리 · 슬루 리밋 · Esc 비상정지)\n"
            "관절맵: 우리 채널 ↔ SDK 슬롯·부호·영점 확정 → dg5f_joint_map.json\n\n"
            "계산·스키마·관절표의 소유자는 각각 dg5f_angles / calibrate_dg5f /\n"
            "dg5f_sdk_bridge다. 이 창은 그걸 호출할 뿐 사본을 갖지 않는다.")

    # ============================ UI 구성 ============================
    def _build_ui(self):
        root = self.root
        self._panels = {}                        # 패널키 → 프레임 (_apply_mode가 배치)

        # ---- 레이아웃 ----
        #   row0: 모드 툴바(전 폭, 고정)
        #   row1: [영상(고정)] [컨트롤 캔버스(스크롤)] [수직 스크롤바]
        #   row2:              [수평 스크롤바]
        #   row3: 상태바(전 폭, 고정)
        # 패널을 다 펼치면 세로 1560px라 어떤 노트북 화면에도 안 들어간다. 그래서 **툴바로
        # 할 일을 고르고 그 일에 필요한 패널만 보여준다**(_apply_mode). 숨긴 패널의 설정은
        # 그대로 살아 있다 — 숨김은 '끄기'가 아니다. 그래서 지금 실제로 무엇이 나가고
        # 있는지(송신 대상·Arm·검증)는 **항상 보이는** 툴바 표시등과 상태바에만 의존한다.
        # **스크롤은 오른쪽 패널에만** 건다 — 영상까지 같이 스크롤되면 슬라이더를 만지려고
        # 내려간 순간 미리보기가 화면 밖으로 사라져서, 손을 보면서 값을 맞출 수가 없다
        # (텔레옵에서 그건 기능 상실이다). ttk.Frame은 스크롤을 못 하므로 패널만
        # Canvas + create_window에 담는다.
        root.columnconfigure(1, weight=1)        # 창을 넓히면 영상이 아니라 패널 쪽이 늘어난다
        root.rowconfigure(1, weight=1)

        # 메뉴·툴바가 참조하는 tk 변수는 그 전에 만들어 둔다.
        self.mode = tk.StringVar(value="보정")
        self.tips_on = tk.BooleanVar(value=True)
        self._build_menubar()
        self._build_toolbar()                    # row0

        # ---- 좌: 영상 (스크롤 밖 = 항상 같은 자리) ----
        # 영상 자리를 미리 480x360으로 예약한다(열/행 minsize). 그러지 않으면 창을 띄우는
        # 순간(=첫 프레임 전)의 라벨 크기는 안내문뿐이라 초기 창이 그 크기로 잡히고,
        # 프레임이 도착해 라벨이 커지는 순간 컨트롤 패널이 오른쪽으로 밀려 **잘린다**.
        # 예약폭은 라벨의 최종 폭(VIDEO_COL_W)과 정확히 같게 맞춘다 — 라벨이 예약폭보다
        # 커지면 예약이 무의미해지므로 padding은 VIDEO_PAD로만 준다.
        self.video = ttk.Label(root, text="카메라 준비 중…\n(모델 로딩 ~4초)",
                               anchor="center", padding=VIDEO_PAD, foreground="#888")
        self.video.grid(row=1, column=0, rowspan=2, sticky="nw", padx=(6, 8), pady=(6, 0))
        root.columnconfigure(0, minsize=VIDEO_COL_W + 14)
        root.rowconfigure(1, minsize=DISPLAY_H + 2 * VIDEO_PAD + 4)

        # ---- 우: 컨트롤 (스크롤 안) ----
        outer = tk.Canvas(root, highlightthickness=0, borderwidth=0)
        outer.grid(row=1, column=1, sticky="nsew")
        vbar = ttk.Scrollbar(root, orient="vertical", command=outer.yview)
        vbar.grid(row=1, column=2, sticky="ns")
        hbar = ttk.Scrollbar(root, orient="horizontal", command=outer.xview)
        hbar.grid(row=2, column=1, sticky="ew")
        # increment을 주지 않으면 휠 한 칸이 '창 높이의 1/10'로 튄다 → 20px 단위로 고정.
        outer.configure(yscrollcommand=vbar.set, xscrollcommand=hbar.set,
                        yscrollincrement=20, xscrollincrement=20)
        self._canvas = outer

        main = ttk.Frame(outer, padding=(0, 6, 6, 6))
        self._main = main
        self._main_item = outer.create_window((0, 0), window=main, anchor="nw")
        # 내용 크기가 바뀌면(채널 전환으로 라벨 길이가 변하는 등) 스크롤 범위를 다시 잡는다.
        main.bind("<Configure>", self._on_content_configure)
        outer.bind("<Configure>", self._on_canvas_configure)
        # 휠: 포커스와 무관하게 창 어디서든(영상 위에서도) 패널이 굴러가게. 단 휠로 값이
        # 바뀌는 위젯(콤보·스핀박스) 위에서는 이중 동작이 되지 않게 넘긴다.
        # ⚠️ X11(리눅스)은 휠을 <MouseWheel>로 주지 않고 <Button-4/5>로 준다 —
        #    <MouseWheel>만 걸면 리눅스에서 휠 스크롤이 통째로 죽는다(방향 판정은 _wheel_dir).
        for seq in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
            root.bind_all(seq, self._on_wheel)
        for seq in ("<Shift-MouseWheel>", "<Shift-Button-4>", "<Shift-Button-5>"):
            root.bind_all(seq, self._on_wheel_x)
        for k in ("<Prior>", "<Next>", "<Home>", "<End>"):
            root.bind_all(k, self._on_page)
        self._vbar, self._hbar = vbar, hbar

        ctrl = ttk.Frame(main)
        ctrl.grid(row=0, column=0, sticky="nsew")
        main.columnconfigure(0, weight=1)         # 창을 넓히면 ①~⑥ 프레임도 같이 넓어진다
                                                 # (안 주면 패널 오른쪽에 빈 띠만 생긴다)

        # (1) 연결 대상
        f = self._panels["send"] = ttk.LabelFrame(ctrl, text="① 송신 대상 (IP·포트)", padding=6)
        self.sim_on = tk.BooleanVar(value=True)
        self.real_on = tk.BooleanVar(value=False)
        self.sim_ip = tk.StringVar(value=DEF_SIM_IP)
        self.sim_port = tk.StringVar(value=str(DEF_SIM_PORT))
        self.real_ip = tk.StringVar(value=DEF_REAL_IP)
        self.real_port = tk.StringVar(value=str(DEF_REAL_PORT))
        ttk.Checkbutton(f, text="Sim (Unity)", variable=self.sim_on).grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.sim_ip, width=15).grid(row=0, column=1)
        ttk.Entry(f, textvariable=self.sim_port, width=6).grid(row=0, column=2)
        ttk.Checkbutton(f, text="Real (로봇/브리지)", variable=self.real_on).grid(row=1, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.real_ip, width=15).grid(row=1, column=1)
        ttk.Entry(f, textvariable=self.real_port, width=6).grid(row=1, column=2)
        self._help(f, "다른 PC로 쏘려면 그 PC의 LAN IP를 넣는다 (같은 WiFi, 방화벽에서 UDP 허용).\n"
                      "Sim = Unity Dg5fReceiver(기본 127.0.0.1:5006).\n"
                      "Real = 별도 프로세스로 띄운 dg5f_sdk_bridge.py(:5007) — 이 창의 ⑦ 직결과는 "
                      "다른 경로다. 리눅스·맥처럼 DGSDK.dll을 못 쓰는 환경에서 이쪽을 쓴다."
                   ).grid(row=2, column=0, columnspan=3, sticky="w", pady=(3, 0))

        # (2) 모드
        f = self._panels["hand"] = ttk.LabelFrame(ctrl, text="② 손·매핑·카메라", padding=6)
        ttk.Label(f, text="손:").grid(row=0, column=0)
        ttk.Radiobutton(f, text="right", value="right", variable=self.hand).grid(row=0, column=1)
        ttk.Radiobutton(f, text="left", value="left", variable=self.hand).grid(row=0, column=2)
        ttk.Label(f, text="매핑:").grid(row=0, column=3, padx=(10, 0))
        ttk.Radiobutton(f, text="direct", value="direct", variable=self.mapmode).grid(row=0, column=4)
        ttk.Radiobutton(f, text="ratio", value="ratio", variable=self.mapmode).grid(row=0, column=5)
        ttk.Label(f, text="cam#").grid(row=1, column=0, pady=(4, 0))
        ttk.Spinbox(f, from_=0, to=8, width=4, textvariable=self.cam_index).grid(row=1, column=1, pady=(4, 0))
        ttk.Button(f, text="카메라 재연결", command=self._request_camera).grid(row=1, column=2, columnspan=2, pady=(4, 0))

        # (3) 채널 파라미터 편집
        f = self._panels["chan"] = ttk.LabelFrame(ctrl, text="③ 채널별 파라미터 (라이브)", padding=6)
        ttk.Label(f, text="채널:").grid(row=0, column=0, sticky="w")
        self.ch_combo = ttk.Combobox(f, textvariable=self.sel_ch, width=22, state="readonly",
                                     values=[f"{JOINT_ID[i]}  {CH[i]}" for i in range(N)])
        self.ch_combo.current(0)
        self.ch_combo.grid(row=0, column=1, columnspan=3, sticky="w")
        self.ch_combo.bind("<<ComboboxSelected>>", self._on_channel_change)

        self.s_hmin = self._mk_scale(f, 1, "사람 min (rad)", -1.8, 1.8, 0.01, self._on_human)
        self.s_hmax = self._mk_scale(f, 2, "사람 max (rad)", -1.8, 1.8, 0.01, self._on_human)
        self.s_rlo = self._mk_scale(f, 3, "로봇 lo (deg)", -160, 160, 1, self._on_robot)
        self.s_rhi = self._mk_scale(f, 4, "로봇 hi (deg)", -160, 160, 1, self._on_robot)

        ov = ttk.Frame(f)
        ov.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Checkbutton(ov, text="수동 오버라이드 (손 무시하고 이 각도로 송신)",
                        variable=self.ov_enabled, command=self._on_override_toggle).pack(anchor="w")
        self.s_manual = self._mk_scale(f, 6, "수동 각도 (deg)", -160, 160, 1, self._on_manual)

        self._help(f, "'로봇 lo/hi'는 direct=clamp범위 · ratio=정규화범위 양쪽에 반영된다.\n"
                      "엄지 1_1은 lo=접힘 / hi=벌림, 1_2는 hi=대향최대(음수).\n"
                      "1_1 값은 항상 왼손 기준 — right 모드에선 자동 반전돼 적용된다."
                   ).grid(row=7, column=0, columnspan=4, sticky="w", pady=(3, 0))

        # (4) 라이브 판독
        f = self._panels["read"] = ttk.LabelFrame(ctrl, text="④ 선택 채널 실시간 값", padding=6)
        self._mono = self._mono_font()            # Font 객체는 참조를 잡아둬야 GC되지 않는다
        self.lbl_read = ttk.Label(f, text="-", font=self._mono)
        self.lbl_read.pack(anchor="w")

        # (5) 로그 기록
        f = self._panels["log"] = ttk.LabelFrame(ctrl, text="⑤ 로그 기록 (CSV)", padding=6)
        self.log_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="랜드마크 → 사람각 → 로봇각 → 송신값 전 구간 기록",
                        variable=self.log_on,
                        command=self._on_log_toggle).grid(row=0, column=0, sticky="w")
        self.lbl_log = ttk.Label(f, text="꺼짐 — 켜면 logs/teleop_<시각>.csv 새로 생성",
                                 foreground="#666", wraplength=340)
        self.lbl_log.grid(row=1, column=0, sticky="w")
        self._help(f, "껐다 켜면 항상 새 파일이 생긴다(덮어쓰기 없음). 약 2MB/분.\n"
                      "t_unix가 Unity 로거와 같은 시계라 unity_dg5f_*.csv / rad_dg5f_*.csv와 "
                      "그대로 조인해서 층간 책임을 가릴 수 있다."
                   ).grid(row=2, column=0, sticky="w", pady=(3, 0))

        # (6) 보정 녹화 — calibrate_dg5f.py를 창 안에서
        f = self._panels["calib"] = ttk.LabelFrame(
            ctrl, text="⑥ 내 손 보정 녹화 (dg5f_calibration.json)", padding=6)
        ttk.Label(f, text="처음 쓰는 사람은 여기부터. 녹화 중 각 동작을 3회 이상 천천히:\n"
                          " ① 완전히 펴기 ↔ 주먹 꽉 쥐기   ② 엄지 대향(손바닥 반대↔새끼쪽)\n"
                          " ③ 엄지 쭉 펴기(여러 방향, 2초씩)  ④ 손가락 쫙 벌리기 ↔ 모으기",
                  wraplength=340).grid(row=0, column=0, columnspan=3, sticky="w")
        self.btn_calib = ttk.Button(f, text="● 보정 녹화 시작", command=self._on_calib_toggle)
        self.btn_calib.grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Button(f, text="현재 보정값 보기", command=self._show_calib).grid(row=1, column=1, padx=4, pady=(4, 0))
        self.lbl_calib = ttk.Label(f, text="대기 — 저장하면 즉시 라이브 반영(재시작 불필요)",
                                   foreground="#666", wraplength=340)
        self.lbl_calib.grid(row=2, column=0, columnspan=3, sticky="w", pady=(3, 0))
        self._help(f, "저장 위치가 calibrate_dg5f.py와 같은 파일(dg5f_calibration.json)이라 "
                      "vision_node 등 다른 도구에도 그대로 적용된다 — 프리셋과 달리 공용이다.\n"
                      "방식: 채널별 백분위 2/98 + 물리캡, 엄지 직진도는 p95.\n"
                      "저장하면 재시작 없이 바로 반영된다(③ 슬라이더도 새 값으로 갱신)."
                   ).grid(row=3, column=0, columnspan=3, sticky="w", pady=(3, 0))

        # (7) 실물 로봇 직결
        f = self._panels["real"] = ttk.LabelFrame(ctrl, text="⑦ 실물 로봇 (DG-5F SDK 직결)", padding=6)
        self.rb_ip = tk.StringVar(value="")
        self.rb_port = tk.StringVar(value="502")
        self.rb_model = tk.StringVar(value="5f_left")
        self.rb_unmirror = tk.BooleanVar(value=False)
        self.rb_dll = tk.StringVar(value=os.path.abspath(SDKB.DEFAULT_DLL))
        self.arm_on = tk.BooleanVar(value=False)
        self.rb_step = tk.StringVar(value="2.0")
        self.rb_hz = tk.StringVar(value="50")
        self.rb_lpf = tk.StringVar(value="0.3")

        ttk.Label(f, text="IP").grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.rb_ip, width=15).grid(row=0, column=1, sticky="w")
        ttk.Label(f, text="포트").grid(row=0, column=2, sticky="e")
        ttk.Entry(f, textvariable=self.rb_port, width=6).grid(row=0, column=3, sticky="w")
        ttk.Label(f, text="모델").grid(row=1, column=0, sticky="w")
        # 목록은 SUPPORTED_MODELS(5f_left/5f_right)뿐이다. S 계열은 손가락당 관절 수부터
        # 달라(15DOF는 3개) 20채널 계약이 성립하지 않는다 — 근거는 그 상수 주석 참조.
        ttk.Combobox(f, textvariable=self.rb_model, width=12, state="readonly",
                     values=list(SDKB.SUPPORTED_MODELS)).grid(row=1, column=1, sticky="w")
        ttk.Checkbutton(f, text="unmirror(왼손 스트림→오른손 실물)",
                        variable=self.rb_unmirror).grid(row=1, column=2, columnspan=2, sticky="w")
        ttk.Label(f, text="DLL").grid(row=2, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.rb_dll, width=30).grid(row=2, column=1, columnspan=2, sticky="ew")
        ttk.Button(f, text="찾기", width=5, command=self._rb_browse).grid(row=2, column=3, sticky="w")

        b = ttk.Frame(f)
        b.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        self.btn_conn = ttk.Button(b, text="연결", command=self._rb_connect)
        self.btn_conn.pack(side="left")
        ttk.Button(b, text="해제", command=self._rb_disconnect).pack(side="left", padx=4)
        ttk.Checkbutton(b, text="구동 시작(Arm)", variable=self.arm_on,
                        command=self._on_arm).pack(side="left", padx=(10, 4))
        # 비상정지는 tk.Button(ttk가 아님) — ttk 테마는 background를 무시해서 빨갛게 못 만든다.
        tk.Button(b, text="■ 비상정지", command=self._estop,
                  bg="#c62828", fg="white", activebackground="#8e0000",
                  activeforeground="white", relief="raised").pack(side="left", padx=6)
        self.lbl_real = ttk.Label(f, text="미연결", foreground="#666", wraplength=340)
        self.lbl_real.grid(row=4, column=0, columnspan=4, sticky="w", pady=(3, 0))
        # 그리퍼가 올려 보내는 상태 — 지금까지는 명령값만 보고 눈 감고 밀던 부분이다.
        fb = ttk.Frame(f)
        fb.grid(row=7, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.lbl_fb = ttk.Label(fb, text="상태 수신 없음", foreground="#666", wraplength=300,
                                font=self._mono_font(8))
        self.lbl_fb.pack(side="left")
        self.autostop_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="모듈 에러·연결 끊김이면 자동 정지",
                        variable=self.autostop_on).grid(row=8, column=0, columnspan=4, sticky="w")

        # 안전·진단 (SDK 호출은 전부 서보 스레드에 맡긴다 — servo와 경쟁시키지 않는다)
        sf = ttk.Frame(f)
        sf.grid(row=9, column=0, columnspan=4, sticky="w", pady=(4, 0))
        self.torque_on = tk.BooleanVar(value=True)
        ttk.Checkbutton(sf, text="토크 제한(HW)", variable=self.torque_on,
                        command=lambda: self.real.set_torque_limit(
                            self.torque_on.get())).pack(side="left")
        ttk.Button(sf, text="자가진단", command=self._rb_diagnose).pack(side="left", padx=4)
        ttk.Button(sf, text="rest로 복귀", command=self.real.go_rest).pack(side="left")
        ttk.Button(sf, text="HW 영점…", command=self._rb_encoder_zero).pack(side="left", padx=4)
        self._help(f, "[토크 제한(HW)] 그리퍼 자체의 토크 제한(SetTorqueLimitMode). 소프트웨어 "
                      "슬루 리밋은 '명령이 튀는 것'만 막지, 손가락이 뭔가에 끼었을 때 힘을 "
                      "줄이지는 못한다 — 이쪽이 2차 방어선이다. 연결할 때 자동으로 켠다.\n\n"
                      "[자가진단] SystemDiagnosis 실행 후 결과를 읽는다. 진단 중에는 SDK가 다른 "
                      "명령을 거부하므로 Arm이 자동으로 꺼진다. 결과 필드의 의미는 SDK 문서에 "
                      "없어 값만 그대로 보여 준다.\n\n"
                      "[rest로 복귀] 전 관절 목표를 0°로 — 슬루 리밋을 밟으며 천천히 돌아간다.\n\n"
                      "[HW 영점] ⚠️ 지금 자세를 **그리퍼 하드웨어의 엔코더 영점**으로 굳힌다. "
                      "프로그램을 꺼도 되돌아오지 않는다. ⑨의 offset(소프트 보정)과 달리 "
                      "장비에 남으므로, 정말 기계적 rest에 있을 때만 쓸 것."
                   ).grid(row=10, column=0, columnspan=4, sticky="w", pady=(2, 0))
        self.lbl_diag = ttk.Label(f, text="", foreground="#666", wraplength=340)
        self.lbl_diag.grid(row=11, column=0, columnspan=4, sticky="w")

        p = ttk.Frame(f)
        p.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(3, 0))
        # 클램프는 ⑨ 관절 대응표가 관절별로 소유한다 — 여기에 전역 ±N을 또 두면 어느 쪽이
        # 이겼는지 알 수 없어진다(⑨에 '전 채널 ±N 채우기' 버튼이 있다).
        for col, (txt, var, lo, hi, inc, w) in enumerate((
                ("슬루 °/틱", self.rb_step, 0.0, 20.0, 0.5, 5),
                ("Hz", self.rb_hz, 5.0, 100.0, 5.0, 5),
                ("LPF α", self.rb_lpf, 0.0, 0.9, 0.05, 5))):
            ttk.Label(p, text=txt).grid(row=0, column=col * 2, sticky="e", padx=(0 if col == 0 else 6, 2))
            ttk.Spinbox(p, from_=lo, to=hi, increment=inc, width=w,
                        textvariable=var).grid(row=0, column=col * 2 + 1, sticky="w")
        warn = ttk.Frame(f)
        warn.grid(row=6, column=0, columnspan=4, sticky="w", pady=(3, 0))
        # 이 한 줄만은 접지 않는다 — 실물이 엉뚱하게 움직일 수 있다는 경고는 툴팁 뒤에
        # 숨기면 안 읽힌다. 자세한 배경만 ⓘ로 넘긴다.
        ttk.Label(warn, text="⚠️ 첫 구동 전 '관절맵' 모드에서 대응표를 확정할 것",
                  foreground="#a33").pack(side="left")
        self._help(warn, "모델은 5f_left / 5f_right 둘만 고를 수 있다. S 계열(5f_s_*, 5f_s15_*)은 "
                         "손가락당 관절 수부터 달라서(15DOF는 3개) 우리 20채널 계약이 성립하지 "
                         "않는다 — 그대로 보내면 인덱스가 밀려 전 손가락이 엉뚱한 채널에 물린다.\n\n"
                         "검증 현황: 매핑·보정은 **왼손 기준으로 맞춰 왔고** 오른손은 일부만 "
                         "확인했다. 오른손으로 돌릴 땐 ⑧로 다시 훑을 것.\n\n"
                         "슬루/Hz는 라이브 반영, LPF는 연결할 때만 적용된다(바꾸려면 재연결).\n\n"
                         "관절 대응표(우리 채널 ↔ SDK 슬롯·부호·영점)는 기본값이 '항등 매핑, "
                         "부호 전부 +1'인 추정치다. 실물에서 확인한 적이 없으므로, 첫 구동은 "
                         "⑧ 검증 모드로 한 관절씩 돌려 보며 ⑨ 표를 고쳐 확정한다.\n\n"
                         "또 연결 시점에 실물이 rest(손 벌린 자세)여야 한다 — 슬루 리밋의 "
                         "기준점이 전 관절 0°이기 때문이다."
                   ).pack(side="left", padx=(8, 0))
        if not sys.platform.startswith("win"):
            # DGSDK.dll은 Windows 전용. 리눅스/맥에서는 ①의 Real(UDP)로 윈도우 PC의
            # dg5f_sdk_bridge.py에 보내는 예전 경로를 그대로 쓰면 된다.
            self.btn_conn.state(["disabled"])
            self.lbl_real.configure(
                text="이 OS에서는 직결 불가 — DGSDK.dll은 Windows 전용. "
                     "① Real 체크로 윈도우 PC의 dg5f_sdk_bridge.py(:5007)에 보내세요.",
                foreground="#a33")

        # (8) 관절 검증 모드
        f = self._panels["verify"] = ttk.LabelFrame(
            ctrl, text="⑧ 관절 검증 모드 (전 채널 rest + 1관절만)", padding=6)
        self.verify_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(f, text="켜면 손·수동 오버라이드를 모두 무시하고 아래 1관절만 움직인다",
                        variable=self.verify_on,
                        command=self._on_verify_toggle).grid(row=0, column=0, columnspan=4, sticky="w")
        ttk.Label(f, text="관절:", width=16).grid(row=1, column=0, sticky="w")
        self.vf_combo = ttk.Combobox(f, width=22, state="readonly",
                                     values=[f"{JOINT_ID[i]}  {CH[i]}" for i in range(N)])
        self.vf_combo.current(0)
        self.vf_combo.grid(row=1, column=1, columnspan=3, sticky="w")
        self.s_verify = self._mk_scale(f, 2, "검증 각도 (deg)", -160, 160, 1, self._on_verify)
        ttk.Button(f, text="0°(rest)로", command=lambda: self.s_verify.set(0)).grid(
            row=3, column=1, sticky="w", pady=(2, 0))
        self._help(f, "dg5f_sdk_bridge.py --pose 를 대신한다. 나머지 19관절이 0°로 고정되므로 "
                      "'이 채널이 실물의 어느 관절을 어느 방향으로 돌리는가'를 한 번에 하나씩 "
                      "확인할 수 있다. 슬루 리밋은 그대로 걸린다.\n\n"
                      "결과를 ⑨ 관절 대응표에 적어 넣으면 된다 — 다른 관절이 움직였으면 그 "
                      "채널의 'SDK 슬롯'을, 반대로 돌면 '부호'를 고친다.\n\n"
                      "⚠️ Unity로 동시에 쏠 때(① Sim)는 손가락 IK를 끄고 볼 것"
                      "(Dg5fFingerIKMode=JointAnglesOnly). 손이 화면에 없으면 리치벡터가 0으로 "
                      "나가는데 Dg5fReceiver.GetFingerTip은 그래도 true를 주므로, IK가 켜져 "
                      "있으면 0 벡터를 목표로 쫓아간다."
                   ).grid(row=4, column=0, columnspan=4, sticky="w", pady=(3, 0))

        # (9) 관절 대응표 — 예전엔 dg5f_sdk_bridge.py 상수를 직접 고쳐야 했던 것
        f = self._panels["jmap"] = ttk.LabelFrame(
            ctrl, text="⑨ 관절 대응표 (우리 채널 → 실물 SDK)", padding=6)
        head = ttk.Frame(f)
        head.grid(row=0, column=0, sticky="ew")
        ttk.Label(head, text="⑧로 한 관절씩 돌려 보고, 실제로 움직인 곳에 맞춰 고친 뒤 "
                             "[적용]. 확정되면 [저장].", wraplength=430).pack(side="left")
        self._help(head, "to_sdk_frame이 하는 계산:\n"
                         "    sdk[슬롯] = 부호 × 우리채널값 + 영점,  그 뒤 [lo, hi]로 클램프\n\n"
                         "· SDK 슬롯 — 이 채널이 실물 몇 번 관절을 미는가. 0~19가 하나씩 다 "
                         "쓰여야 한다(중복되면 적용을 거부한다 — 어떤 관절은 명령을 아예 "
                         "못 받게 되기 때문).\n"
                         "· 부호 — ⑧에서 +를 보냈는데 반대로 돌면 −1.\n"
                         "· 영점 — 우리 0°와 실물 0°가 다를 때의 차이[deg].\n"
                         "· lo/hi — 그 관절에 허용할 안전 범위[deg]. 처음엔 좁게 잡을 것.\n\n"
                         "[저장]은 dg5f_joint_map.json에 쓴다. dg5f_sdk_bridge.py도 기동할 때 "
                         "그 파일을 읽으므로, 여기서 확정하면 별도 브리지로 돌려도 같은 대응이 "
                         "적용된다 — 소스를 고칠 필요가 없다."
                   ).pack(side="left", padx=(6, 0))

        tbl = ttk.Frame(f)
        tbl.grid(row=1, column=0, sticky="ew", pady=(4, 0))
        for c, (t, w) in enumerate((("채널", 13), ("SDK 슬롯", 8), ("부호", 6),
                                    ("영점°", 7), ("lo°", 7), ("hi°", 7), ("실제°", 7))):
            ttk.Label(tbl, text=t, width=w, anchor="w",
                      font=("", 8, "bold")).grid(row=0, column=c, sticky="w", padx=1)
        self.jm_act = []             # '실제°' 라벨 20개 — 그리퍼가 올려 보낸 그 슬롯의 각도
        self.jm_vars = []            # [(sdk, sign, offset, lo, hi) StringVar ×20]
        for i, name in enumerate(CH):
            ttk.Label(tbl, text=f"{JOINT_ID[i]} {name}", width=13,
                      anchor="w").grid(row=i + 1, column=0, sticky="w", padx=1)
            v_sdk = tk.StringVar(value=str(i))
            v_sgn = tk.StringVar(value="+1")
            v_off = tk.StringVar(value="0")
            v_lo = tk.StringVar(value="-130")
            v_hi = tk.StringVar(value="130")
            ttk.Spinbox(tbl, from_=0, to=N - 1, width=6, textvariable=v_sdk).grid(
                row=i + 1, column=1, padx=1)
            ttk.Combobox(tbl, values=("+1", "-1"), width=4, state="readonly",
                         textvariable=v_sgn).grid(row=i + 1, column=2, padx=1)
            for c, var in ((3, v_off), (4, v_lo), (5, v_hi)):
                ttk.Spinbox(tbl, from_=-180, to=180, increment=5, width=6,
                            textvariable=var).grid(row=i + 1, column=c, padx=1)
            act = ttk.Label(tbl, text="—", width=7, anchor="e", foreground="#37a")
            act.grid(row=i + 1, column=6, padx=1)
            self.jm_act.append(act)
            self.jm_vars.append((v_sdk, v_sgn, v_off, v_lo, v_hi))

        b0 = ttk.Frame(f)
        b0.grid(row=2, column=0, sticky="ew", pady=(5, 0))
        ttk.Button(b0, text="🔍 관절 탐색 마법사", command=self._jm_wizard).pack(side="left")
        ttk.Button(b0, text="영점 자동(현재 자세=0)",
                   command=self._jm_zero_from_fb).pack(side="left", padx=3)
        self._help(b0, "[관절 탐색 마법사] SDK 슬롯 0~19를 하나씩 살짝 움직이고, 그때 실제로 "
                       "움직인 관절이 무엇인지 사람이 골라 주면 표를 자동으로 채운다.\n\n"
                       "왜 전자동이 안 되나: 피드백(joint[])도 **SDK 슬롯 인덱스**라, '슬롯 7이 "
                       "지금 20°다'는 알아도 '슬롯 7이 검지 PIP다'는 알 수 없다. 그건 손을 보는 "
                       "사람만 안다. 마법사는 그 판단만 남기고 나머지(명령·대기·도달 확인·"
                       "표 기입·되돌리기)를 전부 대신한다.\n\n"
                       "[영점 자동] 지금 자세를 전 관절 0°로 삼아 offset 열을 채운다. 실물을 "
                       "rest(손 벌린 자세)로 둔 상태에서 누를 것.\n"
                       "⚠️ 영점은 **슬롯**의 성질인데 표에서는 채널 행에 적힌다. 그래서 "
                       "대응이 확정된 **뒤에** 눌러야 한다 — 마법사를 나중에 돌리면 영점만 "
                       "옛 대응에 남아 엉뚱한 행에 붙는다. (마법사는 시작할 때 영점을 스스로 "
                       "재서 끝에 옳은 행에 넣으므로, 마법사를 쓰면 이 버튼은 필요 없다.)"
                   ).pack(side="left", padx=(6, 0))

        b = ttk.Frame(f)
        b.grid(row=3, column=0, sticky="ew", pady=(5, 0))
        ttk.Button(b, text="적용", command=self._jm_apply).pack(side="left")
        ttk.Button(b, text="저장", command=self._jm_save).pack(side="left", padx=3)
        ttk.Button(b, text="불러오기", command=self._jm_load).pack(side="left")
        ttk.Button(b, text="항등으로 초기화", command=self._jm_reset).pack(side="left", padx=3)
        ttk.Label(b, text="전 채널 ±").pack(side="left", padx=(10, 0))
        self.jm_bulk = tk.StringVar(value="40")
        ttk.Spinbox(b, from_=5, to=130, increment=5, width=5,
                    textvariable=self.jm_bulk).pack(side="left")
        ttk.Button(b, text="클램프 채우기", command=self._jm_bulk_clamp).pack(side="left", padx=3)
        self.lbl_jmap = ttk.Label(f, text="", foreground="#666", wraplength=430)
        self.lbl_jmap.grid(row=4, column=0, sticky="w", pady=(3, 0))

        # (10) 프리셋 저장/불러오기
        f = self._panels["preset"] = ttk.Frame(ctrl)
        ttk.Button(f, text="프리셋 저장", command=self.save_preset).pack(side="left", padx=2)
        ttk.Button(f, text="프리셋 불러오기", command=self.load_preset).pack(side="left", padx=2)
        ttk.Button(f, text="채널 리셋", command=self.reset_channel).pack(side="left", padx=2)

        # 상태바 — 스크롤 밖(항상 보이는 자리)에 고정
        self.status = ttk.Label(root, text="", anchor="w", relief="sunken")
        self.status.grid(row=3, column=0, columnspan=3, sticky="ew")

        self._apply_mode()                       # 첫 모드의 패널만 배치

    # ---- 스크롤 배관 ----
    # 휠을 캔버스 스크롤로 쓰지 않고 그냥 넘길 위젯 = **자기 클래스에 <MouseWheel> 바인딩이
    # 있는 것들만**. bind_all("all" 태그)은 클래스 바인딩보다 **뒤에** 실행되므로, 여기서
    # "break"를 해도 값 변경은 이미 일어난 뒤다 → 넘기는 게 아니라 '이중 동작을 피한다'는 뜻.
    # 실측(Tk 8.6): TCombobox·TSpinbox·Listbox만 휠 바인딩이 있고 Scale/TScale은 없다.
    # ③이 슬라이더로 가득해서 예전엔 그 위에서 휠이 완전히 죽어 ⑤·⑥까지 내려갈 수가 없었다.
    _NO_WHEEL = ("TCombobox", "TSpinbox", "Listbox", "Text", "Treeview")

    def _on_content_configure(self, _e=None):
        box = self._canvas.bbox("all")
        if box is not None:                      # 위젯이 아직 없으면(초기 1프레임) bbox=None
            self._canvas.configure(scrollregion=box)

    def _on_canvas_configure(self, e):
        # 창이 내용보다 넓어지면 내부 프레임도 같이 늘려 준다(오른쪽에 빈 띠가 생기지 않게).
        inner = self._canvas.nametowidget(self._canvas.itemcget(self._main_item, "window"))
        self._canvas.itemconfigure(self._main_item, width=max(e.width, inner.winfo_reqwidth()))

    @staticmethod
    def _wheel_dir(e):
        """휠 이벤트 → -1(위로)/+1(아래로). 플랫폼 3종의 차이를 여기서 흡수한다:
             Windows : <MouseWheel>, delta = ±120
             macOS   : <MouseWheel>, delta = ±1 (트랙패드는 더 큰 값)
             X11     : <Button-4>(위)/<Button-5>(아래), **delta = 0**
           → X11에서 delta 부호만 보면 0 > 0 이 False라서 항상 아래로만 굴러간다."""
        if getattr(e, "num", None) in (4, 5):
            return -1 if e.num == 4 else 1
        return -1 if e.delta > 0 else 1

    def _on_wheel(self, e):
        if e.widget.winfo_class() in self._NO_WHEEL:
            return
        self._canvas.yview_scroll(3 * self._wheel_dir(e), "units")
        return "break"

    def _on_wheel_x(self, e):
        if e.widget.winfo_class() in self._NO_WHEEL:
            return
        self._canvas.xview_scroll(3 * self._wheel_dir(e), "units")
        return "break"

    def _on_page(self, e):
        """PgUp/PgDn/Home/End — 휠이나 스크롤바를 안 쓰고도 아래쪽(⑤·⑥)에 닿게.
        텍스트를 입력하는 중(Entry/Spinbox)에는 캐럿 이동을 방해하지 않게 넘긴다."""
        if e.widget.winfo_class() in ("TEntry", "Entry", "TSpinbox", "Spinbox", "Text"):
            return
        if e.keysym == "Prior":
            self._canvas.yview_scroll(-1, "pages")
        elif e.keysym == "Next":
            self._canvas.yview_scroll(1, "pages")
        elif e.keysym == "Home":
            self._canvas.yview_moveto(0.0)
        else:                                    # End
            self._canvas.yview_moveto(1.0)
        return "break"

    @staticmethod
    def _mono_font(size=10):
        """④ 판독용 고정폭 글꼴. 자릿수를 맞춘 포맷('{:+7.1f}')이라 비례 글꼴이면 숫자가
        흔들려 읽기 어렵다. "Consolas"는 **윈도우 전용**이므로(리눅스/맥엔 없어 비례 글꼴로
        대체됨) 설치된 것 중 앞선 것을 고르고, 하나도 없으면 Tk가 플랫폼별로 정의해 둔
        TkFixedFont(고정폭 보장)로 떨어진다."""
        have = set(tkfont.families())
        for fam in ("Consolas",              # Windows
                    "Menlo", "SF Mono",      # macOS
                    "DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono",  # Linux
                    "Courier New"):          # 3종 공통 폴백
            if fam in have:
                return tkfont.Font(family=fam, size=size)
        fnt = tkfont.nametofont("TkFixedFont").copy()
        fnt.configure(size=size)
        return fnt

    def _mk_scale(self, parent, row, label, lo, hi, res, cmd):
        ttk.Label(parent, text=label, width=16).grid(row=row, column=0, sticky="w")
        var = tk.DoubleVar()
        s = tk.Scale(parent, from_=lo, to=hi, resolution=res, orient="horizontal",
                     length=260, variable=var, command=lambda _v: cmd())
        s.grid(row=row, column=1, columnspan=3, sticky="w")
        s.var = var
        return s

    # ============================ 채널 편집 콜백 ============================
    def _cur_ch(self):
        return CH[self.ch_combo.current()]

    def _on_channel_change(self, _e=None):
        self._load_channel_into_sliders(self._cur_ch())

    def _load_channel_into_sliders(self, ch):
        self._loading = True
        hmn, hmx = get_human_range(ch)
        rlo, rhi = get_robot_range(ch)
        self.s_hmin.set(round(hmn, 3))
        self.s_hmax.set(round(hmx, 3))
        self.s_rlo.set(round(rlo, 1))
        self.s_rhi.set(round(rhi, 1))
        i = _ch_idx(ch)
        self.ov_enabled.set(i in self.overrides)
        self.s_manual.set(self.overrides.get(i, 0.0))
        self._loading = False

    def _on_human(self):
        if self._loading:
            return
        set_human_range(self._cur_ch(), self.s_hmin.var.get(), self.s_hmax.var.get())

    def _on_robot(self):
        if self._loading:
            return
        set_robot_range(self._cur_ch(), self.s_rlo.var.get(), self.s_rhi.var.get())

    def _on_override_toggle(self):
        i = _ch_idx(self._cur_ch())
        if self.ov_enabled.get():
            self.overrides[i] = self.s_manual.var.get()
        else:
            self.overrides.pop(i, None)
        self._ov_rev += 1          # 다음 _sync_settings에서 워커 스냅샷 갱신

    def _on_manual(self):
        if self._loading:
            return
        if self.ov_enabled.get():
            self.overrides[_ch_idx(self._cur_ch())] = self.s_manual.var.get()
            self._ov_rev += 1

    def reset_channel(self):
        """선택 채널을 dg5f_angles 원본 기본값으로 되돌린다(모듈 재로딩 없이 근사 복원은 어려워 안내만)."""
        messagebox.showinfo("채널 리셋",
                            "원본 기본값 복원은 프리셋 불러오기로 하거나 프로그램을 재시작하세요.\n"
                            "(현재 세션에서 바꾼 값만 프리셋에 저장됩니다.)")

    def _on_log_toggle(self):
        """메인 스레드 전용. 켜면 새 파일, 끄면 남은 큐를 비우고 닫는다."""
        if self.log_on.get():
            try:
                path = self.logger.start()
            except OSError as e:
                self.log_on.set(False)
                self.lbl_log.configure(text=f"로그 파일 열기 실패: {e}")
                messagebox.showerror("로그", f"로그 파일을 열 수 없습니다:\n{e}")
                return
            self.lbl_log.configure(text=f"기록 중 → {path}")
        else:
            self.logger.stop()
            self.lbl_log.configure(
                text=f"중지 — {self.logger.count}행 저장됨: {self.logger.path}")

    # ============================ ⑥ 보정 녹화 ============================
    def _on_calib_toggle(self):
        if self.calib.active:
            self._calib_finish()
            return
        self.calib.start()
        self.btn_calib.configure(text="■ 보정 완료·저장")
        self.lbl_calib.configure(text="녹화 중… 손이 화면에 보이는 상태로 ①~④를 반복하세요",
                                 foreground="#a33")

    def _calib_finish(self):
        """녹화 중지 → 저장 → **재시작 없이** 라이브 반영.

        calibrate_dg5f는 여기서(함수 안에서) 임포트한다 — 모듈 최상단에서 하면 cv2·mediapipe가
        딸려 와 창 뜨는 데 4.5초가 더 걸린다(07-27 개편이 통째로 무효화된다). 이 시점엔
        워커가 이미 둘 다 로드해 뒀으니 sys.modules 캐시라 사실상 0초다."""
        self.calib.stop()
        self.btn_calib.configure(text="● 보정 녹화 시작")
        import calibrate_dg5f as CAL          # 백분위·물리캡·스키마의 소유자
        n = self.calib.n
        if n < CAL.MIN_SAMPLES:
            self.lbl_calib.configure(
                text=f"손이 보인 프레임 {n}개뿐 — 저장하지 않았습니다(최소 {CAL.MIN_SAMPLES}). "
                     "카메라에 손이 잡히는지 확인하고 다시 녹화하세요.", foreground="#a33")
            return
        out, lines = CAL.build_calibration(self.calib.samples, self.calib.straight)
        if not out["human_ranges"]:
            self.lbl_calib.configure(text="저장할 채널이 없습니다 — 다시 녹화하세요.",
                                     foreground="#a33")
            return
        path = CAL.save_calibration(out)
        # 라이브 반영: dg5f_angles 전역을 ③ 슬라이더와 같은 경로로 갈아끼운다.
        for ch, rng in out["human_ranges"].items():
            if ch in CH:
                set_human_range(ch, rng["min"], rng["max"])
        A.set_thumb_straight(out.get("thumb_straight_ratio"))   # 없으면 기본값으로 복귀
        self._load_channel_into_sliders(self._cur_ch())
        self.lbl_calib.configure(
            text=f"저장 완료 — {n}프레임 → {os.path.basename(path)} (라이브 반영됨)",
            foreground="#060")
        messagebox.showinfo("보정 결과", f"{path}\n\n" + "\n".join(lines))

    def _show_calib(self):
        if not os.path.exists(CALIB_PATH):
            messagebox.showinfo("보정값", f"보정 파일이 아직 없습니다:\n{CALIB_PATH}\n\n"
                                          "⑥에서 녹화하면 만들어집니다(없어도 기본값으로 동작).")
            return
        with open(CALIB_PATH, encoding="utf-8") as fp:
            d = json.load(fp)
        hr = d.get("human_ranges", {})
        body = "\n".join(f"  {k:12s} {v['min']:7.3f} ~ {v['max']:7.3f}" for k, v in hr.items())
        messagebox.showinfo(
            "현재 보정값",
            f"{CALIB_PATH}\nv{d.get('version')} / {d.get('created')} / {len(hr)}채널\n"
            f"thumb_straight_ratio = {d.get('thumb_straight_ratio', '(없음 → 기본값)')}\n\n{body}")

    # ============================ ⑦ 실물 로봇 ============================
    @staticmethod
    def _fnum(var, default, lo, hi):
        """스핀박스 문자열 → 범위 안 float. 입력 중이라 숫자가 아니면 default를 그대로 둔다
        (타이핑 도중 Hz가 0이 되는 사고 방지 — ① IP의 inet_pton 보류와 같은 취지)."""
        try:
            v = float(var.get())
        except (ValueError, tk.TclError):
            return default
        return min(hi, max(lo, v))

    def _apply_real_params(self):
        """⑦ 스핀박스 → _RealHand. 위젯마다 콜백을 다는 대신 10Hz로 밀어 넣는다."""
        r = self.real
        r.max_step = self._fnum(self.rb_step, r.max_step, 0.0, 20.0)
        r.hz = self._fnum(self.rb_hz, r.hz, 5.0, 100.0)
        r.unmirror = self.rb_unmirror.get()
        r.autostop = self.autostop_on.get()

    def _rb_browse(self):
        path = filedialog.askopenfilename(title="DGSDK.dll 선택",
                                          filetypes=[("DLL", "*.dll"), ("모든 파일", "*.*")])
        if path:
            self.rb_dll.set(os.path.abspath(path))

    def _rb_connect(self):
        ip = self.rb_ip.get().strip()
        try:
            socket.inet_pton(socket.AF_INET, ip)
        except OSError:
            messagebox.showerror("실물 연결", "그리퍼 IP를 점4자리로 입력하세요 (예: 169.254.186.72).")
            return
        try:
            port = int(self.rb_port.get())
            if not 0 < port < 65536:
                raise ValueError
        except ValueError:
            messagebox.showerror("실물 연결", "포트가 올바르지 않습니다 (Modbus TCP 기본 502).")
            return
        dll = os.path.abspath(self.rb_dll.get().strip())
        if not os.path.exists(dll):
            messagebox.showerror("실물 연결", f"DGSDK.dll을 찾을 수 없습니다:\n{dll}")
            return
        # 콤보박스는 readonly라 손으로는 못 고르지만, 옛 프리셋이 S 계열 이름을 들고 올 수 있다.
        if self.rb_model.get() not in SDKB.SUPPORTED_MODELS:
            messagebox.showerror(
                "실물 연결",
                f"지원하지 않는 모델입니다: {self.rb_model.get()}\n\n"
                "S 계열은 손가락당 관절 수가 달라(15DOF는 3개) 이 프로그램의 20채널 계약이 "
                f"성립하지 않습니다.\n쓸 수 있는 모델: {', '.join(SDKB.SUPPORTED_MODELS)}")
            return
        self.arm_on.set(False)            # 연결은 연결일 뿐 — 구동은 사람이 따로 켠다
        self.real.armed = False
        self._estopped = False
        self._apply_real_params()
        self.real.connect(ip, port, self.rb_model.get(), dll,
                          self._fnum(self.rb_lpf, 0.3, 0.0, 0.9))
        # 연결 요청 뒤에 큐잉 — 서보 스레드가 connect를 끝낸 다음 순서로 처리한다.
        self.real.set_torque_limit(self.torque_on.get())

    def _rb_disconnect(self):
        self.arm_on.set(False)
        self.real.disconnect()

    def _rb_diagnose(self):
        if not self.real.connected:
            messagebox.showwarning("자가진단", "먼저 ⑦에서 연결하세요.")
            return
        self.arm_on.set(False)
        self.lbl_diag.configure(text="자가진단 실행 중… (Arm이 꺼집니다)", foreground="#666")
        self.real.diagnose()

    def _rb_encoder_zero(self):
        """하드웨어 엔코더 영점. **장비에 남는 설정**이라 확인을 두 번 받는다."""
        if not self.real.connected:
            messagebox.showwarning("HW 영점", "먼저 ⑦에서 연결하세요.")
            return
        if not messagebox.askokcancel(
                "하드웨어 영점", "⚠️ 지금 자세를 그리퍼의 엔코더 영점으로 굳힙니다.\n\n"
                                 "· 이 설정은 **그리퍼에 남습니다** — 프로그램을 꺼도 "
                                 "되돌아오지 않습니다\n"
                                 "· 지금 자세가 기계적 rest가 아니면 이후 모든 각도가 "
                                 "그만큼 틀어집니다\n"
                                 "· 소프트 보정만 원하면 ⑨의 '영점 자동'을 쓰세요\n\n"
                                 "계속할까요?"):
            return
        if not messagebox.askokcancel("하드웨어 영점", "정말 실행합니다. 되돌릴 수 없습니다."):
            return
        self.arm_on.set(False)
        self.real.encoder_zero()

    def _on_arm(self):
        if not self.arm_on.get():
            self.real.armed = False
            return
        if not self.real.connected:
            self.arm_on.set(False)
            messagebox.showwarning("구동 시작", "먼저 '연결'로 그리퍼에 접속하세요.")
            return
        self._apply_real_params()
        if not messagebox.askokcancel(
                "구동 시작", "지금부터 실물이 움직입니다.\n\n"
                f"· 보내는 값: {'⑧ 검증 각도(1관절)' if self.verify_on.get() else '손 추종'}\n"
                f"· 틱당 최대 {self.real.max_step:g}°씩만 접근 (슬루 리밋)\n"
                f"· 클램프는 ⑨ 대응표의 관절별 lo/hi · {self.real.hz:.0f}Hz\n"
                "· 시작 기준점은 rest(전 관절 0°)입니다 — 실물이 지금 rest 자세여야 합니다\n\n"
                "주변을 비우고, 비상정지 버튼에 손이 닿는 위치에서 진행하세요."):
            self.arm_on.set(False)
            return
        self._estopped = False
        self.real.armed = True

    def _estop(self):
        """비상정지 — 새 명령 송신을 즉시 끊는다. 실물은 마지막 명령 자세에서 멈춘다
        (SDK에 전원을 끊는 수단은 없다). 재개는 사람이 Arm을 다시 켜야만 된다."""
        self.real.armed = False
        self.arm_on.set(False)
        self._estopped = True

    # ============================ ⑨ 관절 대응표 ============================
    def _jm_collect(self):
        """표 → dg5f_sdk_bridge 형식 dict. 숫자로 못 읽는 칸은 안전한 기본값으로 떨어진다."""
        ch = {}
        for i, name in enumerate(CH):
            v_sdk, v_sgn, v_off, v_lo, v_hi = self.jm_vars[i]
            ch[name] = {
                "sdk": int(self._fnum(v_sdk, i, 0, N - 1)),
                "sign": -1.0 if v_sgn.get().strip().startswith("-") else 1.0,
                "offset": self._fnum(v_off, 0.0, -180.0, 180.0),
                "clamp": [self._fnum(v_lo, -130.0, -180.0, 180.0),
                          self._fnum(v_hi, 130.0, -180.0, 180.0)],
            }
        return {"note": "dg5f_teleop_gui ⑨ 관절 대응표", "version": SDKB.JOINT_MAP_VERSION,
                "created": time.strftime("%Y-%m-%d %H:%M"), "channels": ch}

    def _jm_show(self, d):
        """dict → 표. 파일/현재 전역을 표에 되비출 때."""
        for name, row in d.get("channels", {}).items():
            if name not in CH:
                continue
            v_sdk, v_sgn, v_off, v_lo, v_hi = self.jm_vars[CH.index(name)]
            lo, hi = row.get("clamp", (-130.0, 130.0))
            v_sdk.set(str(int(row["sdk"])))
            v_sgn.set("-1" if float(row.get("sign", 1.0)) < 0 else "+1")
            v_off.set(f"{float(row.get('offset', 0.0)):g}")
            v_lo.set(f"{float(lo):g}")
            v_hi.set(f"{float(hi):g}")

    def _jm_apply(self, quiet=False):
        """표를 검사하고 실물 경로에 반영. 검사는 여기서(동기), 실제 대입은 _RealHand가
        서보 스레드에 맡긴다 — 전역 4개를 메인 스레드가 갈아끼우면 서보가 '새 ORDER +
        옛 SIGN'인 한 틱을 내보낼 수 있다."""
        d = self._jm_collect()
        warn, built = SDKB.validate_joint_map(d)
        if built is None:
            self.lbl_jmap.configure(text="적용 안 됨 — " + "; ".join(warn), foreground="#a33")
            if not quiet:
                messagebox.showerror("관절 대응표", "\n".join(warn))
            return False
        where = self.real.apply_map(d)
        self.lbl_jmap.configure(
            text=f"적용됨 ({where})" + ("  ⚠ " + "; ".join(warn) if warn else ""),
            foreground="#060")
        return True

    def _jm_save(self):
        if not self._jm_apply(quiet=True):   # 적용되지 않는 표는 저장하지 않는다
            messagebox.showerror("관절 대응표", self.lbl_jmap.cget("text"))
            return
        path = SDKB.save_joint_map(self._jm_collect())
        self.lbl_jmap.configure(text=f"저장됨 → {os.path.basename(path)} "
                                     "(dg5f_sdk_bridge.py도 기동 시 이 파일을 읽는다)",
                                foreground="#060")

    def _jm_load(self):
        ok, msg = SDKB.load_joint_map()
        if ok:
            self._jm_show(SDKB.current_joint_map())
        self.lbl_jmap.configure(text=msg, foreground="#060" if ok else "#a33")

    def _jm_reset(self):
        self._jm_show({"channels": {n: {"sdk": i, "sign": 1.0, "offset": 0.0,
                                        "clamp": [-130.0, 130.0]}
                                    for i, n in enumerate(CH)}})
        self.lbl_jmap.configure(text="항등 매핑으로 되돌림 — [적용]을 눌러야 실물에 반영된다",
                                foreground="#666")

    def _jm_zero_from_fb(self):
        """연결 직후(첫 명령 전) 자세를 전 관절 0°로 삼아 offset 열을 채운다.

        to_sdk_frame은 `sdk = sign*ours + offset` 이므로, 우리 0이 그 자세를 뜻하게 하려면
        offset = 그 자세의 실측 각도다. 표의 offset은 **슬롯이 아니라 채널** 행에 있으므로,
        채널이 물고 있는 슬롯의 값을 넣는다.

        ⚠️ 기준은 **rest_fb(첫 명령 이전 스냅샷)**이지 지금 값이 아니다. Arm을 켜는 순간
           손은 rest(0)로 끌려가므로, 그 뒤의 값을 쓰면 방금 우리가 명령한 값을 되읽어
           영점이 항상 0으로 나온다(드라이 테스트에서 실제로 그렇게 나왔다)."""
        rest = self.real.rest_fb
        if rest is None:
            messagebox.showwarning("영점 자동", "아직 기준 자세를 못 읽었습니다.\n"
                                                "⑦에서 연결한 뒤 (Arm을 켜기 **전에**) "
                                                "잠시 기다렸다 다시 누르세요.")
            return
        if not messagebox.askokcancel(
                "영점 자동", "연결 직후(첫 명령 전) 읽은 자세를 '전 관절 0°'로 삼아 영점 열을 "
                             "채웁니다.\n\n그 시점에 실물이 rest(손 벌린 자세)였어야 합니다. "
                             "굽힌 상태로 연결했다면 그 자세가 0이 되어 이후 명령이 전부 "
                             "틀어집니다 — 그런 경우 rest로 두고 ⑦에서 해제 후 다시 연결하세요."):
            return
        for i in range(N):
            slot = int(self._fnum(self.jm_vars[i][0], i, 0, N - 1))
            self.jm_vars[i][2].set(f"{rest[slot]:.1f}")
        self.lbl_jmap.configure(text="영점을 현재 자세로 채웠다 — [적용]을 눌러야 반영된다",
                                foreground="#666")

    def _jm_wizard(self):
        """관절 탐색 마법사 — SDK 슬롯을 하나씩 움직여 보고 사람이 '무슨 관절이었나'만 답한다.

        전자동이 안 되는 이유는 _help 툴팁과 같다: 피드백도 슬롯 인덱스라 '슬롯 7이 20°'는
        알아도 '슬롯 7 = 검지 PIP'는 손을 보는 사람만 안다. 그래서 사람은 **판단만** 하고
        명령·대기·도달 확인·표 기입·되돌리기는 전부 여기서 한다."""
        if not self.real.connected:
            messagebox.showwarning("관절 탐색", "먼저 ⑦에서 실물에 연결하세요.")
            return
        if not self.real.armed:
            messagebox.showwarning("관절 탐색", "⑦에서 Arm을 켜야 실물이 움직입니다.")
            return
        _JointMapWizard(self)

    def _jm_bulk_clamp(self):
        d = abs(self._fnum(self.jm_bulk, 40.0, 5.0, 130.0))
        for _v_sdk, _v_sgn, _v_off, v_lo, v_hi in self.jm_vars:
            v_lo.set(f"{-d:g}")
            v_hi.set(f"{d:g}")
        self.lbl_jmap.configure(text=f"전 채널 클램프 ±{d:g}° — [적용]을 눌러야 반영된다",
                                foreground="#666")

    # ============================ ⑧ 관절 검증 ============================
    def _on_verify_toggle(self):
        if self.verify_on.get() and self.real.armed and not messagebox.askokcancel(
                "관절 검증 모드", "구동 중(Arm)입니다. 검증 모드로 바꾸면 나머지 19관절이 "
                "rest(0°)로 이동합니다.\n계속할까요?"):
            self.verify_on.set(False)
            return
        self._sync_settings()

    def _on_verify(self):
        self._sync_settings()      # 다음 프레임을 기다리지 않고 즉시 반영

    def _request_camera(self):
        """카메라 (재)연결 요청만 걸고 즉시 리턴 — 오픈은 캡처 스레드가 한다.
        (예전엔 이 버튼이 UI 스레드에서 VideoCapture를 열어 6~25초 프리즈였다.)"""
        self._cam_req_index = self.cam_index.get()
        self._cam_req += 1
        self.cam_status = f"cam{self._cam_req_index} 재연결 요청…"

    # ============================ 캡처 스레드 ============================
    def _capture_loop(self):
        global cv2
        import cv2 as _cv2                      # 0.6초 — UI 밖에서
        cv2 = _cv2
        self._ev_cv2.set()

        cap = None
        req = 0
        fail = 0
        t_fps, n_fps = time.perf_counter(), 0
        while not self._stop.is_set():
            if req != self._cam_req or cap is None:
                req = self._cam_req
                if cap is not None:
                    cap.release()
                    cap = None
                idx = self._cam_req_index
                self.cam_status = f"cam{idx} 여는 중…"
                t0 = time.perf_counter()
                # ⚠️ cap.set() 절대 추가하지 말 것. 2026-07-27 실측(4회 반복):
                #    FOURCC/W/H/FPS 4개를 넣으면 6.3~18.9초를 먹는데(set 하나당 2~4.3초)
                #    read 지연·해상도·fps는 넣든 안 넣든 33ms / 640x480 / 30fps로 동일했다.
                #    MSMF는 set(FOURCC, MJPG)에 False를 반환(무시)한다. 즉 순수 손해.
                #    프레임 지연도 전용 캡처 스레드가 계속 비워주므로 버퍼가 쌓이지 않는다.
                cap = (cv2.VideoCapture(idx) if CAM_BACKEND is None
                       else cv2.VideoCapture(idx, CAM_BACKEND))
                if not cap.isOpened():
                    self.cam_status = f"cam{idx} 열기 실패 — cam# 확인"
                    self.cam_fps = 0.0           # 실패 중에 옛 fps를 계속 보여주면 안 된다
                    cap.release()
                    cap = None
                    self._stop.wait(1.5)         # 실패 폭주 방지
                    continue
                self.cam_status = f"cam{idx} 연결 ({time.perf_counter() - t0:.1f}s)"
                fail = 0

            ok, frame = cap.read()
            if not ok:
                fail += 1
                if fail > 30:                    # ~1초간 계속 실패 → 재오픈
                    self.cam_status = "프레임 끊김 — 재연결 중…"
                    self.cam_fps = 0.0
                    cap.release()
                    cap = None
                    fail = 0
                else:
                    self._stop.wait(0.01)
                continue
            fail = 0
            self.frame_slot.put(frame)

            n_fps += 1
            t = time.perf_counter()
            if t - t_fps >= 1.0:
                self.cam_fps = n_fps / (t - t_fps)
                t_fps, n_fps = t, 0

        if cap is not None:
            cap.release()

    # ============================ 처리 스레드 ============================
    def _process_loop(self):
        global mp
        import mediapipe as _mp                 # 3.9초 — 창이 뜬 뒤 백그라운드에서
        mp = _mp
        hands = mp.solutions.hands.Hands(
            model_complexity=MP_MODEL_COMPLEXITY, max_num_hands=1,
            min_detection_confidence=0.6, min_tracking_confidence=0.6)
        draw_landmarks = mp.solutions.drawing_utils.draw_landmarks
        connections = mp.solutions.hands.HAND_CONNECTIONS
        self._ev_cv2.wait()                     # flip/cvtColor/resize에 cv2 필요

        seq = 0
        t_prev = None
        t_fps, n_fps = time.perf_counter(), 0
        while not self._stop.is_set():
            seq, frame = self.frame_slot.wait_new(seq, timeout=0.3)
            if frame is None:                   # 프레임이 안 온다(카메라 실패/재연결 중)
                self.proc_fps = 0.0             # 상태바가 옛 fps로 거짓말하지 않게
                t_prev = None                   # 끊긴 구간의 dt로 필터 주파수를 오염시키지 않기
                continue
            st = self._settings                 # 참조 한 번만 읽어 프레임 내내 일관되게 사용

            frame = cv2.flip(frame, 1)          # 거울 모드(보기 편의, 각도 계산 무관)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            res = hands.process(rgb)
            detected = bool(res.multi_hand_landmarks)

            # G: 필터 주파수를 실제 처리 주기에 맞춘다. 고정 30Hz로 두면 루프가 19fps일 때
            #    OneEuroFilter의 dx 추정이 틀려 추종 지연이 실제보다 커진다.
            t_now = time.perf_counter()
            if t_prev is not None:
                dt = t_now - t_prev
                if dt > 0:
                    f = min(max(1.0 / dt, 5.0), 120.0)      # 순간 튐 방어
                    self._filter_freq += 0.2 * (f - self._filter_freq)
            t_prev = t_now

            sent = None
            xyz = None                          # 로그용 — 미검출 프레임은 랜드마크가 없다
            mapped = None
            self._tx_ok = False                 # _send_packet이 실제로 보내면 True로 바꾼다
            if detected:
                lms = res.multi_hand_landmarks[0]
                draw_landmarks(frame, lms, connections)
                xyz = A.landmarks_to_xyz(lms, rgb.shape)   # rgb = 표시용 축소 전 원본
                raw = A.compute_raw(xyz)
                mapped = list(A.map_to_dg5f(raw, st.hand, st.mapmode))
                self.last_raw = list(raw)
                self.last_mapped = mapped       # 오버라이드·필터 전 값(_pack_and_send는 사본을 쓴다)
                self.calib.add(raw, xyz)        # ⑥ 보정 녹화 — 켜져 있을 때만 쌓인다

            # ⑧ 관절 검증 모드는 손·오버라이드보다 **먼저** 본다. 손이 화면에 없어도 계속
            # 보내야 하고(실물 관절 대응 확인이 목적), 나머지 19관절은 rest(0°)로 고정한다.
            if st.verify is not None:
                mapped = [0.0] * N
                mapped[st.verify[0]] = st.verify[1]
                sent = self._pack_and_send(mapped, self.last_raw, xyz, st)
            elif detected:
                sent = self._pack_and_send(mapped, self.last_raw, xyz, st)
            elif st.overrides:
                # 손 없어도 오버라이드가 있으면 중립(0)에 오버라이드만 얹어 송신(장비 단독 테스트)
                mapped = [0.0] * N
                sent = self._pack_and_send(mapped, self.last_raw, None, st)
            elif self.last_vals is not None:
                self._send_packet(self.last_vals + self.last_raw, st)   # occlusion hold
                sent = self.last_vals[:N]

            # 로그: 위 분기가 실제로 쓴 값을 그대로 남긴다(raw는 hold 시 직전 값 = 실사용값).
            # time.time()은 UTC 초 — Unity 로거와 같은 시계라야 사후 조인이 된다.
            self.logger.log(time.time(), detected, st.hand, st.mapmode, self._tx_ok,
                            xyz, self.last_raw, mapped, sent)

            # 표시용 축소는 여기서(워커) 한다 — UI 스레드는 paste만.
            # 랜드마크가 그려진 BGR을 먼저 줄이고 그 다음 색변환(작은 쪽이 싸다).
            h, w = frame.shape[:2]
            if w > DISPLAY_W:
                frame = cv2.resize(frame, (DISPLAY_W, max(1, round(h * DISPLAY_W / w))))
            disp = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.result_slot.put(_Result(disp, detected, self.last_raw, self.last_mapped, sent))

            n_fps += 1
            if t_now - t_fps >= 1.0:
                self.proc_fps = n_fps / (t_now - t_fps)
                t_fps, n_fps = t_now, 0

        hands.close()

    def _pack_and_send(self, mapped, raw, xyz, st):
        """처리 스레드 전용. 반환값 = 필터 후 20채널(= 실제 UDP로 나간 값)."""
        mapped = list(mapped)
        for i, deg in st.overrides.items():      # 수동 오버라이드 적용(검증 모드면 비어 있다)
            mapped[i] = deg
        fr = self._filter_freq
        vals_ang = [self.filters[n](v, fr) for n, v in zip(CH, mapped)]
        # 실물은 UDP와 **같은 값**을 받는다(필터 후). 여기서는 대입만 — Modbus 왕복은
        # 서보 스레드가 한다. Arm 전에도 넣어둔다: Arm을 켜는 순간 '지금 자세'부터 시작한다.
        self.real.submit(vals_ang)

        if xyz is not None:
            tip, pinch_d = A.compute_thumb_tip(xyz)
            self.pinch_on = (pinch_d < A.PINCH_OFF) if self.pinch_on else (pinch_d < A.PINCH_ON)
            ftips = A.compute_finger_tips(xyz)
            wtips = A.compute_wrist_tip_vectors(xyz)
            tip_f = [f(v, fr) for f, v in zip(self.tip_filters, tip)]
            ftips_f = [f(v, fr) for f, v in zip(self.ftip_filters, ftips)]
            wtips_f = [f(v, fr) for f, v in zip(self.wtip_filters, wtips)]
            vals = (vals_ang + tip_f + [1.0 if self.pinch_on else 0.0]
                    + [self.pinch_filter(pinch_d, fr)] + ftips_f + wtips_f)
        else:
            # 손 없는 오버라이드 송신 — 각도만 채우고 나머지는 0
            vals = vals_ang + [0.0] * 32

        self.last_vals = vals
        self._send_packet(vals + list(raw), st)
        return vals_ang

    def _send_packet(self, payload72, st):
        now = time.time()
        if now - self.last_send < 1.0 / SEND_HZ_CAP:
            return
        self.last_send = now
        try:
            pkt = struct.pack(A.PACKET_FMT, *payload72)
        except struct.error:
            return
        for ip, port in st.targets:              # 파싱은 _sync_settings에서 이미 끝냈다
            try:
                self.sock.sendto(pkt, (ip, port))
                self.pkt_count += 1
                self._tx_ok = True               # 로그 tx 열 — 이 프레임 값이 실제로 나갔다
            except OSError:
                pass

    # ============================ UI 루프 (메인 스레드) ============================
    def _sync_settings(self):
        """tk 변수를 읽어 워커용 불변 스냅샷으로 교체. **메인 스레드에서만** 호출.
        바뀐 게 없으면 아무것도 안 한다(30Hz로 돌아도 공짜)."""
        key = (self.hand.get(), self.mapmode.get(),
               self.sim_on.get(), self.sim_ip.get(), self.sim_port.get(),
               self.real_on.get(), self.real_ip.get(), self.real_port.get(),
               self._ov_rev,
               self.verify_on.get(), self.vf_combo.current(), self.s_verify.var.get())
        if key == self._settings_key:
            return
        self._settings_key = key

        targets = []
        bad = []
        for label, on, ip, port in (("sim", key[2], key[3], key[4]),
                                    ("real", key[5], key[6], key[7])):
            if not on:
                continue
            ip = ip.strip()
            try:
                p = int(port)
                if not 0 < p < 65536:
                    raise ValueError
            except ValueError:                   # 포트 입력 중(빈칸 등)
                bad.append(f"{label} 포트?")
                continue
            # ⚠️ 여기서 반드시 걸러야 한다. 점4자리가 아닌 문자열을 sendto에 넘기면 Windows가
            #    호스트명으로 보고 DNS 조회를 시도하며 **2.7초 블로킹**한다(2026-07-27 실측).
            #    IP를 한 글자씩 입력하면 '1','19','192','192.' 넷이 전부 조회로 들어가
            #    한 번 입력에 누적 10.8초 동안 송신 스레드가 멈췄다 → 미리보기가 얼어붙음.
            #    inet_pton 검사는 1~3us. 엄격 검사(inet_aton은 '192'를 0.0.0.192로 통과시켜
            #    엉뚱한 곳으로 쏘므로 쓰지 말 것).
            try:
                socket.inet_pton(socket.AF_INET, ip)
            except OSError:
                bad.append(f"{label} IP?")       # 입력 완료 전까지 송신 보류(오발신 방지)
                continue
            targets.append((ip, p))
        self._bad_targets = bad
        # ⑧ 검증 모드에서는 오버라이드를 **비워서** 넘긴다 — '한 관절만'이라는 약속이
        # 남아 있던 오버라이드 때문에 깨지면 관절 대응 확인이 무의미해진다.
        verify = (key[10], float(key[11])) if key[9] else None
        self._settings = _Settings(key[0], key[1],
                                   {} if verify else dict(self.overrides),
                                   tuple(targets), verify)

    def _ui_tick(self):
        t0 = time.perf_counter()
        self._sync_settings()

        seq, r = self.result_slot.peek()          # 절대 블로킹 없음
        if r is not None and seq != self._shown_seq:
            self._shown_seq = seq
            self._last_result = r
            self._last_new_t = time.perf_counter()
            self._blit(r.disp)
            self._ui_n += 1

        now = time.perf_counter()
        if now - self._slow_t >= 1.0 / READOUT_HZ:    # F: 판독/상태바는 10Hz면 충분
            self._slow_t = now
            self._update_readout()
            self._update_status()
        if now - self._ui_t >= 1.0:
            self.ui_fps = self._ui_n / (now - self._ui_t)
            self._ui_t, self._ui_n = now, 0

        # D: 고정 20ms를 덧붙이지 않고 '남은 시간'만 쉰다.
        left = UI_PERIOD_MS - (time.perf_counter() - t0) * 1000.0
        self.root.after(max(1, int(left)), self._ui_tick)

    def _blit(self, rgb):
        """numpy(RGB) → Tk 라벨. 매 프레임 PhotoImage를 새로 만들면 4~13ms + Tk 이미지
        객체 churn이라, 크기가 같으면 paste로 픽셀만 갈아끼운다(480x360에서 ~1.7ms)."""
        img = Image.fromarray(rgb)
        if self._photo is None or (self._photo.width(), self._photo.height()) != img.size:
            self._photo = ImageTk.PhotoImage(img)
            self.video.configure(image=self._photo, text="")
            self.video.image = self._photo       # GC 방지
        else:
            self._photo.paste(img)

    def _update_readout(self):
        i = self.ch_combo.current()
        ch = CH[i]
        r = self._last_result
        if r is None:
            self.lbl_read.configure(text=f"{JOINT_ID[i]} {ch}\n(대기 중…)")
            return
        raw = r.raw[i]
        mapped = r.mapped[i]
        sent = float("nan") if r.sent is None else r.sent[i]
        ov = f"  [OVERRIDE {self.overrides[i]:+.0f}]" if i in self.overrides else ""
        self.lbl_read.configure(
            text=f"{JOINT_ID[i]} {ch}\n"
                 f"raw   = {raw:+.4f} rad ({np.degrees(raw):+7.1f} deg)\n"
                 f"mapped= {mapped:+7.1f} deg{ov}\n"
                 f"sent  = {sent:+7.1f} deg  (필터후=UDP)")

    def _update_feedback(self):
        """그리퍼 상태 → ⑦ 요약 + ⑨ '실제°' 열. 10Hz(READOUT_HZ)로만 돈다."""
        st = self.real.fb
        if st is None:
            if self.real.connected:
                self.lbl_fb.configure(text="상태 수신 대기…", foreground="#666")
            else:
                self.lbl_fb.configure(text="상태 수신 없음", foreground="#666")
            for lb in self.jm_act:
                lb.configure(text="—")
            return
        d, op = self.real.diag, self.real.last_op
        if d is not None:
            self.lbl_diag.configure(
                text=f"자가진단: process={d['process']} step={d['step']} "
                     f"jointId={d['jointId']} period={d['period']} joint={d['joint']} "
                     f"temp={d['temperature']}  (실행={d['run']}, 읽기={d['get']}; "
                     "0=DG_RESULT_NONE)",
                foreground="#060" if d["run"] == 0 and d["get"] == 0 else "#a33")
        elif op is not None:
            self.lbl_diag.configure(
                text=f"{op[0]}: DG_RESULT={op[1]}" + (" (정상)" if op[1] == 0 else " ⚠ 실패"),
                foreground="#060" if op[1] == 0 else "#a33")
        hot = max(st["temperature"])
        amp = max(abs(c) for c in st["current"])
        self.lbl_fb.configure(
            text=f"실제각 {min(st['joint']):+6.1f}~{max(st['joint']):+6.1f}°  "
                 f"최대전류 {amp:5d}mA  최고온도 {hot:4.1f}℃\n"
                 f"moving={st['moving']} 도달={st['arrived']} 제어주기={st['period']} "
                 f"fw={st['fw']} 에러={st['err']}",
            foreground="#a33" if st["err"] else "#060")
        for i, lb in enumerate(self.jm_act):
            slot = int(self._fnum(self.jm_vars[i][0], i, 0, N - 1))
            lb.configure(text=f"{st['joint'][slot]:+.1f}")

    def _update_status(self):
        tgt = []
        if self.sim_on.get():
            tgt.append(f"sim {self.sim_ip.get()}:{self.sim_port.get()}")
        if self.real_on.get():
            tgt.append(f"real {self.real_ip.get()}:{self.real_port.get()}")
        if self._bad_targets:
            tgt.append("⚠ " + "/".join(self._bad_targets) + " 확인 — 송신 보류")
        r = self._last_result
        stall = time.perf_counter() - self._last_new_t
        if r is None:
            state = self.cam_status
        elif stall > 1.5:
            # 처리 스레드가 어딘가에 막혔다는 신호. 이 표시가 보이면 미리보기가 멈춘 게
            # UI 탓이 아니라 캡처/처리 쪽이 막힌 것 — 원인 좁히기에 쓴다.
            state = f"영상 정지 {stall:.0f}s — {self.cam_status}"
        else:
            state = "손 인식" if r.detected else "미검출(hold)"
        if self.logger.active:
            drop = f", 유실 {self.logger.dropped}" if self.logger.dropped else ""
            self.lbl_log.configure(
                text=f"기록 중 {self.logger.count}행{drop} → {os.path.basename(self.logger.path)}")
        if self.calib.active:
            self.lbl_calib.configure(
                text=f"녹화 중 {self.calib.n}프레임 — ①~④를 3회 이상 반복하고 "
                     "'보정 완료·저장'을 누르세요")

        # ⑦ 실물: 스핀박스 값을 밀어 넣고, 서보 스레드가 올린 상태를 받아 온다.
        self._apply_real_params()
        if self._estopped and not self.real.armed:
            rs = "■ 비상정지 — 마지막 자세 유지. 재개하려면 Arm을 다시 켜세요"
        else:
            rs = self.real.state
        self.lbl_real.configure(text=rs, foreground="#a33" if not self.real.connected else "#060")
        if self.arm_on.get() and not self.real.armed:
            self.arm_on.set(False)               # 서보 스레드가 오류로 내린 경우 체크박스 동기화
        self._update_feedback()

        # 툴바 표시등 — 패널을 숨겨도 '지금 살아 있는 것'은 여기서 항상 보인다.
        # 비상정지는 실물이 붙어 있는 동안 계속 눌릴 수 있어야 한다(모드와 무관).
        self.btn_estop_bar.configure(state="normal" if self.real.connected else "disabled")
        if self.real.armed:
            lamp, fg = "● ARM 실물 구동 중", "#c62828"
        elif self._estopped:
            lamp, fg = "■ 비상정지", "#c62828"
        elif self.real.connected:
            lamp, fg = "○ 실물 연결됨(대기)", "#060"
        elif self.calib.active:
            lamp, fg = f"● 보정 녹화 {self.calib.n}", "#a33"
        elif tgt and not self._bad_targets:
            lamp, fg = f"→ 송신 {len(tgt)}", "#666"
        else:
            lamp, fg = "송신 없음", "#999"
        if self.verify_on.get():
            lamp += " · VERIFY"
        self.lamp.configure(text=lamp, fg=fg)

        extra = ""
        if self.real.connected or self._estopped:
            extra += f" | real {rs.split(' —')[0].split(' ·')[0]}"
        if self.verify_on.get():
            i = self.vf_combo.current()
            extra += f" | VERIFY {JOINT_ID[i]} {self.s_verify.var.get():+.0f}°"
        self.status.configure(
            text=f"{state} | cam {self.cam_fps:4.1f} / proc {self.proc_fps:4.1f} / ui {self.ui_fps:4.1f} fps | "
                 f"filt {self._filter_freq:4.1f}Hz | pkt {self.pkt_count} | "
                 f"mode={self.mapmode.get()}/{self.hand.get()} | "
                 f"→ {', '.join(tgt) or '(대상 없음)'}{extra}")

    # ============================ 프리셋 ============================
    def _collect(self):
        return {
            "mode": self.mode.get(),
            "hand": self.hand.get(), "mapmode": self.mapmode.get(),
            "sim": [self.sim_on.get(), self.sim_ip.get(), self.sim_port.get()],
            "real": [self.real_on.get(), self.real_ip.get(), self.real_port.get()],
            "human_ranges": {ch: get_human_range(ch) for ch in CH},
            "robot_ranges": {ch: get_robot_range(ch) for ch in CH},
            "overrides": {CH[i]: v for i, v in self.overrides.items()},
            # ⑦ 실물 접속·안전 파라미터. **연결/Arm 상태는 저장하지 않는다** —
            # 프리셋을 불러오는 것만으로 실물이 움직이기 시작하면 안 된다.
            "real_sdk": {"ip": self.rb_ip.get(), "port": self.rb_port.get(),
                         "model": self.rb_model.get(), "unmirror": self.rb_unmirror.get(),
                         "dll": self.rb_dll.get(), "max_step": self.rb_step.get(),
                         "hz": self.rb_hz.get(),
                         "lpf": self.rb_lpf.get()},
        }

    def save_preset(self):
        path = filedialog.asksaveasfilename(initialfile=os.path.basename(PRESET_PATH),
                                            initialdir=DATA_DIR, defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(self._collect(), fp, ensure_ascii=False, indent=2)
        messagebox.showinfo("저장", f"프리셋 저장됨:\n{path}")

    def load_preset(self):
        path = filedialog.askopenfilename(initialdir=DATA_DIR, filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, encoding="utf-8") as fp:
            d = json.load(fp)
        self.hand.set(d.get("hand", "right"))
        self.mapmode.set(d.get("mapmode", "ratio"))
        for key, on_v, ip_v, port_v in (("sim", self.sim_on, self.sim_ip, self.sim_port),
                                        ("real", self.real_on, self.real_ip, self.real_port)):
            if key in d:
                on, ip, port = d[key]
                on_v.set(on); ip_v.set(ip); port_v.set(str(port))
        for ch, (lo, hi) in d.get("human_ranges", {}).items():
            if ch in CH:
                set_human_range(ch, lo, hi)
        for ch, (lo, hi) in d.get("robot_ranges", {}).items():
            if ch in CH:
                set_robot_range(ch, lo, hi)
        self.overrides = {_ch_idx(ch): v for ch, v in d.get("overrides", {}).items() if ch in CH}
        rs = d.get("real_sdk", {})               # 옛 프리셋엔 없다 → 현재 값 유지
        for k, var in (("ip", self.rb_ip), ("port", self.rb_port), ("model", self.rb_model),
                       ("dll", self.rb_dll), ("max_step", self.rb_step), ("hz", self.rb_hz),
                       ("lpf", self.rb_lpf)):
            if k in rs:
                var.set(str(rs[k]))
        if self.rb_model.get() not in SDKB.SUPPORTED_MODELS:
            # S 계열을 담아 둔 옛 프리셋 — 조용히 두면 연결할 때야 막히므로 지금 되돌린다.
            bad, fallback = self.rb_model.get(), SDKB.SUPPORTED_MODELS[0]
            self.rb_model.set(fallback)
            messagebox.showwarning("프리셋", f"프리셋의 모델 '{bad}'은 지원하지 않아 "
                                             f"{fallback}로 되돌렸습니다.")
        if "unmirror" in rs:
            self.rb_unmirror.set(bool(rs["unmirror"]))
        self._ov_rev += 1
        if d.get("mode") in self.MODES:
            self.mode.set(d["mode"])
            self._on_mode()
        self._load_channel_into_sliders(self._cur_ch())
        self._sync_settings()
        messagebox.showinfo("불러오기", f"프리셋 적용됨:\n{path}")

    # ============================ 종료 ============================
    def on_close(self):
        self._stop.set()
        for th in (self._th_cap, self._th_proc):
            # 무거운 임포트/카메라 오픈 중이면 안 끝날 수 있다 → daemon이라 프로세스 종료를 막지 않음
            if th.is_alive():
                th.join(timeout=1.5)
        # 처리 스레드가 멈춘 뒤에 실물을 내린다(더 이상 새 목표가 들어오지 않는 상태에서
        # SystemStop → Disconnect). daemon 스레드에 맡기고 그냥 죽으면 그리퍼가 DEVELOPER
        # 모드에 물린 채로 남아 다음 연결이 실패할 수 있다.
        self.real.shutdown()
        # 처리 스레드를 먼저 세운 뒤 로거를 닫아야 남은 큐가 유실 없이 파일로 나간다.
        self.logger.stop()
        try:
            self.sock.close()
        finally:
            self.root.destroy()


def main():
    # 리눅스: X를 건드릴 수 있는 임포트를 tk.Tk() 전에 메인 스레드에서 끝낸다(위 주석 참조).
    if PRELOAD_HEAVY:
        print("[dg5f_gui] X11 스레드 안전을 위해 cv2·mediapipe를 메인 스레드에서 먼저 로드합니다"
              " (DG5F_PRELOAD=0 으로 끌 수 있음)…", flush=True)
        print(f"[dg5f_gui] 로드 완료 {_preload_heavy():.1f}s — 창을 띄웁니다", flush=True)

    root = tk.Tk()
    gui = TeleopGUI(root)
    # Canvas는 내용 크기를 따라가지 않으므로(기본 378x265) 창 크기를 직접 잡는다.
    # 높이는 DEFAULT_H 고정 — 모드를 바꿔도 창은 그대로 있고 안쪽만 스크롤된다.
    # (최소 크기도 그 안에서 함께 정한다.)
    gui._set_default_geometry()
    root.mainloop()


if __name__ == "__main__":
    main()
