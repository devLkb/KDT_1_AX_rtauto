# -*- coding: utf-8 -*-
"""dg5f 스크립트 공용 경로 규칙 — 로그·보정 파일이 서로 어긋나거나 덮어써지지 않게 한곳에 모음.

왜 이 파일이 있나 (2026-07-16):
  ① **덮어쓰기 사고**: 로그 파일명이 분 단위(%H%M)라 같은 분에 두 번 실행하면 뒤가 앞을
     소리 없이 지웠다. 7/6에 이미 한 번 로그를 통째로 잃고 "실행마다 새 파일"로 고쳤는데
     분 단위까지만 고친 탓에 함정이 남아 있었다. → 초 단위 + 그래도 겹치면 접미사.
  ② **저장/로드 경로 불일치**: calibrate는 CWD 상대(`open("dg5f_calibration.json","w")`)로
     저장하는데 dg5f_angles는 스크립트 기준 절대경로로 읽었다. dg5f 폴더 안에서 실행하면
     우연히 맞지만 밖에서 실행하면 보정이 딴 데 저장되고 로드는 못 찾는다.
     → 저장·로드가 이 모듈의 **같은 상수**를 쓴다.

  ③ **exe(PyInstaller)에서 저장이 증발하던 문제** (2026-07-31): 경로를 전부
     `os.path.dirname(__file__)` 기준으로 잡았는데, 얼린 앱에서 그건 번들 압축해제
     폴더(sys._MEIPASS)다. onefile은 종료할 때 그 폴더를 통째로 지우므로 보정값·관절
     대응표·로그가 다음 실행 때 사라진다(onedir이면 남지만 Program Files 설치 시
     쓰기 권한이 없다). → **읽기 전용 번들(BUNDLE_DIR)과 쓰기 가능 데이터(DATA_DIR)를
     분리**한다. 소스로 실행할 땐 둘 다 스크립트 폴더라 예전과 동작이 같다.

⚠️ 새 로그를 추가할 땐 반드시 unique_log_path()를 쓸 것. 직접 strftime + open(...,"w") 금지.
⚠️ 쓰기 전에는 ensure_data_dir(), 읽을 때는 read_path()를 통과시킬 것.
"""
import os
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
FROZEN = getattr(sys, "frozen", False)


def _utf8_console():
    """콘솔 인코딩을 UTF-8로 고정한다. **여기 있는 이유**: 이 모듈이 dg5f 스크립트들의
    공통 임포트 뿌리라서, dg5f_angles가 임포트 시점에 찍는 보정 로드 메시지보다 먼저 돈다.

    2026-07-31 exe 자가진단에서 잡힌 실제 크래시:
        UnicodeEncodeError: 'cp949' codec can't encode character '\\u2014'
        (dg5f_angles._load_calibration의 "보정 범위 폭 0 — 기본값 유지" 출력)
    한글 윈도우의 기본 콘솔 코드페이지가 cp949라 em대시(—)를 못 찍고 **임포트 단계에서
    프로세스가 죽는다.** exe만의 문제가 아니라 cmd.exe에서 소스를 돌려도 같다.
    errors="replace"까지 주는 건, 인코딩을 못 바꾸는 환경에서도 출력 하나 때문에
    죽지는 않게 하기 위함이다."""
    for s in (sys.stdout, sys.stderr):
        if s is None:                    # 창 모드(console=False) exe — print는 무해한 no-op
            continue
        try:
            s.reconfigure(encoding="utf-8", errors="replace")
        except (AttributeError, OSError, ValueError):
            pass                         # 재설정 불가한 스트림 — 그대로 두고 넘어간다


_utf8_console()

# 번들에 **동봉되어 배포되는** 읽기 전용 자원 (기본 보정값, DGSDK.dll 등).
# PyInstaller가 압축을 푸는 곳 = sys._MEIPASS. 소스 실행 시엔 스크립트 폴더.
BUNDLE_DIR = getattr(sys, "_MEIPASS", _HERE)

# 사용자가 만들어 내는 것(보정·관절 대응표·로그·프리셋)이 쌓이는 **쓰기 가능** 폴더.
# exe일 때 실행파일 폴더를 쓰지 않는 이유: C:\Program Files\ 에 설치하면 관리자 권한
# 없이는 못 쓴다. LOCALAPPDATA는 설치 위치와 무관하게 항상 쓸 수 있다.
DATA_DIR = (os.path.join(os.environ.get("LOCALAPPDATA") or os.path.expanduser("~"), "dg5f")
            if FROZEN else _HERE)

# 로그 — 실행 CWD와 무관 (어디서 실행하든 같은 곳에 쌓인다)
LOG_DIR = os.path.join(DATA_DIR, "logs")

# 보정 파일: calibrate_dg5f.py(저장)와 dg5f_angles.py(로드)가 **이 상수 하나**를 공유
CALIB_PATH = os.path.join(DATA_DIR, "dg5f_calibration.json")

# 실물 관절 대응표(우리 채널 ↔ SDK 슬롯·부호·영점·클램프).
# dg5f_teleop_gui의 ⑨ 표가 저장하고, dg5f_sdk_bridge가 기동 시 로드한다 — GUI로 확정한
# 대응이 별도 브리지 프로세스에도 그대로 먹어야 하므로 **같은 상수 하나**를 공유한다
# (여기가 갈라지면 "GUI에선 맞는데 브리지로 돌리면 딴 관절이 움직인다"가 된다).
JOINT_MAP_PATH = os.path.join(DATA_DIR, "dg5f_joint_map.json")

# GUI 프리셋도 같은 규칙 — 예전엔 dg5f_teleop_gui._base_dir()가 따로 계산했다.
PRESET_PATH = os.path.join(DATA_DIR, "dg5f_gui_preset.json")


def ensure_data_dir():
    """쓰기 직전에 부른다. LOCALAPPDATA\\dg5f 는 처음 실행 때 없다."""
    os.makedirs(DATA_DIR, exist_ok=True)
    return DATA_DIR


def read_path(path):
    """읽을 때 쓰는 경로. 사용자 폴더에 없으면 **번들에 동봉된 기본본**으로 떨어진다.

    exe를 갓 설치한 사람은 LOCALAPPDATA에 아무것도 없다. 그때 보정 파일이 없다고
    기본 상수로 도는 것보다, 동봉해 둔 기본 보정값을 읽는 편이 낫다(사용자가 ⑥으로
    자기 손을 재면 그때부터는 사용자 폴더 쪽이 이긴다)."""
    if os.path.exists(path) or BUNDLE_DIR == DATA_DIR:
        return path
    bundled = os.path.join(BUNDLE_DIR, os.path.basename(path))
    return bundled if os.path.exists(bundled) else path


def unique_log_path(prefix, ext=".csv", log_dir=None):
    """logs/<prefix>_<YYYYMMDD_HHMMSS><ext> — 이미 있으면 _2, _3… 을 붙여 **절대 덮지 않는다**.

    초 단위라 실질적으로 겹치지 않지만, 겹쳤을 때 조용히 지우는 것보다 파일이 하나 더
    생기는 편이 낫다(로그는 지워지면 복구 불가, 늘어나는 건 나중에 지우면 그만).
    """
    d = log_dir or LOG_DIR
    os.makedirs(d, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    n = 1
    while True:
        name = f"{prefix}_{stamp}{ext}" if n == 1 else f"{prefix}_{stamp}_{n}{ext}"
        path = os.path.join(d, name)
        try:
            # 반환 즉시 자리 선점 — 경로만 계산해 돌려주면 호출자가 open하기 전에 다른
            # 프로세스가 같은 이름을 채갈 수 있다. 'x'(배타 생성)로 원자적으로 예약한다.
            open(path, "x", encoding="utf-8").close()
            return path
        except FileExistsError:
            n += 1
