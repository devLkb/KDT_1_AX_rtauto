# -*- mode: python ; coding: utf-8 -*-
"""dg5f_teleop_gui.py → 단일 exe (PyInstaller 6.x, Windows x64).

빌드:  <vision venv python> -m PyInstaller dg5f_teleop.spec --noconfirm
결과:  dist/dg5f_teleop.exe   (onefile — 받는 사람은 이 파일 하나만 있으면 된다)

동봉하는 것(BUNDLE_DIR = sys._MEIPASS 로 풀린다):
  DGSDK.dll              실물 구동용. 동반 DLL 없이 단독 로드되는 것을 확인하고 넣었다
                         (2026-07-31 실측). dg5f_sdk_bridge._find_dll()이 여기부터 찾는다.
  dg5f_calibration.json  기본 보정값. 처음 실행한 사람이 ⑥으로 자기 손을 재기 전까지 쓴다
                         (dg5f_paths.read_path 폴백). ⑥으로 저장하면 그때부터 사용자
                         폴더(%LOCALAPPDATA%\\dg5f) 쪽이 이긴다.
⚠️ 사용자가 만들어 내는 것(보정·관절 대응표·로그·프리셋)은 여기 넣지 않는다. 번들은
   읽기 전용이고 onefile은 종료 시 지워진다 — 쓰기는 전부 DATA_DIR로 간다.

⚠️ mediapipe는 collect_all이 필수다. hand_landmark_full.tflite(5.2MB)·
   hand_landmark_tracking_cpu.binarypb 같은 **데이터 파일을 PyInstaller가 자동으로
   찾지 못한다.** 빠뜨리면 창은 뜨고 카메라도 열리는데 손만 안 잡히는, 원인 찾기
   어려운 형태로 실패한다.
"""
import os
from PyInstaller.utils.hooks import collect_all

HERE = os.path.abspath(os.getcwd())
SDK_DLL = os.path.abspath(os.path.join(
    HERE, "..", "태슬로sdk", "DGSDKSample_ver_2_0_1", "DGSDK", "DGSDK.dll"))

datas, binaries, hiddenimports = [], [], []
for pkg in ("mediapipe", "google.protobuf"):
    d, b, h = collect_all(pkg)
    datas += d
    binaries += b
    hiddenimports += h

if os.path.exists(SDK_DLL):
    binaries.append((SDK_DLL, "."))
else:
    print(f"[spec] ⚠️ DGSDK.dll 없음 — 실물 연결은 ⑦에서 직접 경로를 잡아야 한다: {SDK_DLL}")

if os.path.exists(os.path.join(HERE, "dg5f_calibration.json")):
    datas.append((os.path.join(HERE, "dg5f_calibration.json"), "."))

a = Analysis(
    ["dg5f_teleop_gui.py"],
    pathex=[HERE],
    binaries=binaries,
    datas=datas,
    # 워커 스레드에서 늦게 임포트하는 것들(cv2/mediapipe)과, 문자열로만 참조되는
    # calibrate_dg5f(⑥에서 함수 안에서 임포트)를 명시한다 — 정적 분석으로는 안 잡힌다.
    hiddenimports=hiddenimports + ["cv2", "mediapipe", "calibrate_dg5f",
                                   "dg5f_angles", "dg5f_sdk_bridge", "dg5f_paths",
                                   "one_euro_filter", "PIL.ImageTk"],
    hookspath=[],
    runtime_hooks=[],
    # 안 쓰는 무거운 것들을 빼서 크기를 줄인다. matplotlib은 mediapipe.drawing_utils가
    # 임포트하지만 우리는 pyplot을 쓰지 않는다 — 다만 임포트 자체는 일어나므로 제외하면
    # 안 된다(MPLBACKEND=Agg로 GUI 백엔드만 막아 둔 상태다).
    excludes=["scipy", "pandas", "notebook", "IPython", "pytest", "PyQt5", "PySide2"],
    noarchive=False,
)
pyz = PYZ(a.pure)

exe = EXE(
    pyz, a.scripts, a.binaries, a.datas, [],
    name="dg5f_teleop",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,                 # UPX는 DLL 압축 중 오작동 사례가 있어 끈다
    runtime_tmpdir=None,
    console=False,             # GUI 앱 — 콘솔 창을 띄우지 않는다
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)
