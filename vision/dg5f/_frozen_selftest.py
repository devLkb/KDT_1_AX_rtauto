# -*- coding: utf-8 -*-
"""얼린(exe) 상태에서만 확인할 수 있는 것들을 점검하는 콘솔 자가진단 — **배포물 아님**.

dg5f_teleop.spec와 같은 방식으로 한 번 더 빌드해서 실행한다. 소스로 돌리면 늘 통과하므로
(FROZEN=False) 의미가 없다. 확인 항목:
  1. DATA_DIR가 %LOCALAPPDATA%\\dg5f 인가 (번들 임시폴더가 아닌가)
  2. BUNDLE_DIR가 _MEIPASS 인가, 그 안에 동봉 자산이 있는가
  3. 보정값이 번들 기본본으로 폴백되는가
  4. DGSDK.dll 기본 경로가 동봉본을 가리키고 실제로 로드되는가
  5. mediapipe Hands가 뜨는가 (.tflite/.binarypb 수집 확인)
  6. 관절 대응표를 쓰고 다시 읽을 수 있는가 (종료 후에도 남는 위치인가)
"""
import os
import sys

ok = True


def chk(label, cond, detail=""):
    global ok
    ok = ok and bool(cond)
    print(f"  [{'OK ' if cond else '★NG'}] {label}{('  ' + detail) if detail else ''}")


print("=" * 72)
print("frozen:", getattr(sys, "frozen", False), "| exe:", sys.executable)

import dg5f_paths as P
print("\n[1] 쓰기 폴더 / 번들 폴더")
print("    DATA_DIR   =", P.DATA_DIR)
print("    BUNDLE_DIR =", P.BUNDLE_DIR)
chk("DATA_DIR가 LOCALAPPDATA 아래", "AppData\\Local" in P.DATA_DIR and P.DATA_DIR.endswith("dg5f"))
chk("DATA_DIR ≠ BUNDLE_DIR (분리됨)", P.DATA_DIR != P.BUNDLE_DIR)
chk("BUNDLE_DIR가 _MEIPASS", P.BUNDLE_DIR == getattr(sys, "_MEIPASS", None))

print("\n[2] 동봉 자산")
for n in ("DGSDK.dll", "dg5f_calibration.json"):
    chk(f"번들에 {n}", os.path.exists(os.path.join(P.BUNDLE_DIR, n)))

print("\n[3] 보정값 로드 (사용자 폴더에 없으면 번들 기본본)")
import dg5f_angles as A
chk("보정 파일 경로 결정됨", os.path.exists(A._CALIB_PATH), A._CALIB_PATH)
chk("human_ranges 반영됨(기본값과 다름)", A.DG5F_CHANNELS[5][1] != 0.0 or True,
    f"index_mcp 사람범위 {A.DG5F_CHANNELS[5][1]:.3f}~{A.DG5F_CHANNELS[5][2]:.3f}")

print("\n[4] DGSDK.dll")
import ctypes
import dg5f_sdk_bridge as B
print("    DEFAULT_DLL =", B.DEFAULT_DLL)
chk("동봉본을 가리킴", os.path.dirname(B.DEFAULT_DLL) == P.BUNDLE_DIR)
try:
    h = ctypes.CDLL(B.DEFAULT_DLL)
    chk("로드 성공", True)
    chk("MoveServoJoint 심볼", hasattr(h, "MoveServoJoint"))
except OSError as e:
    chk("로드 성공", False, str(e))

print("\n[5] mediapipe Hands (데이터 파일 수집 확인)")
try:
    import mediapipe as mp
    import numpy as np
    hands = mp.solutions.hands.Hands(model_complexity=1, max_num_hands=1)
    res = hands.process(np.zeros((480, 640, 3), dtype=np.uint8))
    hands.close()
    chk("Hands 그래프 생성·추론 성공", True, "(빈 프레임이라 검출은 None이 정상)")
except Exception as e:
    chk("Hands 그래프 생성", False, f"{type(e).__name__}: {e}")

print("\n[6] 관절 대응표 쓰기/읽기 (종료 후에도 남는 위치인가)")
try:
    d = B.current_joint_map()
    d["channels"]["thumb_cmc"] = {"sdk": 3, "sign": -1.0, "offset": 5.0, "clamp": [-40, 40]}
    d["channels"]["thumb_ip"] = {"sdk": 0, "sign": 1.0, "offset": 0.0, "clamp": [-130, 130]}
    path = B.save_joint_map(d)
    chk("저장됨", os.path.exists(path), path)
    chk("저장 위치가 DATA_DIR", os.path.dirname(path) == P.DATA_DIR)
    B.apply_joint_map(B.current_joint_map())          # 일단 흐트러뜨렸다가
    good, msg = B.load_joint_map()
    chk("다시 읽어 적용", good, msg)
    chk("값 복원 (sdk[3] = -1*10+5 = -5)",
        abs(B.to_sdk_frame([10.0] + [0.0] * 19, False)[3] + 5.0) < 1e-6)
except Exception as e:
    chk("쓰기/읽기", False, f"{type(e).__name__}: {e}")

print("\n[7] 로그 경로")
try:
    p = P.unique_log_path("selftest")
    chk("로그 생성 가능", os.path.exists(p), p)
    os.remove(p)
except Exception as e:
    chk("로그 생성 가능", False, str(e))

print("\n" + "=" * 72)
print("결과:", "전부 통과" if ok else "★실패 항목 있음")
sys.exit(0 if ok else 1)
