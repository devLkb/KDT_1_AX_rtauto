# -*- coding: utf-8 -*-
"""실물 Tesollo DG-5F SDK 브리지 — vision_node UDP 패킷 [0..19] 관절각[deg]을
DGSDK.dll(ctypes)로 실물 그리퍼에 중계한다.

구조 (Unity 트윈과 실물을 같은 스트림으로 동시 구동):
  vision_node_dg5f.py [left|right] --bridge
     ├→ Unity Dg5fReceiver (127.0.0.1:5006, 트윈)
     └→ 이 브리지 (127.0.0.1:5007) → DGSDK.dll → 실물 (Modbus TCP :502, DEVELOPER 모드)

⚠️ 대상 모델은 **DG-5F 왼손/오른손(5f_left, 5f_right)뿐**이다. S 계열(5f_s_*, 5f_s15_*)은
   손가락당 관절 수가 달라(15DOF는 3개) 이 파일이 전제하는 20채널 계약이 성립하지 않는다.
   근거와 판단은 SUPPORTED_MODELS 주석 참조. 검증은 왼손 기준이고 오른손은 일부만 했다.

SDK 근거 (태슬로sdk/DGSDKSample_ver_2_0_1, 2026-07-20 확인):
  - MAX_JOINT_COUNT=20 (5손가락×4관절), 각도 단위 degrees — 우리 20채널과 1:1
  - 초기화 순서: SetGripperSystem → ConnectToGripper → SetGripperOption → SystemStart
  - 실시간 구동: MoveServoJoint(float[20]) — DEVELOPER 모드 전용, 모션타임 무시
  - 구조체 레이아웃: DGDataTypes.h (GripperSystemSetting/GripperSetting) 그대로 ctypes 매핑

사용:
  python dg5f_sdk_bridge.py                          # 드라이런 — DLL 안 씀, 수신값만 출력(패킷 경로 검증)
  python dg5f_sdk_bridge.py --ip 169.254.186.72      # 실물 연결 (기본 모델 5f_left)
  python dg5f_sdk_bridge.py --ip <IP> --model 5f_right --unmirror
      --unmirror: vision_node를 left로 돌리면서(왼손 Unity 트윈) 실물이 오른손일 때 —
                  왼손 미러 채널 부호를 되돌려 오른손 규약으로 변환
  종료: Ctrl+C (SystemStop + Disconnect 자동)

⚠️ 첫 실물 구동 전 필수 확인 (모르면 움직이지 말 것):
  1. JOINT_ORDER/JOINT_SIGN/JOINT_OFFSET_DEG — 우리 채널(엄지1_1..새끼5_4, URDF 기준)과
     실물 관절 번호·방향·영점 대응은 **미검증**. --pose 로 한 관절씩 살살 보내며 확정할 것.
  2. 처음엔 --max-step 을 작게(기본 2°/틱) + 손 벌린 rest 자세에서 시작.
"""
import argparse
import ctypes
import json
import os
import socket
import struct
import sys
import time

from dg5f_paths import JOINT_MAP_PATH, BUNDLE_DIR, ensure_data_dir, read_path

# ---------------- 우리 패킷 계약 (dg5f_angles와 동일) ----------------
N_JOINTS = 20
MIN_PACKET_BYTES = 4 * N_JOINTS          # v1(<20f>) 이상이면 앞 20f만 사용 (수신기 관례와 동일)
# 왼손 스트림 → 오른손 실물 변환(--unmirror)용. dg5f_angles.LEFT_MIRROR_CHANNELS와 같은 내용을
# 채널 인덱스로 고정 (임포트하면 보정 로드 출력이 섞여서 상수로 복사; 채널 순서 변경 시 함께 수정).
CHANNEL_NAMES = [
    "thumb_cmc", "thumb_opp", "thumb_mcp", "thumb_ip",
    "index_abd", "index_mcp", "index_pip", "index_dip",
    "middle_abd", "middle_mcp", "middle_pip", "middle_dip",
    "ring_abd", "ring_mcp", "ring_pip", "ring_dip",
    "pinky_cmc", "pinky_lat", "pinky_mcp", "pinky_pip",
]
MIRROR_IDX = [0, 1, 2, 3, 4, 8, 12, 16, 17]   # LEFT_MIRROR_CHANNELS 해당 인덱스

# ---------------- 우리 채널 → SDK float[20] 대응 ----------------
# SDK 배열은 손가락당 4개 × 5그룹(F1..F5, 샘플 MoveJointFinger 참조). F1=엄지로 가정
# (DG_GRASP_MODE_5F_2FINGER_1_AND_2 = 엄지+검지 핀치 → F1이 엄지) — 우리 순서와 같으면 항등.
#
# ⚠️ 아래는 **실물 검증 전 기본값(추정)**이다. 확정은 dg5f_teleop_gui.py의 ⑨ 관절 대응표에서
#    ⑧ 관절 검증 모드로 한 관절씩 돌려 보며 하고, 저장하면 JOINT_MAP_PATH에 기록된다.
#    이 모듈은 기동 시 그 파일을 자동으로 읽는다(load_joint_map) — 그래서 GUI로 확정한 대응이
#    별도 브리지 프로세스에도 그대로 먹는다. **소스를 직접 고칠 필요는 없다.**
JOINT_ORDER = list(range(20))            # sdk[i] = ours[JOINT_ORDER[i]]
JOINT_SIGN = [1.0] * 20                  # 방향 반대 관절은 -1로
JOINT_OFFSET_DEG = [0.0] * 20            # 영점 차이는 여기로 (sdk = sign*ours + offset)
# 안전 클램프 — URDF 리밋보다 넉넉하되 물리 밖 금지. 실물 리밋 확인 후 좁힐 것.
JOINT_CLAMP = [(-130.0, 130.0)] * 20

JOINT_MAP_VERSION = 1


def current_joint_map():
    """지금 전역 4개 → 저장/편집용 dict. **우리 채널 이름**을 키로 쓴다(채널 순서가 바뀌어도
    안전하게 붙도록). sdk = 그 채널이 몰고 가는 SDK 슬롯 번호."""
    ch = {}
    for slot in range(N_JOINTS):
        name = CHANNEL_NAMES[JOINT_ORDER[slot]]
        lo, hi = JOINT_CLAMP[slot]
        ch[name] = {"sdk": slot, "sign": float(JOINT_SIGN[slot]),
                    "offset": float(JOINT_OFFSET_DEG[slot]),
                    "clamp": [float(lo), float(hi)]}
    return {"note": "dg5f_teleop_gui ⑨ 관절 대응표가 저장 — dg5f_sdk_bridge가 기동 시 읽음.",
            "version": JOINT_MAP_VERSION, "created": time.strftime("%Y-%m-%d %H:%M"),
            "channels": ch}


def validate_joint_map(d):
    """dict → (경고 목록, 전역에 넣을 4-튜플 or None). **아무것도 바꾸지 않는다.**

    sdk 슬롯이 0..19의 순열이 아니면(중복/누락) 어떤 실물 관절은 명령을 아예 못 받으므로
    쓸 수 없는 표로 보고 None을 돌려준다 — 그런 표로 실물을 돌리면 '어떤 손가락은 안
    움직인다'가 되어 관절 확인 자체가 불가능해진다."""
    ch = d.get("channels", {})
    slots, order = {}, [None] * N_JOINTS
    warn = []
    for name, row in ch.items():
        if name not in CHANNEL_NAMES:
            warn.append(f"알 수 없는 채널 '{name}' — 무시")
            continue
        s = int(row["sdk"])
        if not 0 <= s < N_JOINTS:
            warn.append(f"{name}: SDK 슬롯 {s}가 0~{N_JOINTS - 1} 밖")
            continue
        if s in slots:
            warn.append(f"SDK 슬롯 {s} 중복 — {slots[s]} / {name}")
            continue
        slots[s] = name
        order[s] = CHANNEL_NAMES.index(name)
    missing = [s for s in range(N_JOINTS) if order[s] is None]
    if missing:
        warn.append(f"SDK 슬롯 {missing} 에 물린 채널이 없음 — 그 관절은 명령을 못 받는다")
        return warn, None
    sign = [1.0] * N_JOINTS
    off = [0.0] * N_JOINTS
    clamp = [(-130.0, 130.0)] * N_JOINTS
    for s, name in slots.items():
        row = ch[name]
        sign[s] = float(row.get("sign", 1.0))
        off[s] = float(row.get("offset", 0.0))
        lo, hi = row.get("clamp", (-130.0, 130.0))
        clamp[s] = (min(float(lo), float(hi)), max(float(lo), float(hi)))
    return warn, (order, sign, off, clamp)


def apply_joint_map(d):
    """dict → 전역 4개. 리스트를 **다 만든 뒤 한꺼번에 대입**한다 — 원소를 하나씩 고치면
    서보 스레드가 절반만 바뀐 표로 한 틱을 보낼 수 있다. 반환값 = 경고 문자열 목록
    (경고가 있으면 적용하지 않았다는 뜻)."""
    warn, built = validate_joint_map(d)
    if built is None:
        return warn
    global JOINT_ORDER, JOINT_SIGN, JOINT_OFFSET_DEG, JOINT_CLAMP
    JOINT_ORDER, JOINT_SIGN, JOINT_OFFSET_DEG, JOINT_CLAMP = built
    return warn


def load_joint_map(path=JOINT_MAP_PATH):
    """파일이 있으면 읽어 적용. 반환 (적용됨?, 메시지). 없으면 기본값(항등) 그대로 간다.
    exe에서는 사용자 폴더에 없을 때 번들 동봉본으로 떨어진다(read_path)."""
    path = read_path(path)
    if not os.path.exists(path):
        return False, f"관절 대응표 없음 — 기본값(항등 매핑, 부호 +1) 사용: {path}"
    try:
        with open(path, "r", encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError) as e:
        return False, f"관절 대응표 읽기 실패({e}) — 기본값 사용"
    warn = apply_joint_map(d)
    if warn:
        return False, "관절 대응표 부적합 — 기본값 유지: " + "; ".join(warn)
    return True, f"관절 대응표 적용: {path} ({d.get('created', '?')})"


def save_joint_map(d, path=JOINT_MAP_PATH):
    ensure_data_dir()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(d, f, ensure_ascii=False, indent=2)
    return path

# ---------------- SDK ctypes 바인딩 (DGDataTypes.h 레이아웃 그대로) ----------------
_SRC_DLL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..",
                        "태슬로sdk", "DGSDKSample_ver_2_0_1", "DGSDK", "DGSDK.dll")


def _find_dll():
    """DGSDK.dll 기본 경로. **exe에 동봉한 것 → exe 옆 → 소스 트리** 순으로 찾는다.

    얼린 앱에서 소스 상대경로(../태슬로sdk/...)는 존재하지 않는다 — 예전엔 그 경로를
    그대로 기본값으로 내서, 받은 사람이 [찾기]로 직접 잡아 주지 않으면 연결이 안 됐다.
    DLL은 동반 DLL 없이 단독 로드되는 것을 확인했으므로(2026-07-31) 번들에 넣는다.
    못 찾으면 소스 경로를 그대로 돌려준다 — 사용자에게 '여기 없다'고 보여 주는 편이
    빈 칸보다 낫다."""
    here = os.path.dirname(os.path.abspath(sys.executable))
    for p in (os.path.join(BUNDLE_DIR, "DGSDK.dll"),     # exe에 동봉
              os.path.join(here, "DGSDK.dll"),           # exe 옆에 둔 경우
              _SRC_DLL):                                 # 소스 트리에서 실행
        if os.path.exists(p):
            return os.path.abspath(p)
    return os.path.abspath(_SRC_DLL)


DEFAULT_DLL = _find_dll()
DG_RESULT_NONE = 0
DG_RESULT_DIAGNOSING_SYSTEM = 20      # GetReceivedGripperData가 진단 모드 중일 때 돌려주는 값
CONTROL_MODE_DEVELOPER = 1
COMMUNICATION_MODE_ETHERNET = 0
# DGDataTypes.h DEVELOPER_MODE_RECEIVED_DATA_TYPE (JOINT=1부터 1씩)
RX_JOINT, RX_CURRENT, RX_TEMPERATURE, RX_VELOCITY = 0x01, 0x02, 0x03, 0x04
RX_FT_SENSOR, RX_GPIO, RX_MODULE_ERROR, RX_CONTROL_PERIOD = 0x05, 0x06, 0x07, 0x08
DEVELOPER_MODE_RECEIVED_DATA_TYPE_JOINT = RX_JOINT      # 옛 이름 유지(외부 참조 대비)

# 그리퍼에 올려 달라고 요청할 데이터 종류. 관절각은 필수, 나머지는 안전·진단용이다.
# 공식 샘플(main.cpp:172)은 8종을 전부 요청하지만 우리는 50Hz로 서보를 밀고 있어서
# 통신량을 늘리지 않으려고 네 종만 받는다(속도·FT센서·GPIO·제어주기는 안 쓴다).
# ⚠️ 이 선택이 실물 통신 주기에 주는 영향은 **미검증**이다 — 하드웨어에서 controlPeriod와
#    실측 서보 Hz를 보고 조정할 것.
REQUESTED_RX = (RX_JOINT, RX_CURRENT, RX_TEMPERATURE, RX_MODULE_ERROR)
MODELS = {  # DGDataTypes.h DG_MODEL (SDK 원문 그대로 — 코드표는 지우지 않는다)
    "5f_left": 0x5F12, "5f_right": 0x5F22,
    "5f_s_left": 0x5F14, "5f_s_right": 0x5F24,
    "5f_s15_left": 0x5F34, "5f_s15_right": 0x5F44,
}

# ⚠️ **실제로 쓸 수 있는 모델은 이 둘뿐이다.** 이 파이프라인 전체가 '손가락 5개 × 관절 4개
# = 20채널, 1_1~5_4' 계약 위에 서 있는데(dg5f_angles.CHANNEL_NAMES → 패킷 [0..19] →
# MoveServoJoint float[20]), S 계열은 그 계약이 성립하지 않는다. URDF 실측 근거:
#   tesollo_model-main/dg5f/dg5f_{left,right}.urdf        revolute 20, 관절명 *_dg_f_j (f=1~5, j=1~4)
#   tesollo_model-main/dg5fs/dg5fs_left.urdf              revolute 20, 관절명 joint_f_j — **명명 규칙이
#       다른 별개 모델**(Unity Dg5fHandDriver는 "_dg_" 접미사로 관절을 찾으므로 바인딩조차 안 된다)
#   tesollo_model-main/dg5fs/dg5fs_15dof_left.urdf        revolute **15**, 손가락당 j=1~3
#       → 20채널을 그대로 보내면 인덱스가 통째로 밀린다. 우리 3번(thumb_ip)이 그쪽 검지 첫
#         관절로 들어가는 식이라 전 손가락이 엉뚱한 채널에 물린다. 매핑을 새로 짜야 하는 일이지
#         '모델만 바꾸면 되는' 일이 아니다.
# 5f_s_*가 위 둘 중 어느 물리 모델인지는 SDK 문서만으로 단정할 수 없어서(코드표에 치수 정보가
# 없다) 함께 막는다. S 계열을 실제로 쓰게 되면 채널 정의부터 다시 잡을 것.
#
# 검증 현황(2026-07-31): 매핑·보정은 **왼손(5f_left) 기준으로 맞춰 왔고** 오른손은 일부만
# 확인했다. 오른손으로 돌릴 땐 ⑧ 관절 검증 모드로 다시 훑을 것.
SUPPORTED_MODELS = ("5f_left", "5f_right")


class GripperSystemSetting(ctypes.Structure):
    _fields_ = [("comport", ctypes.c_char * 32),
                ("ip", ctypes.c_char * 32),
                ("port", ctypes.c_int),
                ("readTimeout", ctypes.c_int),
                ("controlMode", ctypes.c_int),
                ("communicationMode", ctypes.c_int),
                ("slaveID", ctypes.c_int),
                ("baudrate", ctypes.c_int)]


class ReceivedGripperData(ctypes.Structure):
    """DGDataTypes.h ReceivedGripperData 그대로. 그리퍼가 주기적으로 올려 보내는 상태값.

    ⚠️ 전부 4바이트 멤버라 기본 정렬로 패딩이 생기지 않는다(헤더의 #pragma pack(1)은
       __linux__ 전용 — Windows 기본 정렬과 결과가 같다).
    joint/current/velocity/temperature는 **SDK 슬롯 인덱스**다. 우리 채널 순서가 아니다 —
    슬롯↔채널 대응은 JOINT_ORDER가 쥐고 있고, 그 표를 확정하는 게 GUI ⑨의 일이다."""
    _fields_ = [("joint", ctypes.c_float * 20),        # deg
                ("current", ctypes.c_int * 20),        # mA
                ("velocity", ctypes.c_int * 20),       # rpm
                ("temperature", ctypes.c_float * 20),  # ℃
                ("TCP", ctypes.c_float * 30),          # 손끝 5개 × (x,y,z,rx,ry,rz)
                ("moving", ctypes.c_int),
                ("targetArrived", ctypes.c_int),
                ("blendMoveState", ctypes.c_int),
                ("currentBlendIndex", ctypes.c_int),
                ("productID", ctypes.c_int),
                ("firmwareVersion", ctypes.c_int),
                ("moduleErrorCode", ctypes.c_int),
                ("controlPeriod", ctypes.c_int)]


class DiagnosisSystem(ctypes.Structure):
    """DGDataTypes.h DiagnosisSystem — SystemDiagnosis()가 돌린 자가진단 결과.
    각 필드는 '검사 항목별 결과값'이고 0이 정상이라는 보장은 문서에 없다 → 값 자체를
    그대로 보여 주고 판단은 사람에게 맡긴다(의미 미검증)."""
    _fields_ = [("process", ctypes.c_int), ("step", ctypes.c_int),
                ("jointId", ctypes.c_int), ("period", ctypes.c_int),
                ("joint", ctypes.c_int), ("temperature", ctypes.c_int)]


# 연결 끊김 콜백. DLL이 **자기 스레드에서** 부르므로 몸통은 대입 한 줄로만 둔다
# (여기서 Tk를 만지거나 오래 걸리는 일을 하면 그 스레드가 물린다).
# ⚠️ CFUNCTYPE 객체는 **파이썬 쪽에서 참조를 붙들고 있어야** 한다 — GC되면 DLL이
#    해제된 함수 포인터를 부르며 프로세스가 죽는다. 그래서 모듈 전역에 남긴다.
_CB_VOID = ctypes.CFUNCTYPE(None)
LINK = {"down": False, "up_count": 0, "down_count": 0}


def _on_disconnected():
    LINK["down"] = True
    LINK["down_count"] += 1


def _on_connected():
    LINK["down"] = False
    LINK["up_count"] += 1


_CB_DISCONNECTED = _CB_VOID(_on_disconnected)
_CB_CONNECTED = _CB_VOID(_on_connected)


class GripperSetting(ctypes.Structure):
    _fields_ = [("jointOffset", ctypes.c_float * 20),
                ("jointInpose", ctypes.c_float * 20),
                ("tcpInpose", ctypes.c_float * 5),
                ("orientationInpose", ctypes.c_float * 5),
                ("receivedDataType", ctypes.c_int * 8),
                ("movingInpose", ctypes.c_float),
                ("jointCount", ctypes.c_int),
                ("fingerCount", ctypes.c_int),
                ("model", ctypes.c_int),
                ("dutyByteLength", ctypes.c_int8)]


class Dg5fSdk:
    """DGSDK.dll 래퍼 — 초기화 시퀀스와 MoveServoJoint만 노출."""

    def __init__(self, dll_path):
        self.dll = ctypes.CDLL(dll_path)   # extern "C" cdecl
        self.dll.SetGripperSystem.argtypes = [GripperSystemSetting]
        self.dll.SetGripperSystem.restype = ctypes.c_int
        self.dll.SetGripperOption.argtypes = [GripperSetting]
        self.dll.SetGripperOption.restype = ctypes.c_int
        for name in ("ConnectToGripper", "DisconnectToGripper",
                     "SystemStart", "SystemStop"):
            getattr(self.dll, name).restype = ctypes.c_int
        self.dll.MoveServoJoint.argtypes = [ctypes.POINTER(ctypes.c_float)]
        self.dll.MoveServoJoint.restype = ctypes.c_int
        self.dll.GetReceivedGripperData.argtypes = [ctypes.POINTER(ReceivedGripperData)]
        self.dll.GetReceivedGripperData.restype = ctypes.c_int
        self.dll.SetLowPassFilterAlpha.argtypes = [ctypes.c_int, ctypes.c_float]
        self.dll.SetLowPassFilterAlpha.restype = ctypes.c_int
        self.dll.SetJointGainPIDAllEqual.argtypes = [ctypes.c_float] * 4
        self.dll.SetJointGainPIDAllEqual.restype = ctypes.c_int
        # 안전·진단 계열 (2026-07-31 추가)
        self.dll.SetTorqueLimitMode.argtypes = [ctypes.c_int]
        self.dll.SetTorqueLimitMode.restype = ctypes.c_int
        self.dll.GetDiagnosisSystem.argtypes = [ctypes.POINTER(DiagnosisSystem)]
        self.dll.GetDiagnosisSystem.restype = ctypes.c_int
        for name in ("SystemDiagnosis", "SetJointEncoderZero"):
            getattr(self.dll, name).restype = ctypes.c_int
        self.dll.CallbackForOnDisconnected.argtypes = [_CB_VOID]
        self.dll.CallbackForOnDisconnected.restype = ctypes.c_int
        self.dll.CallbackForOnConnected.argtypes = [_CB_VOID]
        self.dll.CallbackForOnConnected.restype = ctypes.c_int
        # ⚠️ MoveJointAll / SetMotionTimeAllEqual 바인딩은 2026-07-31에 **삭제**했다.
        #    한 번도 호출한 적 없는 죽은 코드였고, MoveServoJoint(DEVELOPER 전용 서보
        #    스트림)와 섞어 쓰면 어느 쪽이 이기는지 문서에 없다. rest 복귀가 필요하면
        #    서보 경로에 슬루를 걸어 내리는 편이 모드 문제 없이 확실하다(_RealHand.go_rest).

    def _check(self, name, res):
        if res != DG_RESULT_NONE:
            raise RuntimeError(f"{name} 실패 — DG_RESULT={res} (DGDataTypes.h 참조)")

    def connect(self, ip, port, model_code):
        sys_set = GripperSystemSetting()
        sys_set.comport = b"COM1"                  # Ethernet 모드에선 미사용(샘플 관례)
        sys_set.ip = ip.encode("ascii")
        sys_set.port = port
        sys_set.readTimeout = 1000
        sys_set.controlMode = CONTROL_MODE_DEVELOPER   # MoveServoJoint 필수 조건
        sys_set.communicationMode = COMMUNICATION_MODE_ETHERNET
        sys_set.slaveID = 1
        sys_set.baudrate = 115200
        self._check("SetGripperSystem", self.dll.SetGripperSystem(sys_set))

        # 끊김/재연결 알림을 **연결 전에** 등록한다 — 연결 도중 끊겨도 놓치지 않게.
        LINK["down"] = False
        self.dll.CallbackForOnDisconnected(_CB_DISCONNECTED)
        self.dll.CallbackForOnConnected(_CB_CONNECTED)

        self._check("ConnectToGripper", self.dll.ConnectToGripper())

        opt = GripperSetting()                     # 배열들은 ctypes가 0으로 초기화
        opt.model = model_code
        opt.movingInpose = 0.4
        # jointInpose = 도달 판정 허용오차[deg]. 0으로 두면 어떤 관절도 '도달'로 안 잡혀
        # targetArrived가 늘 0일 수 있다. 공식 샘플(main.cpp:170)이 전 관절 10을 넣으므로
        # 그대로 맞춘다 — 서보 스트림에는 영향이 없지만 피드백 해석이 샘플과 같아진다.
        for i in range(N_JOINTS):
            opt.jointInpose[i] = 10.0
        for i, t in enumerate(REQUESTED_RX):       # 관절각 + 전류·온도·모듈에러
            opt.receivedDataType[i] = t
        self._check("SetGripperOption", self.dll.SetGripperOption(opt))

        # 샘플(GripperConnect)과 동일한 보수적 게인 — 실물 튜닝은 별도
        self.dll.SetJointGainPIDAllEqual(1.0, 5.0, 0.05, 0.1)
        self._check("SystemStart", self.dll.SystemStart())

    # ---- 안전·진단 (전부 서보 스레드에서만 부를 것 — DLL 호출을 servo와 경쟁시키지 않는다) ----
    def set_torque_limit(self, on):
        """하드웨어 토크 제한. 소프트웨어 슬루 리밋보다 확실한 2차 방어선이다
        (슬루는 '명령이 튀는 것'만 막지, 손가락이 뭔가에 끼었을 때 힘은 못 줄인다)."""
        return self.dll.SetTorqueLimitMode(1 if on else 0)

    def diagnose(self):
        """자가진단 실행 → 결과 dict. 결과 필드의 의미는 문서에 없어 값 그대로 돌려준다."""
        r = self.dll.SystemDiagnosis()
        d = DiagnosisSystem()
        r2 = self.dll.GetDiagnosisSystem(ctypes.byref(d))
        return {"run": r, "get": r2, "process": d.process, "step": d.step,
                "jointId": d.jointId, "period": d.period, "joint": d.joint,
                "temperature": d.temperature}

    def encoder_zero(self):
        """⚠️ 지금 자세를 **하드웨어 엔코더 영점**으로 굳힌다. 그리퍼에 남는 설정이라
        이 프로그램을 꺼도 되돌아오지 않는다 — 호출부가 반드시 확인을 받을 것."""
        return self.dll.SetJointEncoderZero()

    def servo(self, deg20):
        """deg20은 **SDK 슬롯 순서**의 20개(= to_sdk_frame 출력)."""
        arr = (ctypes.c_float * 20)(*deg20)
        return self.dll.MoveServoJoint(arr)

    def read_state(self):
        """그리퍼가 올려 보낸 상태 → dict. 실패하면 None.

        여기서 나오는 joint/current/temperature는 **SDK 슬롯 인덱스**다(우리 채널 순서 아님).
        그래서 '몇 번 슬롯이 실제로 몇 도인가'는 알 수 있지만 '그 슬롯이 검지 PIP인가'는
        알 수 없다 — 그건 사람이 보고 알려 줘야 하고, GUI ⑨ 탐색 마법사가 그 일을 돕는다."""
        d = ReceivedGripperData()
        res = self.dll.GetReceivedGripperData(ctypes.byref(d))
        if res != DG_RESULT_NONE:
            return None                      # 진단 모드(DG_RESULT_DIAGNOSING_SYSTEM) 등
        return {"joint": list(d.joint), "current": list(d.current),
                "temperature": list(d.temperature), "moving": int(d.moving),
                "arrived": int(d.targetArrived), "err": int(d.moduleErrorCode),
                "period": int(d.controlPeriod), "fw": int(d.firmwareVersion),
                "pid": int(d.productID)}

    def close(self):
        try:
            self.dll.SystemStop()
        finally:
            self.dll.DisconnectToGripper()


def to_sdk_frame(ours, unmirror):
    """우리 채널 20개 → SDK float[20] (미러 복원 → 재배열 → 부호/영점 → 클램프)."""
    v = list(ours)
    if unmirror:
        for i in MIRROR_IDX:
            v[i] = -v[i]
    out = []
    for i in range(N_JOINTS):
        d = JOINT_SIGN[i] * v[JOINT_ORDER[i]] + JOINT_OFFSET_DEG[i]
        lo, hi = JOINT_CLAMP[i]
        out.append(min(hi, max(lo, d)))
    return out


def main():
    ap = argparse.ArgumentParser(description="DG-5F 실물 SDK 브리지")
    ap.add_argument("--ip", default=None, help="그리퍼 IP — 생략 시 드라이런(수신값 출력만)")
    ap.add_argument("--port", type=int, default=502, help="그리퍼 Modbus TCP 포트 (기본 502)")
    ap.add_argument("--model", default="5f_left", choices=SUPPORTED_MODELS,
                    help="실물 모델 (기본 5f_left). S 계열(5f_s_*, 5f_s15_*)은 20채널 계약이 "
                         "성립하지 않아 제외 — SUPPORTED_MODELS 주석 참조")
    ap.add_argument("--listen", type=int, default=5007,
                    help="UDP 수신 포트 (vision_node --bridge와 동일해야 함, 기본 5007)")
    ap.add_argument("--dll", default=DEFAULT_DLL, help="DGSDK.dll 경로")
    ap.add_argument("--hz", type=float, default=50.0, help="실물 송신 상한 Hz (기본 50)")
    ap.add_argument("--max-step", type=float, default=2.0,
                    help="틱당 관절 최대 변화량[deg] — 점프 방지 슬루 리밋 (기본 2.0)")
    ap.add_argument("--lpf", type=float, default=0.3,
                    help="SDK 내장 저역필터 alpha (0=사용 안 함, 기본 0.3)")
    ap.add_argument("--unmirror", action="store_true",
                    help="왼손 스트림(vision_node left)을 오른손 실물 규약으로 부호 변환")
    ap.add_argument("--pose", default=None,
                    help="검증용 1회 포즈: 'idx:deg[,idx:deg...]' 나머지 0으로 MoveServoJoint 후 종료. "
                         "예) --pose 6:20 (검지 pip만 20°)")
    ap.add_argument("--joint-map", default=JOINT_MAP_PATH,
                    help="관절 대응표 JSON (GUI ⑨에서 저장한 것). 없으면 항등 매핑")
    args = ap.parse_args()

    # ⚠️ 대응표는 **연결·수신보다 먼저** 읽는다. 나중에 읽으면 첫 몇 패킷이 옛 대응으로 나간다.
    print("[대응표]", load_joint_map(args.joint_map)[1])

    dry = args.ip is None
    sdk = None
    if not dry:
        dll_path = os.path.abspath(args.dll)
        if not os.path.exists(dll_path):
            print(f"[오류] DLL 없음: {dll_path}")
            return
        sdk = Dg5fSdk(dll_path)
        print(f"[연결] {args.ip}:{args.port} model={args.model}(0x{MODELS[args.model]:X}) "
              f"DEVELOPER 모드")
        sdk.connect(args.ip, args.port, MODELS[args.model])
        if args.lpf > 0:
            sdk.dll.SetLowPassFilterAlpha(1, ctypes.c_float(args.lpf))
        print("[연결] SystemStart 완료")

        if args.pose is not None:   # 관절 대응 검증 모드 — 한 포즈 보내고 종료
            target = [0.0] * N_JOINTS
            for item in args.pose.split(","):
                i, d = item.split(":")
                target[int(i)] = float(d)
            print(f"[pose] MoveServoJoint {target}")
            sdk.servo(target)
            time.sleep(1.0)
            sdk.close()
            return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.listen))
    sock.settimeout(0.2)
    print(f"[수신] UDP :{args.listen} 대기 — vision_node_dg5f.py [left|right] --bridge 로 송신"
          + (" (드라이런: 실물 송신 없음)" if dry else ""))

    period = 1.0 / args.hz
    last_sent_t = 0.0
    last_cmd = None          # 슬루 리밋 기준 (첫 패킷은 그대로 통과)
    last_print = 0.0
    stale_warned = False
    try:
        while True:
            try:
                data, _ = sock.recvfrom(4096)
                stale_warned = False
            except socket.timeout:
                if not stale_warned and last_cmd is not None:
                    print("[hold] 패킷 끊김 — 마지막 자세 유지(실물은 위치 유지)")
                    stale_warned = True
                continue
            if len(data) < MIN_PACKET_BYTES:
                continue
            ours = struct.unpack_from(f"<{N_JOINTS}f", data)
            target = to_sdk_frame(ours, args.unmirror)

            now = time.time()
            if now - last_sent_t < period:
                continue
            # 슬루 리밋 — 트래킹 점프/오클루전 복귀 시 실물이 튀지 않게 틱당 변화 제한
            if last_cmd is not None and args.max_step > 0:
                step = args.max_step
                target = [p + min(step, max(-step, t - p))
                          for p, t in zip(last_cmd, target)]
            last_cmd = target
            last_sent_t = now

            if dry:
                if now - last_print >= 0.5:
                    print("[dry]", " ".join(f"{v:6.1f}" for v in target))
                    last_print = now
            else:
                res = sdk.servo(target)
                if res != DG_RESULT_NONE and now - last_print >= 0.5:
                    print(f"[경고] MoveServoJoint DG_RESULT={res}")
                    last_print = now
    except KeyboardInterrupt:
        print("\n[종료] Ctrl+C")
    finally:
        if sdk is not None:
            sdk.close()
            print("[종료] SystemStop + Disconnect 완료")


if __name__ == "__main__":
    main()
