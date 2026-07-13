"""
MediaPipe 손 landmark(3D 좌표) -> SVH 9개 관절 각도 변환.

MediaPipe Hands는 21개 landmark를 반환합니다. 인덱스 규약:
  0        : 손목(WRIST)
  1-4      : 엄지 (CMC, MCP, IP, TIP)
  5-8      : 검지 (MCP, PIP, DIP, TIP)
  9-12     : 중지
  13-16    : 약지
  17-20    : 새끼

여기 입력되는 landmark는 "삼각측량으로 복원한 실제 3D 좌표(mm 등)"를
가정합니다. 단일 카메라라면 mediapipe의 (x,y,z)를 그대로 넣어도
각도 경향은 나오지만 z가 부정확합니다.

SVH는 9개 구동기(DoF)입니다. 아래 SVH_CHANNELS가 그 9개이며,
각 채널의 (svh_min, svh_max)는 SVH URDF(schunk_svh_hand_right.urdf)의
<limit lower/upper> 값(rad)으로 채워져 있습니다. 채널 순서는 SVH ROS
드라이버 JTC(cfg/schunk_svh_driver.yaml) / SVHChannel enum 순서와 동일합니다.
검지·중지는 근위(proximal)/원위(distal) 독립 모터 2개, 약지·새끼는 단일
모터 1개로 실제 하드웨어 구성에 맞춰 정의했습니다.
"""
import numpy as np

# --- landmark 인덱스 상수 ---
WRIST = 0
THUMB = [1, 2, 3, 4]
INDEX = [5, 6, 7, 8]
MIDDLE = [9, 10, 11, 12]
RING = [13, 14, 15, 16]
PINKY = [17, 18, 19, 20]


def _angle(a, b, c):
    """점 b를 꼭짓점으로 하는 벡터 (b->a),(b->c) 사이 각도[rad]."""
    a, b, c = np.asarray(a), np.asarray(b), np.asarray(c)
    v1 = a - b
    v2 = c - b
    n1 = np.linalg.norm(v1)
    n2 = np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    cosang = np.dot(v1, v2) / (n1 * n2)
    cosang = np.clip(cosang, -1.0, 1.0)
    return float(np.arccos(cosang))


def _flexion(lm, finger):
    """
    손가락 굽힘 정도[rad]. MCP와 PIP 각도의 합을 굽힘 지표로 사용.
    편 상태에서 약 0, 완전히 굽히면 커집니다.
    finger: [MCP, PIP, DIP, TIP] landmark 인덱스 리스트.
    """
    mcp, pip, dip, tip = finger
    a_mcp = np.pi - _angle(lm[WRIST], lm[mcp], lm[pip])
    a_pip = np.pi - _angle(lm[mcp], lm[pip], lm[dip])
    return a_mcp + a_pip


def _mcp_flex(lm, finger):
    """MCP(근위) 굽힘각[rad]: 손목-MCP-PIP. 편 상태 ~0, 굽히면 커짐."""
    mcp, pip, dip, tip = finger
    return np.pi - _angle(lm[WRIST], lm[mcp], lm[pip])


def _pip_flex(lm, finger):
    """PIP(원위) 굽힘각[rad]: MCP-PIP-DIP. 편 상태 ~0, 굽히면 커짐."""
    mcp, pip, dip, tip = finger
    return np.pi - _angle(lm[mcp], lm[pip], lm[dip])


def _thumb_flexion(lm):
    cmc, mcp, ip, tip = THUMB
    return (np.pi - _angle(lm[cmc], lm[mcp], lm[ip])) + \
           (np.pi - _angle(lm[mcp], lm[ip], lm[tip]))


def _thumb_opposition(lm):
    """엄지 대향: 엄지 끝이 새끼 MCP 쪽으로 향하는 정도."""
    return _angle(lm[INDEX[0]], lm[WRIST], lm[THUMB[3]])


def _spread(lm):
    """검지~새끼 MCP들이 벌어진 정도(검지-새끼 MCP 벡터가 이루는 각)."""
    return _angle(lm[INDEX[0]], lm[WRIST], lm[PINKY[0]])


def compute_svh_angles(lm):
    """
    lm: (21, 3) 배열의 3D landmark.
    반환: dict {channel_name: raw_angle_rad}  (아직 SVH 범위로 정규화 전)
    """
    # 순서 = SVH 드라이버 JTC(cfg/schunk_svh_driver.yaml) / SVHChannel enum 순서.
    # 검지·중지는 근위/원위 독립 모터 2개, 약지·새끼는 단일 모터(합산 굽힘) 1개.
    return {
        "thumb_flexion":          _thumb_flexion(lm),
        "thumb_opposition":       _thumb_opposition(lm),
        "index_finger_distal":    _pip_flex(lm, INDEX),
        "index_finger_proximal":  _mcp_flex(lm, INDEX),
        "middle_finger_distal":   _pip_flex(lm, MIDDLE),
        "middle_finger_proximal": _mcp_flex(lm, MIDDLE),
        "ring_finger":            _flexion(lm, RING),
        "pinky":                  _flexion(lm, PINKY),
        "finger_spread":          _spread(lm),
    }


# =========================================================================
# SVH 9개 채널 정의 + 매핑 범위.
# (human_min, human_max): 사람 손 관측 각도 범위(rad).
#   -> 2026-07-02 calibrate_raw.py 실측값 기반으로 보정 완료.
#      (관측 min/max에 여유를 주고 순간 노이즈 극단값은 다듬음)
# (svh_min, svh_max)    : SVH 관절의 물리적 가동 범위(rad).
#   -> 2026-07-02 SVH URDF(schunk_svh_hand_right.urdf) <limit lower/upper>로 채움.
#      순서 = JTC(cfg/schunk_svh_driver.yaml) / SVHChannel enum 순서.
#      URDF는 lower=편 상태(0), upper=굽힘/벌림 최대(양수) → 사람 각도와 같은 방향.
#      단 thumb_opposition은 방향 미확정 → Phase 3 시각화에서 반대면 min/max 교체.
# =========================================================================
SVH_CHANNELS = [
    # name,                    human_min, human_max, svh_min, svh_max
    # human_min/max는 임시 기본값 -> calibrate_raw.py 재보정으로 덮어씀.
    ("thumb_flexion",          0.10, 2.00,   0.0, 0.9704),
    # thumb_opposition: 2026-07-08 라이브 로그로 역방향 확정 → svh_min/max 스왑(반전)
    ("thumb_opposition",       0.10, 0.55,   0.9879, 0.0),
    ("index_finger_distal",    0.10, 1.80,   0.0, 1.334),
    ("index_finger_proximal",  0.05, 1.20,   0.0, 0.79849),
    ("middle_finger_distal",   0.10, 1.80,   0.0, 1.334),
    ("middle_finger_proximal", 0.05, 1.20,   0.0, 0.79849),
    ("ring_finger",            0.12, 3.30,   0.0, 0.98175),
    ("pinky",                  0.15, 3.25,   0.0, 0.98175),
    ("finger_spread",          0.30, 1.10,   0.0, 0.5829),
]


def map_to_svh(raw_angles):
    """
    raw_angles(dict) -> SVH 9채널 위치 리스트[rad], 물리 범위로 clamp.
    반환 순서는 SVH_CHANNELS 순서와 동일(= ROS 드라이버 joint 순서에 맞추세요).
    """
    out = []
    for name, hmin, hmax, smin, smax in SVH_CHANNELS:
        # [임시] finger_spread 상수 0 고정 — 주먹 쥘 때 벌어짐 억제.
        #        추후 굽힘 기반 게이팅으로 대체 예정. 원복: 아래 if 블록 삭제.
        if name == "finger_spread":
            out.append(0.0)
            continue
        v = raw_angles[name]
        # 사람 범위 -> 0~1 정규화
        t = (v - hmin) / (hmax - hmin) if hmax > hmin else 0.0
        t = min(1.0, max(0.0, t))
        # 0~1 -> SVH 범위
        out.append(smin + t * (smax - smin))
    return out


CHANNEL_NAMES = [c[0] for c in SVH_CHANNELS]


# =========================================================================
# 자동 보정 로딩:
#   calibrate_raw.py가 저장한 calibration.json이 같은 폴더에 있으면
#   각 채널의 human_min/human_max를 자동으로 덮어씁니다.
#   (svh_min/svh_max는 건드리지 않음 - 그건 URDF 값이라 별도 관리)
#   파일이 없으면 위의 기본값을 그대로 사용하므로 안전합니다.
# =========================================================================
import json
import os

_CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "calibration.json")


def _load_calibration():
    global SVH_CHANNELS, CHANNEL_NAMES
    if not os.path.exists(_CALIB_PATH):
        return False
    try:
        with open(_CALIB_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception as e:
        print(f"[svh_angles] calibration.json 읽기 실패({e}) - 기본값 사용")
        return False

    hr = data.get("human_ranges", {})
    updated = []
    for name, hmin, hmax, smin, smax in SVH_CHANNELS:
        if name in hr:
            new_min = float(hr[name]["min"])
            new_max = float(hr[name]["max"])
            updated.append((name, new_min, new_max, smin, smax))
        else:
            updated.append((name, hmin, hmax, smin, smax))
    SVH_CHANNELS = updated
    CHANNEL_NAMES = [c[0] for c in SVH_CHANNELS]
    return True


# 모듈 임포트 시 자동 적용
if _load_calibration():
    print(f"[svh_angles] 보정값 로드됨: {_CALIB_PATH}")
