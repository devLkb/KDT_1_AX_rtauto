# -*- coding: utf-8 -*-
"""MediaPipe 손 landmark(21×3) → Tesollo DG5F 20관절 각도[deg] 변환.

DG5F 구조 (dg5f_right.urdf 실측, 2026-07-13 팁 좌표로 손가락 확정):
  손가락 1 = 엄지:  1_1 CMC굽힘[-22,77] / 1_2 대향회전[-155,0] / 1_3 MCP[±90] / 1_4 IP[±90]
  손가락 2~4 = 검지/중지/약지: n_1 벌림 / n_2 MCP[0,115] / n_3 PIP[±90] / n_4 DIP[±90]
  손가락 5 = 새끼:  5_1 손바닥접기[0,60] / 5_2 MCP[-15,90] / 5_3 PIP / 5_4 DIP
  (mimic 없음 — 20관절 전부 독립 구동)

패킷 순서(고정, Unity Dg5fHandDriver와 계약):
  [0..3] 엄지 1_1,1_2,1_3,1_4 / [4..7] 검지 2_1..2_4 / [8..11] 중지 / [12..15] 약지 / [16..19] 새끼

⚠️ 벌림(n_1)·새끼접기(5_1)는 GATED=True 동안 중립 0° 고정 (SVH spread 과민 전례).
   굽힘 채널 라이브 검증 후 단계적으로 해제.
⚠️ 방향(부호) 미확정 채널: 엄지 대향(1_2)·CMC(1_1) — 라이브 검증에서 반대면
   해당 채널 (dg_min, dg_max)를 스왑 (SVH thumb_opposition 전례와 동일한 해법).
"""
import json
import math
import os

import numpy as np

# --- MediaPipe landmark 인덱스 ---
WRIST = 0
THUMB = [1, 2, 3, 4]      # CMC, MCP, IP, TIP
INDEX = [5, 6, 7, 8]      # MCP, PIP, DIP, TIP
MIDDLE = [9, 10, 11, 12]
RING = [13, 14, 15, 16]
PINKY = [17, 18, 19, 20]


def _angle(a, b, c):
    """점 b를 꼭짓점으로 하는 두 벡터 사이 각[rad]."""
    v1, v2 = np.asarray(a) - np.asarray(b), np.asarray(c) - np.asarray(b)
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0
    return float(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)))


def _bend(lm, i, j, k):
    """관절 j의 굽힘각[rad]: 펴면 0, 굽히면 +."""
    return math.pi - _angle(lm[i], lm[j], lm[k])


def _abduction(lm, finger):
    """손바닥 평면에 투영한 벌림각[rad, 부호有]. 중지 근위지골 방향 기준.
    +: 검지 쪽, -: 새끼 쪽. (게이트 해제 후 사용 예정)"""
    lm = np.asarray(lm)
    palm_n = np.cross(lm[INDEX[0]] - lm[WRIST], lm[PINKY[0]] - lm[WRIST])
    n = np.linalg.norm(palm_n)
    if n < 1e-9:
        return 0.0
    palm_n /= n
    ref = lm[MIDDLE[1]] - lm[MIDDLE[0]]          # 중지 근위지골
    v = lm[finger[1]] - lm[finger[0]]            # 대상 근위지골
    ref -= palm_n * np.dot(ref, palm_n)          # 평면 투영
    v -= palm_n * np.dot(v, palm_n)
    ang = _angle(ref + lm[finger[0]], lm[finger[0]], v + lm[finger[0]])
    side = np.dot(np.cross(ref, v), palm_n)      # 부호: 검지쪽 +
    return ang if side >= 0 else -ang


def _thumb_elevation(lm):
    """엄지 중수골(CMC→MCP)의 손바닥 평면 이탈각[rad, 부호有] — thumb_cmc 신프록시.
    옛 프록시(손목-CMC-MCP 3점각)는 세 점이 준일직선이라 노이즈 과대(§20-2 게이트 원인).
    평면 이탈각(arcsin)은 전 구간 조건수 양호. 부호: 손바닥 법선 쪽 = +.
    라이브에서 방향 반대면 채널 테이블 (dg_min, dg_max) 스왑 (관례 동일)."""
    lm = np.asarray(lm)
    n = np.cross(lm[INDEX[0]] - lm[WRIST], lm[PINKY[0]] - lm[WRIST])
    nn = np.linalg.norm(n)
    if nn < 1e-9:
        return 0.0
    n /= nn
    v = lm[THUMB[1]] - lm[THUMB[0]]
    vn = np.linalg.norm(v)
    if vn < 1e-9:
        return 0.0
    return float(np.arcsin(np.clip(np.dot(v / vn, n), -1.0, 1.0)))


def compute_raw(lm):
    """landmark → 20채널 raw 사람각도[rad], 패킷 순서."""
    return [
        # 엄지
        _thumb_elevation(lm),                          # 0 thumb_cmc (앞뒤, 신프록시)
        _angle(lm[INDEX[0]], lm[WRIST], lm[THUMB[3]]), # 1 thumb_opp (SVH와 동일 proxy)
        _bend(lm, THUMB[0], THUMB[1], THUMB[2]),       # 2 thumb_mcp
        _bend(lm, THUMB[1], THUMB[2], THUMB[3]),       # 3 thumb_ip
        # 검지
        _abduction(lm, INDEX),                         # 4 index_abd (게이트)
        _bend(lm, WRIST, INDEX[0], INDEX[1]),          # 5 index_mcp
        _bend(lm, INDEX[0], INDEX[1], INDEX[2]),       # 6 index_pip
        _bend(lm, INDEX[1], INDEX[2], INDEX[3]),       # 7 index_dip
        # 중지
        _abduction(lm, MIDDLE),                        # 8 (게이트)
        _bend(lm, WRIST, MIDDLE[0], MIDDLE[1]),        # 9
        _bend(lm, MIDDLE[0], MIDDLE[1], MIDDLE[2]),    # 10
        _bend(lm, MIDDLE[1], MIDDLE[2], MIDDLE[3]),    # 11
        # 약지
        _abduction(lm, RING),                          # 12 (게이트)
        _bend(lm, WRIST, RING[0], RING[1]),            # 13
        _bend(lm, RING[0], RING[1], RING[2]),          # 14
        _bend(lm, RING[1], RING[2], RING[3]),          # 15
        # 새끼 — ⚠️관절 의미가 다른 손가락과 다름 (2026-07-13 왼손 관절 스윕 실측):
        #   5_1=손바닥접기, 5_2=측면 기울임(굽힘 아님! 굽힘성분 0.42/측면 0.81),
        #   5_3=굽힘(0.98), 5_4=굽힘(0.99) → 굽힘 관절이 2개뿐.
        #   사람 MCP→5_3, (PIP+DIP)평균→5_4, 5_1·5_2는 게이트 중립.
        _bend(lm, WRIST, PINKY[0], PINKY[1]) * 0.5,    # 16 pinky_cmc → 5_1 (게이트)
        _abduction(lm, PINKY),                         # 17 pinky_lat → 5_2 측면 (게이트)
        _bend(lm, WRIST, PINKY[0], PINKY[1]),          # 18 pinky_mcp → 5_3
        (_bend(lm, PINKY[0], PINKY[1], PINKY[2])
         + _bend(lm, PINKY[1], PINKY[2], PINKY[3])) / 2.0,  # 19 pinky_pip → 5_4 (pip·dip 평균)
    ]


# =========================================================================
# 채널 테이블: (이름, human_min, human_max[rad], dg_min, dg_max[deg], gated)
#   사람각 human_min→dg_min, human_max→dg_max 선형 매핑 후 dg 범위로 clamp.
#   방향 반전 = (dg_min, dg_max) 스왑. gated=True면 중립값(dg_neutral) 고정 송신.
#   human_min/max는 dg5f_calibration.json 이 있으면 자동 덮어씀.
#   dg_min/max 기본값은 URDF 리밋 안쪽의 보수적 구간(하드리밋 충돌 방지).
# =========================================================================
GATED_NEUTRAL_DEG = 0.0

# 왼손 모델용 부호 반전 채널 (2026-07-13 좌/우 URDF 리밋 비교로 도출).
#   좌우 리밋이 (lower,upper)→(-upper,-lower)로 뒤집힌 관절 = 값도 부호 반전 필요.
#   벌림(abd) 계열은 리밋이 대칭이어도 물리 방향이 미러라 전부 포함.
#   대칭 리밋(±90) 채널은 왼손 주먹 프로브 시각 검증으로 확정(2026-07-13):
#     엄지 mcp/ip = 반전 필요(+60이면 엄지가 주먹 밖으로 꺾임), 새끼 pip/dip = 반전 불필요
#     (-70이면 주먹 밖으로 펴짐), 검지~약지 pip/dip = 반전 불필요.
#   새끼: 5_3/5_4(굽힘)는 좌우 모두 양수=굽힘(스윕 실측) → 반전 없음.
#   엄지 mcp/ip 반전은 핀치 탐색으로 재확인(2026-07-13): 왼손 음수 조합만 엄지-검지 4.9cm 도달.
LEFT_MIRROR_CHANNELS = {
    "thumb_cmc", "thumb_opp", "thumb_mcp", "thumb_ip",
    "index_abd", "middle_abd", "ring_abd",
    "pinky_cmc", "pinky_lat",
}

DG5F_CHANNELS = [
    # name          hmin   hmax    dg_min  dg_max  gated
    # thumb_cmc: 신프록시(중수골 평면 이탈각)로 게이트 해제(2026-07-14) — 엄지 앞뒤 재현.
    # 옛 3점각 프록시는 노이즈로 게이트했었음(§20-2). ⚠️보정 재실행 필요(calibrate_dg5f).
    ("thumb_cmc",   0.15,  0.85,    0.0,   65.0,  False),
    # thumb_opp: 방향은 핀치 탐색으로 확정. dg_min -15 = 휴지 상태를 약간 대향으로 —
    # 대향 0 근처에서 엄지 굽힘 방향이 시각적으로 반전돼 보이는 결합 특성 완화.
    ("thumb_opp",   0.10,  0.55,  -15.0, -120.0,  False),
    ("thumb_mcp",   0.05,  0.90,    0.0,   80.0,  False),
    ("thumb_ip",    0.05,  1.20,    0.0,   80.0,  False),
    ("index_abd",  -0.30,  0.30,  -25.0,   20.0,  True),
    ("index_mcp",   0.05,  1.20,    0.0,  110.0,  False),
    ("index_pip",   0.10,  1.80,    0.0,   85.0,  False),
    ("index_dip",   0.05,  1.20,    0.0,   80.0,  False),
    ("middle_abd", -0.30,  0.30,  -20.0,   20.0,  True),
    ("middle_mcp",  0.05,  1.20,    0.0,  110.0,  False),
    ("middle_pip",  0.10,  1.80,    0.0,   85.0,  False),
    ("middle_dip",  0.05,  1.20,    0.0,   80.0,  False),
    ("ring_abd",   -0.30,  0.30,  -12.0,   28.0,  True),
    ("ring_mcp",    0.05,  1.20,    0.0,  105.0,  False),
    ("ring_pip",    0.10,  1.80,    0.0,   85.0,  False),
    ("ring_dip",    0.05,  1.20,    0.0,   80.0,  False),
    ("pinky_cmc",   0.05,  0.60,    0.0,   50.0,  True),
    ("pinky_lat",  -0.30,  0.30,  -12.0,   12.0,  True),   # 5_2 측면 기울임 — 게이트
    ("pinky_mcp",   0.05,  1.20,    0.0,   85.0,  False),  # → 5_3 (실질 MCP 굽힘)
    ("pinky_pip",   0.10,  1.80,    0.0,   80.0,  False),  # → 5_4 (pip·dip 평균)
]

CHANNEL_NAMES = [c[0] for c in DG5F_CHANNELS]


def map_to_dg5f(raw, hand="right"):
    """raw 사람각도 20개[rad] → DG5F 관절각 20개[deg] (게이트/미러/clamp 적용).

    hand="left"면 LEFT_MIRROR_CHANNELS 채널 값을 부호 반전 (왼손 URDF는
    해당 관절 리밋이 좌우 대칭으로 뒤집혀 있음). Unity 쪽에서 프리팹 자체
    리밋으로 한 번 더 clamp 하므로 초과분은 안전.
    """
    out = []
    for v, (name, hmin, hmax, dmin, dmax, gated) in zip(raw, DG5F_CHANNELS):
        if gated:
            out.append(GATED_NEUTRAL_DEG)
            continue
        t = (v - hmin) / (hmax - hmin) if hmax > hmin else 0.0
        t = min(1.0, max(0.0, t))
        deg = dmin + t * (dmax - dmin)
        if hand == "left" and name in LEFT_MIRROR_CHANNELS:
            deg = -deg
        out.append(deg)
    return out


# =========================================================================
# 엄지 손끝 위치 리타게팅 (v2 프로토콜 — 관절각 20개 + 엄지끝 위치 3 + 핀치 1)
#   채널별 선형 매핑은 엄지의 결합 운동(대향×굽힘×접기)을 재현 못함 →
#   엄지만 "손바닥 해부학 좌표계 기준 엄지끝 위치"를 보내고 Unity에서 IK(CCD).
#   좌표계: 원점=중지 MCP, ez=손목→중지MCP(손가락 방향), ey=새끼MCP→검지MCP(측면),
#           ex=cross(ey,ez)(손바닥 법선). 해부학 랜드마크 기반이라 좌/우·거울 불변.
#   단위: |손목→중지MCP| 길이로 정규화 (무차원 — Unity가 로봇 치수로 복원).
# =========================================================================
# 핀치 히스테리시스(vision_node에서 적용): 걸림 <PINCH_ON, 풀림 >PINCH_OFF
#   단일 임계는 경계 근처에서 플래그가 깜빡여 엄지가 두 목표 사이를 왕복(까딱임) — 실측 교훈.
PINCH_ON, PINCH_OFF = 0.30, 0.42


def compute_thumb_tip(lm):
    """landmark → (엄지끝 정규화 좌표[3], 엄지-검지 끝거리 비율). 손바닥 해부학 좌표계."""
    lm = np.asarray(lm)
    wrist, mid_mcp = lm[WRIST], lm[MIDDLE[0]]
    hand_len = np.linalg.norm(mid_mcp - wrist)
    if hand_len < 1e-6:
        return (0.0, 0.0, 0.0), 0.0
    ez = (mid_mcp - wrist) / hand_len
    ey_raw = lm[INDEX[0]] - lm[PINKY[0]]
    ex = np.cross(ey_raw, ez)
    ex /= max(np.linalg.norm(ex), 1e-9)
    ey = np.cross(ez, ex)
    v = (lm[THUMB[3]] - mid_mcp) / hand_len
    tip = (float(np.dot(v, ex)), float(np.dot(v, ey)), float(np.dot(v, ez)))
    pinch_d = float(np.linalg.norm(lm[THUMB[3]] - lm[INDEX[3]]) / hand_len)
    return tip, pinch_d


# --- 보정 파일 자동 로드 (calibrate_raw.py 방식과 동일 스키마) ---
_CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "dg5f_calibration.json")


def _load_calibration():
    global DG5F_CHANNELS
    if not os.path.exists(_CALIB_PATH):
        return False
    try:
        with open(_CALIB_PATH, "r", encoding="utf-8") as f:
            hr = json.load(f).get("human_ranges", {})
    except Exception as e:
        print(f"[dg5f_angles] 보정 파일 읽기 실패({e}) — 기본값 사용")
        return False
    DG5F_CHANNELS = [
        (n, float(hr[n]["min"]) if n in hr else hmin,
            float(hr[n]["max"]) if n in hr else hmax, dmin, dmax, g)
        for (n, hmin, hmax, dmin, dmax, g) in DG5F_CHANNELS]
    return True


if _load_calibration():
    print(f"[dg5f_angles] 보정값 로드됨: {_CALIB_PATH}")
