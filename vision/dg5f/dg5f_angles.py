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

from dg5f_paths import CALIB_PATH

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
        # 엄지 — ⚠️2026-07-14 프록시 교차 재배선(§20-7, 로봇 기하 실측 근거):
        #   1_1은 앞뒤 관절이 아니라 손바닥 평면 안 스윕(풀스윕 시 법선 1.5cm/전방 13.4cm),
        #   깊이(앞뒤)는 1_2(대향 롤)가 만듦(굽힌 채 1_2 스윕 시 법선 3→8.3cm).
        #   따라서 사람 가로 스윕→1_1, 사람 앞뒤(elevation)→1_2 로 교차 배선.
        #   (§20-5의 elevation→1_1 직결은 관절 오배정이었음)
        _angle(lm[INDEX[0]], lm[WRIST], lm[THUMB[3]]), # 0 → 1_1: 가로 스윕 proxy
        _thumb_elevation(lm),                          # 1 → 1_2: 앞뒤(elevation) proxy
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
    # ⚠️채널 이름은 관절 기준(thumb_cmc=1_1, thumb_opp=1_2)이고, 프록시는 §20-7에서
    # 교차 재배선됨: thumb_cmc(1_1)의 사람각 = 가로 스윕, thumb_opp(1_2) = 앞뒤 elevation.
    # human 기본범위도 프록시에 맞춰 교차. 보정 파일 값도 2026-07-14 함께 스왑됨.
    ("thumb_cmc",   0.10,  0.55,    0.0,   65.0,  False),
    # thumb_opp(1_2): 사람 elevation(앞뒤) → 대향 롤. dg_min -15 = 휴지 약간 대향(§20-2),
    # dg_max -120: 실측상 깊이(법선) 성분은 1_2≈80°서 최대, 120°까지 전방 유지(§20-7).
    ("thumb_opp",   0.15,  0.85,  -15.0, -120.0,  False),
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
# 엄지 손끝 위치 리타게팅 (패킷 [20..22] 위치 3 + [23] 핀치 + [24] 끝거리비)
#   채널별 선형 매핑은 엄지의 결합 운동(대향×굽힘×접기)을 재현 못함 →
#   엄지만 "손끝 위치"를 보내고 Unity에서 IK(CCD).
#   축: ez=손목→중지MCP(손가락 방향), ey=새끼MCP→검지MCP(측면), ex=cross(ey,ez)
#       (손바닥 법선). 해부학 랜드마크 기반이라 좌/우·거울 불변.
#   값(2026-07-15 §26 '펴짐 비율' 재재정의 — Unity Dg5fThumbIK 방향별 도달 테이블과 계약):
#       리치벡터 = 방향(단위벡터, 해부학 축 성분) × 크기(펴짐 비율 0~1)
#       크기 = |엄지끝−CMC| 직선 / (같은 프레임 엄지 마디합 × 직진도 상한) — "직진도".
#       Unity가 가상 앵커 + 방향 × 펴짐비율 × 그 방향 로봇 최대도달(FK 테이블)로 복원.
#   왜(§26, 2026-07-15): §25의 "직선/(손길이×보정비율)"은 보정 세션의 최대치를 상수로
#       박는 방식이라 MediaPipe z 압축의 **방향 의존 오차**를 못 넘김 — 라이브 실측에서
#       손가락 방향으로 뻗으면 |n|→1.0, 손바닥 옆/앞으로 뻗으면 |n|→0.69~0.74
#       (thumbik_20260715_1744/1803.csv). 사람이 쭉 펴도 로봇 목표가 70%에 머무는 원인.
#       직진도(같은 프레임, 같은 랜드마크끼리의 비)는 삼각부등식으로 항상 0~1이고,
#       일직선이면 이방성 스케일 오차(z 압축)가 분자·분모에 똑같이 걸려 비가 보존 —
#       어느 방향으로 뻗어도 1.0. 카메라·세션 간 보정 이전 문제도 소멸(무보정 동작).
#   §25의 이전 정의·근거(마디합→최대도달 재정의, 1_2 정책)는 WORKLOG §25 참조.
# =========================================================================
# 핀치 히스테리시스(vision_node에서 적용): 걸림 <PINCH_ON, 풀림 >PINCH_OFF
#   단일 임계는 경계 근처에서 플래그가 깜빡여 엄지가 두 목표 사이를 왕복(까딱임) — 실측 교훈.
PINCH_ON, PINCH_OFF = 0.30, 0.42

# 사람 엄지 직진도 상한 = 쭉 폈을 때의 (직선/마디합) 실측치. MediaPipe 랜드마크는 쭉 펴도
# 완전 일직선이 아니라서(자연 굴곡) 1.0에 못 미침 — 이 값으로 나눠 "쭉 폄 = 1.0"으로 정렬.
# calibrate_dg5f.py v3가 p95로 실측 저장(선택). 기본 0.97은 전형적 직진도의 보수적 근사.
DEFAULT_THUMB_STRAIGHT = 0.97
CALIB_VERSION = 3  # v3: thumb_straight_ratio(직진도). v2 thumb_reach_ratio(직선/손길이)는 폐기.
_thumb_straight_calib = None

# 손가락(검지~새끼) 직진도 상한 — 엄지(0.97)와 분리한다. 해부학이 달라서: 엄지는 쭉 펴도
# IP에 자연 굴곡이 남지만 손가락은 거의 일직선이라 직진도가 1.0에 더 가깝다.
# ⚠️ 실측 전 잠정값(2026-07-16). 틀렸을 때 방향:
#    - 실제보다 **낮으면** 사람이 다 펴기 전에 비율이 1.0 포화 → 로봇이 먼저 쭉 펴짐(무해)
#    - 실제보다 **높으면** 쭉 펴도 로봇이 덜 펴짐 → §26에서 엄지가 겪은 바로 그 증상
#    그래서 보수적으로 낮게 잡는다. 라이브에서 쭉 폈을 때 [send] |idx|가 1.0에 못 미치면
#    이 값을 낮출 것. (엄지처럼 calibrate가 손가락별 p95를 저장하게 만드는 건 추후 과제)
DEFAULT_FINGER_STRAIGHT = 0.98

# 리치벡터를 싣는 손가락(패킷 [25..36] 순서 고정) — 엄지는 [20..22]에 따로 있어 제외.
TIP_FINGERS = [("index", INDEX), ("middle", MIDDLE), ("ring", RING), ("pinky", PINKY)]
# v5 새 방식(로봇 관점 IK)용: 손목→끝 벡터를 싣는 5손가락 (엄지 포함, 엄지부터).
WRIST_TIP_FINGERS = [("thumb", THUMB), ("index", INDEX), ("middle", MIDDLE),
                     ("ring", RING), ("pinky", PINKY)]

# =========================================================================
# 패킷 레이아웃 (Unity Dg5fReceiver와 계약 — 수신기는 **길이로** 버전 판별)
#   v1 <20f>: [0..19] 관절각[deg]
#   v2 <24f>: + [20..22] 엄지 리치벡터, [23] 핀치 플래그
#   v3 <25f>: + [24] 엄지-검지 끝거리비
#   v4 <37f>: + [25..36] 검지/중지/약지/새끼 리치벡터 (각 3, TIP_FINGERS 순서)
#   v5 <52f>: + [37..51] 손목→끝 벡터 5개 (엄지·검지·중지·약지·새끼, WRIST_TIP_FINGERS 순서,
#             각 3 = 해부학 프레임 성분 ÷ 손길이) — 새 방식(로봇 관점 IK, ikMode=RobotRootTipVector)
# ⚠️ 수신기 판별이 `>=`라 상위 버전을 하위 Unity에 쏴도 앞부분만 읽고 뒤는 무시된다(양방향 호환).
#    v5를 v4까지 아는 Unity에 쏴도 손목→끝 필드만 무시, 나머지 동작 불변.
# =========================================================================
PACKET_FMT = "<52f"
PACKET_LEN = 52


def _anat_frame(lm):
    """손바닥 해부학 좌표계 → (ex, ey, ez, hand_len). 손길이 퇴화면 None.

    축: ez=손목→중지MCP(손가락 방향), ey=새끼MCP→검지MCP(측면), ex=cross(ey,ez)(손바닥 법선).
    해부학 랜드마크 기반이라 좌/우·거울 불변 — 엄지·손가락 전부 이 프레임을 공유한다.
    """
    wrist, mid_mcp = lm[WRIST], lm[MIDDLE[0]]
    hand_len = float(np.linalg.norm(mid_mcp - wrist))
    if hand_len < 1e-6:
        return None
    ez = (mid_mcp - wrist) / hand_len
    ey_raw = lm[INDEX[0]] - lm[PINKY[0]]
    ex = np.cross(ey_raw, ez)
    ex /= max(np.linalg.norm(ex), 1e-9)
    ey = np.cross(ez, ex)
    return ex, ey, ez, hand_len


def _reach_vector(lm, chain_idx, cap, ex, ey, ez):
    """리치벡터 = 해부학 축 단위방향 × 펴짐 비율(직진도). 퇴화 프레임이면 (0,0,0).

    chain_idx = [앵커, 중간1, 중간2, 끝] landmark 인덱스 (엄지=CMC 기준, 손가락=MCP 기준).
    직진도 = |끝−앵커| ÷ (같은 프레임 마디합 × 상한), 1.0 클램프.
    같은 프레임 같은 랜드마크끼리의 비라서 삼각부등식으로 항상 0~1이고, 일직선이면
    이방성 z 압축이 분자·분모에 똑같이 걸려 비가 보존 — 어느 방향으로 뻗어도 1.0 (§26).
    """
    d = lm[chain_idx[3]] - lm[chain_idx[0]]
    straight = float(np.linalg.norm(d))
    chain = float(np.linalg.norm(lm[chain_idx[1]] - lm[chain_idx[0]])
                  + np.linalg.norm(lm[chain_idx[2]] - lm[chain_idx[1]])
                  + np.linalg.norm(lm[chain_idx[3]] - lm[chain_idx[2]]))
    if straight < 1e-9 or chain < 1e-9:
        return (0.0, 0.0, 0.0)
    m = min(1.0, straight / (chain * cap))
    u = d / straight
    return (m * float(np.dot(u, ex)), m * float(np.dot(u, ey)), m * float(np.dot(u, ez)))


def compute_thumb_tip(lm):
    """landmark → (엄지 리치벡터[3, '펴짐 비율' 0~1], 엄지-검지 끝거리 비율).

    리치벡터 = 해부학 좌표계 단위 방향 × 펴짐 비율. 펴짐 비율(직진도) =
    |엄지끝−CMC| / (같은 프레임 CMC→MCP→IP→끝 마디합 × 직진도 상한), 1.0 클램프.
    같은 프레임 같은 랜드마크끼리의 비라서 카메라 깊이 오차·손 크기와 무관하게
    쭉 펴면 방향 불문 ~1.0 (송신 단계에서 0~1 보장 확립).
    pinch_d는 기존과 동일하게 손길이 정규화 — 핀치 임계(0.30/0.42 등) 튜닝 유지.
    """
    lm = np.asarray(lm)
    frame = _anat_frame(lm)
    if frame is None:
        return (0.0, 0.0, 0.0), 0.0
    ex, ey, ez, hand_len = frame
    cap = _thumb_straight_calib if _thumb_straight_calib is not None else DEFAULT_THUMB_STRAIGHT
    tip = _reach_vector(lm, THUMB, cap, ex, ey, ez)
    pinch_d = float(np.linalg.norm(lm[THUMB[3]] - lm[INDEX[3]]) / hand_len)
    return tip, pinch_d


def compute_finger_tips(lm):
    """landmark → 검지·중지·약지·새끼 리치벡터 12 float (패킷 [25..36], TIP_FINGERS 순서).

    엄지와 **같은** §26 직진도 정의를 그대로 쓴다 — 앵커만 CMC 대신 각 손가락 MCP.
    사람 MCP ↔ 로봇 n_1 피벗 대응: 로봇 n_1은 벌림 관절이라 마디 길이에 기여하지 않아
    사람 3마디 ↔ 로봇 4관절이어도 체인 대응이 성립한다.
    Unity는 IK 컴포넌트가 붙은 손가락 값만 골라 쓴다 — 나머지는 그냥 무시(각도 방식 유지).
    """
    lm = np.asarray(lm)
    frame = _anat_frame(lm)
    if frame is None:
        return [0.0] * 12
    ex, ey, ez, _ = frame
    out = []
    for _name, idx in TIP_FINGERS:
        out.extend(_reach_vector(lm, idx, DEFAULT_FINGER_STRAIGHT, ex, ey, ez))
    return out


def compute_wrist_tip_vectors(lm):
    """landmark → 손목→각 손가락 끝 벡터 5개 = 15 float (패킷 [37..51], WRIST_TIP_FINGERS 순서).

    새 방식(로봇 관점 IK)용. 각 벡터 = (손가락끝 − 손목)을 해부학 프레임 (ex,ey,ez)로 분해한 뒤
    손길이(손목→중지MCP)로 나눈 것 — 카메라 거리·손 크기 무관(정규화)하고 좌/우·회전 불변.
    Unity는 로봇 손목(palm)에서 이 벡터 × 로봇 손길이 방향으로 목표를 찍어 IK를 푼다.
    리치벡터(§26)와 달리 '직진도 0~1'이 아니라 실제 손목→끝 변위라 크기가 1을 넘는다
    (편 손가락은 대략 1.3~2.0). 로봇/사람 마디 비율 차이는 도달 불가 시 CCD가 최근접에서 멈춰 흡수.
    """
    lm = np.asarray(lm)
    frame = _anat_frame(lm)
    if frame is None:
        return [0.0] * (3 * len(WRIST_TIP_FINGERS))
    ex, ey, ez, hand_len = frame
    wrist = lm[WRIST]
    out = []
    for _name, idx in WRIST_TIP_FINGERS:
        v = lm[idx[3]] - wrist                       # idx[3] = 끝 landmark
        out.extend([float(np.dot(v, ex)) / hand_len,
                    float(np.dot(v, ey)) / hand_len,
                    float(np.dot(v, ez)) / hand_len])
    return out


# --- 보정 파일 자동 로드 (calibrate_raw.py 방식과 동일 스키마) ---
# ⚠️ 경로는 dg5f_paths.CALIB_PATH 하나로 통일 — calibrate_dg5f.py(저장)와 여기(로드)가
#    서로 다른 경로를 쓰다 2026-07-16에 발각됨(저장은 CWD 상대, 로드는 스크립트 기준).
_CALIB_PATH = CALIB_PATH


def _load_calibration():
    global DG5F_CHANNELS, _thumb_straight_calib
    if not os.path.exists(_CALIB_PATH):
        return False
    try:
        with open(_CALIB_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        hr = data.get("human_ranges", {})
    except Exception as e:
        print(f"[dg5f_angles] 보정 파일 읽기 실패({e}) — 기본값 사용")
        return False
    # 버전 게이트: v1 thumb_chain_ratio(마디합)·v2 thumb_reach_ratio(직선/손길이)는
    # 의미가 달라 읽지 않음 — v3 직진도는 무보정으로도 동작하므로 경고 없이 기본값 사용.
    ver = int(data.get("version", 1))
    ts = data.get("thumb_straight_ratio")
    if ver >= CALIB_VERSION and ts:
        _thumb_straight_calib = float(ts)
        print(f"[dg5f_angles] 엄지 직진도 상한 보정값 사용: {_thumb_straight_calib:.3f}")
    else:
        print(f"[dg5f_angles] 엄지 직진도 상한 = 기본값 {DEFAULT_THUMB_STRAIGHT} "
              f"(보정 파일 v{ver}, thumb_straight_ratio 없음 — 무보정 동작 정상. "
              "더 정밀히 맞추려면 calibrate_dg5f.py 재실행). human_ranges는 그대로 사용.")
    DG5F_CHANNELS = [
        (n, float(hr[n]["min"]) if n in hr else hmin,
            float(hr[n]["max"]) if n in hr else hmax, dmin, dmax, g)
        for (n, hmin, hmax, dmin, dmax, g) in DG5F_CHANNELS]
    return True


if _load_calibration():
    print(f"[dg5f_angles] 보정값 로드됨: {_CALIB_PATH}")
