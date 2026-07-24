import json
import math
import os
import numpy as np

from dg5f_paths import CALIB_PATH

# --- MediaPipe landmark 인덱스 ---
WRIST = 0                 # 손목
THUMB = [1, 2, 3, 4]      # 엄지 CMC, MCP, IP, TIP
INDEX = [5, 6, 7, 8]      # 검지 MCP, PIP, DIP, TIP
MIDDLE = [9, 10, 11, 12]  # 중지 MCP, PIP, DIP, TIP
RING = [13, 14, 15, 16]   # 약지 MCP, PIP, DIP, TIP
PINKY = [17, 18, 19, 20]  # 소지 MCP, PIP, DIP, TIP

# _angle(a, b, c): 점 b를 꼭짓점으로 하는 두 벡터(b→a, b→c) 사이 각(rad)을 구하는 함수
def _angle(a, b, c):
    # 점 b를 꼭짓점으로 하는 두 벡터 만들기
    v1, v2 = np.asarray(a) - np.asarray(b), np.asarray(c) - np.asarray(b)
    # 미디어파이프에서 랜드마크는 a[x,y,z] 처럼 리스트로 받기때문에 np.asarray()로 NumPy 배열로 변환처리 후
    # 벡터 연산(빼기, 내적, 길이 계산 등)을 하여 점 b를 꼭짓점으로 하는 두 벡터를 구함.

    # 구한 두 벡터의 길이를 계산
    n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
    # np.linalg.norm(): 벡터의 크기(길이)를 구하는 함수, 길이 = √(x² + y² + z²) 이용

    # 벡터의 길이가 0(또는 거의 0)인 경우 예외 처리
    if n1 < 1e-6 or n2 < 1e-6:
        return 0.0

    # cosθ에서 각도 추출
    return float(np.arccos(np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)))
    # np.dot(v1, v2) / (n1 * n2): 벡터의 내적 공식 이용하여 cosθ 값 구하기
    # |a|*|b|*cosθ = a⋅b
    # cosθ = a⋅b/|a|*|b|
    # np.clip(): cosθ 값은 -1~1 사이로 나와야하지만 컴퓨터 부동소수점 연산 오차로 범위를 벗어날 수 있으므로, -1~1 범위로 제한 처리
    # np.arccos(): 코사인(cos) 값으로부터 실제 각도(θ)를 구하는 함수

# _bend(): 손가락 관절의 굽힘각(Flexion)을 계산하는 함수
def _bend(lm, i, j, k):
    return math.pi - _angle(lm[i], lm[j], lm[k])
    # - lm : MediaPipe의 21개 랜드마크 좌표
    # - i : 이전 관절
    # - j : 현재 관절(굽힘을 계산할 관절)
    # - k : 다음 관절
    # 위에서 구현한 _angle() 함수에 미디어파이프 전체 랜드마크 좌표에서 자신이 구할 관절의 내부 각도를 구하고
    # math.pi(180°) 에서 구한 관절 내부 각도를 빼서 손가락 관절을 완전히 폈을때 기준 얼만큼 굽혔는지 그 각도를 구함.

# _abduction(): 중지의 근위지골(9→10) 방향을 기준으로 손가락이 얼마나 좌우로 벌어졌는지 계산하는 함수
def _abduction(lm, finger):
    lm = np.asarray(lm)           # 랜드마크 리스트를 NumPy 배열로 변환처리

    # 손바닥 평면의 법선 벡터 구하는 처리
    palm_n = np.cross(            # np.cross(): 두 벡터의 외적 계산 함수로 손바닥 평면의 법선 벡터 구하기
        lm[INDEX[0]] - lm[WRIST], # 손목→검지 MCP 벡터 구하기
        lm[PINKY[0]] - lm[WRIST]  # 손목→소지 MCP 벡터 구하기
      )

    # 손바닥 법선 벡터(palm_n)의 길이 구하기 처리
    # np.linalg.norm(): 벡터의 크기(길이)를 구하는 함수, 길이 = √(x² + y² + z²) 이용
    n = np.linalg.norm(palm_n)

    # 벡터의 길이가 0(또는 거의 0)인 경우 예외 처리
    if n < 1e-9:
        return 0.0

    palm_n /= n                                  # 법선벡터 정규화(벡터길이를 1로 정규화)
    ref = lm[MIDDLE[1]] - lm[MIDDLE[0]]          # 중지 근위지골(9→10 벡터)
    v = lm[finger[1]] - lm[finger[0]]            # 대상 근위지골(대상 손가락 mcp→pip 벡터)
    
    # 중지 근위지골(9→10)벡터에서 손바닥 평면 방향 벡터 성분만 남기도록 처리
    ref -= palm_n * np.dot(ref, palm_n)          # np.dot(ref, palm_n): 중지 방향 벡터(ref)에서 손바닥 법선 방향 성분만을 남기도록 내적
                                                 # palm_n * np.dot(ref, palm_n): 내적연산으로 구한 법선 방향 벡터성분을 법선 벡터와 곱해 벡터로 만들고
                                                 # 그 값을 중지 방향 벡터(ref)에서 벡터 연산(빼기)하여 손바닥 평면 방향 벡터 성분만을 남기도록 처리.
    
    # 대상 손가락 mcp→pip 벡터에서 손바닥 평면 방향 성분만 남기도록 처리                                             
    v -= palm_n * np.dot(v, palm_n)              # 위와 근위지골(9→10)벡터와 동일하게 처리
    
    # _angle() 함수로
    ang = _angle(
        ref + lm[finger[0]],    # ref + lm[finger[0]]: 대상손가락의 mcp에서 9→10벡터의 손바닥 평면 성분벡터만큼 이동한 좌표
        lm[finger[0]],          # lm[finger[0]]      : 대상손가락의 mcp 좌표
        v + lm[finger[0]]       # v + lm[finger[0]]  : 대상손가락의 mcp에 대상 손가락 mcp→pip 벡터의 손바닥 평면 성분벡터만큼 이동한 좌표
    ) 
    
    # 중지를 기준으로 대상 손가락이 검지 쪽으로 벌어졌는지, 새끼 쪽으로 벌어졌는지를 판별
    side = np.dot(np.cross(ref, v), palm_n)
    # np.cross(ref, v): 9→10벡터에서 손바닥 평면 방향 성분만 남긴 벡터와 대상 손가락 mcp→pip 벡터에서 손바닥 평면 방향 성분만 남긴 벡터를 외적한 결과. 
    #                   두 벡터가 같은 평면(손바닥 평면)에 있으므로 결과는 palm_n 축과 나란함
    # np.dot(np.cross(ref, v), palm_n) : 위 외적 결과와 손바닥 평면 법선 벡터를 내적하여 palm_n(법선) 방향 성분의 크기만 스칼라로 남김.
    #                                    이렇게 구한 스칼라의 부호로 +면 손등 방향, -면 손바닥 방향인지 확인
    #                                    오른손 법칙에 따라 위 결과가 +면 시계방향, -면 반시계방향으로 측정한 각도
    
    # 위에서 구한 side로 최종 ang이 어느쪽으로 벌어진 것인지 부호를 결정.
    return ang if side >= 0 else -ang
    
# _thumb_abduction(): 엄지 CMC의 벌림(abduction)각을 구하는 함수.
#   엄지 근위 마디(1→2) 벡터를 손바닥 평면에 투영한 뒤,
#   손 축(손목→검지 MCP, 0→5)의 평면 투영과 이루는 각을 잰다.
#   TIP(4번)을 쓰지 않으므로 MCP·IP 굽힘이 값에 섞이지 않는다.
def _thumb_abduction(lm):
    lm = np.asarray(lm)

    # 손바닥 평면 법선 (기존 _thumb_elevation과 동일)
    n = np.cross(lm[INDEX[0]] - lm[WRIST],
                 lm[PINKY[0]] - lm[WRIST])
    nn = np.linalg.norm(n)
    if nn < 1e-9:
        return 0.0
    n /= nn

    ref = lm[INDEX[0]] - lm[WRIST]     # 손 축 벡터 (0→5)
    v   = lm[THUMB[1]] - lm[THUMB[0]]  # 엄지 근위 마디 벡터 (1→2) ← TIP 아님!

    # 두 벡터 모두 손바닥 평면에 투영 (법선 성분 제거 — _abduction과 동일 처리)
    ref = ref - n * np.dot(ref, n)
    v   = v   - n * np.dot(v, n)

    nr, nv = np.linalg.norm(ref), np.linalg.norm(v)
    if nr < 1e-9 or nv < 1e-9:
        return 0.0

    # 평면 위 두 벡터 사이 각 (크기)
    ang = float(np.arccos(np.clip(np.dot(ref / nr, v / nv), -1.0, 1.0)))

    # 부호 판정 (_abduction과 동일한 방식): 외적을 법선에 투영해 방향 결정
    side = np.dot(np.cross(ref, v), n)
    return ang if side >= 0 else -ang

# _thumb_elevation(): 엄지 손가락이 손바닥 평면으로부터 얼만큼 각도로 들려 있는지 각도를 구하는 함수
def _thumb_elevation(lm):
    lm = np.asarray(lm)    # 랜드마크 리스트를 NumPy 배열로 변환처리
    
    # 손목→중지mcp 벡터와 손목→새끼 mcp 벡터의 외적을 구해 손바닥 평면의 법선 벡터 구하기
    n = np.cross(
        lm[INDEX[0]] - lm[WRIST],    # 0→5 백터
        lm[PINKY[0]] - lm[WRIST]     # 0→17 벡터
    )
    
    # 손바닥 법선 벡터의 길이 구하기
    nn = np.linalg.norm(n)
    
    # 벡터의 길이가 0(또는 거의 0)인 경우 예외 처리
    if nn < 1e-9:
        return 0.0
    n /= nn                            # 법선벡터 정규화(벡터길이를 1로 정규화)
    
    v = lm[THUMB[1]] - lm[THUMB[0]]    # 엄지 방향 벡터(1→2 벡터)
    vn = np.linalg.norm(v)             # 엄지 방향 벡터의 길이 구하기
    
    # 벡터의 길이가 0(또는 거의 0)인 경우 예외 처리
    if vn < 1e-9:
        return 0.0
    v /= vn                            # 엄지 방향 벡터 정규화(벡터길이를 1로 정규화)
        
    # 엄지 손가락이 손바닥 평면으로부터 얼만큼 각도로 들려 있는지 각도 계산
    return float(np.arcsin(np.clip(np.dot(v, n), -1.0, 1.0)))
    # np.dot(v, n): 정규화된 엄지벡터와 정규화된 손바닥 법선 벡터를 내적하여 두 벡터 사이 각도의 cosθ을 구함.
    #               cos θ = cos(90° − φ) = sin φ 이므로 dot(v, n) = cos θ = sin φ 
    # np.clip(): cosθ 값은 -1~1 사이로 나와야하지만 컴퓨터 부동소수점 연산 오차로 범위를 벗어날 수 있으므로, -1~1 범위로 제한 처리
    # np.arcsin(): sin값을 값으로부터 실제 각도(φ)를 구하는 함수


# _thumb_opposition(): 엄지 '대향(opposition)'각 — 엄지가 손바닥을 가로질러 손가락 쪽으로
#   얼마나 넘어왔는지. 2026-07-21 신규(기존 _thumb_elevation 대체). 왜 바꾸나:
#   MediaPipe는 대향을 크게 잘 잡는데(엄지끝이 소지끝에 거의 닿음), _thumb_elevation은
#   엄지 근위마디의 '손바닥에서 들린 각(arcsin dot v·n)'만 재서 **손 축 성분에 희석**돼
#   과소평가(실측 max ~30°, 로봇 대향은 75° 필요)했다. 여기선 엄지 근위마디에서 **손 축 성분을
#   먼저 제거**하고, 남은 '손 축에 수직인 평면(폭 w × 깊이 n)' 안에서 깊이방향 회전각을 재
#   대향을 온전히 잡는다(손 축 희석 제거 → elevation보다 큰 범위).
def _thumb_opposition(lm):
    lm = np.asarray(lm)
    # 손바닥 법선 n (기존 elevation과 동일 부호 — 대향할수록 depth +)
    n = np.cross(lm[INDEX[0]] - lm[WRIST], lm[PINKY[0]] - lm[WRIST])
    nn = np.linalg.norm(n)
    if nn < 1e-9:
        return 0.0
    n /= nn
    # 손 축 a (손목→중지 MCP) — 손가락이 뻗는 위쪽 방향
    a = lm[MIDDLE[0]] - lm[WRIST]
    an = np.linalg.norm(a)
    if an < 1e-9:
        return 0.0
    a /= an
    # 엄지 근위 마디(cmc→mcp, 1→2)에서 손 축 성분 제거 → 손 축에 수직인 성분만 남김
    #   (TIP 안 씀 → MCP·IP 굽힘이 대향값에 안 섞임)
    v = lm[THUMB[1]] - lm[THUMB[0]]
    v = v - np.dot(v, a) * a
    if np.linalg.norm(v) < 1e-9:
        return 0.0
    w = np.cross(n, a)                 # 손바닥 폭 축(검지↔소지 방향)
    depth = float(np.dot(v, n))        # 손바닥 밖(대향)으로 나가는 성분 — 대향 시 +
    width = float(np.dot(v, w))        # 폭 방향(평면 내) 성분
    # 대향각 = 평면 내(width)에서 평면 밖(depth)으로 회전한 각. 손 축 성분을 뺐으므로
    #   elevation(arcsin, 손 축에 희석)보다 큰 범위. rest≈0, 완전대향→90° 근처.
    return float(np.arctan2(depth, abs(width) + 1e-9))


# ── 엄지 평면 벌림(1_1) ↔ 대향(1_2) crosstalk 게이트 (기본 OFF) ─────────────────
# 배경(2026-07-21 로그 실측): _thumb_abduction(1_1)이 _thumb_opposition(1_2)과 상관 +0.98 —
#   둘이 같은 엄지 중수골(1→2) 벡터의 두 구면각(평면내 방위각=벌림/접힘, 평면밖 들림=대향)이라
#   엄지 CMC 안장관절에서 자연스럽게 함께 움직인다. 단일 뼈 벡터라 완전 분리는 원래 불가.
# ★ 사용자 요구(2026-07-21): 엄지는 대향+벌림/접힘이 '섞여서' 움직여야 하는 경우가 많다 →
#   두 프록시를 인위적으로 디커플링/게이팅하지 말고 그대로 내보내 1_1·1_2가 함께 움직이게 한다.
#   따라서 아래 게이트는 기본 OFF. (fold 중 1_1을 죽이던 이전 "옆벌림만" 처방은 이 요구와 상충.)
# THUMB_ABD_OPP_GATE=True로 켜면: 대향(opp)이 깊어질수록 벌림(1_1)을 감쇠(fold 중 1_1→중립).
#   실험용 — 특정 태스크에서 벌림 crosstalk가 방해될 때만.
THUMB_ABD_OPP_GATE = False   # 기본 OFF = 대향/벌림 혼합 허용(사용자 요구)
OPP_GATE_LO = 0.20   # (게이트 ON일 때) 이 대향각(rad) 이하: 벌림 그대로 통과
OPP_GATE_HI = 0.45   # (게이트 ON일 때) 이 대향각(rad) 이상: 벌림 완전 차단


def _opp_gate(opp):
    """대향각(opp, rad)이 클수록(=fold 깊을수록) 0에 가까운 게이트값[0,1] 반환. (게이트 ON일 때만 사용)"""
    if opp <= OPP_GATE_LO:
        return 1.0
    if opp >= OPP_GATE_HI:
        return 0.0
    return (OPP_GATE_HI - opp) / (OPP_GATE_HI - OPP_GATE_LO)


# _palm_fold():
def _palm_fold(lm):
    lm = np.asarray(lm)    # 랜드마크 리스트를 NumPy 배열로 변환처리

    # 요측(엄지쪽) 손바닥 평면의 법선 벡터 — 비교적 손바닥 접힘 운동에 안 딸려가는 검지·중지로 기준면을 만든다
    n_rad = np.cross(                  # np.cross(): 두 벡터의 외적으로 요측 평면 법선 구하기
	      lm[INDEX[0]] - lm[MIDDLE[0]],  # 중지 MCP→검지 MCP 벡터(9→5 벡터)
	      lm[MIDDLE[0]] - lm[WRIST]      # 손목→중지 MCP 벡터(0→9 벡터)
    )
    
    # 척측(새끼쪽) 손바닥 평면의 법선 벡터 — 비교적 손바닥 접힘 운동에 딸려가는 약지·새끼 MCP로 만든다
    n_uln = np.cross(
	      lm[RING[0]] - lm[WRIST],       # 손목→약지 MCP 벡터(0→13 벡터)
	      lm[PINKY[0]] - lm[WRIST]       # 손목→소지 MCP 벡터(0→17 벡터)
    )
      
    # 두 손바닥 평면이 접히는 경첩(힌지)축 벡터 구하기. 손목→약지 MCP 벡터(0→13 벡터)
    hinge = lm[RING[0]] - lm[WRIST]

    # 세 벡터의 길이를 구해 하나라도 0(또는 거의 0)이면 예외 처리
    nr, nu, nh = np.linalg.norm(n_rad), np.linalg.norm(n_uln), np.linalg.norm(hinge)
    if nr < 1e-9 or nu < 1e-9 or nh < 1e-9:
        return 0.0
    
    # 전부 단위벡터로 정규화
    n_rad, n_uln, hinge = n_rad / nr, n_uln / nu, hinge / nh

    # 두 법선 사이의 '부호 있는' 각도를 힌지축 기준으로 구하기
    x = float(np.dot(n_rad, n_uln))                    # x: 두 손바닥 평면이 접힌 각도의 cos 성분
    y = float(np.dot(np.cross(n_rad, n_uln), hinge))   # y: 두 손바닥 평면의 법선벡터의 외적을 계산하여 두 손바닥 평면의 법선벡터로 이루어진
                                                       # 평면의 법선벡터(np.cross(a, b)=|a|·|b|·sin θ)를 구하고, 
                                                       # np.cross() 벡터와 힌지축 벡터는 서로 동일한 축에 놓인 두 벡터이므로
                                                       # 내적 연산을 통해 부호를 결정하여 최종 성분을 sin 구함.

    # np.arctan2(y, x): (y, x)로부터 각도(θ)를 구하는 함수 — 단순 arctan과 달리 접힘/폄 방향(부호)까지 포함된 결과를 얻을 수 있음.
    return float(np.arctan2(y, x))


# DIP(원위 관절) 굽힘을 PIP에서 유도하는 결합계수 (2026-07-21, 격리 디버깅 결과 대응).
#   왜: DIP는 _bend(PIP→DIP→TIP)로 재는데 DIP·TIP의 z(깊이)가 MediaPipe에서 부실해 노이즈 큼
#       (사용자 관찰: 2_4·3_4·4_4 "z로 잘 추정 못함"). 해부학적으로 DIP 굴곡은 신전건 메커니즘상
#       PIP에 종속(사람도 DIP만 독립으로 못 굽힘)이라 측정 대신 PIP에서 유도하는 게 표준·강건.
#   값: DIP≈0.66~0.75×PIP(해부학). 보정파일 middle/ring 비율도 ~0.75. 튜닝: 원위가 덜/더 굽으면 ↕.
DIP_PIP_COUPLING = 0.75


def compute_raw(lm):
    # 엄지 벌림/접힘(1_1)과 대향(1_2)은 안장관절서 자연히 섞여 움직임 → 두 프록시를 독립적으로
    #   그대로 내보내 로봇 1_1·1_2가 함께(복합) 움직이게 한다(사용자 요구). 게이트는 기본 OFF.
    _opp_thumb = _thumb_opposition(lm)
    _abd_thumb = _thumb_abduction(lm)
    if THUMB_ABD_OPP_GATE:                             # (기본 False) 켜면 fold 중 벌림 감쇠 — 실험용
        _abd_thumb *= _opp_gate(_opp_thumb)
    return [
        # 엄지
        _abd_thumb,                                    # thumb_cmc(1_1): 엄지 손바닥평면 안 운동 = 벌림(검지와 수직)↔접힘(손가락과 평행). 프록시=cmc→mcp 벡터와 0→5 벡터의 손바닥평면 성분 사이각. FK: 로봇 1_1 −77°=평행(접음)/+22°=수직(벌림)
        _opp_thumb,                                    # thumb_opp(1_2): 엄지 평면 밖 대향각(손바닥서 떠서 가로질러 넘어옴). 옛 _thumb_elevation(들림각)은 손축 희석으로 과소평가라 교체(2026-07-21)
        _bend(lm, THUMB[0], THUMB[1], THUMB[2]),       # thumb_mcp(2번) 관절의 각도
        _bend(lm, THUMB[1], THUMB[2], THUMB[3]),       # thumb_ip(3번) 관절의 각도
        # 검지
        _abduction(lm, INDEX),                         # 중지의 근위지골(9→10) 방향을 기준으로 검지가 얼마나 좌우로 벌어졌는지 각도
        _bend(lm, WRIST, INDEX[0], INDEX[1]),          # index_mcp(5번) 관절의 각도
        _bend(lm, INDEX[0], INDEX[1], INDEX[2]),       # index_pip(6번) 관절의 각도
        DIP_PIP_COUPLING * _bend(lm, INDEX[0], INDEX[1], INDEX[2]),   # index_dip(7번): PIP에서 유도(측정 z 부실). =k×PIP
        # 중지
        _abduction(lm, MIDDLE),                        # middle_abd — 기준(9→10)과 자기 자신 비교라 항상 ≈0 (중립 유지용)
        _bend(lm, WRIST, MIDDLE[0], MIDDLE[1]),        # middle_mcp(9번) 관절의 각도
        _bend(lm, MIDDLE[0], MIDDLE[1], MIDDLE[2]),    # middle_pip(10번) 관절의 각도
        DIP_PIP_COUPLING * _bend(lm, MIDDLE[0], MIDDLE[1], MIDDLE[2]),  # middle_dip(11번): PIP에서 유도. =k×PIP
        # 약지
        _abduction(lm, RING),                          # ring_abd — 중지(9→10) 기준 약지 벌림각
        _bend(lm, WRIST, RING[0], RING[1]),            # ring_mcp(13번) 관절의 각도
        _bend(lm, RING[0], RING[1], RING[2]),          # ring_pip(14번) 관절의 각도
        DIP_PIP_COUPLING * _bend(lm, RING[0], RING[1], RING[2]),       # ring_dip(15번): PIP에서 유도. =k×PIP
        # 새끼
        _palm_fold(lm),                                # pinky_cmc → 손바닥접기 각도, 5_1 대응용
        _abduction(lm, PINKY),                         # pinky_lat → 중지의 근위지골(9→10) 방향을 기준으로 새끼가 얼마나 좌우로 벌어졌는지 각도, 5_2 대응용
        _bend(lm, WRIST, PINKY[0], PINKY[1]),          # pinky_mcp(17번) 관절의 각도, 5_3 대응용
        (1.0 + DIP_PIP_COUPLING) * _bend(lm, PINKY[0], PINKY[1], PINKY[2]),  # pinky 원위(5_4): 로봇은 원위관절 1개뿐 → 사람 PIP+DIP 합을 PIP에서 유도((1+k)×PIP). 옛 (PIP+측정DIP)/2는 DIP z부실로 과소(사용자: "덜 움직임")
    ]

# 채널 테이블:
GATED_NEUTRAL_DEG = 0.0

# 왼손 모델용 부호 반전 채널
#   ★thumb_cmc 제외(2026-07-21): 1_1은 이제 |abd|→로봇[접힘,벌림] 양방향 선형매핑으로 왼손모델
#     각을 '직접' 산출하므로 여기서 또 반전하면 안 됨(FK가 왼손 URDF 기준이었음). 우수 모델은
#     THUMB_CMC_FOLD_DEG/SPREAD_DEG 부호를 뒤집어 대응(주석 참고).
LEFT_MIRROR_CHANNELS = {
    "thumb_opp", "thumb_mcp", "thumb_ip",
    "index_abd", "middle_abd", "ring_abd",
    "pinky_cmc", "pinky_lat",
}

# 벌림(좌우) 채널 — 사람 벌림각(rad)을 로봇 벌림각(deg)으로 **1:1 직접 매핑**(로봇 범위로 clamp).
#   percentile min/max 정규화를 쓰면 안 되는 이유(2026-07-20 실측): 보정 세션에서 벌림을 조금만 해도
#   사람 범위가 좁게 잡혀(예 index 양수쪽 hmax=0.096rad=5.5°) 로봇 dmax=20°까지 ~3.6배 증폭 →
#   "사람보다 훨씬 많이 벌어짐". 로봇 벌림 가동범위(±20~30°)가 사람 손가락 벌림 범위(±20~25°)와
#   비슷해 1:1이 자연스럽다. raw=0(중지와 평행=모음)→0°는 자동으로 성립(부호 있는 각이라).
#   ABD_GAIN을 키우면 더 과장, 줄이면 더 절제 — 라이브 체감으로 미세조정.
# thumb_cmc는 0을 사이에 두지 않는 벌림(hmin/hmax 둘 다 음수)이라 여기서 제외 — 기존 선형 유지.
ABDUCTION_CHANNELS = {"index_abd", "middle_abd", "ring_abd", "pinky_lat"}
ABD_GAIN = 1.0  # 로봇도 = 사람 벌림각(deg) × 이 값. 1.0 = 1:1(증폭 없음).

# 엄지 깊이 대향(thumb_opp): 신 프록시 _thumb_opposition 사용. 2026-07-21 라이브 실측 —
#   사람은 엄지를 손바닥 안쪽까지 깊게 대향하는데 로봇은 use clamp -75°(≈손바닥 법선)에서 잘렸다.
#   URDF 한계는 155°이므로 clamp를 -155°(전 범위)로 열고, 프록시가 ~83°에서 천장(atan2)이라 그 범위를
#   쓰려면 GAIN>1 필요 → 1.5. (사람 대향 83° → 로봇 125°로 손바닥 안쪽까지 접힘.) 너무 과하면 ↓, 부족하면 ↑.
THUMB_OPP_GAIN = 1.5
# 어느 부호의 raw 대향각을 '대향(fold-in)'으로 볼지. +1.0=기존(v>0을 대향으로). 라이브에서 대향
#   방향이 반대면(격리 1_2 테스트로 확인) -1.0으로 뒤집을 것. LEFT_MIRROR(로봇 좌우)와는 별개 제어.
THUMB_OPP_SIGN = 1.0
# 비율(ratio) 모드 전용: 사람 대향각(양의 대향, rad)의 '완전대향' 상한. 이 값에서 t=1(로봇 최대 대향).
#   보정값(0.013~0.428)은 옛 프록시(_thumb_elevation) 기준이라 새 _thumb_opposition(라이브 0~1.4rad)엔
#   안 맞아 조기 포화 → 반응 소실. 라이브 실측 max≈1.42rad(rad_dg5f 로그)를 반영해 1.4로 고정.
#   대향이 부족하면(끝까지 못 감) ↓, 너무 일찍 포화하면 ↑.
THUMB_OPP_RATIO_HI = 1.4

# 엄지 손바닥평면 벌림/접힘(thumb_cmc, 1_1): |abd| → 로봇 [접힘, 벌림] 양방향 선형매핑.
#   ★왜 부호/미러가 아니라 크기 매핑인가(2026-07-21 실데이터 진단):
#     _thumb_abduction 실측값은 파이프라인상 '항상 음수'(보정 -1.4~-30°, lmprobe -7.6~-22°)이고,
#     벌림(엄지-검지 수직)=|각| 큼 / 접힘(엄지 손가락과 평행)=|각| 작음(≈0). 즉 두 동작이 부호가
#     아니라 '크기'로 구분된다. 게다가 이 프록시 부호는 손 방향·거울상에 불변(검증) → 부호 뒤집기는
#     방향 교정이 아니라 채널을 죽일 뿐. 그래서 |abd|를 로봇 양방향각에 직접 선형매핑한다.
#   FK(왼손 URDF): 로봇 1_1 −77°=평행(접음) / +22°=수직(벌림) / 0=중립. LEFT_MIRROR 제외(직접 산출).
#   튜닝: 벌림이 부족/과하면 SPREAD_DEG, 접힘이 부족/과하면 FOLD_DEG. 사람 |abd| 범위가 다르면 H_*.
#   ⚠️우수(right) 모델은 FOLD_DEG/SPREAD_DEG 부호를 뒤집을 것(왼손 기준 값임).
THUMB_CMC_FOLD_DEG   = -65.0   # 사람이 엄지 완전히 접었을 때(손가락과 평행) 로봇 1_1 목표각
THUMB_CMC_SPREAD_DEG = 22.0    # 사람이 엄지 완전히 벌렸을 때(검지와 수직) 로봇 1_1 목표각
THUMB_CMC_H_FOLD     = 0.03    # 접힘일 때 사람 |abd|(rad, 작음). 이 값→FOLD_DEG
THUMB_CMC_H_SPREAD   = 0.52    # 벌림일 때 사람 |abd|(rad, 큼).   이 값→SPREAD_DEG

DG5F_CHANNELS = [
    # name          hmin   hmax    dg_min  dg_max  gated
    # ★2026-07-21: map_to_dg5f 전 채널 1:1 전환. dg_min/dg_max = 로봇 clamp 경계(가동범위).
    #   hmin/hmax는 매핑에서 미사용(보정 로드가 덮어써도 무해) — 기록/스키마용으로만 남김.
    #   굽힘 채널 dg_max = 로봇 관절 최대각(사람이 더 굽히면 여기서 포화).
    # thumb_cmc(1_1): 2026-07-21 |abd|→로봇[접힘,벌림] 양방향 선형매핑(THUMB_CMC_* 상수·분기 참조).
    #   dmin/dmax(0,65)·hmin/hmax는 이 채널에선 미사용(분기가 자체 [FOLD_DEG,SPREAD_DEG]로 클램프).
    #   실사용 범위 = [THUMB_CMC_FOLD_DEG, THUMB_CMC_SPREAD_DEG] = 기본 [-65, +22]°(왼손).
    ("thumb_cmc",  -0.52, -0.03,    0.0,   65.0,  False),
    # thumb_opp(1_2, 깊이 대향): dmax -75→-155(2026-07-21) — 로봇 clamp가 -75(≈손바닥 법선)에서
    #   잘려 사람처럼 손바닥 안쪽까지 못 접혔음. URDF 한계 155°까지 열어 깊은 대향 허용(GAIN 1.5와 함께).
    #   dmin=0(평면). 신 프록시 _thumb_opposition + THUMB_OPP_GAIN로 이 범위를 채운다.
    ("thumb_opp",   0.15,  0.85,    0.0, -155.0,  False),
    ("thumb_mcp",   0.05,  0.90,    0.0,   80.0,  False),
    ("thumb_ip",    0.05,  1.20,    0.0,   80.0,  False),
    ("index_abd",  -0.30,  0.30,  -25.0,   20.0,  False),
    ("index_mcp",   0.05,  1.20,    0.0,  110.0,  False),
    ("index_pip",   0.10,  1.80,    0.0,   85.0,  False),
    ("index_dip",   0.05,  1.20,    0.0,   80.0,  False),
    ("middle_abd", -0.30,  0.30,  -20.0,   20.0,  False),
    ("middle_mcp",  0.05,  1.20,    0.0,  110.0,  False),
    ("middle_pip",  0.10,  1.80,    0.0,   85.0,  False),
    ("middle_dip",  0.05,  1.20,    0.0,   80.0,  False),
    ("ring_abd",   -0.30,  0.30,  -12.0,   28.0,  False),
    ("ring_mcp",    0.05,  1.20,    0.0,  105.0,  False),
    ("ring_pip",    0.10,  1.80,    0.0,   85.0,  False),
    ("ring_dip",    0.05,  1.20,    0.0,   80.0,  False),
    # pinky_cmc(5_1, 손바닥 접기): 2026-07-20 게이트(사용자 요청) — 앞굽힘을 가장 크게 망치는 오염원.
    #   프록시 raw는 컵핑(22°)>굽힘오염(7.7°)로 신호가 있으나, 사람범위 6.9°→로봇 50°로 7.2배 증폭돼
    #   굽힘만 해도 로봇 5_1이 40° 스윙 + 정지 노이즈 14°도 증폭. SNR 나빠 유지가치 낮음.
    #   컵핑을 조금 살리려면 gated False + dg (50,0)→(12,0)으로 '아주 적게 제한'이 대안(주석 참고).
    ("pinky_cmc",   0.09,  0.21,   50.0,    0.0,  True),
    # pinky_lat(5_2): 2026-07-20 게이트 해제(True→False) — 사용자 요청("옆으로 벌림이 안 됨").
    #   새끼 굽힘이 이 값에 섞이는 crosstalk(~50%)는 감수. 0 중심 매핑이라 crosstalk 영향은 ±12°로 제한.
    ("pinky_lat",  -0.30,  0.30,  -12.0,   12.0,  False),
    ("pinky_mcp",   0.05,  1.20,    0.0,   85.0,  False),
    ("pinky_pip",   0.10,  1.80,    0.0,   80.0,  False),
]

CHANNEL_NAMES = [c[0] for c in DG5F_CHANNELS]

# ── 비율(0~1) 매핑 모드용 로봇 관절 URDF 기구 리밋[deg] ────────────────────────
#   출처: joint_ranges.py SPEC의 URDF <limit> 열(이번 세션 왼손 미러 URDF 기준).
#   ★2026-07-21 사용자 요청: map_to_dg5f(mode="ratio")가 이 범위를 로봇 [min,max]로 써서
#     사람각을 [사람min,사람max]→[0,1]→[로봇min,로봇max]로 정규화 매핑한다(아래 _map_ratio).
#   ⚠️ 기구 리밋은 실제 사용범위(DG5F_CHANNELS dg_min/dg_max)보다 넓다(pip/dip ±90 등).
#     → 비율이 안 쓰는 극단 영역까지 분산돼 같은 사람동작에도 로봇 움직임이 작아진다(사용자 인지·선택).
#     또 굽힘 관절 리밋 하한이 −90(과신전 여유)이라 사람 rest(≈사람min)가 로봇 −90 근처로 갈 수 있음.
#     rest 포즈가 뒤로 젖혀 보이면 RATIO_ROBOT_RANGE를 "use"로 바꿔 재평가할 것(한 줄).
URDF_LIMITS_DEG = {
    "thumb_cmc": (-77.0,  22.0), "thumb_opp": (0.0, 155.0),
    "thumb_mcp": (-90.0,  90.0), "thumb_ip":  (-90.0, 90.0),
    "index_abd": (-20.0,  31.0), "index_mcp": (0.0, 115.0),
    "index_pip": (-90.0,  90.0), "index_dip": (-90.0, 90.0),
    "middle_abd": (-25.0, 25.0), "middle_mcp": (0.0, 115.0),
    "middle_pip": (-90.0, 90.0), "middle_dip": (-90.0, 90.0),
    "ring_abd": (-32.0,  15.0),  "ring_mcp":  (0.0, 110.0),
    "ring_pip": (-90.0,  90.0),  "ring_dip":  (-90.0, 90.0),
    "pinky_cmc": (-60.0,  0.0),  "pinky_lat": (-90.0, 15.0),
    "pinky_mcp": (-90.0,  90.0), "pinky_pip": (-90.0, 90.0),
}

# 비율 모드가 로봇 [min,max]로 무엇을 쓸지: "urdf"=기구 리밋(사용자 선택) / "use"=DG5F_CHANNELS 사용범위.
RATIO_ROBOT_RANGE = "urdf"

# ── 비율 모드 채널별 로봇 출력 한계[deg] (URDF/use 대신 이 값을 최우선 사용) ─────────────
#   2026-07-22 라이브 관절디버깅 피드백 반영. URDF 리밋(±90 등)이면 사람보다 과하게 움직이거나
#   (특히 굽힘 관절) 사람이 못 하는 뒤꺾임(과신전, rmin=−90)까지 나온다. → 사람 실제 ROM으로 조인다.
#   여기 '없는' 채널은 URDF 리밋(RATIO_ROBOT_RANGE)로 폴백 — 벌림(abduction) 채널은 URDF가 더
#   사람같이 벌어진다는 피드백이라 일부러 뺐다(index_abd 등). (rmin,rmax) 의미는 채널별 주석 참조.
#   ⚠️ 첫 튜닝값 — 라이브 보고 이 숫자만 조정하면 됨.
RATIO_LIMIT = {
    # ── 엄지 ──
    # thumb_cmc(1_1): rmin=접힘(fold,−)/rmax=벌림(spread,+). 접힘 OK(−65 유지), 벌림 과함 +22→+10.
    "thumb_cmc": (-65.0, 10.0),
    # thumb_mcp(1_3)/thumb_ip(1_4): 안쪽 굽힘만 사람스러움 → URDF −90(과신전) 제거, 0(1자)..80.
    "thumb_mcp": (0.0, 80.0),
    "thumb_ip":  (0.0, 80.0),
    # (thumb_opp 1_2는 단방향 특수분기 → THUMB_OPP_RATIO_MAX_DEG로 따로 제한)
    # ── 손가락 굽힘(mcp/pip/dip): ★공통 — 사람은 뒤로 못 꺾음 → rmin=0(과신전 금지). rmax=사람 굽힘 한계 ──
    # 검지: 2_2 굽힘 잘 되나 약간 과함 → rmax 110→95. 2_3/2_4는 URDF −90이라 손 펴면 뒤로 꺾이던 것 제거.
    "index_mcp": (0.0, 95.0), "index_pip": (0.0, 85.0), "index_dip": (0.0, 80.0),
    # 중지·약지·새끼도 동일한 뒤꺾임 문제라 예방적으로 rmin=0 적용(rmax=각 손가락 사람 굽힘 한계).
    "middle_mcp": (0.0, 95.0),  "middle_pip": (0.0, 85.0), "middle_dip": (0.0, 80.0),
    "ring_mcp":   (0.0, 105.0), "ring_pip":   (0.0, 85.0), "ring_dip":   (0.0, 80.0),
    "pinky_mcp":  (0.0, 85.0),  "pinky_pip":  (0.0, 80.0),
    # ── 벌림(abduction)은 일부러 없음 → URDF 폴백(index_abd 등 ratio-URDF가 더 사람같이 벌어짐, 피드백) ──
}

# thumb_opp(1_2) 비율 모드 손바닥쪽(대향) 최대각[deg]. 부호는 direct와 동일(−, left-mirror가 +로).
#   URDF −155면 엄지가 새끼 손바닥에 닿을 만큼 과회전 → 사람 대향 한계로 축소(−95).
#   더 깊게 대향하려면 크기↑(예 −110), 덜이면 크기↓(예 −80).
#   ※ 손등쪽(음의 대향)은 MediaPipe z가 부실해 신뢰 불가 → rest(0)로 고정(로봇 관절도 대개 단방향).
THUMB_OPP_RATIO_MAX_DEG = -95.0


def _robot_range(name, dmin, dmax):
    """비율 모드용 로봇 [min,max](deg). 채널별 한계(RATIO_LIMIT) 최우선 →
    없으면 RATIO_ROBOT_RANGE에 따라 URDF 리밋 또는 DG5F 사용범위."""
    if name in RATIO_LIMIT:
        return RATIO_LIMIT[name]
    if RATIO_ROBOT_RANGE == "urdf" and name in URDF_LIMITS_DEG:
        return URDF_LIMITS_DEG[name]
    return dmin, dmax


def _map_ratio(raw, hand):
    """사람 관절 프록시(rad) → DG5F 로봇 관절각(deg). **비율(0~1) 정규화 매핑**(2026-07-21 신규 모드).
    채널마다 t = clamp01((v − 사람min)/(사람max − 사람min)) 로 사람각을 0~1 비율로 바꾸고,
    로봇각 = 로봇min + t·(로봇max − 로봇min) 로 로봇 범위에 그대로 실어보낸다(사람비율=로봇비율).
      • 사람 min/max = DG5F_CHANNELS의 hmin/hmax(= calibration.json human_ranges, 실행 시 로드).
      • 로봇 min/max = _robot_range() (기본 URDF 기구 리밋, RATIO_ROBOT_RANGE로 전환).
    ⚠️ 정규화라 보정 신선도에 민감: 보정 때 관절을 끝까지 안 움직였거나(폭 좁음) 프록시 정의가
       보정 이후 바뀐 채널(2026-07-21 thumb_cmc/thumb_opp)은 t가 빗나갈 수 있음 → 재보정 권장.
       폭 0(예 middle_abd)은 t=0(로봇min 고정)으로 처리(0나누기 방지)."""
    out = []
    for v, (name, hmin, hmax, dmin, dmax, gated) in zip(raw, DG5F_CHANNELS):
        if gated:
            out.append(GATED_NEUTRAL_DEG)
            continue
        if name == "thumb_cmc":
            # ★thumb_cmc(1_1)는 부호가 아니라 |abd| '크기'로 접힘/벌림을 구분한다(direct 모드와
            #   동일 원리 — _thumb_abduction 원값은 항상 음수, 작은|v|=접힘·큰|v|=벌림). 균일 비율식을
            #   그대로 쓰면 v가 hmin(큰 음수)일수록 t=0→rmin이 돼 '벌림→접힘'으로 안/바깥이 뒤집힌다.
            #   → |v|를 사람 [|min|,|max|] 비율로 바꿔 로봇 [접힘(rmin), 벌림(rmax)]에 실어 방향을 맞춘다.
            rmin, rmax = _robot_range(name, dmin, dmax)   # URDF (-77접힘, +22벌림)
            a_lo, a_hi = min(abs(hmin), abs(hmax)), max(abs(hmin), abs(hmax))
            aspan = a_hi - a_lo
            t = (abs(v) - a_lo) / aspan if aspan > 1e-9 else 0.0
            t = min(1.0, max(0.0, t))
            deg = rmin + t * (rmax - rmin)
        elif name == "thumb_opp":
            # ★thumb_opp(1_2)는 '깊이 방향(양의 대향)'만 유효한 단방향 동작이다(direct의 max(0,·)와
            #   동일). 프록시 _thumb_opposition은 양·음(대향/후퇴) 양방향이라 균일 비율식을 쓰면 rest(v≈0)가
            #   중간값으로 매핑돼 반쯤 대향된 채 시작하고, 게다가 보정값(0.013~0.428)이 옛 프록시 기준이라
            #   조기 포화(→ 반응 소실)한다. → 양의 대향 성분만 [0, THUMB_OPP_RATIO_HI] 비율로 로봇 대향
            #   범위(dmax=−155, URDF 방향·크기)에 실는다. 부호는 direct와 동일 → 아래 left-mirror가 좌수 URDF(+)로 맞춤.
            opp = max(0.0, THUMB_OPP_SIGN * v)
            t = min(1.0, opp / THUMB_OPP_RATIO_HI) if THUMB_OPP_RATIO_HI > 1e-9 else 0.0
            deg = t * THUMB_OPP_RATIO_MAX_DEG            # 0(rest) .. MAX(−95, 사람 대향 한계)
        elif name in ABDUCTION_CHANNELS:
            # ★벌림은 0중심 양방향 신호(v=0=중립=손가락 평행). 균일 비율식은 중립을 0°로 안 보내
            #   (예 pinky_lat v=0→+41.6°) rest에서 손가락이 옆으로 휘어버림 → 0을 중심으로 각 방향을
            #   독립 선형매핑해 v=0→0°를 보장한다. 방향(부호)은 균일식과 동일 → 방향감 불변, 오프셋만 제거.
            rmin, rmax = _robot_range(name, dmin, dmax)
            if v >= 0:
                t = v / hmax if hmax > 1e-9 else 0.0      # 0..1 (v: 0..사람 양(+)쪽 최대)
                deg = min(1.0, t) * rmax
            else:
                t = v / hmin if hmin < -1e-9 else 0.0     # 0..1 (v: 0..사람 음(−)쪽 최대)
                deg = min(1.0, t) * rmin
        else:
            rmin, rmax = _robot_range(name, dmin, dmax)
            span = hmax - hmin
            t = (v - hmin) / span if abs(span) > 1e-9 else 0.0
            t = min(1.0, max(0.0, t))                     # 사람 비율 [0,1]
            deg = rmin + t * (rmax - rmin)                # 로봇 비율 → 로봇각
            deg = min(max(rmin, rmax), max(min(rmin, rmax), deg))  # 안전 clamp
        if hand == "left" and name in LEFT_MIRROR_CHANNELS:
            deg = -deg
        out.append(deg)
    return out


def map_to_dg5f(raw, hand="right", mode="direct"):
    """사람 관절 프록시(rad) → DG5F 로봇 관절각(deg).
    mode="direct"(기본): **전 채널 1:1 직접매핑**(2026-07-21). 사람 관절각을 그대로 로봇각으로
      보내고 로봇 가동범위[dmin,dmax]로만 clamp. 사람이 로봇 한계보다 크게 움직이면 포화.
      옛 percentile 선형이 만들던 증폭(예 index_mcp 4.8×)을 근본 제거. hmin/hmax 미사용.
    mode="ratio": **비율(0~1) 정규화 매핑**. [사람min,사람max]→[0,1]→[로봇min,로봇max](_map_ratio 참조).
      사람/로봇 범위를 각각 0~1 비율로 바꿔 사람비율을 로봇비율로 그대로 옮긴다."""
    if mode == "ratio":
        return _map_ratio(raw, hand)
    out = []
    for v, (name, _hmin, _hmax, dmin, dmax, gated) in zip(raw, DG5F_CHANNELS):
        if gated:
            out.append(GATED_NEUTRAL_DEG)
            continue
        if name in ABDUCTION_CHANNELS:
            # 벌림 1:1: 사람 벌림각(rad)→deg × ABD_GAIN, [dmin,dmax] clamp. raw=0→0°(중립) 자동 성립.
            # middle_abd는 기준(중지 자기 자신)이라 v≈0 → deg≈0 (0나누기 없음 — 1:1은 나눗셈 안 함).
            deg = math.degrees(v) * ABD_GAIN
            deg = min(dmax, max(dmin, deg))
        elif name == "thumb_opp":
            # 깊이 대향 1:1: THUMB_OPP_SIGN이 정한 '대향 방향' 성분만 취해 로봇 깊이각으로.
            #   dmin=0/dmax=-155 부호 맞춰 음수. 대향 방향이 반대면 THUMB_OPP_SIGN 뒤집기.
            deg = -max(0.0, THUMB_OPP_SIGN * math.degrees(v)) * THUMB_OPP_GAIN
            deg = max(dmax, min(dmin, deg))
        elif name == "thumb_cmc":
            # 벌림/접힘: |abd|(벌림 크기, 방향·거울 불변)를 [H_FOLD,H_SPREAD]→[FOLD_DEG,SPREAD_DEG] 선형.
            #   작은 |abd|(접힘)→FOLD_DEG(−), 큰 |abd|(벌림)→SPREAD_DEG(+). 왼손 각 직접 산출(미러 제외).
            span = THUMB_CMC_H_SPREAD - THUMB_CMC_H_FOLD
            t = (abs(v) - THUMB_CMC_H_FOLD) / span if span > 1e-9 else 0.0
            t = min(1.0, max(0.0, t))
            deg = THUMB_CMC_FOLD_DEG + t * (THUMB_CMC_SPREAD_DEG - THUMB_CMC_FOLD_DEG)
        else:
            # 굽힘(flexion) 1:1: 사람 관절각(_bend는 항상 ≥0, rad)→deg 그대로, [0,dmax(로봇최대)] clamp.
            # 사람이 로봇 기구한계보다 더 굽히면 한계각에서 포화(사용범위 내내 1:1, 끝에서만 어긋남).
            deg = math.degrees(v)
            deg = min(dmax, max(dmin, deg))
        if hand == "left" and name in LEFT_MIRROR_CHANNELS:
            deg = -deg
        out.append(deg)
    return out

# 엄지 손끝 위치 리타게팅
PINCH_ON, PINCH_OFF = 0.30, 0.42

# 사람 엄지 직진도 상한
DEFAULT_THUMB_STRAIGHT = 0.97
CALIB_VERSION = 3  # v3: thumb_straight_ratio(직진도). v2 thumb_reach_ratio(직선/손길이)는 폐기.
_thumb_straight_calib = None

# 손가락(검지~새끼) 직진도 상한
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
#   v6 <72f>: + [52..71] compute_raw()의 20채널 **라디안 원값**(매핑·필터 전, 프록시 출력 그대로) —
#             디버그용. Unity가 비전 라디안 vs 로봇 관절 라디안을 직접 비교/로깅하는 데 씀.
# ⚠️ 수신기 판별이 `>=`라 상위 버전을 하위 Unity에 쏴도 앞부분만 읽고 뒤는 무시된다(양방향 호환).
#    v6를 v5까지 아는 Unity에 쏴도 라디안 필드만 무시, 나머지 동작 불변.
# =========================================================================
PACKET_FMT = "<72f"
PACKET_LEN = 72


def _anat_frame(lm):
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
    ver = int(data.get("version", 1))
    ts = data.get("thumb_straight_ratio")
    if ver >= CALIB_VERSION and ts:
        _thumb_straight_calib = float(ts)
        print(f"[dg5f_angles] 엄지 직진도 상한 보정값 사용: {_thumb_straight_calib:.3f}")
    else:
        print(f"[dg5f_angles] 엄지 직진도 상한 = 기본값 {DEFAULT_THUMB_STRAIGHT} "
              f"(보정 파일 v{ver}, thumb_straight_ratio 없음 — 무보정 동작 정상. "
              "더 정밀히 맞추려면 calibrate_dg5f.py 재실행). human_ranges는 그대로 사용.")
    # 보정 범위 폭이 0이면 기본값 유지 — map_to_dg5f가 폭 0이면 t=0(dg_min 고정)이 되는 함정.
    # 예: middle_abd는 기준(중지)과 자기 자신 비교라 정의상 항상 0 → 보정이 (0,0)을 저장함
    # (2026-07-20 실보정에서 발각: 중지 3_1이 -20°로 상시 누움).
    def _rng(n, hmin, hmax):
        if n in hr:
            lo, hi = float(hr[n]["min"]), float(hr[n]["max"])
            if hi - lo > 1e-6:
                return lo, hi
            print(f"[dg5f_angles] {n}: 보정 범위 폭 0 — 기본값({hmin}, {hmax}) 유지")
        return hmin, hmax

    DG5F_CHANNELS = [
        (n, *_rng(n, hmin, hmax), dmin, dmax, g)
        for (n, hmin, hmax, dmin, dmax, g) in DG5F_CHANNELS]
    return True


if _load_calibration():
    print(f"[dg5f_angles] 보정값 로드됨: {_CALIB_PATH}")
