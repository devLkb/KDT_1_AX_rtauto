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


def compute_raw(lm):
    return [
        # 엄지
        _thumb_abduction(lm),                          # 엄지cmc→mcp 벡터와 손목→검지mcp(0→5) 벡터의 손바닥 평면 성분만을 남겨 얼만큼 벌려져 있는지 각도. 엄지 cmc의 수평 성분
        _thumb_elevation(lm),                          # 엄지 손가락이 손바닥 평면(손목→검지, 손목→소지 평면)으로부터 얼만큼 각도로 들려 있는지 각도. 즉 엄지 cmc의 수직 성분
        _bend(lm, THUMB[0], THUMB[1], THUMB[2]),       # thumb_mcp(2번) 관절의 각도
        _bend(lm, THUMB[1], THUMB[2], THUMB[3]),       # thumb_ip(3번) 관절의 각도
        # 검지
        _abduction(lm, INDEX),                         # 중지의 근위지골(9→10) 방향을 기준으로 검지가 얼마나 좌우로 벌어졌는지 각도
        _bend(lm, WRIST, INDEX[0], INDEX[1]),          # index_mcp(5번) 관절의 각도
        _bend(lm, INDEX[0], INDEX[1], INDEX[2]),       # index_pip(6번) 관절의 각도
        _bend(lm, INDEX[1], INDEX[2], INDEX[3]),       # index_dip(7번) 관절의 각도
        # 중지
        _abduction(lm, MIDDLE),                        # middle_abd — 기준(9→10)과 자기 자신 비교라 항상 ≈0 (중립 유지용)
        _bend(lm, WRIST, MIDDLE[0], MIDDLE[1]),        # middle_mcp(9번) 관절의 각도
        _bend(lm, MIDDLE[0], MIDDLE[1], MIDDLE[2]),    # middle_pip(10번) 관절의 각도
        _bend(lm, MIDDLE[1], MIDDLE[2], MIDDLE[3]),    # middle_dip(11번) 관절의 각도
        # 약지
        _abduction(lm, RING),                          # ring_abd — 중지(9→10) 기준 약지 벌림각
        _bend(lm, WRIST, RING[0], RING[1]),            # ring_mcp(13번) 관절의 각도
        _bend(lm, RING[0], RING[1], RING[2]),          # ring_pip(14번) 관절의 각도
        _bend(lm, RING[1], RING[2], RING[3]),          # ring_dip(15번) 관절의 각도
        # 새끼
        _palm_fold(lm),                                # pinky_cmc → 손바닥접기 각도, 5_1 대응용
        _abduction(lm, PINKY),                         # pinky_lat → 중지의 근위지골(9→10) 방향을 기준으로 새끼가 얼마나 좌우로 벌어졌는지 각도, 5_2 대응용
        _bend(lm, WRIST, PINKY[0], PINKY[1]),          # pinky_mcp(17번) 관절의 각도, 5_3 대응용
        (_bend(lm, PINKY[0], PINKY[1], PINKY[2])       # pinky_mcp(18번), pinky_mcp(19번) 관절의 각도, 5_4 대응용
         + _bend(lm, PINKY[1], PINKY[2], PINKY[3])) / 2.0,       
    ]

# 채널 테이블:
GATED_NEUTRAL_DEG = 0.0

# 왼손 모델용 부호 반전 채널
LEFT_MIRROR_CHANNELS = {
    "thumb_cmc", "thumb_opp", "thumb_mcp", "thumb_ip",
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

# 엄지 깊이 대향(thumb_opp)도 진짜 각도형 프록시(_thumb_elevation=arcsin, 0=평면)라 벌림과 같은
#   1:1 직접 매핑으로 뺀다 — 보정(사람 24°→로봇 105° 4.4배)이 만든 증폭 제거. 사람 대향각(들림)을
#   그대로 로봇 깊이각으로. GAIN 1.0=무증폭(자연스럽지만 대향 약함). 파지 도달 부족하면 ↑(단 평면 대비
#   깊이가 다시 커지면 엄지가 앞으로 기우는 문제 재발 — 평면 cmc 스윙 ~63°와 균형 보며 조정).
THUMB_OPP_GAIN = 1.0

# 엄지 평면 벌림(thumb_cmc, 1_1)도 진짜 각도형 프록시(_thumb_abduction=평면상 부호각)라
#   thumb_opp와 같은 1:1 직접 매핑으로 뺀다 — 옛 선형[-0.52,-0.03]rad→[65,0]°이 만든 ~1.66배
#   증폭(2026-07-21 로그 실측: 사람 벌림 34° → 로봇 56°) 제거. 사람 벌림각(deg)을 그대로 로봇
#   벌림각으로. v<0=벌림만 취해 양수 deg(우수 프레임), 왼손은 LEFT_MIRROR가 부호 반전.
#   GAIN 1.0=무증폭. 파지 시 벌림이 부족하면 ↑(단 사람 대비 과장되면 다시 부자연). ⚠️우수(right)
#   방향은 미검증 — 현재 방향 보존은 왼손 기준(라이브에서 벌림 추종 방향 재확인 필요).
THUMB_CMC_GAIN = 1.0

DG5F_CHANNELS = [
    # name          hmin   hmax    dg_min  dg_max  gated
    # thumb_cmc(1_1): 2026-07-21 1:1 직접매핑으로 전환(map_to_dg5f 아래 thumb_cmc 분기 참조).
    #   dmin/dmax는 이제 선형보간 끝점이 아니라 clamp 경계[0, 65]°(우수 프레임, 양수=벌림)로 쓰인다.
    #   hmin/hmax(-0.52,-0.03)는 1:1 분기에서 미사용(보정 로드가 덮어써도 무해).
    ("thumb_cmc",  -0.52, -0.03,    0.0,   65.0,  False),
    # thumb_opp(1_2, 깊이 대향): dmax -120→-75 + dmin -15→0(2026-07-20) — ①사람 대향각(~24°)이
    #   로봇 105°로 4.4배 증폭돼 깊이 스윙(81°)이 평면 스윙(62°)을 압도 → -75로 축소.
    #   ②dmin -15는 사람 엄지가 평평(elev≈0)해도 로봇을 항상 15° 앞으로 기울이는 '상시 깊이 오프셋'
    #   (로그 실측 opp 최소 16°) → 0으로 바꿔 평평한 엄지는 평면에 눕힘. 최대 대향(dmax)은 불변.
    #   (파지 도달은 여전히 희생 — 정밀 파지는 IK 모드 담당.)
    ("thumb_opp",   0.15,  0.85,    0.0,  -75.0,  False),
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

def map_to_dg5f(raw, hand="right"):
    out = []
    for v, (name, hmin, hmax, dmin, dmax, gated) in zip(raw, DG5F_CHANNELS):
        if gated:
            out.append(GATED_NEUTRAL_DEG)
            continue
        if name in ABDUCTION_CHANNELS:
            # 1:1 직접 매핑: 사람 벌림각(rad)→deg × ABD_GAIN, 로봇 가동범위[dmin,dmax]로 clamp.
            # raw=0→0°(중지 정렬)은 자동 성립. 좁은 percentile 범위 증폭 문제를 근본 제거.
            deg = math.degrees(v) * ABD_GAIN
            deg = min(dmax, max(dmin, deg))
        elif name == "thumb_opp":
            # 깊이 대향도 1:1 직접(무증폭): 사람 들림각(v≥0)을 그대로 로봇 깊이각으로.
            # dmin=0(평면)/dmax=-75(최대 대향) 부호에 맞춰 음수로, [dmax,dmin]=[-75,0] clamp.
            deg = -math.degrees(max(0.0, v)) * THUMB_OPP_GAIN
            deg = max(dmax, min(dmin, deg))
        elif name == "thumb_cmc":
            # 평면 벌림 1:1 직접(무증폭): 사람 벌림각(v<0=벌림)을 그대로 로봇 벌림각으로.
            # v<0만 취해 양수 deg(우수 프레임), v≥0(모음/역)은 0. 왼손은 아래 LEFT_MIRROR가 부호 반전.
            # dmin=0/dmax=65로 로봇 벌림 가동범위만 clamp(옛 선형보간의 1.66배 증폭 제거).
            deg = -math.degrees(min(0.0, v)) * THUMB_CMC_GAIN
            deg = min(dmax, max(dmin, deg))
        else:
            t = (v - hmin) / (hmax - hmin) if hmax > hmin else 0.0
            t = min(1.0, max(0.0, t))
            deg = dmin + t * (dmax - dmin)
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
