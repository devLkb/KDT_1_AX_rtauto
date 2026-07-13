"""
One Euro Filter - 실시간 손 트래킹 노이즈 억제용.

느린 움직임에서는 지연을 줄이고(부드럽게), 빠른 움직임에서는
반응성을 유지하는 적응형 저역통과 필터입니다.
각 관절 각도 채널마다 독립 인스턴스를 하나씩 씁니다.

참고: Casiez et al., "1€ Filter" (CHI 2012)
"""
import math


class _LowPass:
    def __init__(self, alpha):
        self._alpha = alpha
        self._y = None

    def __call__(self, x, alpha=None):
        if alpha is not None:
            self._alpha = alpha
        if self._y is None:
            self._y = x
        else:
            self._y = self._alpha * x + (1.0 - self._alpha) * self._y
        return self._y

    @property
    def last(self):
        return self._y


class OneEuroFilter:
    def __init__(self, freq=60.0, min_cutoff=1.0, beta=0.007, d_cutoff=1.0):
        """
        freq       : 대략적인 샘플링 주파수(Hz). 프레임레이트로 설정.
        min_cutoff : 낮출수록 더 부드럽지만 지연 증가. 0.5~1.5에서 튜닝.
        beta       : 높일수록 빠른 움직임에 더 민감(지연 감소). 0.001~0.05 튜닝.
        d_cutoff   : 미분 신호용 컷오프. 보통 1.0 고정.
        """
        self.freq = freq
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self._x = _LowPass(self._alpha(min_cutoff))
        self._dx = _LowPass(self._alpha(d_cutoff))
        self._last_x = None

    def _alpha(self, cutoff):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        te = 1.0 / self.freq
        return 1.0 / (1.0 + tau / te)

    def __call__(self, x, freq=None):
        if freq is not None and freq > 0:
            self.freq = freq

        if self._last_x is None:
            dx = 0.0
        else:
            dx = (x - self._last_x) * self.freq
        self._last_x = x

        edx = self._dx(dx, self._alpha(self.d_cutoff))
        cutoff = self.min_cutoff + self.beta * abs(edx)
        return self._x(x, self._alpha(cutoff))
