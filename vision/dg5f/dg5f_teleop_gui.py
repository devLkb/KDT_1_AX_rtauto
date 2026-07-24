# -*- coding: utf-8 -*-
"""DG5F 텔레오퍼레이션 GUI — 웹캠+MediaPipe로 손을 20관절 각도[deg]로 만들어 UDP로 쏘되,
송신 대상(IP/포트)·모드·보정값(사람 범위)·관절 제한(로봇 범위)·관절 각도(수동)를
**실행 중에 UI로 바꿔** 볼 수 있는 컨트롤 패널.  (headless 버전=vision_node_dg5f.py)

이 파일이 exe 타깃(1단계). 계산 로직은 dg5f_angles를 그대로 재사용 —
  raw = compute_raw(landmarks)            # 사람 관절 프록시(rad)
  mapped = map_to_dg5f(raw, hand, mode)   # 로봇 관절각(deg)  ← UDP로 나가는 값
UI에서 바꾼 값은 dg5f_angles의 모듈 전역(DG5F_CHANNELS / RATIO_LIMIT / 엄지 상수)에
**라이브로 반영**된다(map_to_dg5f가 호출 시점에 그 전역을 읽으므로 재시작 불필요).

패킷은 vision_node와 동일한 v6 <72f>:
  [0..19] 관절각[deg] / [20..22] 엄지 tip / [23] 핀치 / [24] 끝거리비
  / [25..36] 손가락 리치 / [37..51] 손목→끝 / [52..71] 라디안 원값(디버그)
→ Unity Dg5fReceiver(sim)와 dg5f_sdk_bridge(real) 둘 다 그대로 받는다.

실행:  <vision venv python> dg5f_teleop_gui.py
"""
import json
import os
import socket
import struct
import sys
import time

import cv2
import mediapipe as mp
import numpy as np
from PIL import Image, ImageTk

import tkinter as tk
from tkinter import ttk, filedialog, messagebox

from one_euro_filter import OneEuroFilter
import dg5f_angles as A

# ------------------------- 기본 설정 (vision_node와 동일 값) -------------------------
CAM_INDEX = 0
FRAME_W, FRAME_H = 640, 480
DEF_SIM_IP, DEF_SIM_PORT = "127.0.0.1", 5006      # Unity 트윈
DEF_REAL_IP, DEF_REAL_PORT = "127.0.0.1", 5007    # 실물 SDK 브리지
SEND_HZ_CAP = 60
FILTER_FREQ, FILTER_MIN_CUTOFF, FILTER_BETA = 30.0, 0.6, 0.0005
TIP_MIN_CUTOFF, TIP_BETA = 0.15, 0.5

N = 20
CH = A.CHANNEL_NAMES                       # 20 채널 이름
JOINT_ID = [f"{i // 4 + 1}_{i % 4 + 1}" for i in range(N)]   # 1_1 .. 5_4


def _base_dir():
    """exe(frozen)면 실행파일 폴더, 아니면 스크립트 폴더 — 프리셋 저장/로드 기준."""
    if getattr(sys, "frozen", False):
        return os.path.dirname(sys.executable)
    return os.path.dirname(os.path.abspath(__file__))


PRESET_PATH = os.path.join(_base_dir(), "dg5f_gui_preset.json")


def landmarks_to_xyz(hand_landmarks):
    pts = np.zeros((21, 3), dtype=np.float64)
    for i, lm in enumerate(hand_landmarks.landmark):
        pts[i] = (lm.x, lm.y, lm.z)
    return pts


# ------------------------- dg5f_angles 전역에 라이브 반영하는 헬퍼 -------------------------
def _ch_idx(ch):
    return CH.index(ch)


def get_human_range(ch):
    _n, hmn, hmx, _dmn, _dmx, _g = A.DG5F_CHANNELS[_ch_idx(ch)]
    return hmn, hmx


def set_human_range(ch, lo, hi):
    i = _ch_idx(ch)
    n, _hmn, _hmx, dmn, dmx, g = A.DG5F_CHANNELS[i]
    A.DG5F_CHANNELS[i] = (n, lo, hi, dmn, dmx, g)   # map_to_dg5f가 이 리스트를 라이브로 읽음


def get_robot_range(ch):
    """현재 매핑이 참고하는 로봇 [lo,hi](deg). ratio 우선순위(RATIO_LIMIT)를 먼저 보여준다."""
    if ch in A.RATIO_LIMIT:
        return A.RATIO_LIMIT[ch]
    _n, _hmn, _hmx, dmn, dmx, _g = A.DG5F_CHANNELS[_ch_idx(ch)]
    return dmn, dmx


def set_robot_range(ch, lo, hi):
    """로봇 범위를 direct(DG5F_CHANNELS dmin/dmax)·ratio(RATIO_LIMIT) 양쪽에 함께 기록 →
    모드 전환해도 일관. 특수 엄지 채널(cmc/opp)은 전용 상수까지 갱신."""
    i = _ch_idx(ch)
    n, hmn, hmx, _dmn, _dmx, g = A.DG5F_CHANNELS[i]
    A.DG5F_CHANNELS[i] = (n, hmn, hmx, lo, hi, g)   # direct clamp + ratio 폴백
    A.RATIO_LIMIT[ch] = (lo, hi)                    # ratio 최우선
    if ch == "thumb_cmc":                           # |abd|→[fold,spread] 선형(direct/ratio 공통)
        A.THUMB_CMC_FOLD_DEG = lo
        A.THUMB_CMC_SPREAD_DEG = hi
    elif ch == "thumb_opp":                         # 단방향 대향 최대각(ratio 전용 상수)
        A.THUMB_OPP_RATIO_MAX_DEG = hi


class TeleopGUI:
    def __init__(self, root):
        self.root = root
        root.title("DG5F 텔레오퍼레이션 컨트롤")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ---- 상태 ----
        self.hand = tk.StringVar(value="right")
        self.mapmode = tk.StringVar(value="ratio")
        self.cam_index = tk.IntVar(value=CAM_INDEX)
        self.sel_ch = tk.StringVar(value=CH[0])
        self.overrides = {}          # {ch_idx: deg}  수동 오버라이드 활성 채널
        self.ov_enabled = tk.BooleanVar(value=False)
        self._loading = False        # 슬라이더 프로그램 세팅 중 콜백 억제
        self.last_vals = None        # occlusion hold
        self.last_raw = [0.0] * N
        self.last_mapped = [0.0] * N
        self.pinch_on = False
        self.pkt_count = 0
        self._fps_t = time.time()
        self._fps_n = 0
        self.fps = 0.0

        # ---- 필터 ----
        self.filters = {n: OneEuroFilter(FILTER_FREQ, FILTER_MIN_CUTOFF, FILTER_BETA) for n in CH}
        self.tip_filters = [OneEuroFilter(FILTER_FREQ, TIP_MIN_CUTOFF, TIP_BETA) for _ in range(3)]
        self.ftip_filters = [OneEuroFilter(FILTER_FREQ, TIP_MIN_CUTOFF, TIP_BETA)
                             for _ in range(3 * len(A.TIP_FINGERS))]
        self.wtip_filters = [OneEuroFilter(FILTER_FREQ, TIP_MIN_CUTOFF, TIP_BETA)
                             for _ in range(3 * len(A.WRIST_TIP_FINGERS))]
        self.pinch_filter = OneEuroFilter(FILTER_FREQ, FILTER_MIN_CUTOFF, 0.001)

        # ---- 네트워크 ----
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.last_send = 0.0

        # ---- MediaPipe / 카메라 ----
        self.mp_hands = mp.solutions.hands.Hands(
            model_complexity=1, max_num_hands=1,
            min_detection_confidence=0.6, min_tracking_confidence=0.6)
        self.cap = None
        self._open_camera()

        self._build_ui()
        self._load_channel_into_sliders(CH[0])
        self.root.after(10, self.tick)

    # ============================ 카메라 ============================
    def _open_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = cv2.VideoCapture(self.cam_index.get())
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

    # ============================ UI 구성 ============================
    def _build_ui(self):
        root = self.root
        main = ttk.Frame(root, padding=6)
        main.grid(row=0, column=0, sticky="nsew")

        # ---- 좌: 영상 ----
        self.video = ttk.Label(main)
        self.video.grid(row=0, column=0, rowspan=3, sticky="nw", padx=(0, 8))

        # ---- 우: 컨트롤 ----
        ctrl = ttk.Frame(main)
        ctrl.grid(row=0, column=1, sticky="nsew")

        # (1) 연결 대상
        f = ttk.LabelFrame(ctrl, text="① 송신 대상 (IP·포트)", padding=6)
        f.grid(row=0, column=0, sticky="ew", pady=3)
        self.sim_on = tk.BooleanVar(value=True)
        self.real_on = tk.BooleanVar(value=False)
        self.sim_ip = tk.StringVar(value=DEF_SIM_IP)
        self.sim_port = tk.StringVar(value=str(DEF_SIM_PORT))
        self.real_ip = tk.StringVar(value=DEF_REAL_IP)
        self.real_port = tk.StringVar(value=str(DEF_REAL_PORT))
        ttk.Checkbutton(f, text="Sim (Unity)", variable=self.sim_on).grid(row=0, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.sim_ip, width=15).grid(row=0, column=1)
        ttk.Entry(f, textvariable=self.sim_port, width=6).grid(row=0, column=2)
        ttk.Checkbutton(f, text="Real (로봇/브리지)", variable=self.real_on).grid(row=1, column=0, sticky="w")
        ttk.Entry(f, textvariable=self.real_ip, width=15).grid(row=1, column=1)
        ttk.Entry(f, textvariable=self.real_port, width=6).grid(row=1, column=2)
        ttk.Label(f, text="※ 다른 PC로 쏘려면 그 PC의 LAN IP 입력 (같은 WiFi, 방화벽 UDP 허용)",
                  foreground="#666").grid(row=2, column=0, columnspan=3, sticky="w", pady=(3, 0))

        # (2) 모드
        f = ttk.LabelFrame(ctrl, text="② 모드", padding=6)
        f.grid(row=1, column=0, sticky="ew", pady=3)
        ttk.Label(f, text="손:").grid(row=0, column=0)
        ttk.Radiobutton(f, text="right", value="right", variable=self.hand).grid(row=0, column=1)
        ttk.Radiobutton(f, text="left(미러)", value="left", variable=self.hand).grid(row=0, column=2)
        ttk.Label(f, text="매핑:").grid(row=0, column=3, padx=(10, 0))
        ttk.Radiobutton(f, text="direct", value="direct", variable=self.mapmode).grid(row=0, column=4)
        ttk.Radiobutton(f, text="ratio", value="ratio", variable=self.mapmode).grid(row=0, column=5)
        ttk.Label(f, text="cam#").grid(row=1, column=0, pady=(4, 0))
        ttk.Spinbox(f, from_=0, to=8, width=4, textvariable=self.cam_index).grid(row=1, column=1, pady=(4, 0))
        ttk.Button(f, text="카메라 재연결", command=self._open_camera).grid(row=1, column=2, columnspan=2, pady=(4, 0))

        # (3) 채널 파라미터 편집
        f = ttk.LabelFrame(ctrl, text="③ 채널별 파라미터 (라이브)", padding=6)
        f.grid(row=2, column=0, sticky="ew", pady=3)
        ttk.Label(f, text="채널:").grid(row=0, column=0, sticky="w")
        self.ch_combo = ttk.Combobox(f, textvariable=self.sel_ch, width=22, state="readonly",
                                     values=[f"{JOINT_ID[i]}  {CH[i]}" for i in range(N)])
        self.ch_combo.current(0)
        self.ch_combo.grid(row=0, column=1, columnspan=3, sticky="w")
        self.ch_combo.bind("<<ComboboxSelected>>", self._on_channel_change)

        self.s_hmin = self._mk_scale(f, 1, "사람 min (rad)", -1.8, 1.8, 0.01, self._on_human)
        self.s_hmax = self._mk_scale(f, 2, "사람 max (rad)", -1.8, 1.8, 0.01, self._on_human)
        self.s_rlo = self._mk_scale(f, 3, "로봇 lo (deg)", -160, 160, 1, self._on_robot)
        self.s_rhi = self._mk_scale(f, 4, "로봇 hi (deg)", -160, 160, 1, self._on_robot)

        ov = ttk.Frame(f)
        ov.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        ttk.Checkbutton(ov, text="수동 오버라이드 (손 무시하고 이 각도로 송신)",
                        variable=self.ov_enabled, command=self._on_override_toggle).pack(anchor="w")
        self.s_manual = self._mk_scale(f, 6, "수동 각도 (deg)", -160, 160, 1, self._on_manual)

        ttk.Label(f, text="※ '로봇 lo/hi'는 direct=clamp범위·ratio=정규화범위 양쪽에 반영. "
                          "엄지 1_1은 lo=접힘/hi=벌림, 1_2는 hi=대향최대(음수).",
                  foreground="#666", wraplength=340).grid(row=7, column=0, columnspan=4, sticky="w", pady=(3, 0))

        # (4) 라이브 판독
        f = ttk.LabelFrame(ctrl, text="④ 선택 채널 실시간 값", padding=6)
        f.grid(row=3, column=0, sticky="ew", pady=3)
        self.lbl_read = ttk.Label(f, text="-", font=("Consolas", 10))
        self.lbl_read.pack(anchor="w")

        # (5) 프리셋 저장/불러오기
        f = ttk.Frame(ctrl)
        f.grid(row=4, column=0, sticky="ew", pady=3)
        ttk.Button(f, text="프리셋 저장", command=self.save_preset).pack(side="left", padx=2)
        ttk.Button(f, text="프리셋 불러오기", command=self.load_preset).pack(side="left", padx=2)
        ttk.Button(f, text="채널 리셋", command=self.reset_channel).pack(side="left", padx=2)

        # 상태바
        self.status = ttk.Label(root, text="", anchor="w", relief="sunken")
        self.status.grid(row=1, column=0, sticky="ew")
        root.columnconfigure(0, weight=1)

    def _mk_scale(self, parent, row, label, lo, hi, res, cmd):
        ttk.Label(parent, text=label, width=16).grid(row=row, column=0, sticky="w")
        var = tk.DoubleVar()
        s = tk.Scale(parent, from_=lo, to=hi, resolution=res, orient="horizontal",
                     length=260, variable=var, command=lambda _v: cmd())
        s.grid(row=row, column=1, columnspan=3, sticky="w")
        s.var = var
        return s

    # ============================ 채널 편집 콜백 ============================
    def _cur_ch(self):
        return CH[self.ch_combo.current()]

    def _on_channel_change(self, _e=None):
        self._load_channel_into_sliders(self._cur_ch())

    def _load_channel_into_sliders(self, ch):
        self._loading = True
        hmn, hmx = get_human_range(ch)
        rlo, rhi = get_robot_range(ch)
        self.s_hmin.set(round(hmn, 3))
        self.s_hmax.set(round(hmx, 3))
        self.s_rlo.set(round(rlo, 1))
        self.s_rhi.set(round(rhi, 1))
        i = _ch_idx(ch)
        self.ov_enabled.set(i in self.overrides)
        self.s_manual.set(self.overrides.get(i, 0.0))
        self._loading = False

    def _on_human(self):
        if self._loading:
            return
        set_human_range(self._cur_ch(), self.s_hmin.var.get(), self.s_hmax.var.get())

    def _on_robot(self):
        if self._loading:
            return
        set_robot_range(self._cur_ch(), self.s_rlo.var.get(), self.s_rhi.var.get())

    def _on_override_toggle(self):
        i = _ch_idx(self._cur_ch())
        if self.ov_enabled.get():
            self.overrides[i] = self.s_manual.var.get()
        else:
            self.overrides.pop(i, None)

    def _on_manual(self):
        if self._loading:
            return
        if self.ov_enabled.get():
            self.overrides[_ch_idx(self._cur_ch())] = self.s_manual.var.get()

    def reset_channel(self):
        """선택 채널을 dg5f_angles 원본 기본값으로 되돌린다(모듈 재로딩 없이 근사 복원은 어려워 안내만)."""
        messagebox.showinfo("채널 리셋",
                            "원본 기본값 복원은 프리셋 불러오기로 하거나 프로그램을 재시작하세요.\n"
                            "(현재 세션에서 바꾼 값만 프리셋에 저장됩니다.)")

    # ============================ 메인 루프 ============================
    def tick(self):
        t0 = time.time()
        ok, frame = self.cap.read()
        if ok:
            frame = cv2.flip(frame, 1)
            res = self.mp_hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            detected = bool(res.multi_hand_landmarks)
            hand = self.hand.get()
            mode = self.mapmode.get()

            if detected:
                mp.solutions.drawing_utils.draw_landmarks(
                    frame, res.multi_hand_landmarks[0], mp.solutions.hands.HAND_CONNECTIONS)
                xyz = landmarks_to_xyz(res.multi_hand_landmarks[0])
                raw = A.compute_raw(xyz)
                mapped = A.map_to_dg5f(raw, hand, mode)
                self.last_raw = list(raw)
                self.last_mapped = list(mapped)
                self._pack_and_send(mapped, raw, xyz)
            elif self.overrides:
                # 손 없어도 오버라이드가 있으면 중립(0)에 오버라이드만 얹어 송신(장비 단독 테스트)
                mapped = [0.0] * N
                self._pack_and_send(mapped, self.last_raw, None)
            elif self.last_vals is not None:
                self._send_packet(self.last_vals + self.last_raw)   # occlusion hold

            self._show_frame(frame, detected)

        self._update_readout()
        # FPS
        self._fps_n += 1
        if t0 - self._fps_t >= 1.0:
            self.fps = self._fps_n / (t0 - self._fps_t)
            self._fps_t, self._fps_n = t0, 0
        self.root.after(1, self.tick)

    def _pack_and_send(self, mapped, raw, xyz):
        # 수동 오버라이드 적용
        for i, deg in self.overrides.items():
            mapped[i] = deg
        vals_ang = [self.filters[n](v) for n, v in zip(CH, mapped)]

        if xyz is not None:
            tip, pinch_d = A.compute_thumb_tip(xyz)
            self.pinch_on = (pinch_d < A.PINCH_OFF) if self.pinch_on else (pinch_d < A.PINCH_ON)
            ftips = A.compute_finger_tips(xyz)
            wtips = A.compute_wrist_tip_vectors(xyz)
            tip_f = [f(v) for f, v in zip(self.tip_filters, tip)]
            ftips_f = [f(v) for f, v in zip(self.ftip_filters, ftips)]
            wtips_f = [f(v) for f, v in zip(self.wtip_filters, wtips)]
            vals = (vals_ang + tip_f + [1.0 if self.pinch_on else 0.0]
                    + [self.pinch_filter(pinch_d)] + ftips_f + wtips_f)
        else:
            # 손 없는 오버라이드 송신 — 각도만 채우고 나머지는 0
            vals = vals_ang + [0.0] * 32

        self.last_vals = vals
        self._send_packet(vals + list(raw))

    def _send_packet(self, payload72):
        now = time.time()
        if now - self.last_send < 1.0 / SEND_HZ_CAP:
            return
        self.last_send = now
        try:
            pkt = struct.pack(A.PACKET_FMT, *payload72)
        except struct.error:
            return
        for on, ip, port in ((self.sim_on.get(), self.sim_ip.get(), self.sim_port.get()),
                             (self.real_on.get(), self.real_ip.get(), self.real_port.get())):
            if not on:
                continue
            try:
                self.sock.sendto(pkt, (ip.strip(), int(port)))
                self.pkt_count += 1
            except (OSError, ValueError):
                pass

    def _show_frame(self, frame, detected):
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.video.configure(image=img)
        self.video.image = img
        tgt = []
        if self.sim_on.get():
            tgt.append(f"sim {self.sim_ip.get()}:{self.sim_port.get()}")
        if self.real_on.get():
            tgt.append(f"real {self.real_ip.get()}:{self.real_port.get()}")
        self.status.configure(
            text=f"{'손 인식' if detected else '미검출(hold)'} | {self.fps:4.1f} fps | "
                 f"pkt {self.pkt_count} | mode={self.mapmode.get()}/{self.hand.get()} | "
                 f"→ {', '.join(tgt) or '(대상 없음)'}")

    def _update_readout(self):
        i = self.ch_combo.current()
        ch = CH[i]
        raw = self.last_raw[i]
        mapped = self.last_mapped[i]
        sent = self.filters[ch]._x.last                # 필터 후 = 실제 UDP 전송값
        sent = float("nan") if sent is None else sent
        ov = f"  [OVERRIDE {self.overrides[i]:+.0f}]" if i in self.overrides else ""
        self.lbl_read.configure(
            text=f"{JOINT_ID[i]} {ch}\n"
                 f"raw   = {raw:+.4f} rad ({np.degrees(raw):+7.1f} deg)\n"
                 f"mapped= {mapped:+7.1f} deg{ov}\n"
                 f"sent  = {sent:+7.1f} deg  (필터후=UDP)")

    # ============================ 프리셋 ============================
    def _collect(self):
        return {
            "hand": self.hand.get(), "mapmode": self.mapmode.get(),
            "sim": [self.sim_on.get(), self.sim_ip.get(), self.sim_port.get()],
            "real": [self.real_on.get(), self.real_ip.get(), self.real_port.get()],
            "human_ranges": {ch: get_human_range(ch) for ch in CH},
            "robot_ranges": {ch: get_robot_range(ch) for ch in CH},
            "overrides": {CH[i]: v for i, v in self.overrides.items()},
        }

    def save_preset(self):
        path = filedialog.asksaveasfilename(initialfile=os.path.basename(PRESET_PATH),
                                            initialdir=_base_dir(), defaultextension=".json",
                                            filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(self._collect(), fp, ensure_ascii=False, indent=2)
        messagebox.showinfo("저장", f"프리셋 저장됨:\n{path}")

    def load_preset(self):
        path = filedialog.askopenfilename(initialdir=_base_dir(), filetypes=[("JSON", "*.json")])
        if not path:
            return
        with open(path, encoding="utf-8") as fp:
            d = json.load(fp)
        self.hand.set(d.get("hand", "right"))
        self.mapmode.set(d.get("mapmode", "ratio"))
        for key, on_v, ip_v, port_v in (("sim", self.sim_on, self.sim_ip, self.sim_port),
                                        ("real", self.real_on, self.real_ip, self.real_port)):
            if key in d:
                on, ip, port = d[key]
                on_v.set(on); ip_v.set(ip); port_v.set(str(port))
        for ch, (lo, hi) in d.get("human_ranges", {}).items():
            if ch in CH:
                set_human_range(ch, lo, hi)
        for ch, (lo, hi) in d.get("robot_ranges", {}).items():
            if ch in CH:
                set_robot_range(ch, lo, hi)
        self.overrides = {_ch_idx(ch): v for ch, v in d.get("overrides", {}).items() if ch in CH}
        self._load_channel_into_sliders(self._cur_ch())
        messagebox.showinfo("불러오기", f"프리셋 적용됨:\n{path}")

    # ============================ 종료 ============================
    def on_close(self):
        try:
            if self.cap is not None:
                self.cap.release()
            self.sock.close()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    TeleopGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
