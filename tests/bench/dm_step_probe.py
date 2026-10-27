#!/usr/bin/env python3
"""
阶跃响应实验：给定 +5° 小步进，量「静差 / 超调 / 上升时间」随 Kp、Kd 的变化。

为什么要做：判据 2「到位并保持 <1°」本质是**稳态误差**，而
    e_ss = (τ_ext − τ_ff) / Kp
所以同一台轴、同一个外载下，改 Kp 应该看到 e_ss 成反比变化 —— 这条能验证
我们的力矩公式理解是否正确，还能反推轴上的实际外载力矩 τ_ext（后面算 τ_ff 要用）。

安全：
  · 步进固定 ±5°（0.0873 rad），最大力矩 = Kp × 0.0873 ≤ 4.4 N·m（Kp=50）
  · 每级开始前读当前角度，目标 = 当前 + 5°，测完回到起点
  · 先清错、TIMEOUT≈90 ms；任何 ERR≠1 立刻失能退出；结束恢复 TIMEOUT=0

用法：python3 tests/bench/dm_step_probe.py [串口] [CAN_ID]
"""
from __future__ import annotations

import statistics
import struct
import sys
import time

sys.path.insert(0, "/Users/a1234/Embedded/custom_controller_ai/pc")
from usb2can_classic import Usb2CanClassic  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem00000000050C1"
CAN_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
MST_ID = 0x11
PARAM_ID = 0x7FF
HZ = 200
PMAX, VMAX, TMAX = 12.5, 10.0, 28.0
STEP = 0.0873          # 5°
TIMEOUT = 3000
EN, DIS, CLR = (bytes([0xFF] * 7 + [c]) for c in (0xFC, 0xFD, 0xFB))

GAINS = [(10.0, 0.2), (20.0, 0.4), (30.0, 0.5), (50.0, 0.8)]


def f2u(x, lo, hi, bits):
    r = (max(lo, min(hi, x)) - lo) / (hi - lo)
    return int(r * ((1 << bits) - 1))


def u2f(u, lo, hi, bits):
    return u / ((1 << bits) - 1) * (hi - lo) + lo


def pack(p, v, kp, kd, t):
    pi = f2u(p, -PMAX, PMAX, 16)
    vi = f2u(v, -VMAX, VMAX, 12)
    ki = f2u(kp, 0, 500, 12)
    di = f2u(kd, 0, 5, 12)
    ti = f2u(t, -TMAX, TMAX, 12)
    return bytes([pi >> 8, pi & 0xFF, vi >> 4, ((vi & 0xF) << 4) | (ki >> 8),
                  ki & 0xFF, di >> 4, ((di & 0xF) << 4) | (ti >> 8), ti & 0xFF])


def dec(d):
    return (u2f((d[1] << 8) | d[2], -PMAX, PMAX, 16),
            u2f((d[3] << 4) | (d[4] >> 4), -VMAX, VMAX, 12),
            d[0] >> 4)


class Rig:
    def __init__(self, bus):
        self.bus = bus

    def poll(self, timeout=0.05):
        t0, out = time.time(), []
        while time.time() - t0 < timeout:
            for f in self.bus.recv():
                if f["id"] == MST_ID and f["cmd"] == 0x11:
                    out.append(dec(f["data"]))
            time.sleep(0.0015)
        return out

    def ask(self, kp=0.0, kd=0.0, p=0.0):
        """发一帧（默认零力矩）并返回最新反馈"""
        self.bus.send(CAN_ID, pack(p, 0.0, kp, kd, 0.0))
        r = self.poll(0.03)
        return r[-1] if r else None

    def write_reg(self, rid, val):
        self.bus.send(PARAM_ID, bytes([CAN_ID & 0xFF, 0, 0x55, rid]) + struct.pack("<I", val))
        time.sleep(0.03)
        self.poll(0.02)

    def pos(self):
        for _ in range(10):
            r = self.ask()
            if r:
                return r[0]
        return None

    def hold(self, kp, kd, p_des, sec):
        """按 200Hz 保持 p_des，采样 (t, pos)"""
        out, t0 = [], time.time()
        while time.time() - t0 < sec:
            self.bus.send(CAN_ID, pack(p_des, 0.0, kp, kd, 0.0))
            for p, v, e in self.poll(1.0 / HZ):
                out.append((time.time() - t0, p, e))
        return out


def run_gain(rig, kp, kd, p0):
    target = p0 + STEP
    samples = rig.hold(kp, kd, target, 2.0)
    if not samples:
        return None
    errs = [e for _, _, e in samples]
    if any(e != 1 for e in errs):
        return {"err": [e for e in errs if e != 1][0]}
    pos = [p for _, p, _ in samples]
    ts = [t for t, _, _ in samples]
    final = statistics.mean(pos[-20:])
    ss_err = abs(final - target)
    lo, hi = min(pos[0], final), max(pos[0], final)
    t10 = t90 = None
    for t, p in zip(ts, pos):
        if t10 is None and (p - lo) >= 0.1 * (hi - lo):
            t10 = t
        if t90 is None and (p - lo) >= 0.9 * (hi - lo):
            t90 = t
    rise = (t90 - t10) if (t10 is not None and t90 is not None) else float("nan")
    overshoot = max(0.0, (max(pos) - final)) if final > pos[0] else max(0.0, (final - min(pos)))
    # 回到起点，避免位置漂移累积
    rig.hold(kp, kd, p0, 1.2)
    return {"final": final, "ss_err": ss_err, "rise": rise, "overshoot": overshoot,
            "peak_vel": max(abs(v) for v in []) if False else 0.0}


def main() -> int:
    print(f"串口 {PORT} ｜ CAN ID {CAN_ID} ｜ MST_ID {MST_ID} ｜ 200 Hz ｜ 步进 {STEP*180/3.14159:.1f}°")
    bus = Usb2CanClassic(PORT, 921600)
    bus.open(1_000_000)
    rig = Rig(bus)
    rig.write_reg(0x09, TIMEOUT)
    bus.send(CAN_ID, CLR)
    time.sleep(0.03)
    bus.send(CAN_ID, EN)
    time.sleep(0.05)
    p0 = rig.pos()
    if p0 is None:
        print("❌ 使能后读不到反馈")
        bus.close()
        return 1
    print(f"使能成功，起始角度 p0 = {p0:.4f} rad；目标 = p0 + {STEP:.4f}\n")

    print(f"{'Kp':>5}{'Kd':>6}{'稳态值(rad)':>14}{'静差(°)':>10}{'上升10-90%(ms)':>16}{'超调(°)':>10}")
    print("-" * 64)
    rows = []
    try:
        for kp, kd in GAINS:
            r = run_gain(rig, kp, kd, p0)
            if r is None or "err" in r:
                print(f"{kp:>5.0f}{kd:>6.2f}   ❌ ERR={r.get('err') if r else '无反馈'}")
                break
            print(f"{kp:>5.0f}{kd:>6.2f}{r['final']:>14.4f}{r['ss_err']*180/3.14159:>10.3f}"
                  f"{r['rise']*1000:>16.1f}{r['overshoot']*180/3.14159:>10.3f}")
            rows.append((kp, r["ss_err"]))
    finally:
        bus.send(CAN_ID, DIS)
        time.sleep(0.05)
        rig.write_reg(0x09, 0)
        bus.close()

    if len(rows) >= 2:
        print("\n静差 × Kp 应近似为常数（= τ_ext），用它反推外载力矩：")
        for kp, e in rows:
            print(f"   Kp={kp:>5.0f} → τ_ext ≈ Kp × e = {kp*e:.3f} N·m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
