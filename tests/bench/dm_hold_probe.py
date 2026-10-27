#!/usr/bin/env python3
"""
第一次「使能 + 保持」实验（阶段 1 的判据 2、3）。

做法（逐级加变量，每级只改一个）：
  A 纯阻尼   Kp=0,   Kd=0.5, τ=0   → 手推有阻力、松手不弹（判据 3 的"最低档"）
  B 弱保持   Kp=10,  Kd=0.3        → p_des = 起始角度（不动），手推后回位
  C 中保持   Kp=30,  Kd=0.5
  D 强保持   Kp=50,  Kd=0.8

每级记录位置时间序列，输出：保持段的 均值/标准差/峰峰值，以及手推后回位误差。
安全：
  · p_des 恒等于"本级起始角度"，不会主动去一个新位置
  · τ_ff 恒为 0；Kp/Kd 从小到大
  · 先写 TIMEOUT≈90 ms（保护开着）、先发清错帧（否则故障锁存）
  · 任何一级出现 ERR≠1(使能) 立刻失能退出
  · 结束恢复 TIMEOUT=0 并失能

用法：python3 tests/bench/dm_hold_probe.py [串口] [CAN_ID]
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
TIMEOUT = 3000                      # ≈90 ms（实测 1000≈20~40 ms）

EN = bytes([0xFF] * 7 + [0xFC])
DIS = bytes([0xFF] * 7 + [0xFD])
CLR = bytes([0xFF] * 7 + [0xFB])


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


def decode(d):
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
                    out.append(decode(f["data"]))
            time.sleep(0.001)
        return out

    def read_pos(self, timeout=0.4):
        """问询式：必须先发一帧才可能收到反馈。发中点帧（不出力）再读。"""
        t0 = time.time()
        last = None
        while time.time() - t0 < timeout:
            self.bus.send(CAN_ID, pack(0.0, 0.0, 0.0, 0.0, 0.0))   # 零力矩，纯问询
            for p, v, e in self.poll(0.02):
                last = p
            if last is not None:
                return last
        return None

    def write_reg(self, rid, val):
        self.bus.send(PARAM_ID, bytes([CAN_ID & 0xFF, 0, 0x55, rid]) + struct.pack("<I", val))
        time.sleep(0.03)
        self.poll(0.02)

    def reset_and_enable(self):
        self.bus.send(CAN_ID, CLR)
        time.sleep(0.03)
        self.poll(0.02)
        self.bus.send(CAN_ID, EN)
        time.sleep(0.03)
        r = self.poll(0.03)
        return r[-1][2] if r else None

    def run(self, kp, kd, seconds, push_from, push_to, p_des):
        """按 200Hz 发 MIT 并采样；返回 (位置序列, 速度序列, 错误)"""
        pos, vel, err = [], [], None
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.bus.send(CAN_ID, pack(p_des, 0.0, kp, kd, 0.0))
            for p, v, e in self.poll(1.0 / HZ):
                pos.append(p)
                vel.append(v)
                err = e
            if err not in (None, 1):
                break
        return pos, vel, err


def stage(rig, name, kp, kd, p_des, sec=4.0):
    cur = rig.read_pos()
    if cur is None:
        print(f"\n── {name}：读不到当前位置，跳过")
        return None
    if kp > 0 and abs(cur - p_des) > 0.05:
        print(f"\n── {name}：⛔ 目标 {p_des:.3f} 与当前 {cur:.3f} 相差 "
              f"{abs(cur-p_des)*180/3.14159:.1f}°，Kp>0 会驱动轴移动 → 拒绝执行")
        return None
    p_des = cur        # 就地保持：目标 = 此刻位置（不主动移动）
    print(f"\n── {name}：Kp={kp}, Kd={kd}, p_des={p_des:.3f} rad（本级起始角）")
    print("   前 1.5 s 稳定；1.5~3 s 是【手推窗口】← 现在推轴")
    pos, vel, err = rig.run(kp, kd, sec, 1.5, 3.0, p_des)
    if err not in (None, 1):
        print(f"   ❌ ERR=0x{err:X}（非使能），本级中止")
        return None
    if not pos:
        print("   ❌ 没采到反馈")
        return None
    hold = pos[: max(1, int(len(pos) * 0.3))]
    push = pos[int(len(pos) * 0.35): int(len(pos) * 0.7)]
    back = pos[int(len(pos) * 0.8):]
    st = {
        "保持均值": statistics.mean(hold),
        "保持标准差": statistics.pstdev(hold) if len(hold) > 1 else 0.0,
        "保持峰峰值": max(hold) - min(hold) if len(hold) > 1 else 0.0,
        "手推最大偏离": max(abs(x - p_des) for x in push) if push else 0.0,
        "松手后回位误差": abs(statistics.mean(back) - p_des) if back else 0.0,
    }
    print(f"   保持：均值={st['保持均值']:.4f} rad  标准差={st['保持标准差']*1000:.2f} mrad  "
          f"峰峰={st['保持峰峰值']*180/3.14159:.3f}°")
    print(f"   手推最大偏离={st['手推最大偏离']*180/3.14159:.2f}°  "
          f"松手后回位误差={st['松手后回位误差']*180/3.14159:.3f}°")
    ok = st["松手后回位误差"] < 0.0175
    print(f"   {'✅' if ok else '⚠️'} 回位误差 {'<' if ok else '≥'} 1°（判据 2）")
    return st


def main() -> int:
    print(f"串口 {PORT} ｜ CAN ID {CAN_ID} ｜ MST_ID {MST_ID} ｜ 200 Hz ｜ TIMEOUT={TIMEOUT}")
    bus = Usb2CanClassic(PORT, 921600)
    bus.open(1_000_000)
    rig = Rig(bus)
    rig.write_reg(0x09, TIMEOUT)
    err = rig.reset_and_enable()
    print(f"清错 + 使能 → ERR={err}（1=使能）")
    if err != 1:
        print("❌ 使能失败，先查故障锁存（需要清错帧）")
        bus.close()
        return 1

    p0 = rig.read_pos()
    if p0 is None:
        print("❌ 使能后读不到反馈，退出（不冒险继续）")
        bus.send(CAN_ID, DIS)
        rig.write_reg(0x09, 0)
        bus.close()
        return 1
    print(f"当前角度 p0 = {p0:.4f} rad（= p_des，本实验不主动移动）")

    try:
        stage(rig, "A 纯阻尼（判据 3 最低档）", 0.0, 0.5, p0, sec=3.5)
        stage(rig, "B 弱保持", 5.0, 0.2, p0, sec=4.0)
        stage(rig, "B2 弱保持+", 10.0, 0.3, p0, sec=4.0)
        stage(rig, "C 中保持", 30.0, 0.5, p0, sec=4.0)
        stage(rig, "D 强保持", 50.0, 0.8, p0, sec=4.0)
    finally:
        bus.send(CAN_ID, DIS)
        time.sleep(0.03)
        rig.write_reg(0x09, 0)
        t = rig.poll(0.05)
        if t:
            print(f"\n收尾：已失能，位置={t[-1][0]:.3f} rad；TIMEOUT 已恢复 0")
        bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
