#!/usr/bin/env python3
"""
L0 探针：达妙 USB2CAN（经典版）→ DM4340，只发【失能帧】，只收反馈。

判据（L3）：
  1. 能收到反馈帧 → 说明波特率、CAN ID、接线三者至少都对
  2. 反馈帧 CAN ID = 电机配置的 MST_ID（用户配置：17）
  3. D[0] 低 4 位 = 电机 CAN ID（配置：1），高 4 位 = 状态（0 失能 / 1 使能）

安全：**不发使能、不发 MIT**，电机保持上电默认的失能状态（轴是松的）。
用法：python3 tests/bench/usb2can_probe.py [串口] [CAN_ID]
"""
from __future__ import annotations

import struct
import sys
import time

sys.path.insert(0, "/Users/a1234/Embedded/custom_controller_ai/pc")
from usb2can_classic import Usb2CanClassic  # noqa: E402  （已验证过字节格式的实现）

DEFAULT_PORT = "/dev/cu.usbmodem00000000050C1"
DEFAULT_CAN_ID = 1
CAN_BITRATE = 1_000_000

ERR_NAME = {0x0: "失能", 0x1: "使能", 0x8: "超压", 0x9: "欠压", 0xA: "过流",
            0xB: "MOS过温", 0xC: "线圈过温", 0xD: "通讯丢失", 0xE: "过载"}

P_MIN, P_MAX = -12.5, 12.5
V_MIN, V_MAX = -10.0, 10.0
T_MIN, T_MAX = -28.0, 28.0


def u2f(u: int, lo: float, hi: float, bits: int) -> float:
    return u / ((1 << bits) - 1) * (hi - lo) + lo


def decode(d: bytes) -> dict:
    pos = u2f((d[1] << 8) | d[2], P_MIN, P_MAX, 16)
    vel = u2f((d[3] << 4) | (d[4] >> 4), V_MIN, V_MAX, 12)
    tau = u2f(((d[4] & 0x0F) << 8) | d[5], T_MIN, T_MAX, 12)
    return {"id": d[0] & 0x0F, "err": d[0] >> 4, "pos": pos, "vel": vel, "tau": tau,
            "t_mos": d[6], "t_rotor": d[7]}


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    can_id = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_CAN_ID

    print(f"串口 {port} @921600 ｜ CAN {CAN_BITRATE/1e6:.0f} Mbps ｜ 目标 CAN ID {can_id}")
    bus = Usb2CanClassic(port, 921600)
    bus.open(CAN_BITRATE)

    # 只发失能帧：7 个 0xFF + 0xFD（手册 p14），帧 ID = CAN ID（MIT 模式）
    disable = bytes([0xFF] * 7 + [0xFD])
    for _ in range(3):                       # 发 3 次：电机可能刚上电，首帧被忽略
        bus.send(can_id, disable)
        time.sleep(0.05)
    print(f"已发失能帧 ×3：id={can_id} data={disable.hex(' ')}")

    t0 = time.time()
    frames = []
    while time.time() - t0 < 1.5:
        for f in bus.recv():
            frames.append(f)
            print(f"  ← cmd=0x{f['cmd']:02X} fmt={f['fmt']} can_id={f['id']}"
                  f"({f['id']:#x}) data={f['data'].hex(' ')}")
        time.sleep(0.01)
    bus.close()

    print(f"\n共收到 {len(frames)} 帧")
    fb = [f for f in frames if f["cmd"] == 0x11]
    if not fb:
        print("❌ 判据1 不成立：1.5 s 内没收到反馈帧")
        print("   排查顺序：① 24V 电源 ② CAN_H/L 是否接反 ③ 终端电阻 ④ 波特率是否 1 Mbps"
              " ⑤ CAN ID 是否 1（若电机被别人改过）")
        return 1

    print("✅ 判据1 成立：收到反馈")
    ids = sorted({f["id"] for f in fb})
    print(f"   反馈帧 CAN ID 集合 = {ids}（期望 = MST_ID，你配置的是 17 / 0x11）")

    d = decode(fb[-1]["data"])
    err_name = ERR_NAME.get(d["err"], f"0x{d['err']:X}")
    print(f"   解析：ID={d['id']} 状态={err_name} 位置={d['pos']:.3f} rad "
          f"速度={d['vel']:.3f} rad/s 力矩={d['tau']:.2f} N·m "
          f"MOS={d['t_mos']}℃ 线圈={d['t_rotor']}℃")
    print(f"   判据2（反馈 ID = MST_ID）：{'成立' if ids == [17] else '⚠️ 与 17 不符，按实际值改配置'}")
    print(f"   判据3（D[0] 低 4 位 = CAN ID）：{'成立' if d['id'] == can_id else '❌ 实得 ' + str(d['id'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
