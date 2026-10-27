#!/usr/bin/env python3
"""
L0 探针：标定 TIMEOUT(0x09) 的单位 —— 掉线保护到底多久生效。

难点：达妙是**问询式**——不发帧就没有反馈，所以不能"停发然后等反馈"。
做法：停发 T ms → 再发一帧 MIT → 看这一帧回执里的 ERR：
      ERR=1(使能) = 没超时；ERR=0xD(通讯丢失) 或 0(失能) = 已超时。

安全：全程只发「中点帧」(p_des=0, v_des=0, Kp=0, Kd=0, t_ff=0) = 不出力，
      轴保持松的；不写 flash（TIMEOUT 只在 RAM，掉电恢复 0）。

判据（L3）：
  1. TIMEOUT 可写可读回（0 → 1）
  2. 存在一个明确的停发时长 T*，越过它 ERR 从 1 变成「已失能」→ 单位可标定
用法：python3 tests/bench/dm_timeout_probe.py [串口] [CAN_ID]
"""
from __future__ import annotations

import struct
import sys
import time

sys.path.insert(0, "/Users/a1234/Embedded/custom_controller_ai/pc")
from usb2can_classic import Usb2CanClassic  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem00000000050C1"
CAN_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
MST_ID = 0x11
PARAM_ID = 0x7FF
BITRATE = 1_000_000
TIMEOUT_VALUE = int(sys.argv[3]) if len(sys.argv) > 3 else 100

# 中点帧：所有量都取区间中点 → p_des=0, v_des=0, Kp=0, Kd=0, t_ff=0（不出力）
#   位置 16 位中点 0x8000；速度/力矩 12 位中点 0x800；Kp/Kd = 0
MID_FRAME = bytes([0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF])
ENABLE = bytes([0xFF] * 7 + [0xFC])
ERR_NAME = {0x0: "失能", 0x1: "使能", 0xD: "通讯丢失"}


def recv_until(bus, rid_check=None, timeout=0.2):
    t0 = time.time()
    while time.time() - t0 < timeout:
        for f in bus.recv():
            if f["id"] != MST_ID or f["cmd"] != 0x11:
                continue
            d = f["data"]
            if rid_check is not None:
                if d[2] == 0x55 and d[3] == rid_check:
                    return d
            else:
                return d
        time.sleep(0.002)
    return None


def read_reg(bus, rid):
    bus.send(PARAM_ID, bytes([CAN_ID & 0xFF, (CAN_ID >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0]))
    d = recv_until(bus, rid_check=None)
    if d is None or d[2] != 0x33 or d[3] != rid:
        return None
    return struct.unpack("<I", bytes(d[4:8]))[0]


def write_reg(bus, rid, value):
    payload = bytes([CAN_ID & 0xFF, (CAN_ID >> 8) & 0xFF, 0x55, rid]) + struct.pack("<I", value)
    bus.send(PARAM_ID, payload)
    d = recv_until(bus, rid_check=rid)
    return None if d is None else struct.unpack("<I", bytes(d[4:8]))[0]


def run_stream(bus, duration_s, hz=200):
    """按 hz 发中点帧，返回最后一帧的 ERR 半字节"""
    period = 1.0 / hz
    t0 = time.time()
    err = None
    while time.time() - t0 < duration_s:
        bus.send(CAN_ID, MID_FRAME)
        d = recv_until(bus, timeout=period)
        if d is not None:
            err = d[0] >> 4
        time.sleep(0.001)
    return err


def trial(bus, pause_ms):
    """使能 → 流 0.3 s → 停发 pause_ms → 发一帧看 ERR"""
    bus.send(CAN_ID, ENABLE)
    recv_until(bus, timeout=0.05)
    err_before = run_stream(bus, 0.3)
    time.sleep(pause_ms / 1000.0)
    bus.send(CAN_ID, MID_FRAME)
    d = recv_until(bus, timeout=0.1)
    err_after = None if d is None else (d[0] >> 4)
    return err_before, err_after


def main() -> int:
    print(f"串口 {PORT} ｜ CAN {BITRATE/1e6:.0f} Mbps ｜ CAN ID {CAN_ID} ｜ MST_ID {MST_ID}")
    bus = Usb2CanClassic(PORT, 921600)
    bus.open(BITRATE)

    before = read_reg(bus, 0x09)
    print(f"TIMEOUT 原值 = {before}")
    got = write_reg(bus, 0x09, TIMEOUT_VALUE)
    back = read_reg(bus, 0x09)
    print(f"写入 {TIMEOUT_VALUE} → 回执 {got} ｜ 读回 {back}")
    if back != TIMEOUT_VALUE:
        print("❌ 判据1 不成立：TIMEOUT 写不进去（可能需要先失能，或该型号不支持）")
        bus.close()
        return 1
    print("✅ 判据1 成立：TIMEOUT 可写可读回\n")

    print(f"{'停发时长':<10}{'流期间ERR':<12}{'恢复后ERR':<12}判定")
    print("-" * 56)
    cross = None
    last_alive = 0
    for pause in (5, 10, 20, 50, 100, 200, 500, 1000, 2000):
        eb, ea = trial(bus, pause)
        name = ERR_NAME.get(ea, f"0x{ea:X}" if ea is not None else "无回帧")
        alive = ea == 0x1
        if alive:
            last_alive = pause
        if (not alive) and cross is None:
            cross = pause
        print(f"{str(pause)+' ms':<10}{ERR_NAME.get(eb, str(eb)):<12}{name:<12}"
              f"{'仍在使能' if alive else '已失能 ← 超时触发'}")
        time.sleep(0.1)



    print()
    if cross is None:
        print("⚠️ 判据2 未定：2000 ms 停发后仍使能 → 阈值 ≥2 s 或单位非线性")
    else:
        lo = last_alive if last_alive else 0
        print(f"✅ 判据2 成立：停发 {lo} ms 仍使能，{cross} ms 已失能 "
              f"→ 真实阈值落在 ({lo}, {cross}] ms")
        if TIMEOUT_VALUE > 0:
            print(f"   换算：阈值 ≈ {lo}~{cross} ms 对应 TIMEOUT={TIMEOUT_VALUE} "
                  f"→ 单位 ≈ {lo/TIMEOUT_VALUE:.1f}~{cross/TIMEOUT_VALUE:.1f} ms/单位")
    bus.send(CAN_ID, bytes([0xFF] * 7 + [0xFD]))   # 收尾：失能
    time.sleep(0.05)
    restore = write_reg(bus, 0x09, 0)
    print(f"已恢复 TIMEOUT=0（读回 {read_reg(bus, 0x09)}）：掉线保护回到出厂关闭状态")
    bus.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
