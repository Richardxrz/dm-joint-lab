#!/usr/bin/env python3
"""
L0 探针（只读）：用 0x7FF 读参数协议把电机的真实配置抄下来。

判据（L3）：
  1. 每个寄存器都能读回（回帧 ID = MST_ID，data[2]=0x33，data[3]=RID）
  2. MST_ID=17 / ESC_ID=1 / CTRL_MODE=1(MIT) 与口头配置一致
  3. PMAX/VMAX/TMAX 与代码里的 DmLimits_DM4340(12.5 / 10 / 28) 是否一致
     —— 不一致就必须改映射，否则位置换算全错
  4. TIMEOUT(0x09) 当前值（手册只给范围不给单位，先记录，再单独标定）

协议出处：手册 p14（读参数：ID=0x7FF，data = CANID_L, CANID_H, 0x33, RID, 0,0,0,0）
寄存器表：手册 p16~p17
安全：只读不写，不发使能，轴不会动。
"""
from __future__ import annotations

import struct
import sys
import time

sys.path.insert(0, "/Users/a1234/Embedded/custom_controller_ai/pc")
from usb2can_classic import Usb2CanClassic  # noqa: E402

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem00000000050C1"
CAN_ID = int(sys.argv[2]) if len(sys.argv) > 2 else 1
PARAM_ID = 0x7FF
CAN_BITRATE = 1_000_000

# (RID, 名字, 类型)  类型：'u' = uint32 ｜ 'f' = float32  —— 出处：手册 p16~p17
REGS = [
    (0x07, "MST_ID  反馈ID", "u"),
    (0x08, "ESC_ID  接收ID", "u"),
    (0x09, "TIMEOUT 超时", "u"),
    (0x0A, "CTRL_MODE 模式(1=MIT)", "u"),
    (0x10, "NPP 极对数", "u"),
    (0x14, "Gr 减速比", "f"),
    (0x15, "PMAX 位置范围", "f"),
    (0x16, "VMAX 速度范围", "f"),
    (0x17, "TMAX 扭矩范围", "f"),
    (0x01, "KT 扭矩系数", "f"),
    (0x23, "can_br 波特率代码", "u"),
    (0x3B, "Tpcb 驱动板温度", "f"),
    (0x3C, "Tmtr 电机温度", "f"),
    (0x50, "p_m 电机侧角度", "f"),
    (0x51, "xout 输出轴角度", "f"),
]


def read_reg(bus, mst_id: int, rid: int) -> bytes | None:
    payload = bytes([CAN_ID & 0xFF, (CAN_ID >> 8) & 0xFF, 0x33, rid, 0, 0, 0, 0])
    bus.send(PARAM_ID, payload)
    t0 = time.time()
    while time.time() - t0 < 0.2:
        for f in bus.recv():
            d = f["data"]
            if (f["id"] == mst_id and d[2] == 0x33 and d[3] == rid
                    and d[0] == (CAN_ID & 0xFF) and d[1] == ((CAN_ID >> 8) & 0xFF)):
                return bytes(d[4:8])
        time.sleep(0.005)
    return None


def main() -> int:
    print(f"串口 {PORT} ｜ 参数 ID 0x7FF ｜ 目标电机 CAN ID {CAN_ID} ｜ 1 Mbps\n")
    bus = Usb2CanClassic(PORT, 921600)
    bus.open(CAN_BITRATE)

    # 先确认 MST_ID（反馈帧 ID）才能筛回帧；未知时先按 0x11 试，失败则扫描
    mst_candidates = [0x11]
    mst_id = None
    for cand in mst_candidates:
        if read_reg(bus, cand, 0x07) is not None:
            mst_id = cand
            break
    if mst_id is None:
        print("⚠️ 用 MST_ID=0x11 读不到回帧，扫描 0x00~0x20 的反馈 ID …")
        for cand in range(0x00, 0x21):
            if read_reg(bus, cand, 0x07) is not None:
                mst_id = cand
                print(f"   命中 MST_ID = {cand} (0x{cand:02X})")
                break
    if mst_id is None:
        print("❌ 判据1 不成立：任何反馈 ID 都读不到 0x07（检查 0x7FF 是否被电机接受）")
        bus.close()
        return 1
    print(f"回帧反馈 ID（MST_ID）= {mst_id} (0x{mst_id:02X})\n")

    print(f"{'RID':<6}{'名称':<26}{'原始 4 字节':<14}{'解析'}")
    print("-" * 70)
    got = 0
    vals: dict[int, float | int] = {}
    for rid, name, kind in REGS:
        raw = read_reg(bus, mst_id, rid)
        if raw is None:
            print(f"0x{rid:02X}  {name:<26}{'—':<14}无响应")
            continue
        got += 1
        if kind == "f":
            v = struct.unpack("<f", raw)[0]
            print(f"0x{rid:02X}  {name:<26}{raw.hex(' '):<14}{v:.4f}")
            vals[rid] = v
        else:
            v = struct.unpack("<I", raw)[0]
            print(f"0x{rid:02X}  {name:<26}{raw.hex(' '):<14}{v}  (0x{v:X})")
            vals[rid] = v
    bus.close()

    print(f"\n共读到 {got}/{len(REGS)} 个寄存器")
    ok = True
    checks = [
        (0x07, 17, "MST_ID = 17"),
        (0x08, 1, "ESC_ID = 1"),
        (0x0A, 1, "CTRL_MODE = 1（MIT）"),
    ]
    for rid, want, label in checks:
        if rid in vals:
            good = int(vals[rid]) == want
            ok &= good
            print(f"{'✅' if good else '❌'} {label}（实得 {int(vals[rid])}）")

    code = vals.get(0x23)
    if code is not None:
        names = {0: "125K", 1: "200K", 2: "250K", 3: "500K", 4: "1M",
                 5: "2M", 6: "2.5M", 7: "3.2M", 8: "4M", 9: "5M"}
        print(f"ℹ️ 波特率代码 = {int(code)} → {names.get(int(code), '未知')}")

    pmax, vmax, tmax = vals.get(0x15), vals.get(0x16), vals.get(0x17)
    if None not in (pmax, vmax, tmax):
        same = abs(pmax - 12.5) < 1e-3 and abs(vmax - 10.0) < 1e-3 and abs(tmax - 28.0) < 1e-3
        print(f"{'✅' if same else '⚠️'} PMAX/VMAX/TMAX = {pmax:.3f} / {vmax:.3f} / {tmax:.3f}"
              f"（代码里是 12.5 / 10 / 28）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
