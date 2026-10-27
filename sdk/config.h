/**
 * @file config.h
 * @brief 全部静态容量集中在此。**每个数字必须带推导**（维护者契约不变量 #4）。
 *
 * 规则：
 *   1. 改数字 → 必须同时改注释里的推导，否则 `tools/contract_check` 报警；
 *   2. 容量用尽是**设计事实**，不是意外：超限必须返回 SDK_RS_NO_RESOURCE；
 *   3. 开机 banner 打印占用率（如 `sub=22/32`），快满时提前看见。
 */
#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

/** CAN 总线数。MC02 板载 3 路 FDCAN — 依据：板载手册 p5「引出 3 路 CAN 信号」、
 *  官方例程 CtrBoard-H7_ALL/Core/Inc/fdcan.h:35-39（hfdcan1/2/3）。 */
#define SDK_MAX_BUS 3u

/** 每条总线的订阅条数（一个帧 ID 一条）。
 *  推导：最坏单总线用例 = 6（达妙 6 轴，共用 Master ID 也是 6 条）
 *        + 8（DJI 电调反馈 0x201~0x208）+ 2（超级电容 0x301/0x302）
 *        + 2（图传/自定义控制器 0x310/0x311）+ 4（预留调试）= 22
 *        22 × 1.45 ≈ 32。
 *  预算校验：RAM = 32 × 16 B = 512 B/总线（3 路共 1.5 KB，占 H723 1 MB 的 0.15%）；
 *           CPU = 2000 帧/s × 32 次比较 ≈ 64k 次/s ≈ 0.05%（@480 MHz）。
 *  即：不是被 RAM/CPU 卡住，而是 32 以上没有用例支撑（YAGNI）。 */
#define SDK_MAX_SUB 32u

/** 设备数（一个结构管理多通道的算一个：C620 四通道 = 1 个设备）。
 *  推导：6 关节 + 1 电调组 + 电容 + IMU + 图传 = 10，× 1.6 = 16。 */
#define SDK_MAX_DEV 16u

/** 周期任务数。
 *  推导：1 kHz 控制环 1 + 200 Hz 下发 1 + 200 Hz 保护巡检 1 + 50 Hz 上报 1
 *        + 10 Hz 遥测 1 + 2 Hz 状态灯 1 = 6，× 2.6 = 16（留给调试与临时任务）。 */
#define SDK_MAX_TASK 16u

#endif /* SDK_CONFIG_H */
