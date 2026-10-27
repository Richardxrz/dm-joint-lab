/**
 * @file bus.h
 * @brief 一条物理 CAN 总线：发送、按帧 ID 分发、统计。
 *
 * 它不知道载荷含义（达妙 MIT / C620 电流 / IMU），只负责「这个 ID 的帧该给谁」。
 *
 * ── 契约（调用方必须遵守，实现方必须保证）────────────────────────────
 * 1. 不分配内存：订阅表是静态数组，满了返回 NO_RESOURCE，绝不扩容、绝不 malloc。
 * 2. 不阻塞：所有函数立即返回；发送队列满返回 BUSY，不等待、不重试。
 * 3. 只在主循环上下文调用（含回调内部）。**严禁在中断里调用**——
 *    ISR 只负责把帧推进 port 的 FIFO，取帧与分发都在 Bus_Poll 里做。
 * 4. 一个帧 ID 允许挂 N 个订阅者，分发时**不短路**（六轴共用 Master ID 的前提）。
 * 5. 统计量是「同一时刻的快照」；「这次调用为什么失败」由返回值给出，两者不混。
 *
 * ── 典型用法 ─────────────────────────────────────────────────────
 *   Bus *bus;
 *   Bus_Create(&can1_port, &bus);                  // 绑定硬件端口
 *   Bus_Subscribe(bus, 0, dm_on_frame, &joint1);   // 六轴都订阅 Master ID 0
 *   ...
 *   while (1) {
 *     Bus_Poll(bus);                               // 取帧 + 分发，不阻塞
 *     ...
 *   }
 */
#ifndef SDK_BUS_H
#define SDK_BUS_H

#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "err.h"

/** 经典 CAN 一帧的数据长度（本 SDK 不支持 CAN-FD）。 */
#define SDK_CAN_DLEN 8u

/* 硬件端口契约（只有接口，实现归 port/mc02 或 port/host）。
 * 依赖方向：sdk --> port/inc（允许）；sdk --> port/mc02（禁止）。 */
#include "can_port.h"

/** 总线对象。内部结构不公开：调用方只拿指针，不能读写字段。 */
typedef struct Bus Bus;

/**
 * @brief 收到订阅的帧时被调用。**在 Bus_Poll 的上下文里执行**。
 * @warning 必须短（不阻塞、不 while 等、不再调 Bus_Poll）；重活丢给 Sched 的周期任务。
 */
typedef void (*BusOnFrame)(void *self, uint32_t id, const uint8_t data[SDK_CAN_DLEN]);

/**
 * @brief 总线统计快照。
 * @note 所有字段都来自**同一次读取的时刻**（不混不同时间的数），解读时先说时间点。
 */
typedef struct {
  /* ---- 发送侧 ---- */
  uint32_t tx_ok;
  /**< 已交给硬件发送队列的帧数。**≠ 对方收到了**：达妙是问询式，对方活着才会回反馈。 */

  uint32_t tx_busy;
  /**< 发送队列满而被拒的帧数（返回 SDK_RS_BUSY）。
   *   涨的正常原因：下发频率 > 总线速率；异常原因：总线被别的设备占满 / 波特率配错。 */

  uint32_t tx_error;
  /**< 硬件报告发送失败（仲裁丢失、Bus-Off、外设错误位）。
   *   非 0 就要查物理层，别只查代码。 */

  /* ---- 接收侧 ---- */
  uint32_t rx_ok;
  /**< 从端口 FIFO 取到的帧数（含没人认领的）。 */

  uint32_t rx_dispatched;
  /**< 至少有一个订阅者匹配帧 ID 的帧数。正常情况下应接近 rx_ok。 */

  uint32_t rx_unknown;
  /**< 没有任何订阅者认领的帧数（野 ID）。
   *   涨的原因：① 订阅漏了 ② 总线上还有别人的设备 ③ 电机 ID 设错。
   *   注意：设备自己按载荷筛掉的帧（如六轴的 D[0] 位）**不计这里**，算 dispatched。 */

  uint32_t rx_hits;
  /**< 订阅者回调累计调用次数。一帧挂 3 个订阅者就 +3。
   *   用来发现「订阅表被重复注册」，不是用来数帧（数帧看 rx_ok）。 */

  uint32_t rx_drop;
  /**< 端口 FIFO 满而丢掉的帧数。
   *   涨的原因：Bus_Poll 调用太慢、回调太长、或主循环里干了别的事。**这是丢数据的直接证据**。 */

  /* ---- 物理层 ---- */
  uint32_t bus_off;
  /**< 进入 Bus-Off 的次数（连续错误导致节点脱离总线）。
   *   恢复由 port 负责；这里只记账，涨了说明线/终端电阻/共地有问题。 */

  /* ---- 最近一次失败 ---- */
  sdk_err_t err_last;
  /**< 最近一次让某个调用失败的错误码（SDK_OK 表示至今没失败过）。
   *   与上面各计数器互补：计数器看趋势，这里看现场。 */
} BusStats;

/**
 * @brief 创建总线并绑定端口（从静态池取，不 malloc）。
 * @param port 端口实现，调用方负责其生命周期；传 NULL 非法。
 * @param out  成功时写入总线指针。
 * @retval SDK_OK
 * @retval SDK_ERR(SDK_MOD_BUS, SDK_RS_INVALID_ARG) port/out 为 NULL
 * @retval SDK_ERR(SDK_MOD_BUS, SDK_RS_NO_RESOURCE) 已达 SDK_MAX_BUS
 */
sdk_err_t Bus_Create(CanPort *port, Bus **out);

/**
 * @brief 发一帧。
 * @retval SDK_OK                 已交给硬件队列（不代表对端收到）
 * @retval SDK_RS_INVALID_ARG     bus 为 NULL / id > 0x7FF
 * @retval SDK_RS_BUSY            发送队列满，**这一帧没发出去**（不排队、不重试）
 * @retval SDK_RS_HW_FAULT        硬件报错；同时 tx_error++ 并写 err_last
 */
sdk_err_t Bus_Send(Bus *bus, uint32_t id, const uint8_t data[SDK_CAN_DLEN]);

/**
 * @brief 订阅一个帧 ID。
 * @note  同一 ID 可订阅多次（不同 self = 合法，分发时依次调用）；
 *        同一 (id, self) 重复订阅 = DUPLICATE，不会产生两份回调。
 * @retval SDK_OK
 * @retval SDK_RS_INVALID_ARG     bus/fn 为 NULL / id > 0x7FF
 * @retval SDK_RS_NO_RESOURCE     订阅表满（SDK_MAX_SUB）
 * @retval SDK_RS_DUPLICATE       同一 (id, self) 已订阅
 */
sdk_err_t Bus_Subscribe(Bus *bus, uint32_t id, BusOnFrame fn, void *self);

/**
 * @brief 退订（按 (id, self) 定位）。
 * @retval SDK_OK
 * @retval SDK_RS_INVALID_ARG     bus 为 NULL
 * @retval SDK_RS_NOT_FOUND       没有这条订阅
 */
sdk_err_t Bus_Unsubscribe(Bus *bus, uint32_t id, void *self);

/**
 * @brief 从端口取走当前所有可用的帧并分发，处理完就返回（不阻塞、不等待新帧）。
 * @note  回调在本函数内执行，回调里的耗时直接计入主循环周期。
 * @retval SDK_OK             正常（含「没有新帧」这种正常情况）
 * @retval SDK_RS_INVALID_ARG bus 为 NULL
 * @retval SDK_RS_HW_FAULT    端口报错；写 err_last（已分发出去的帧不受影响）
 */
sdk_err_t Bus_Poll(Bus *bus);

/**
 * @brief 取统计快照（同一时刻的全部计数）。
 * @retval SDK_OK
 * @retval SDK_RS_INVALID_ARG bus/out 为 NULL
 */
sdk_err_t Bus_GetStats(const Bus *bus, BusStats *out);

/**
 * @brief 当前订阅条数。
 * @return 0..SDK_MAX_SUB；bus 为 NULL 返回 0。用于测试与开机自检日志。
 */
uint8_t Bus_SubCount(const Bus *bus);


/* ===== WHY（维护者面：为什么这么设计，消费者不用读） =====
 *
 * 【D11 一个帧 ID 允许挂 N 个订阅者，分发不短路】
 *   前提（用户 2026-10-27 选方案 A）：六台达妙**共用**同一个 MST_ID(17)，
 *   所以六台的反馈帧在总线上 ID 完全相同，bus 层无法用 ID 区分，只能全部叫到，
 *   由每台设备自己看 D[0] 低 4 位判断是不是自己的。
 *   代价：一帧被分发 6 次（16 条表 × 5 周期 ≈ 0.17 µs，可忽略）。
 *   若将来改成"每台独立 MST_ID"（方案 B），本设计仍然兼容，只是每条订阅的 ID 不同。
 *
 * 【D12 不承诺分发顺序】任何设备不得假设别的订阅者何时被调用。
 *   原因：将来若把线性表换成哈希/分段表，顺序会变；依赖顺序 = 隐式耦合，且不会报错。
 *
 * 【D13 未注册 ID 必须计数】电机 ID 配错的现场表现是"完全没反应"，
 *   rx_unknown 是唯一便宜的观测面。
 *
 * 【D14 失败语义三选一】表满 = NO_RESOURCE（加容量）、重复注册 = DUPLICATE（改代码）、
 *   参数非法 = INVALID_ARG（改调用）。三者处置不同，不能合并。
 *   注意：**同 id 不同 self 合法**（六轴就是这种），只有 (id,self) 完全相同才是 DUPLICATE。
 *
 * 【D15 回调上下文】在 Bus_Poll 内部执行。允许回调里 Bus_Send（只进硬件队列，不碰订阅表）；
 *   禁止回调里再调 Bus_Poll —— 否则外层遍历中途嵌套一轮遍历：rx_ok 被数两次、
 *   同一设备的回调交错重入、递归深度不可控。
 *
 * 【D16 不透明 + 静态池】调用方拿不到表，就改不了表；静态池保证无 malloc。
 *
 * 【D17 统计与错误分家】返回值只答"这次为什么失败"；累计量走 GetStats()。
 *   混用会让"累计错误数"冒充"本次失败原因"。
 *
 * 【D18 单次 Poll 最多 64 帧 —— 推导】
 *   链条（目标：主循环周期有上界 → 保护巡检最坏延迟 ≤10 ms）：
 *     ① 允许单次 Poll 占用其中 15%  → 时间预算 1.5 ms
 *     ② 单帧成本 = 分发 0.33 µs（32 条上限时）+ 回调 ≤20 µs（**L4 目标，未实测**）
 *     ③ N = 1.5 ms / 20.3 µs ≈ 73  →  取 2 的幂 = 64
 *   注意：port 的 FIFO 深度只有 8，所以 64 实际几乎摸不到 —— 它是"防御性上界"，
 *   真正起作用的约束是 FIFO 深度。回调 20 µs 一旦实测超出，此推导要重算。
 */

#endif /* SDK_BUS_H */
