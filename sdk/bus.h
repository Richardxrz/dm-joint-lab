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

#include "err.h"

/** 经典 CAN 一帧的数据长度（本 SDK 不支持 CAN-FD）。 */
#define SDK_CAN_DLEN 8u

/** 同时可用的总线数。MC02 = FDCAN1 + FDCAN2。 */
#define SDK_MAX_BUS 2u

/** 每条总线的订阅表容量（一个 6 轴臂 = 6 条订阅，留余量）。 */
#define SDK_MAX_SUB 32u

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

#endif /* SDK_BUS_H */
