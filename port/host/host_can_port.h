/**
 * @file host_can_port.h
 * 解决什么：Mac/Linux 上的假 CAN 端口 —— 让 Bus 不接硬件也能跑测试
 * 不解决什么：真总线时序 / Bus-Off / FIFO 溢出的硬件行为（那些只能上板验）
 * 判据在哪：tests/host/main.c 里所有 bus 用例都用它当输入端
 *
 * 用法：
 *   HostCan hc; HostCan hc = {0}; HostCan_Init(&hc);
 *   Bus *bus; Bus_Create(HostCan_Port(&hc), &bus);      // 交给 Bus
 *   HostCan_InjectFrame(&hc, 17, data);                 // 假装收到一帧
 *   Bus_Poll(bus);                                      // 触发分发
 *   HostCan_SentCount(&hc);                             // 看 Bus 发了几帧
 */
#ifndef PORT_HOST_CAN_PORT_H
#define PORT_HOST_CAN_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "can_port.h"

/** 接收 FIFO 深度。取 8 是为了与真实 port(mc02) 保持一致，让"塞满"的用例有意义。 */
#define HOST_CAN_RX_DEPTH 8u

/** 发送记录条数（只用于测试回看，超出后覆盖最旧）。 */
#define HOST_CAN_TX_LOG 16u

/** 假端口对象。第一个成员必须是 CanPort，方便 HostCan_Port() 取地址。 */
typedef struct {
  CanPort port;
  /* ---- 接收：测试注入、Bus 取走 ---- */
  struct {
    uint32_t id;
    uint8_t data[CAN_PORT_DLEN];
  } rx[HOST_CAN_RX_DEPTH];
  uint8_t rx_head, rx_tail, rx_len;
  /* ---- 发送：Bus 调 send 时记录 ---- */
  struct {
    uint32_t id;
    uint8_t data[CAN_PORT_DLEN];
  } tx[HOST_CAN_TX_LOG];
  uint32_t tx_count;
  CanPortResult send_result; /**< 强制 send 的返回值（默认 OK） */
  /* ---- 计数器（Bus 的统计靠它们） ---- */
  uint32_t dropped, bus_off_cnt, hw_err_cnt;
} HostCan;

/** 初始化（填 ops 与 ctx）。返回后可直接交给 Bus_Create。 */
void HostCan_Init(HostCan *hc);

/** @return 可交给 Bus_Create 的端口指针（即 &hc->port）。 */
CanPort *HostCan_Port(HostCan *hc);

/**
 * @brief 注入一帧"收到的 CAN"。
 * @return false = FIFO 已满（该帧被丢弃并计入 dropped，与真实端口的溢出行为一致）。
 */
bool HostCan_InjectFrame(HostCan *hc, uint32_t id, const uint8_t data[CAN_PORT_DLEN]);

/** FIFO 里还有几帧没被 Bus 取走。 */
uint8_t HostCan_PendingRx(const HostCan *hc);

/** 强制下一次（以及之后每次）send 的返回值：CAN_PORT_OK / BUSY / ERROR。 */
void HostCan_SetSendResult(HostCan *hc, CanPortResult r);

/** 直接拨动计数器，用于验证 Bus 是否把它们搬进 BusStats。 */
void HostCan_SetCounters(HostCan *hc, uint32_t dropped, uint32_t bus_off, uint32_t hw_errs);

/** Bus 一共调用了几次 send（含失败的那些）。 */
uint32_t HostCan_SentCount(const HostCan *hc);

/** 读第 index 次发送的记录（0 = 最早）。@return false = index 越界。 */
bool HostCan_GetSent(const HostCan *hc, uint32_t index, uint32_t *id,
                     uint8_t data[CAN_PORT_DLEN]);

/** 清空发送记录（每组用例开头调用，避免上一组的残留）。 */
void HostCan_ResetLog(HostCan *hc);

#endif /* PORT_HOST_CAN_PORT_H */
