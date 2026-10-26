/**
 * @file can_port.h
 * @brief CAN 硬件端口契约：Bus 只通过这张表碰硬件。
 *
 * 两个实现：`port/mc02/`（FDCAN + HAL）、`port/host/`（测试替身，可注入帧）。
 * `sdk/` 不认识本文件之外的任何硬件细节。
 *
 * ── 契约 ──────────────────────────────────────────────────────────
 * 1. **只由 Bus 调用**。应用代码不要直接调 `ops->send`——绕过统计与路由。
 * 2. **不解析协议**：不认 MIT/C620 的字节含义，只搬 8 字节。
 * 3. **不阻塞**：`send` 排队失败就返回 BUSY；`recv` 没帧就返回 EMPTY。
 *    禁止在实现里 while 等待硬件标志——那是主循环卡死的常见来源。
 * 4. **初始化归实现方**：波特率、过滤器、FIFO 配置在 `CanPort` 交给 `Bus_Create` 之前完成。
 *    Bus 不做硬件初始化，也不检查波特率。
 * 5. **中断归实现方**：ISR 只把帧推进内部 FIFO 并计数，**不在中断里回调 Bus**。
 * 6. **时间无关**：本接口不接收、不返回时间；时基请看 `time_port.h`。
 * 7. **容量**：内部 FIFO 建议 ≥ 8 帧。1 kHz 下发时，主循环一轮可能积压 2~3 帧；
 *    FIFO 满必须丢并计数（`dropped`），不允许覆盖未读数据而不计。
 */
#ifndef PORT_CAN_PORT_H
#define PORT_CAN_PORT_H

#include <stdint.h>

/** 一帧的数据长度。本 SDK 只用经典 CAN（不支持 CAN-FD 的长帧）。 */
#define CAN_PORT_DLEN 8u

/** 标准帧 ID 上限（11 位）。 */
#define CAN_PORT_STD_ID_MAX 0x7FFu

/**
 * @brief 端口操作结果。三个失败值是**三种不同的现实**，实现方不要合并。
 */
typedef enum {
  CAN_PORT_OK = 0,  /**< send：已入队 ｜ recv：取到一帧 */
  CAN_PORT_EMPTY,   /**< recv 专用：当前没有帧。**这是正常情况，不是错误** */
  CAN_PORT_BUSY,    /**< send 专用：队列满，这一帧没发出去（不排队、不重试） */
  CAN_PORT_ERROR,   /**< 硬件错误：外设错误位、Bus-Off、FIFO 异常 */
} CanPortResult;

/**
 * @brief 端口实现的操作表。全部字段必须非 NULL（实现时不许留空，留空即违约）。
 */
typedef struct {
  /**
   * @brief 发送一帧。
   * @param id 标准帧 ID（调用方保证 ≤ CAN_PORT_STD_ID_MAX）
   * @return CAN_PORT_OK / CAN_PORT_BUSY / CAN_PORT_ERROR
   * @note  OK 只代表"进了硬件队列"，不代表对端收到。
   */
  CanPortResult (*send)(void *ctx, uint32_t id, const uint8_t data[CAN_PORT_DLEN]);

  /**
   * @brief 取一帧（非阻塞）。
   * @param id   成功时写回帧 ID
   * @param data 成功时写回 8 字节
   * @return CAN_PORT_OK（已填充 id/data）/ CAN_PORT_EMPTY（没帧）/ CAN_PORT_ERROR
   * @note  Bus 会一直调到 EMPTY 为止，所以实现不得在 EMPTY 时做任何耗时动作。
   */
  CanPortResult (*recv)(void *ctx, uint32_t *id, uint8_t data[CAN_PORT_DLEN]);

  /** @return 累计因 FIFO 满而丢弃的帧数（只增不减；掉电清零）。 */
  uint32_t (*dropped)(void *ctx);

  /** @return 累计进入 Bus-Off 的次数（只增不减）。 */
  uint32_t (*bus_off)(void *ctx);

  /** @return 累计其他硬件错误次数（溢出之外的错误位、外设异常）。 */
  uint32_t (*hw_errors)(void *ctx);
} CanPortOps;

/**
 * @brief 端口对象。由实现方静态定义并填好 ops/ctx，再交给 `Bus_Create`。
 * @note  生命周期由创建者负责：Bus 只持有指针，不释放、不复制。
 */
typedef struct CanPort {
  const CanPortOps *ops; /**< 操作表，静态常量 */
  void *ctx;             /**< 实现私有上下文（HAL 句柄 / 测试记录器） */
} CanPort;

/**
 * @brief 实现方自检用：操作表是否填全。测试用它卡"忘了实现某个 op"。
 * @return true = 五个函数指针都非 NULL。
 */
static inline int CanPort_IsComplete(const CanPort *port) {
  return port != 0 && port->ops != 0 && port->ops->send != 0 && port->ops->recv != 0 &&
         port->ops->dropped != 0 && port->ops->bus_off != 0 && port->ops->hw_errors != 0;
}

#endif /* PORT_CAN_PORT_H */
