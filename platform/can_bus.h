/**
 * @file can_bus.h
 * @brief CAN 发送通道抽象 —— module 层只认识它，不认识 FDCAN / HAL。
 *
 * 依赖方向：module --> platform <-- bsp
 *   bsp 提供实现（FDCAN），测试提供替身（记录帧），module 两边都不用改。
 */
#ifndef PLATFORM_CAN_BUS_H
#define PLATFORM_CAN_BUS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define CAN_FRAME_DLEN 8u

/** 一帧经典 CAN（远程帧/非 8 字节帧本工程不用）。 */
typedef struct {
  uint32_t id;
  uint8_t data[CAN_FRAME_DLEN];
} CanFrame;

/** 发送通道。ctx 是实现方的私有上下文（HAL 句柄 / 测试记录器）。 */
typedef struct {
  bool (*send)(void *ctx, uint32_t id, const uint8_t data[CAN_FRAME_DLEN]);
  void *ctx;
} CanBus;

/**
 * @brief 发送一帧。
 * @return true = 已交给硬件发送队列；false = 没交出去（bus 未注入 / 队列满）。
 * @note  true 不等于"对方收到了"，只等于"我交出去了"。
 */
static inline bool CanBus_Send(const CanBus *bus, uint32_t id,
                               const uint8_t data[CAN_FRAME_DLEN]) {
  if (bus == NULL || bus->send == NULL) {
    return false;
  }
  return bus->send(bus->ctx, id, data);
}

#endif /* PLATFORM_CAN_BUS_H */
