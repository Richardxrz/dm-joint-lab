/**
 * @file err.h
 * @brief SDK 统一错误码：一个 32 位值 = 模块号 + 原因码。
 *
 * 设计约束（三条）：
 *   1. 每个函数只返回「这次调用为什么失败」，不返回累计量（累计量走 *_GetStats）；
 *   2. 0 是唯一的成功值，任何非 0 都必须能被打印成人话；
 *   3. 模块号固定 8 位、原因码固定 24 位，方便日志里直接 filter。
 */
#ifndef SDK_ERR_H
#define SDK_ERR_H

#include <stdint.h>

/** 0 = OK，其余 = 高 8 位模块号 | 低 24 位原因码。 */
typedef uint32_t sdk_err_t;

#define SDK_OK ((sdk_err_t)0u)

/** 模块号：谁报的错。 */
typedef enum {
  SDK_MOD_NONE = 0,
  SDK_MOD_BUS = 1,    /**< 总线路由 */
  SDK_MOD_DEVICE = 2, /**< 设备注册 */
  SDK_MOD_SCHED = 3,  /**< 周期调度 */
  SDK_MOD_PORT = 4,   /**< 硬件端口（can/time/log） */
  SDK_MOD_APP = 5,    /**< 业务 */
} SdkModule;

/** 原因码：为什么失败。 */
typedef enum {
  SDK_RS_OK = 0,
  SDK_RS_INVALID_ARG = 1,  /**< 空指针 / 越界 / 未填的配置项 */
  SDK_RS_NOT_INITED = 2,   /**< 对象没初始化就调用了 */
  SDK_RS_BUSY = 3,         /**< 资源正被占用（发送队列满），没排队 */
  SDK_RS_NO_RESOURCE = 4,  /**< 静态表满（设计容量用尽），没扩容 */
  SDK_RS_DUPLICATE = 5,    /**< 同一身份重复注册 */
  SDK_RS_NOT_FOUND = 6,    /**< 要操作的对象不存在 */
  SDK_RS_TIMEOUT = 7,      /**< 等到了但超时（有明确时限的操作才有） */
  SDK_RS_HW_FAULT = 8,     /**< 硬件报错（Bus-Off / 外设错误位） */
  SDK_RS_PROTOCOL = 9,     /**< 协议层不一致（字段越界、校验失败） */
} SdkReason;

#define SDK_ERR(mod, reason) \
  (((sdk_err_t)(uint32_t)(mod) << 24) | ((sdk_err_t)(uint32_t)(reason) & 0x00FFFFFFu))

/** @return 模块号名（"BUS"/"DEVICE"…）；未知返回 "UNKNOWN"。 */
const char *Sdk_ErrModule(sdk_err_t err);

/** @return 原因码名（"INVALID_ARG"/"BUSY"…）；未知返回 "UNKNOWN"。 */
const char *Sdk_ErrReason(sdk_err_t err);

#endif /* SDK_ERR_H */
