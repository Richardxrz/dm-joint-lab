/**
 * @file dm_motor.h
 * @brief 达妙（DAMIAO）电机 MIT 模式：组帧、解析反馈、在线判定。
 *
 * 这一层只做"帧 <-> 量"的翻译和状态记账，不做：
 *   - 轨迹/插值（归 app）
 *   - PID（MIT 模式下 Kp/Kd 由上位给，电机内部闭环）
 *   - 总线仲裁、重发（归 bsp）
 *
 * 单位（全部为减速器输出端）：
 *   位置 rad ｜ 速度 rad/s ｜ 力矩 N·m ｜ 温度 °C（反馈里是整数摄氏度）
 *
 * 协议依据（实测核对过的两处）：
 *   - MIT 控制帧 8 字节：pos(16) | vel(12) | kp(12) | kd(12) | tau(12)
 *   - 反馈帧：data[1..2] pos(16) | data[3]+data[4]高4位 vel(12)
 *             | data[4]低4位+data[5] tau(12) | data[6] MOS 温度 | data[7] 转子温度
 * 未核实：data[0]（推测为 状态<<4 | 电机ID，见 10-架构与协议.md §三），故原样透出 status_raw。
 */
#ifndef MODULE_DM_MOTOR_H
#define MODULE_DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "can_bus.h"

/** 特殊帧命令字（8 字节全 0xFF，最后一字节换成这里之一）。 */
typedef enum {
  DM_CMD_ENABLE = 0xFC,      /**< 使能 */
  DM_CMD_DISABLE = 0xFD,     /**< 失能 */
  DM_CMD_SET_ZERO = 0xFE,    /**< 把当前位置设为零点 */
  DM_CMD_CLEAR_ERROR = 0xFB, /**< 清错误 */
} DmCommand;

/** 一条轴的量程。越界值会被饱和，不会溢出成反向跑。 */
typedef struct {
  float pos_min, pos_max; /**< rad */
  float vel_min, vel_max; /**< rad/s */
  float tau_min, tau_max; /**< N·m */
  float kp_max;           /**< 0..500 */
  float kd_max;           /**< 0..5 */
} DmLimits;

typedef struct {
  CanBus bus;      /**< 注入的发送通道；无 send 则一切命令失败 */
  uint8_t can_id;  /**< 电机 CAN ID（控制帧 ID 就是它） */
  uint8_t feedback_id; /**< 反馈帧 ID。达妙默认 = 主控 ID（0）。实测不符合就改这一格 */
  DmLimits limits;
  uint32_t rx_timeout_ms; /**< >0 = 超过这么久没反馈判掉线；0 = 不判 */
} DmMotorConfig;

/** 一帧反馈解析出来的瞬时量。 */
typedef struct {
  float pos;        /**< rad */
  float vel;        /**< rad/s */
  float tau;        /**< N·m */
  float temp_mos;   /**< °C */
  float temp_rotor; /**< °C */
  uint8_t status_raw;   /**< data[0] 原值（状态位域未核实） */
  uint32_t rx_count;    /**< 累计收到的反馈帧数 */
  uint32_t last_rx_ms;  /**< 最后一帧反馈的时刻 */
} DmFeedback;

typedef struct {
  DmMotorConfig cfg;
  DmFeedback fb;
  bool inited;
} DmMotor;

/** DM4340（24 V，40:1）量程：±12.5 rad / ±10 rad/s / ±28 N·m（达妙官方 SDK 表）。 */
DmLimits DmLimits_DM4340(void);

/** @return false = 参数非法（m/cfg 为空、bus 无 send、feedback_id 未填）。 */
bool DmMotor_Init(DmMotor *m, const DmMotorConfig *cfg);

/** 使能 / 失能 / 清零 / 清错。@return 帧是否交给总线。 */
bool DmMotor_Enable(DmMotor *m);
bool DmMotor_Disable(DmMotor *m);
bool DmMotor_SetZero(DmMotor *m);
bool DmMotor_ClearError(DmMotor *m);

/**
 * @brief 下发一条 MIT 指令（立刻组帧发走，不自持）。
 * @note  入参按 cfg.limits 饱和；NaN 当 0 处理。
 * @return 帧是否交给总线。
 */
bool DmMotor_MitCommand(DmMotor *m, float pos, float vel, float kp, float kd, float tau);

/**
 * @brief 喂一帧收到的 CAN。id 不等于 cfg.feedback_id 时返回 false 且不改状态。
 * @param now_ms 本帧到达时刻（调用方时基，便于测试拨表）
 * @return true = 这帧是这条轴的反馈，已更新 fb。
 */
bool DmMotor_OnFrame(DmMotor *m, uint32_t id, const uint8_t data[CAN_FRAME_DLEN],
                     uint32_t now_ms);

/**
 * @brief 在线判定。
 * @param now_ms 当前毫秒时基（由调用方给，便于测试拨表；差值已做 32 位回绕处理）
 * @return 收到过反馈，且（未配置超时 或 距上一帧 <= rx_timeout_ms）
 */
bool DmMotor_IsOnline(const DmMotor *m, uint32_t now_ms);

#endif /* MODULE_DM_MOTOR_H */
