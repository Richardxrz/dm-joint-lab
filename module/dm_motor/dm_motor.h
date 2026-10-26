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

/**
 * @brief 反馈帧 D[0] 高 4 位的状态（docs/协议.md §四，手册 p12）。
 * @note  0/1 是正常状态，0x8~0xE 是故障；判故障用 DmState_IsFault()。
 */
typedef enum {
  DM_STATE_DISABLED = 0x0,       /**< 失能（上电默认） */
  DM_STATE_ENABLED = 0x1,        /**< 使能 */
  DM_STATE_OVER_VOLTAGE = 0x8,   /**< 超压 */
  DM_STATE_UNDER_VOLTAGE = 0x9,  /**< 欠压 */
  DM_STATE_OVER_CURRENT = 0xA,   /**< 过流 */
  DM_STATE_MOS_OVER_TEMP = 0xB,  /**< MOS 过温 */
  DM_STATE_COIL_OVER_TEMP = 0xC, /**< 线圈过温 */
  DM_STATE_COMM_LOST = 0xD,      /**< 通讯丢失 */
  DM_STATE_OVERLOAD = 0xE,       /**< 过载 */
} DmState;

/** @return true = 0x8~0xE 的故障态；0/1 与未定义值返回 false。 */
bool DmState_IsFault(DmState state);

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
  uint8_t feedback_id; /**< 反馈帧 ID = Master ID（手册 p12，默认 0）。同一条总线上所有达妙电机共用它 */
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
  uint8_t id;       /**< 反馈帧里的电机 ID（D[0] 低 4 位） */
  DmState state;    /**< 反馈帧里的状态（D[0] 高 4 位） */
  uint32_t rx_count;    /**< 累计收到本轴的反馈帧数 */
  uint32_t foreign_cnt; /**< 落在本轴反馈 ID 上、ID 却不是自己的帧数（多轴共用 Master ID 时看串轴） */
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
 * @brief 喂一帧收到的 CAN，两级匹配：① 帧 ID == cfg.feedback_id ② D[0] 低 4 位 == cfg.can_id。
 * @param now_ms 本帧到达时刻（调用方时基，便于测试拨表）
 * @retval true  两级都匹配：已解出瞬时量并计入 rx_count
 * @retval false 不匹配：瞬时量一律不动
 * @note  第②级失败时 foreign_cnt++ —— 多台电机共用 Master ID 时，"串轴"这件事必须看得见，
 *        不能被当成"这一帧不存在"。
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
