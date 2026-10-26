#include "dm_motor.h"

#include <math.h>
#include <string.h>

#define DM_POS_BITS 16u
#define DM_VEL_BITS 12u
#define DM_KP_BITS 12u
#define DM_KD_BITS 12u
#define DM_TAU_BITS 12u

static uint32_t uint_max_for(uint8_t bits) { return (1u << bits) - 1u; }

/* 浮点 -> 整数：先饱和再用比例映射，保证不会溢出成反向跑 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
  if (isnan(x)) {
    x = 0.0f;
  }
  if (x < x_min) {
    x = x_min;
  }
  if (x > x_max) {
    x = x_max;
  }
  const float span = x_max - x_min;
  if (span <= 0.0f) {
    return 0u;
  }
  const float ratio = (x - x_min) / span;
  return (uint16_t)(ratio * (float)uint_max_for(bits));
}

/* 整数 -> 浮点：与 float_to_uint 严格互逆 */
static float uint_to_float(uint16_t u, float x_min, float x_max, uint8_t bits) {
  const float span = x_max - x_min;
  if (span <= 0.0f) {
    return x_min;
  }
  return ((float)u / (float)uint_max_for(bits)) * span + x_min;
}

bool DmState_IsFault(DmState state) {
  switch (state) {
  case DM_STATE_OVER_VOLTAGE:
  case DM_STATE_UNDER_VOLTAGE:
  case DM_STATE_OVER_CURRENT:
  case DM_STATE_MOS_OVER_TEMP:
  case DM_STATE_COIL_OVER_TEMP:
  case DM_STATE_COMM_LOST:
  case DM_STATE_OVERLOAD:
    return true;
  default:
    return false;
  }
}

DmLimits DmLimits_DM4340(void) {
  DmLimits lim;
  lim.pos_min = -12.5f;
  lim.pos_max = 12.5f;
  lim.vel_min = -10.0f;
  lim.vel_max = 10.0f;
  lim.tau_min = -28.0f;
  lim.tau_max = 28.0f;
  lim.kp_max = 500.0f;
  lim.kd_max = 5.0f;
  return lim;
}

bool DmMotor_Init(DmMotor *m, const DmMotorConfig *cfg) {
  if (m == NULL || cfg == NULL || cfg->bus.send == NULL) {
    return false;
  }
  memset(m, 0, sizeof(*m));
  m->cfg = *cfg;
  m->inited = true;
  return true;
}

static bool send_command(DmMotor *m, DmCommand cmd) {
  if (m == NULL || !m->inited) {
    return false;
  }
  uint8_t frame[CAN_FRAME_DLEN];
  memset(frame, 0xFF, sizeof(frame));
  frame[CAN_FRAME_DLEN - 1u] = (uint8_t)cmd;
  return CanBus_Send(&m->cfg.bus, m->cfg.can_id, frame);
}

bool DmMotor_Enable(DmMotor *m) { return send_command(m, DM_CMD_ENABLE); }
bool DmMotor_Disable(DmMotor *m) { return send_command(m, DM_CMD_DISABLE); }
bool DmMotor_SetZero(DmMotor *m) { return send_command(m, DM_CMD_SET_ZERO); }
bool DmMotor_ClearError(DmMotor *m) { return send_command(m, DM_CMD_CLEAR_ERROR); }

bool DmMotor_MitCommand(DmMotor *m, float pos, float vel, float kp, float kd, float tau) {
  if (m == NULL || !m->inited) {
    return false;
  }
  const DmLimits *lim = &m->cfg.limits;

  const uint16_t p = float_to_uint(pos, lim->pos_min, lim->pos_max, DM_POS_BITS);
  const uint16_t v = float_to_uint(vel, lim->vel_min, lim->vel_max, DM_VEL_BITS);
  const uint16_t kpi = float_to_uint(kp, 0.0f, lim->kp_max, DM_KP_BITS);
  const uint16_t kdi = float_to_uint(kd, 0.0f, lim->kd_max, DM_KD_BITS);
  const uint16_t t = float_to_uint(tau, lim->tau_min, lim->tau_max, DM_TAU_BITS);

  uint8_t frame[CAN_FRAME_DLEN];
  frame[0] = (uint8_t)(p >> 8);
  frame[1] = (uint8_t)(p & 0xFFu);
  frame[2] = (uint8_t)(v >> 4);
  frame[3] = (uint8_t)(((v & 0x0Fu) << 4) | (kpi >> 8));
  frame[4] = (uint8_t)(kpi & 0xFFu);
  frame[5] = (uint8_t)(kdi >> 4);
  frame[6] = (uint8_t)(((kdi & 0x0Fu) << 4) | (t >> 8));
  frame[7] = (uint8_t)(t & 0xFFu);

  return CanBus_Send(&m->cfg.bus, m->cfg.can_id, frame);
}

bool DmMotor_OnFrame(DmMotor *m, uint32_t id, const uint8_t data[CAN_FRAME_DLEN], uint32_t now_ms) {
  if (m == NULL || !m->inited || data == NULL) {
    return false;
  }
  if (id != (uint32_t)m->cfg.feedback_id) {
    return false;
  }
  /* 手册 p12：D[0] = ID|ERR<<4，低 4 位是电机 ID。共用 Master ID 时靠这一级区分各轴 */
  const uint8_t id_nibble = (uint8_t)(data[0] & 0x0Fu);
  const uint8_t err_nibble = (uint8_t)(data[0] >> 4);
  if (id_nibble != (uint8_t)(m->cfg.can_id & 0x0Fu)) {
    m->fb.foreign_cnt++;
    return false;
  }

  const uint16_t p_int = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
  const uint16_t v_int = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
  const uint16_t t_int = (uint16_t)(((uint16_t)(data[4] & 0x0Fu) << 8) | data[5]);

  const DmLimits *lim = &m->cfg.limits;
  m->fb.pos = uint_to_float(p_int, lim->pos_min, lim->pos_max, DM_POS_BITS);
  m->fb.vel = uint_to_float(v_int, lim->vel_min, lim->vel_max, DM_VEL_BITS);
  m->fb.tau = uint_to_float(t_int, lim->tau_min, lim->tau_max, DM_TAU_BITS);
  m->fb.temp_mos = (float)data[6];
  m->fb.temp_rotor = (float)data[7];
  m->fb.id = id_nibble;
  /* 高 4 位只有低 3 位有定义（0/1/8~E），>0xE 的值不猜，原样截低 4 位存 */
  m->fb.state = (DmState)err_nibble;
  m->fb.rx_count++;
  m->fb.last_rx_ms = now_ms;
  return true;
}

bool DmMotor_IsOnline(const DmMotor *m, uint32_t now_ms) {
  if (m == NULL || !m->inited || m->fb.rx_count == 0u) {
    return false;
  }
  if (m->cfg.rx_timeout_ms == 0u) {
    return true;
  }
  return (uint32_t)(now_ms - m->fb.last_rx_ms) <= m->cfg.rx_timeout_ms;
}
