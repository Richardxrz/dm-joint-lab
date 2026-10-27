/**
 * @file main.c
 * @brief dm_motor 的规格测试（L2）：只依赖 .h 契约，不需要硬件。
 *
 * 判据层级：契约测试（模块自己承诺的语义），不是业务验收。
 * 一格失败即打印 期望 vs 实际，不做归因。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "dm_motor.h"
#include "host_can_port.h"

static int g_fail;
static int g_total;

static void report(bool ok, const char *what) {
  g_total++;
  if (!ok) {
    g_fail++;
  }
  printf("%s %s\n", ok ? "✅" : "❌", what);
}

static void check_int(long want, long got, const char *what) {
  char buf[160];
  snprintf(buf, sizeof(buf), "%s（期望 %ld，实际 %ld）", what, want, got);
  report(want == got, buf);
}

static void check_float(double want, double got, double eps, const char *what) {
  char buf[160];
  snprintf(buf, sizeof(buf), "%s（期望 %.6f，实际 %.6f）", what, want, got);
  report(fabs(want - got) <= eps, buf);
}

static void check_bytes(const uint8_t want[8], const uint8_t got[8], const char *what) {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "%s（期望 %02X %02X %02X %02X %02X %02X %02X %02X，实际 %02X %02X %02X %02X %02X %02X %02X %02X）",
           what, want[0], want[1], want[2], want[3], want[4], want[5], want[6], want[7], got[0],
           got[1], got[2], got[3], got[4], got[5], got[6], got[7]);
  report(memcmp(want, got, 8) == 0, buf);
}

/* ---------- 测试替身：记录最后一次发出的帧 ---------- */

typedef struct {
  int count;
  uint32_t id;
  uint8_t data[8];
} FakeBus;

static bool fake_send(void *ctx, uint32_t id, const uint8_t data[8]) {
  FakeBus *b = (FakeBus *)ctx;
  b->count++;
  b->id = id;
  memcpy(b->data, data, 8);
  return true;
}

static bool fake_send_fail(void *ctx, uint32_t id, const uint8_t data[8]) {
  (void)ctx;
  (void)id;
  (void)data;
  return false;
}

static DmMotorConfig make_cfg(FakeBus *bus) {
  DmMotorConfig cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.bus.send = fake_send;
  cfg.bus.ctx = bus;
  cfg.can_id = 1;
  cfg.feedback_id = 0; /* 达妙默认：反馈走主控 ID */
  cfg.limits = DmLimits_DM4340();
  cfg.rx_timeout_ms = 100;
  return cfg;
}

/* ---------- 用例 ---------- */

static void t_limits(void) {
  DmLimits lim = DmLimits_DM4340();
  check_float(12.5, lim.pos_max, 1e-6, "DM4340 位置上限 = +12.5 rad");
  check_float(-12.5, lim.pos_min, 1e-6, "DM4340 位置下限 = -12.5 rad");
  check_float(10.0, lim.vel_max, 1e-6, "DM4340 速度上限 = +10 rad/s");
  check_float(28.0, lim.tau_max, 1e-6, "DM4340 力矩上限 = +28 N·m");
  check_float(500.0, lim.kp_max, 1e-6, "Kp 上限 = 500");
  check_float(5.0, lim.kd_max, 1e-6, "Kd 上限 = 5");
}

static void t_mit_zero(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  report(DmMotor_Init(&m, &cfg), "t_mit_zero: Init 成功");
  report(DmMotor_MitCommand(&m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f), "t_mit_zero: 命令已交给总线");
  const uint8_t want[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  check_bytes(want, bus.data, "t_mit_zero: 8 字节 = 中位(0 rad/0 rad·s⁻¹/0 N·m)");
  check_int(1, bus.id, "t_mit_zero: 控制帧 ID = 电机 CAN ID");
}

static void t_mit_extremes(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  DmMotor_MitCommand(&m, 12.5f, 10.0f, 500.0f, 5.0f, 28.0f);
  const uint8_t want_max[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  check_bytes(want_max, bus.data, "t_mit_extremes: 全上限 → 全 0xFF");

  DmMotor_MitCommand(&m, -12.5f, -10.0f, 0.0f, 0.0f, -28.0f);
  const uint8_t want_min[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  check_bytes(want_min, bus.data, "t_mit_extremes: 全下限 → 全 0x00");
}

static void t_mit_clamp(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  /* 超界值必须饱和，不能溢出成反向 */
  DmMotor_MitCommand(&m, 100.0f, -100.0f, 1e6f, 1e6f, -1e6f);
  const uint8_t want[8] = {0xFF, 0xFF, 0x00, 0x0F, 0xFF, 0xFF, 0xF0, 0x00};
  check_bytes(want, bus.data, "t_mit_clamp: 超界入参饱和到端点");
}

static void t_mit_nan(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  DmMotor_MitCommand(&m, NAN, 0.0f, 0.0f, 0.0f, 0.0f);
  const uint8_t want[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
  check_bytes(want, bus.data, "t_mit_nan: NaN 位置当 0 处理（不产生 0xFFFF 或随机值）");
}

static void t_command_frames(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  const uint8_t tail_enable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  const uint8_t tail_disable[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
  const uint8_t tail_zero[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
  const uint8_t tail_clear[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};

  DmMotor_Enable(&m);
  check_bytes(tail_enable, bus.data, "t_command_frames: 使能 = 7×FF + FC");
  DmMotor_Disable(&m);
  check_bytes(tail_disable, bus.data, "t_command_frames: 失能 = 7×FF + FD");
  DmMotor_SetZero(&m);
  check_bytes(tail_zero, bus.data, "t_command_frames: 设零 = 7×FF + FE");
  DmMotor_ClearError(&m);
  check_bytes(tail_clear, bus.data, "t_command_frames: 清错 = 7×FF + FB");
  check_int(1, bus.id, "t_command_frames: 特殊帧 ID = 电机 CAN ID");
  check_int(4, bus.count, "t_command_frames: 四条命令各发一帧，无重复发送");
}

static void t_decode_endpoints(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  /* pos=+12.5 / vel=+10 / tau=+28 / MOS 60℃ / 转子 45℃ */
  const uint8_t fb[8] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 60, 45};
  report(DmMotor_OnFrame(&m, 0, fb, 1000), "t_decode: 帧被识别为该轴反馈");
  check_float(12.5, m.fb.pos, 1e-4, "t_decode: 位置 = +12.5 rad");
  check_float(10.0, m.fb.vel, 1e-4, "t_decode: 速度 = +10 rad/s");
  check_float(28.0, m.fb.tau, 1e-4, "t_decode: 力矩 = +28 N·m");
  check_float(60.0, m.fb.temp_mos, 1e-6, "t_decode: MOS 温度 = 60 ℃");
  check_float(45.0, m.fb.temp_rotor, 1e-6, "t_decode: 转子温度 = 45 ℃");

  /* 12 位字段跨字节拆分：vel=0x123, tau=0x456 */
  const uint8_t fb2[8] = {0x11, 0x80, 0x00, 0x12, 0x34, 0x56, 0, 0};
  DmMotor_OnFrame(&m, 0, fb2, 1010);
  const double v_want = (0x123 / 4095.0) * 20.0 - 10.0;
  const double t_want = (0x456 / 4095.0) * 56.0 - 28.0;
  check_float(v_want, m.fb.vel, 1e-4, "t_decode: 12 位速度跨字节拼接正确（0x123）");
  check_float(t_want, m.fb.tau, 1e-4, "t_decode: 12 位力矩跨字节拼接正确（0x456）");
}

static void t_decode_reject(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  const uint8_t fb[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 60, 45};
  report(!DmMotor_OnFrame(&m, 0x7, fb, 1000), "t_decode_reject: 非本轴 ID 的帧被拒");
  check_int(0, (long)m.fb.rx_count, "t_decode_reject: 被拒的帧不计入 rx_count");
  check_float(0.0, m.fb.pos, 1e-9, "t_decode_reject: 被拒的帧不改位置");
}

static void t_timeout(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  report(!DmMotor_IsOnline(&m, 500), "t_timeout: 从未收到反馈 → 不在线");

  const uint8_t fb[8] = {0x01, 0, 0, 0, 0, 0, 0, 0};
  DmMotor_OnFrame(&m, 0, fb, 1000);
  report(DmMotor_IsOnline(&m, 1050), "t_timeout: 距上一帧 50 ms（阈 100）→ 在线");
  report(DmMotor_IsOnline(&m, 1100), "t_timeout: 正好 100 ms → 仍在线");
  report(!DmMotor_IsOnline(&m, 1101), "t_timeout: 101 ms → 判定掉线");

  /* 32 位时基回绕：0xFFFFFFF0 → 0x10 只过了 32 tick */
  DmMotor m2;
  DmMotorConfig cfg2 = make_cfg(&bus);
  DmMotor_Init(&m2, &cfg2);
  DmMotor_OnFrame(&m2, 0, fb, 0xFFFFFFF0u);
  report(DmMotor_IsOnline(&m2, 0x00000010u), "t_timeout: 时基回绕后仍算作 32 ms（无符号差值）");
}

static void t_two_motors(void) {
  FakeBus bus = {0};
  DmMotor a, b;
  DmMotorConfig cfg_a = make_cfg(&bus);
  cfg_a.feedback_id = 0;
  DmMotorConfig cfg_b = make_cfg(&bus);
  cfg_b.can_id = 2;
  cfg_b.feedback_id = 1;
  DmMotor_Init(&a, &cfg_a);
  DmMotor_Init(&b, &cfg_b);

  const uint8_t fb[8] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 1, 2};
  DmMotor_OnFrame(&b, 1, fb, 500);
  check_int(0, (long)a.fb.rx_count, "t_two_motors: 第二条轴的反馈不串到第一条轴");
  check_int(1, (long)b.fb.rx_count, "t_two_motors: 第二条轴自己收到 1 帧");
  check_int(0, (long)a.fb.foreign_cnt, "t_two_motors: 第一条轴不把别人的帧记成 foreign");
}

static void t_state_decode(void) {
  FakeBus bus = {0};
  DmMotor m;
  DmMotorConfig cfg = make_cfg(&bus);
  DmMotor_Init(&m, &cfg);

  const struct {
    uint8_t err;
    DmState want;
    bool fault;
    const char *name;
  } cases[] = {
      {0x0, DM_STATE_DISABLED, false, "失能"},
      {0x1, DM_STATE_ENABLED, false, "使能"},
      {0x8, DM_STATE_OVER_VOLTAGE, true, "超压"},
      {0x9, DM_STATE_UNDER_VOLTAGE, true, "欠压"},
      {0xA, DM_STATE_OVER_CURRENT, true, "过流"},
      {0xB, DM_STATE_MOS_OVER_TEMP, true, "MOS 过温"},
      {0xC, DM_STATE_COIL_OVER_TEMP, true, "线圈过温"},
      {0xD, DM_STATE_COMM_LOST, true, "通讯丢失"},
      {0xE, DM_STATE_OVERLOAD, true, "过载"},
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    uint8_t fb[8] = {0};
    fb[0] = (uint8_t)((cases[i].err << 4) | 1u);
    DmMotor_OnFrame(&m, 0, fb, 1000);
    char buf[96];
    snprintf(buf, sizeof(buf), "t_state_decode: ERR=0x%X → %s（state=%d）", cases[i].err,
             cases[i].name, (int)m.fb.state);
    report(m.fb.state == cases[i].want, buf);
    snprintf(buf, sizeof(buf), "t_state_decode: ERR=0x%X → IsFault=%d", cases[i].err,
             (int)cases[i].fault);
    report(DmState_IsFault(m.fb.state) == cases[i].fault, buf);
  }
  /* 未定义的高 4 位值（0xF）不许当故障猜 */
  uint8_t fb_unknown[8] = {0xF1, 0, 0, 0, 0, 0, 0, 0};
  DmMotor_OnFrame(&m, 0, fb_unknown, 1000);
  report(!DmState_IsFault(m.fb.state), "t_state_decode: 未定义 ERR=0xF 不判为故障");
}

static void t_foreign_id(void) {
  FakeBus bus = {0};
  DmMotor a, b;
  DmMotorConfig cfg_a = make_cfg(&bus); /* can_id 1, feedback_id 0 */
  DmMotorConfig cfg_b = make_cfg(&bus);
  cfg_b.can_id = 2; /* 共用同一个 Master ID 0 */
  DmMotor_Init(&a, &cfg_a);
  DmMotor_Init(&b, &cfg_b);

  /* 同一 Master ID 下，A 的反馈（D[0]=0x01）不能被 B 吃掉 */
  const uint8_t fb_a[8] = {0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 60, 45};
  report(DmMotor_OnFrame(&a, 0, fb_a, 100) == true, "t_foreign_id: A 收到自己的帧");
  report(DmMotor_OnFrame(&b, 0, fb_a, 100) == false, "t_foreign_id: B 拒收 A 的帧");
  check_float(0.0, b.fb.pos, 1e-9, "t_foreign_id: B 的位置未被 A 的帧改写");
  check_int(1, (long)b.fb.foreign_cnt, "t_foreign_id: B 的 foreign_cnt = 1（串轴可见）");
  check_int(1, (long)a.fb.rx_count, "t_foreign_id: A 的 rx_count 不受影响");
}

static void t_fail_paths(void) {
  FakeBus bus = {0};
  DmMotor m;

  DmMotorConfig bad = make_cfg(&bus);
  bad.bus.send = NULL;
  report(!DmMotor_Init(&m, &bad), "t_fail_paths: 没注入 send 的配置 → Init 失败");

  memset(&m, 0, sizeof(m));
  report(!DmMotor_MitCommand(&m, 0, 0, 0, 0, 0), "t_fail_paths: 未 Init 就下发 → 返回 false");
  report(!DmMotor_Enable(&m), "t_fail_paths: 未 Init 就使能 → 返回 false");

  DmMotorConfig cfg = make_cfg(&bus);
  cfg.bus.send = fake_send_fail;
  DmMotor m2;
  DmMotor_Init(&m2, &cfg);
  report(!DmMotor_MitCommand(&m2, 0, 0, 0, 0, 0), "t_fail_paths: 总线发送失败 → 命令返回 false");
}


/* ---------- 假端口自检：测试替身本身必须先是对的 ---------- */

static void t_host_port(void) {
  HostCan hc;
  HostCan_Init(&hc);
  CanPort *port = HostCan_Port(&hc);
  report(CanPort_IsComplete(port), "t_host_port: ops 五项填全（CanPort_IsComplete）");

  uint32_t id = 0;
  uint8_t buf[CAN_PORT_DLEN] = {0};
  report(hc.port.ops->recv(hc.port.ctx, &id, buf) == CAN_PORT_EMPTY,
         "t_host_port: 空 FIFO 的 recv 返回 EMPTY（不是 ERROR）");

  const uint8_t f1[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  report(HostCan_InjectFrame(&hc, 0x11, f1), "t_host_port: 注入 1 帧成功");
  check_int(1, HostCan_PendingRx(&hc), "t_host_port: FIFO 里剩 1 帧");
  report(hc.port.ops->recv(hc.port.ctx, &id, buf) == CAN_PORT_OK, "t_host_port: recv 取到一帧");
  check_int(0x11, (long)id, "t_host_port: 取回的帧 ID 正确");
  report(memcmp(buf, f1, 8) == 0, "t_host_port: 取回的 8 字节内容一致");
  report(hc.port.ops->recv(hc.port.ctx, &id, buf) == CAN_PORT_EMPTY,
         "t_host_port: 取空后 recv 回到 EMPTY");

  /* FIFO 满：第 9 帧被丢并计数（与真实端口契约一致） */
  HostCan_Init(&hc);
  int ok = 0;
  for (int i = 0; i < (int)HOST_CAN_RX_DEPTH; i++) {
    ok += HostCan_InjectFrame(&hc, 0x11, f1) ? 1 : 0;
  }
  check_int((long)HOST_CAN_RX_DEPTH, ok, "t_host_port: FIFO 满之前全部注入成功");
  report(!HostCan_InjectFrame(&hc, 0x11, f1), "t_host_port: 第 9 帧被拒");
  check_int(1, (long)hc.port.ops->dropped(hc.port.ctx),
            "t_host_port: 溢出计入 dropped");

  /* 发送侧记录 + 强制失败 */
  HostCan_Init(&hc);
  const uint8_t t1[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0, 0, 0x07, 0xFF};
  hc.port.ops->send(hc.port.ctx, 1, t1);
  hc.port.ops->send(hc.port.ctx, 2, t1);
  check_int(2, (long)HostCan_SentCount(&hc), "t_host_port: 两次 send 都被记录");
  uint8_t got[8] = {0};
  uint32_t gid = 0;
  report(HostCan_GetSent(&hc, 0, &gid, got) && gid == 1 && memcmp(got, t1, 8) == 0,
         "t_host_port: 第 0 次发送的 ID 与内容可回看");

  HostCan_SetSendResult(&hc, CAN_PORT_BUSY);
  report(hc.port.ops->send(hc.port.ctx, 3, t1) == CAN_PORT_BUSY,
         "t_host_port: 可强制 send 返回 BUSY");
  check_int(2, (long)HostCan_SentCount(&hc), "t_host_port: 失败的那次不进发送记录");
}

int main(void) {
  printf("=== dm_motor 规格测试（host，无硬件） ===\n");
  t_limits();
  t_mit_zero();
  t_mit_extremes();
  t_mit_clamp();
  t_mit_nan();
  t_command_frames();
  t_decode_endpoints();
  t_decode_reject();
  t_timeout();
  t_two_motors();
  t_state_decode();
  t_foreign_id();
  t_fail_paths();
  t_host_port();
  printf("---\n%d 条判据，失败 %d 条\n", g_total, g_fail);
  return g_fail == 0 ? 0 : 1;
}
