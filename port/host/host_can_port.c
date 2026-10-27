#include "host_can_port.h"

#include <string.h>

/* ---- CanPortOps 的五个实现；ctx 一律是 HostCan* ---- */

static CanPortResult host_send(void *ctx, uint32_t id, const uint8_t data[CAN_PORT_DLEN]) {
  HostCan *hc = (HostCan *)ctx;
  if (hc->send_result != CAN_PORT_OK) {
    return hc->send_result; /* 强制失败：不计入发送记录，与"没发出去"一致 */
  }
  hc->tx[hc->tx_count % HOST_CAN_TX_LOG].id = id;
  memcpy(hc->tx[hc->tx_count % HOST_CAN_TX_LOG].data, data, CAN_PORT_DLEN);
  hc->tx_count++;
  return CAN_PORT_OK;
}

static CanPortResult host_recv(void *ctx, uint32_t *id, uint8_t data[CAN_PORT_DLEN]) {
  HostCan *hc = (HostCan *)ctx;
  if (hc->rx_len == 0u) {
    return CAN_PORT_EMPTY; /* 没帧是正常态，不是错误 */
  }
  *id = hc->rx[hc->rx_tail].id;
  memcpy(data, hc->rx[hc->rx_tail].data, CAN_PORT_DLEN);
  hc->rx_tail = (uint8_t)((hc->rx_tail + 1u) % HOST_CAN_RX_DEPTH);
  hc->rx_len--;
  return CAN_PORT_OK;
}

static uint32_t host_dropped(void *ctx) { return ((HostCan *)ctx)->dropped; }
static uint32_t host_bus_off(void *ctx) { return ((HostCan *)ctx)->bus_off_cnt; }
static uint32_t host_hw_errors(void *ctx) { return ((HostCan *)ctx)->hw_err_cnt; }

static const CanPortOps k_ops = {
    .send = host_send,
    .recv = host_recv,
    .dropped = host_dropped,
    .bus_off = host_bus_off,
    .hw_errors = host_hw_errors,
};

void HostCan_Init(HostCan *hc) {
  if (hc == NULL) {
    return;
  }
  memset(hc, 0, sizeof(*hc));
  hc->port.ops = &k_ops;
  hc->port.ctx = hc;
  hc->send_result = CAN_PORT_OK;
}

CanPort *HostCan_Port(HostCan *hc) { return hc == NULL ? NULL : &hc->port; }

bool HostCan_InjectFrame(HostCan *hc, uint32_t id, const uint8_t data[CAN_PORT_DLEN]) {
  if (hc == NULL || data == NULL) {
    return false;
  }
  if (hc->rx_len >= HOST_CAN_RX_DEPTH) {
    /* 与真实端口契约一致：满了丢新帧并计数，不覆盖未读数据 */
    hc->dropped++;
    return false;
  }
  const uint8_t slot = (uint8_t)((hc->rx_tail + hc->rx_len) % HOST_CAN_RX_DEPTH);
  hc->rx[slot].id = id;
  memcpy(hc->rx[slot].data, data, CAN_PORT_DLEN);
  hc->rx_len++;
  return true;
}

uint8_t HostCan_PendingRx(const HostCan *hc) { return hc == NULL ? 0u : hc->rx_len; }

void HostCan_SetSendResult(HostCan *hc, CanPortResult r) {
  if (hc != NULL) {
    hc->send_result = r;
  }
}

void HostCan_SetCounters(HostCan *hc, uint32_t dropped, uint32_t bus_off, uint32_t hw_errs) {
  if (hc == NULL) {
    return;
  }
  hc->dropped = dropped;
  hc->bus_off_cnt = bus_off;
  hc->hw_err_cnt = hw_errs;
}

uint32_t HostCan_SentCount(const HostCan *hc) { return hc == NULL ? 0u : hc->tx_count; }

bool HostCan_GetSent(const HostCan *hc, uint32_t index, uint32_t *id,
                     uint8_t data[CAN_PORT_DLEN]) {
  if (hc == NULL || index >= hc->tx_count || index >= HOST_CAN_TX_LOG) {
    return false;
  }
  if (id != NULL) {
    *id = hc->tx[index].id;
  }
  if (data != NULL) {
    memcpy(data, hc->tx[index].data, CAN_PORT_DLEN);
  }
  return true;
}

void HostCan_ResetLog(HostCan *hc) {
  if (hc == NULL) {
    return;
  }
  memset(hc->tx, 0, sizeof(hc->tx));
  hc->tx_count = 0u;
}
