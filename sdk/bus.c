#include "bus.h"

#include <stddef.h>
#include <string.h>

/* 内部表结构：调用方拿不到 Bus 的字段（不透明），只能通过 API 改表。 */
typedef struct {
  uint32_t id;
  BusOnFrame fn;
  void *self;
  bool used;
} BusSub;

struct Bus {
  uint32_t magic;      /* 防止把野指针当 Bus 用 */
  CanPort *port;
  BusSub subs[SDK_MAX_SUB];
  uint8_t sub_count;
  bool polling;        /* D15：回调重入保护，Bus_Poll 期间置 1 */
  BusStats st;
};

#define BUS_MAGIC 0x42555331u

/* 静态池：最多 SDK_MAX_BUS 个，不 malloc、不释放（D16） */
static Bus s_pool[SDK_MAX_BUS];
static uint8_t s_pool_used;

static bool bus_ok(const Bus *bus) { return bus != NULL && bus->magic == BUS_MAGIC; }

static sdk_err_t bus_fail(Bus *bus, SdkReason why) {
  if (bus_ok(bus)) {
    bus->st.err_last = SDK_ERR(SDK_MOD_BUS, why);
  }
  return SDK_ERR(SDK_MOD_BUS, why);
}

sdk_err_t Bus_Create(CanPort *port, Bus **out) {
  if (port == NULL || out == NULL || port->ops == NULL) {
    return SDK_ERR(SDK_MOD_BUS, SDK_RS_INVALID_ARG);
  }
  if (s_pool_used >= SDK_MAX_BUS) {
    return SDK_ERR(SDK_MOD_BUS, SDK_RS_NO_RESOURCE);
  }
  Bus *bus = &s_pool[s_pool_used++];
  memset(bus, 0, sizeof(*bus));
  bus->magic = BUS_MAGIC;
  bus->port = port;
  *out = bus;
  return SDK_OK;
}

sdk_err_t Bus_Send(Bus *bus, uint32_t id, const uint8_t data[SDK_CAN_DLEN]) {
  if (!bus_ok(bus) || data == NULL || id > 0x7FFu) {
    return SDK_ERR(SDK_MOD_BUS, SDK_RS_INVALID_ARG);
  }
  if (bus->port->ops->send == NULL) {
    return bus_fail(bus, SDK_RS_INVALID_ARG);
  }
  const CanPortResult r = bus->port->ops->send(bus->port->ctx, id, data);
  switch (r) {
  case CAN_PORT_OK:
    bus->st.tx_ok++;
    return SDK_OK;
  case CAN_PORT_BUSY:
    bus->st.tx_busy++;
    return bus_fail(bus, SDK_RS_BUSY);
  case CAN_PORT_ERROR:
  default:
    bus->st.tx_error++;
    return bus_fail(bus, SDK_RS_HW_FAULT);
  }
}

sdk_err_t Bus_Subscribe(Bus *bus, uint32_t id, BusOnFrame fn, void *self) {
  if (!bus_ok(bus) || fn == NULL || id > 0x7FFu) {
    return SDK_ERR(SDK_MOD_BUS, SDK_RS_INVALID_ARG);
  }
  for (uint8_t i = 0; i < SDK_MAX_SUB; i++) {
    if (bus->subs[i].used && bus->subs[i].id == id && bus->subs[i].self == self) {
      return bus_fail(bus, SDK_RS_DUPLICATE); /* 同 (id,self) 重复；同 id 不同 self 合法 */
    }
  }
  for (uint8_t i = 0; i < SDK_MAX_SUB; i++) {
    if (!bus->subs[i].used) {
      bus->subs[i].used = true;
      bus->subs[i].id = id;
      bus->subs[i].fn = fn;
      bus->subs[i].self = self;
      bus->sub_count++;
      return SDK_OK;
    }
  }
  return bus_fail(bus, SDK_RS_NO_RESOURCE);
}

sdk_err_t Bus_Unsubscribe(Bus *bus, uint32_t id, void *self) {
  if (!bus_ok(bus)) {
    return SDK_ERR(SDK_MOD_BUS, SDK_RS_INVALID_ARG);
  }
  for (uint8_t i = 0; i < SDK_MAX_SUB; i++) {
    if (bus->subs[i].used && bus->subs[i].id == id && bus->subs[i].self == self) {
      memset(&bus->subs[i], 0, sizeof(bus->subs[i]));
      bus->sub_count--;
      return SDK_OK;
    }
  }
  return bus_fail(bus, SDK_RS_NOT_FOUND);
}

sdk_err_t Bus_GetStats(const Bus *bus, BusStats *out) {
  if (!bus_ok(bus) || out == NULL) {
    return SDK_ERR(SDK_MOD_BUS, SDK_RS_INVALID_ARG);
  }
  *out = bus->st; /* 一次性拷贝 = 同一时刻的快照（D17） */
  return SDK_OK;
}

uint8_t Bus_SubCount(const Bus *bus) { return bus_ok(bus) ? bus->sub_count : 0u; }
