/*
  a314_proto.c, see a314_proto.h
*/

#include "a314_proto.h"
#include <string.h>

#define A2R_BASE   0
#define R2A_BASE   256
#define CAP_BASE   512

#define CAP_R2A_TAIL   0
#define CAP_A2R_HEAD   1
#define CAP_A2R_TAIL   2
#define CAP_R2A_HEAD   3

#define PKT_CONNECT           4
#define PKT_CONNECT_RESPONSE  5
#define PKT_DATA              6
#define PKT_EOS               7
#define PKT_RESET             8

#define CONNECT_OK        0
#define CONNECT_UNKNOWN   3     // "service not available", as a314d answers

#define READ_FRAME_REQ    1
#define WRITE_FRAME_REQ   2
#define READ_FRAME_RES    3
#define WRITE_FRAME_RES   4

#define IRQ_PI_CLEAR      0x02  // REG_IRQ_CLR | REG_IRQ_PI
#define IRQ_AMIGA_SET     0x81  // REG_IRQ_SET | REG_IRQ_CP

static const char service_name[] = "ethernet";

static void ev(struct a314 *a, uint8_t e, uint16_t addr) {
  if(a->ops.event) a->ops.event(a->ops.ctx, e, addr);
}

void a314_init(struct a314 *a, const struct a314_ops *ops, uint32_t ram_size) {
  memset(a, 0, sizeof(*a));
  a->ops = *ops;
  a->ram_size = ram_size;
}

static void reset_state(struct a314 *a) {
  a->connected = false;
  a->n_rx = 0;
  a->q_rd = a->q_wr = 0;
}

/* ---- R2A ---- */

static bool q_full(const struct a314 *a)  { return (uint8_t)(a->q_wr - a->q_rd) >= A314_R2A_SLOTS; }

static bool q_put(struct a314 *a, uint8_t type, const uint8_t *data, uint8_t len) {
  if(q_full(a)) { a->st_dropped++; return false; }
  struct a314_pkt *p = &a->q[a->q_wr % A314_R2A_SLOTS];
  p->len = len; p->type = type; p->stream = a->stream;
  memcpy(p->data, data, len);
  a->q_wr++;
  return true;
}

static void put_frame_msg(struct a314 *a, uint32_t addr, uint16_t len, uint16_t kind) {
  uint8_t m[8] = { addr >> 24, addr >> 16, addr >> 8, addr, len >> 8, len, kind >> 8, kind };
  q_put(a, PKT_DATA, m, 8);
}

// as many waiting packets as fit into the R2A ring, then tell the Amiga
static void flush_r2a(struct a314 *a) {
  uint8_t tail = a->cap[CAP_R2A_TAIL], head = a->cap[CAP_R2A_HEAD];
  unsigned left = 255 - (uint8_t)(tail - head);
  unsigned n = 0;

  while(a->q_rd != a->q_wr) {
    const struct a314_pkt *p = &a->q[a->q_rd % A314_R2A_SLOTS];
    unsigned plen = 3 + p->len;
    if(left < plen) break;
    a->buf[n++] = p->len; a->buf[n++] = p->type; a->buf[n++] = p->stream;
    memcpy(a->buf + n, p->data, p->len); n += p->len;
    left -= plen;
    a->q_rd++;
  }
  if(!n) return;

  unsigned at_end = 256 - tail, first = n < at_end ? n : at_end;
  a->ops.mem_write(a->ops.ctx, R2A_BASE + tail, a->buf, first);
  if(first < n) a->ops.mem_write(a->ops.ctx, R2A_BASE, a->buf + first, n - first);
  a->cap[CAP_R2A_TAIL] = (uint8_t)(tail + n);
  a->ops.mem_write(a->ops.ctx, CAP_BASE + CAP_R2A_TAIL, &a->cap[CAP_R2A_TAIL], 1);
  a->ops.irq_write(a->ops.ctx, IRQ_AMIGA_SET);
}

/* ---- A2R ---- */

static bool frame_ok(const struct a314 *a, uint32_t addr, uint32_t len) {
  return len >= 14 && len <= A314_FRAME_MAX && addr + len <= a->ram_size;
}

static void handle_data(struct a314 *a, const uint8_t *d, uint8_t len) {
  if(len != 8) return;
  uint32_t addr = ((uint32_t)d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3];
  uint16_t flen = (d[4] << 8) | d[5];
  uint16_t kind = (d[6] << 8) | d[7];

  if(kind == WRITE_FRAME_REQ) {
    ev(a, A314_EV_TX_REQ, addr);
    if(frame_ok(a, addr, flen)) {
      a->ops.mem_read(a->ops.ctx, addr, a->buf + 256, flen);
      a->ops.frame_from_amiga(a->ops.ctx, a->buf + 256, flen);
      a->st_from_amiga++;
    } else a->st_dropped++;                          // the buffer is given back in any case
    put_frame_msg(a, addr, flen, WRITE_FRAME_RES);
    ev(a, A314_EV_TX_DONE, addr);
  } else if(kind == READ_FRAME_REQ) {
    // flen is the size of the buffer; a buffer that is not usable is simply never filled
    if(addr + A314_FRAME_MAX <= a->ram_size && flen >= A314_FRAME_MAX && a->n_rx < A314_RX_SLOTS) {
      a->rx[a->n_rx].addr = addr; a->rx[a->n_rx].len = flen; a->n_rx++;
      ev(a, A314_EV_RX_FREE, addr);
    } else a->st_dropped++;
  }
}

static void handle_packet(struct a314 *a, uint8_t type, uint8_t stream, const uint8_t *d, uint8_t len) {
  switch(type) {
  case PKT_CONNECT:
    if(len == sizeof(service_name) - 1 && !memcmp(d, service_name, len) && !a->connected) {
      a->connected = true; a->stream = stream; a->n_rx = 0; a->st_connect++;
      uint8_t res = CONNECT_OK;
      q_put(a, PKT_CONNECT_RESPONSE, &res, 1);
    } else {
      uint8_t res = CONNECT_UNKNOWN;
      a->st_refused++;
      uint8_t n = len < sizeof(a->last_refused) - 1 ? len : sizeof(a->last_refused) - 1;
      memcpy(a->last_refused, d, n); a->last_refused[n] = 0;
      uint8_t cur = a->stream; a->stream = stream;      // the response carries the id of the request
      q_put(a, PKT_CONNECT_RESPONSE, &res, 1);
      a->stream = cur;
    }
    break;
  case PKT_DATA:
    if(a->connected && stream == a->stream) handle_data(a, d, len);
    break;
  case PKT_EOS:
    break;                                           // not used by the ethernet service
  case PKT_RESET:
    if(a->connected && stream == a->stream) { reset_state(a); a->st_reset++; }
    break;
  }
}

static void receive_a2r(struct a314 *a) {
  uint8_t head = a->cap[CAP_A2R_HEAD], tail = a->cap[CAP_A2R_TAIL];
  unsigned len = (uint8_t)(tail - head);
  if(!len) return;

  if(head < tail) a->ops.mem_read(a->ops.ctx, A2R_BASE + head, a->buf, len);
  else {
    a->ops.mem_read(a->ops.ctx, A2R_BASE + head, a->buf, 256 - head);
    if(tail) a->ops.mem_read(a->ops.ctx, A2R_BASE, a->buf + (256 - head), tail);
  }

  // the frame of a WRITE_FRAME_REQ goes to buf + 256, the ring copy in buf[0..255] stays
  const uint8_t *p = a->buf, *end = a->buf + len;
  while(end - p >= 3) {
    uint8_t plen = p[0], type = p[1], stream = p[2];
    p += 3;
    if(end - p < plen) break;                        // cannot happen, the Amiga writes whole packets
    handle_packet(a, type, stream, p, plen);
    p += plen;
  }

  a->cap[CAP_A2R_HEAD] = tail;
  a->ops.mem_write(a->ops.ctx, CAP_BASE + CAP_A2R_HEAD, &a->cap[CAP_A2R_HEAD], 1);
}

void a314_service(struct a314 *a) {
  a->ops.irq_write(a->ops.ctx, IRQ_PI_CLEAR);      // first clear, then look: nothing gets lost

  uint8_t v[3];
  if(!a->magic) {
    a->ops.mem_read(a->ops.ctx, CAP_BASE + 4, v, 3);
    if(v[1] != 0xa3 || v[2] != 0x14) return;       // a314.device has not been started
    a->restart = v[0];
    a->magic = true;
  }

  a->st_service++;
  a->ops.mem_read(a->ops.ctx, CAP_BASE, a->cap, 4);

  a->ops.mem_read(a->ops.ctx, CAP_BASE + 4, v, 1);
  if(v[0] != a->restart) {                         // a314.device was started again: everything is gone
    reset_state(a);
    a->restart = v[0];
    a->st_restart++;
    a->ops.mem_read(a->ops.ctx, CAP_BASE, a->cap, 4);
  }

  receive_a2r(a);
  flush_r2a(a);
}

/* ---- frames to the Amiga ---- */

bool a314_can_send(const struct a314 *a) {
  return a->connected && a->n_rx > 0 && !q_full(a);
}

bool a314_send_frame(struct a314 *a, const uint8_t *frame, uint16_t len) {
  if(!a314_can_send(a) || len < 14 || len > A314_FRAME_MAX) return false;

  uint16_t addr = a->rx[0].addr;
  a->n_rx--;
  memmove(&a->rx[0], &a->rx[1], a->n_rx * sizeof(a->rx[0]));

  a->ops.mem_write(a->ops.ctx, addr, frame, len);
  put_frame_msg(a, addr, len, READ_FRAME_RES);
  a->st_to_amiga++;

  // the ring state of the Amiga may have changed since a314_service(), a314_service() is the only reader
  // of r2a_head, so a fresh copy is needed before writing
  a->ops.mem_read(a->ops.ctx, CAP_BASE + CAP_R2A_HEAD, &a->cap[CAP_R2A_HEAD], 1);
  flush_r2a(a);
  ev(a, A314_EV_RX_FILLED, addr);
  return true;
}
