/*
  a314_proto.h

  The Pi side of the A314 protocol (a314d + the "ethernet" service of the A314 project, CC0
  clockport variant) for the FPGA Companion. No lwIP, no SPI, no RTOS in here: the shared
  memory and the interrupt bits are reached through callbacks, so that the module can be
  tested on the host (test/a314) and the same code runs on the RP2040 (rp2040/a314_port.c).

  Shared memory (a314_cp.v in the core), byte addresses:

      0 .. 255   A2R ring  Amiga -> Companion, packets  {len, type, stream_id, payload}
    256 .. 511   R2A ring  Companion -> Amiga
    512 .. 515   r2a_tail (written here), a2r_head (here), a2r_tail (Amiga), r2a_head (Amiga)
    516          restart counter of a314.device (changes every time the driver starts)
    517, 518     0xA3, 0x14  (valid once a314.device has been started)
    520 ..       memory the Amiga allocates (frame buffers of a314eth.device, 4 x 1518 bytes)

  Rings hold at most 255 bytes (head == tail is empty). Packet types: 4 CONNECT, 5 CONNECT_RESPONSE,
  6 DATA, 7 EOS, 8 RESET. The service "ethernet" carries 8 byte DATA packets {u32 address,
  u16 length, u16 kind} (big endian):
      kind 1 READ_FRAME_REQ   Amiga: buffer at address (length = size) is free for a received frame
      kind 2 WRITE_FRAME_REQ  Amiga: buffer at address holds a frame of length bytes to send
      kind 3 READ_FRAME_RES   here: frame of length bytes is in the buffer
      kind 4 WRITE_FRAME_RES  here: frame has been taken, the buffer is free again
*/

#ifndef A314_PROTO_H
#define A314_PROTO_H

#include <stdint.h>
#include <stdbool.h>

#define A314_FRAME_MAX     1518   // RAW_MTU of a314eth.device
#define A314_RX_SLOTS      4      // free receive buffers that are remembered (the driver has 2)
#define A314_R2A_SLOTS     8      // packets waiting for room in the R2A ring

struct a314_ops {
  void *ctx;
  // shared memory of the core, address = byte address as the Amiga sees it
  void (*mem_read)(void *ctx, uint16_t addr, uint8_t *dst, uint16_t len);
  void (*mem_write)(void *ctx, uint16_t addr, const uint8_t *src, uint16_t len);
  // write REG_IRQ of the Pi side: bit 7 = set, bits 1:0 = which. 0x02 clears the Pi interrupt, 0x81 raises INT6 of the Amiga
  void (*irq_write)(void *ctx, uint8_t val);
  // ethernet frame the Amiga wants to send (valid until return)
  void (*frame_from_amiga)(void *ctx, const uint8_t *frame, uint16_t len);
  // optional (may be NULL): buffer events for timing statistics, addr = frame buffer of the Amiga
  void (*event)(void *ctx, uint8_t ev, uint16_t addr);
};

// events for a314_ops.event
#define A314_EV_RX_FREE    1     // READ_FRAME_REQ: the Amiga offers an empty receive buffer
#define A314_EV_RX_FILLED  2     // a frame has been written to that buffer and announced
#define A314_EV_TX_REQ     3     // WRITE_FRAME_REQ: the Amiga has a frame to send in a buffer
#define A314_EV_TX_DONE    4     // the frame has been taken, the buffer goes back to the Amiga

struct a314_pkt {
  uint8_t len, type, stream;
  uint8_t data[8];
};

struct a314 {
  struct a314_ops ops;
  uint32_t ram_size;              // bytes of shared memory, frames outside are refused

  bool     magic;                 // 0xA3 0x14 seen
  uint8_t  restart;               // restart counter we know
  bool     connected;
  uint8_t  stream;
  uint8_t  cap[4];                // r2a_tail, a2r_head, a2r_tail, r2a_head as last read/written
  bool     r2a_dirty;             // r2a_tail changed, Amiga to be notified

  struct { uint16_t addr, len; } rx[A314_RX_SLOTS];
  uint8_t  n_rx;
  struct a314_pkt q[A314_R2A_SLOTS];
  uint8_t  q_rd, q_wr;            // free running

  uint32_t st_connect, st_to_amiga, st_from_amiga, st_dropped, st_restart;
  uint32_t st_refused, st_reset, st_service;   // connects refused, streams closed by the Amiga, a314_service() with magic
  uint8_t  last_refused[12];                   // service name of the last refused connect, 0 terminated
  uint8_t  buf[256 + A314_FRAME_MAX];
};

void a314_init(struct a314 *a, const struct a314_ops *ops, uint32_t ram_size);

// the Pi interrupt bit is set (or just try it): clears it, takes everything the Amiga has queued,
// sends what is waiting for the Amiga. Safe to call at any time
void a314_service(struct a314 *a);

// a frame for the Amiga: needs a free receive buffer of the Amiga. false = no room, try again after a314_service()
bool a314_can_send(const struct a314 *a);
bool a314_send_frame(struct a314 *a, const uint8_t *frame, uint16_t len);

static inline bool a314_connected(const struct a314 *a) { return a->connected; }

#endif // A314_PROTO_H
