// Host test of a314_proto.c: the protocol engine of the Pi side against the model of the core
// (a314_cp.v) and of a314.device + a314eth.device. The memory is reached directly (backdoor),
// the SPI byte protocol is tested in test_a314_port.c.
#include "a314_model.h"
#include "../../src/a314_proto.h"

static struct a314 pi;
static uint8_t from_amiga[64][RAW_MTU]; static int from_amiga_len[64]; static int n_from_amiga;

static void m_read(void *c, uint16_t a, uint8_t *d, uint16_t n)       { (void)c; memcpy(d, core.ram + (a & (M_RAM - 1)), n); }
static void m_write(void *c, uint16_t a, const uint8_t *s, uint16_t n) { (void)c; memcpy(core.ram + (a & (M_RAM - 1)), s, n); }
static void m_irq(void *c, uint8_t v)                                  { (void)c; irq_write(v); }
static void m_frame(void *c, const uint8_t *f, uint16_t n) {
  (void)c;
  if(n_from_amiga < 64) { memcpy(from_amiga[n_from_amiga], f, n); from_amiga_len[n_from_amiga++] = n; }
}
static void pi_init(void) {
  struct a314_ops ops = { NULL, m_read, m_write, m_irq, m_frame };
  a314_init(&pi, &ops, M_RAM);
  n_from_amiga = 0;
}
// what the interrupt handler of the Companion does
static void pi_run(void) { if(core.pi_irq) a314_service(&pi); }
static void run(void) { for(int i = 0; i < 3; i++) { pi_run(); a_service(); } }

static void mkframe(uint8_t *f, int len, int seed) {
  for(int i = 0; i < len; i++) f[i] = (i * 31 + seed * 7 + (i >> 8)) & 0xff;
}

int main(void) {
  static uint8_t f[RAW_MTU], g[RAW_MTU];

  core_reset(); pi_init(); sim_clear_amiga();

  // 1. nothing happens before a314.device has been started
  CHECK(a314_can_send(&pi) == false, "not connected: cannot send");
  core.pi_irq = true; pi_run();
  CHECK(core.pi_irq == false && !pi.magic, "interrupt cleared, no magic yet, nothing else happens");
  CHECK(a314_send_frame(&pi, f, 60) == false, "no frame without a connection");

  // 2. start of the driver, CONNECT "ethernet"
  a_start();
  CHECK(core.pi_irq, "set_pi_irq raises the interrupt for the Pi");
  pi_run();
  CHECK(pi.magic && !pi.connected, "magic found, still no stream");
  a_connect("ethernet", eth_stream); run();
  CHECK(pi.connected && eth_connected && eth_connect_result == 0, "CONNECT ethernet accepted (result 0)");
  CHECK(core.pi_irq == false, "pi_irq cleared by the Pi");

  // 3. second connect while connected, and an unknown service: result 3
  eth_connect_result = -1; a_connect("ethernet", 2); run();
  CHECK(eth_connect_result == 3 && pi.stream == eth_stream, "second ethernet connect refused, old stream kept");
  eth_connect_result = -1; a_connect("telnetd", 3); run();
  CHECK(eth_connect_result == 3, "unknown service refused");

  // 4. the Amiga offers receive buffers, the Pi puts frames into them
  mkframe(f, 60, 1);
  CHECK(!a314_can_send(&pi), "no receive buffer yet");
  a_post_rx_buffer(0); a_post_rx_buffer(1); run();
  CHECK(pi.n_rx == 2 && a314_can_send(&pi), "two receive buffers known");
  CHECK(a314_send_frame(&pi, f, 60), "frame 1 sent");
  mkframe(g, 1514, 2);
  CHECK(a314_send_frame(&pi, g, 1514), "frame 2 (1514 bytes) sent");
  CHECK(!a314_can_send(&pi) && !a314_send_frame(&pi, f, 60), "both buffers used: refused");
  CHECK(core.cp_irq, "Amiga interrupted");
  a_service();
  CHECK(a_rx_n == 2 && a_rxq[0].len == 60 && !memcmp(a_rxq[0].data, f, 60) &&
	a_rxq[1].len == 1514 && !memcmp(a_rxq[1].data, g, 1514), "Amiga got both frames unchanged, in order");
  pi_run();
  CHECK(pi.n_rx == 2 && a314_can_send(&pi), "buffers re-posted by the driver, usable again");
  a_rx_n = 0;

  // 5. frames from the Amiga (2 send buffers)
  mkframe(f, 100, 3); mkframe(g, 1514, 4);
  CHECK(a_send_frame(f, 100) && a_send_frame(g, 1514), "two frames written");
  CHECK(!a_send_frame(f, 100), "both send buffers in use");
  run();
  CHECK(n_from_amiga == 2 && from_amiga_len[0] == 100 && !memcmp(from_amiga[0], f, 100) &&
	from_amiga_len[1] == 1514 && !memcmp(from_amiga[1], g, 1514), "Pi received both frames unchanged");
  CHECK(a_tx_done == 2 && a_send_frame(f, 100), "WRITE_FRAME_RES frees the buffers");
  run(); n_from_amiga = 0; a_tx_done = 0;

  // 6. long run: both directions, all sizes, the rings wrap many times
  {
    static struct { int len, seed; } exp_pi[1000], exp_am[1000];
    int n_exp_pi = 0, n_exp_am = 0, pi_got = 0, am_got = 0, bad = 0;
    static uint8_t e[RAW_MTU];
    for(int i = 0; i < 600; i++) {
      int len = 14 + (i * 97) % 1501;
      mkframe(f, len, i);
      if(i % 2) {                                         // Pi -> Amiga
        if(a314_send_frame(&pi, f, len)) { exp_am[n_exp_am].len = len; exp_am[n_exp_am++].seed = i; }
      } else {                                            // Amiga -> Pi
        if(a_send_frame(f, len)) { exp_pi[n_exp_pi].len = len; exp_pi[n_exp_pi++].seed = i; }
      }
      a_service(); pi_run(); a_service(); pi_run();
      for(int k = 0; k < n_from_amiga; k++, pi_got++) {
        mkframe(e, exp_pi[pi_got].len, exp_pi[pi_got].seed);
        if(from_amiga_len[k] != exp_pi[pi_got].len || memcmp(e, from_amiga[k], from_amiga_len[k])) bad++;
      }
      n_from_amiga = 0;
      for(int k = 0; k < a_rx_n; k++, am_got++) {
        mkframe(e, exp_am[am_got].len, exp_am[am_got].seed);
        if(a_rxq[k].len != exp_am[am_got].len || memcmp(e, a_rxq[k].data, a_rxq[k].len)) bad++;
      }
      a_rx_n = 0;
    }
    run();
    for(int k = 0; k < n_from_amiga; k++, pi_got++) {
      mkframe(e, exp_pi[pi_got].len, exp_pi[pi_got].seed);
      if(from_amiga_len[k] != exp_pi[pi_got].len || memcmp(e, from_amiga[k], from_amiga_len[k])) bad++;
    }
    n_from_amiga = 0;
    for(int k = 0; k < a_rx_n; k++, am_got++) {
      mkframe(e, exp_am[am_got].len, exp_am[am_got].seed);
      if(a_rxq[k].len != exp_am[am_got].len || memcmp(e, a_rxq[k].data, a_rxq[k].len)) bad++;
    }
    a_rx_n = 0;
    printf("   long run: %d frames Amiga->Pi, %d frames Pi->Amiga\n", n_exp_pi, n_exp_am);
    CHECK(n_exp_pi > 100 && n_exp_am > 100, "plenty of frames both ways");
    CHECK(pi_got == n_exp_pi && am_got == n_exp_am, "every frame arrived, none twice");
    CHECK(bad == 0, "all frames unchanged and in order");
    CHECK(pi.st_dropped == 0, "nothing dropped");
  }

  // 7. R2A ring full: packets wait, nothing is overwritten, they follow when the Amiga has read
  {
    run(); a_rx_n = 0;
    uint8_t tail = cap.r2a_tail;
    // the Amiga has not read the ring for a while: make 4 free bytes (head 5 bytes behind the tail)
    cap.r2a_head = tail + 5; a_write_amiga_cap();
    mkframe(f, 70, 99);
    int before_q = pi.q_wr - pi.q_rd;
    bool ok = a314_send_frame(&pi, f, 70);
    CHECK(ok, "frame accepted although the R2A ring is full");
    CHECK(pi.q_wr - pi.q_rd == before_q + 1, "its notification waits in the queue");
    CHECK(core.ram[CAP_BASE + 0] == tail, "r2a_tail not moved");
    cap.r2a_head = tail; a_write_amiga_cap();                  // the Amiga has read everything
    a_set_pi_irq(); pi_run();
    CHECK(pi.q_wr == pi.q_rd && core.ram[CAP_BASE + 0] == (uint8_t)(tail + 11), "notification delivered after the Amiga made room");
    a_service();
    CHECK(a_rx_n == 1 && a_rxq[0].len == 70, "frame arrives"); a_rx_n = 0;
  }

  // 8. RESET of the stream closes it, a new CONNECT works
  a_send_pkt(8, eth_stream, 0, NULL); run();
  CHECK(!pi.connected && !a314_can_send(&pi), "RESET closes the stream, buffers forgotten");
  a_connect("ethernet", 5); eth_stream = 5; run();
  CHECK(pi.connected && pi.stream == 5, "new connection on stream 5");
  memset(buf_pending, 0, sizeof(buf_pending));
  a_post_rx_buffer(0); a_post_rx_buffer(1); run();
  CHECK(a314_can_send(&pi), "buffers of the new stream");

  // 9. a314.device restarts (Amiga reset): connection state is dropped, CONNECT works again
  sim_clear_amiga(); eth_stream = 1;
  a_start(); run();
  CHECK(!pi.connected && pi.st_restart == 1, "restart counter changed: connection and buffers dropped");
  a_connect("ethernet", eth_stream); run();
  CHECK(pi.connected && eth_connected, "CONNECT ethernet after the restart");
  a_post_rx_buffer(0); a_post_rx_buffer(1); run();
  mkframe(f, 200, 5);
  CHECK(a314_send_frame(&pi, f, 200), "frame after restart"); a_service();
  CHECK(a_rx_n == 1 && !memcmp(a_rxq[0].data, f, 200), "arrives"); a_rx_n = 0; pi_run();

  // 10. bad requests: outside of the RAM, too long, too short. The Amiga always gets its buffer back
  {
    n_from_amiga = 0; a_tx_done = 0;
    uint8_t m[8];
    put_msg(m, M_RAM - 100, 1514, 2); a_send_pkt(6, eth_stream, 8, m); run();      // ends behind the RAM
    put_msg(m, 520, 1519, 2);         a_send_pkt(6, eth_stream, 8, m); run();      // longer than RAW_MTU
    put_msg(m, 520, 10, 2);           a_send_pkt(6, eth_stream, 8, m); run();      // shorter than an ethernet header
    CHECK(n_from_amiga == 0 && pi.st_dropped == 3, "three bad WRITE_FRAME_REQ: no frame handed on");
    CHECK(a_tx_done == 3, "three WRITE_FRAME_RES anyway");
    uint32_t n0 = pi.n_rx;
    put_msg(m, M_RAM - 100, RAW_MTU, 1); a_send_pkt(6, eth_stream, 8, m); run();   // READ buffer outside of the RAM
    put_msg(m, 520, 100, 1);             a_send_pkt(6, eth_stream, 8, m); run();   // too small for a frame
    CHECK(pi.n_rx == n0, "unusable receive buffers are not remembered");
    a_send_pkt(6, eth_stream, 5, (const uint8_t *)"abcde"); run();                // DATA of a wrong length
    a_send_pkt(6, 77, 8, m); run();                                                // DATA of a foreign stream
    a_send_pkt(7, eth_stream, 0, NULL); run();                                     // EOS: ignored
    CHECK(pi.connected, "garbage does not close the connection");
  }

  // 9b. more buffers than slots: the surplus is dropped, no overflow
  {
    uint8_t m[8];
    for(int i = 0; i < 6; i++) { put_msg(m, (uint32_t)(520 + i * 100), RAW_MTU, 1); a_send_pkt(6, eth_stream, 8, m); }
    run();
    CHECK(pi.n_rx <= A314_RX_SLOTS, "receive buffer list is bounded");
  }

  printf("%s (%d Checks, %d Fehler)\n", fails ? "FAIL" : "PASS", checks, fails);
  return fails != 0;
}
