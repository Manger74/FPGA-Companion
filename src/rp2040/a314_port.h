/*
  a314_port.h

  A314 (clockport variant, Amiga a314.device + a314eth.device) as network interface, see a314_port.c.
  The core implements the shared memory and the interrupt bits (a314_cp.v, SPI target 6), the
  Companion plays the Pi: the "ethernet" service of a314d, frames become an lwIP interface whose
  peer (the Amiga) is forwarded to the lan like the PPP and PLIP peers.

  config.ini:  [A314]  ENABLE=1   switch it on (default 0)
                       IP=...     address of the Companion on this link, default 192.168.253.1
                       PEER=...   address of the Amiga, default 192.168.253.2
*/

#ifndef A314_PORT_H
#define A314_PORT_H

// read [A314] from config.ini and start the task if enabled (call after config.ini has been read)
void a314_port_init(void);

// interrupt of the core (the Amiga called set_pi_irq). May be called from any task
void a314_handle_event(void);

#endif // A314_PORT_H
