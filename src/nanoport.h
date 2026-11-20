/*
  nanoport.h - network bridge for the NanoMig nanoport network card
*/

#ifndef NANOPORT_H
#define NANOPORT_H

void nanoport_init(void);
void nanoport_handle_event(void);   // core interrupt 5

#endif // NANOPORT_H
