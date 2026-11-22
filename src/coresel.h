/*
  coresel.h - core selector
*/

#ifndef CORESEL_H
#define CORESEL_H

#include "sdc.h"

void coresel_init(void);              // after the xml config has been read
void coresel_autoboot(void);          // once the usb stick has been mounted
void coresel_forget(void);            // a core is loaded without the selector
sdc_dir_entry_t *coresel_list(void);  // the cores on the usb stick
const char *coresel_load(const char *core);  // only returns with an error message

#endif // CORESEL_H
