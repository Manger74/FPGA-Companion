/*
  inifile.h
 */

#ifndef INIFILE_H
#define INIFILE_H

#define INIFILE_OPTION_HOTKEY   0   // HID key code
#define INIFILE_OPTION_LED      1   // 0 = blink, 1 = on, 0 = off

int inifile_read(char *);
void inifile_write(const char *);
int inifile_option_get(int id);
const char *inifile_get_current(void);  // returns last loaded .ini filename

#endif // INIFILE_H
