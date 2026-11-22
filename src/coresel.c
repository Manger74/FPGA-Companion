/*
  coresel.c - core selector

  Every core has a folder <core> of its own. The bitstream is stored on the
  usb stick as /cores/<core>/core.bin (or core.fs), the files of the core
  (ini files, disk and rom images) are on the sd card in /cores/<core>. The
  core can't access anything outside its folder on the sd card.

  The selection is kept in /sd/cores/coresel.ini:

    core=<core>     the folder selected last
    name=<name>     the name of that core from its xml config
    pending=1       the core has just been loaded, its name is to be learned

  The core running is told by the name in its xml config. Another one is
  e.g. running after a power cycle when the FPGA booted from its own flash.
  With "[SYSTEM] AUTOBOOT=on" in config.ini the selected core is loaded
  then once the usb stick has been mounted.
*/

#include <string.h>
#include <stdio.h>

#ifdef ESP_PLATFORM
#include <freertos/FreeRTOS.h>
#else
#include <FreeRTOS.h>
#endif

#include "coresel.h"
#include "config.h"
#include "inifile.h"
#include "mcu_hw.h"
#include "debug.h"

#define coresel_debugf(a, ...) debugf("CORESEL: " a, ##__VA_ARGS__)

#ifdef ENABLE_JTAG

#define CORESEL_USB_DIR  "/usb/cores"
#define CORESEL_SD_DIR   CARD_MOUNTPOINT "/cores"
#define CORESEL_STATE    CORESEL_SD_DIR "/coresel.ini"
#define CORESEL_NAME_MAX 64

typedef struct {
  char core[CORESEL_NAME_MAX];
  char name[CORESEL_NAME_MAX];
  char pending[2];
} coresel_state_t;

static char *autoboot = NULL;   // core to be loaded once the usb stick is there

// a core name is a plain folder name
static bool core_ok(const char *core) {
  return core[0] && !strchr(core, '/') && sdc_path_ok(core) && strlen(core) < CORESEL_NAME_MAX;
}

// path of the bitstream of core on the usb stick. The caller has the file system locked.
static bool core_file(char *path, size_t size, const char *core) {
  static const char *const files[] = { "core.bin", "core.fs" };
  FILINFO fno;
  for(unsigned i=0;i<sizeof(files)/sizeof(*files);i++) {
    snprintf(path, size, CORESEL_USB_DIR "/%s/%s", core, files[i]);
    if(f_stat(path, &fno) == FR_OK && !(fno.fattrib & AM_DIR)) return true;
  }
  return false;
}

// copy the value of a "key=value" line
static void state_value(char *dst, size_t size, const char *line, const char *key) {
  size_t kl = strlen(key);
  if(strncmp(line, key, kl) || line[kl] != '=') return;
  size_t n = strcspn(line+kl+1, "\r\n");
  if(n >= size) n = size-1;
  memcpy(dst, line+kl+1, n);
  dst[n] = '\0';
}

static bool state_read(coresel_state_t *st) {
  memset(st, 0, sizeof(*st));

  FIL fil;
  sdc_lock();
  if(f_open(&fil, CORESEL_STATE, FA_OPEN_EXISTING | FA_READ) == FR_OK) {
    char line[CORESEL_NAME_MAX+16];
    while(f_gets(line, sizeof(line), &fil)) {
      state_value(st->core, sizeof(st->core), line, "core");
      state_value(st->name, sizeof(st->name), line, "name");
      state_value(st->pending, sizeof(st->pending), line, "pending");
    }
    f_close(&fil);
  }
  sdc_unlock();

  return core_ok(st->core);
}

static bool state_write(const coresel_state_t *st) {
  char buf[sizeof(*st)+32];
  int len = snprintf(buf, sizeof(buf), "core=%s\nname=%s\npending=%s\n", st->core, st->name, st->pending);

  FIL fil;
  UINT bw = 0;
  sdc_lock();
  FRESULT r = f_mkdir(CORESEL_SD_DIR);
  bool ok = (r == FR_OK || r == FR_EXIST) && f_open(&fil, CORESEL_STATE, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK;
  if(ok) {
    ok = f_write(&fil, buf, len, &bw) == FR_OK && bw == (UINT)len;
    if(f_close(&fil) != FR_OK) ok = false;
  }
  sdc_unlock();
  return ok;
}

void coresel_init(void) {
  coresel_state_t st;
  if(!cfg || !cfg->name || !state_read(&st)) return;

  // the name as it would be stored
  char name[CORESEL_NAME_MAX];
  snprintf(name, sizeof(name), "%s", cfg->name);
  coresel_debugf("selected '%s' (%s%s), running '%s'", st.core, st.name, st.pending[0]=='1'?", pending":"", name);

  if(st.pending[0] == '1') {
    // the selected core has just been loaded, learn its name
    strcpy(st.name, name);
    st.pending[0] = '\0';
    if(!state_write(&st)) coresel_debugf("unable to write %s", CORESEL_STATE);
  } else if(strcmp(st.name, name)) {
    // another core is running, e.g. the one from flash after a power cycle
    if(inifile_config_get_int("system", "autoboot", 0)) {
      autoboot = pvPortMalloc(strlen(st.core)+1);
      strcpy(autoboot, st.core);
    }
    return;
  }

  // restrict the core to its folder if there is one
  char dir[sizeof(CORESEL_SD_DIR)+CORESEL_NAME_MAX+1];
  snprintf(dir, sizeof(dir), CORESEL_SD_DIR "/%s", st.core);
  FILINFO fno;
  sdc_lock();
  bool is_dir = f_stat(dir, &fno) == FR_OK && (fno.fattrib & AM_DIR);
  sdc_unlock();
  if(is_dir) sdc_set_root(dir);
}

void coresel_autoboot(void) {
  if(!autoboot) return;

  // try only once
  char *core = autoboot;
  autoboot = NULL;
  coresel_debugf("autoboot '%s'", core);
  const char *err = coresel_load(core);
  if(err) coresel_debugf("autoboot failed: %s", err);
  vPortFree(core);
}

void coresel_forget(void) {
  sdc_lock();
  f_unlink(CORESEL_STATE);
  sdc_unlock();
}

sdc_dir_entry_t *coresel_list(void) {
  static sdc_dir_entry_t *list = NULL;

  // free the previous list
  while(list) {
    sdc_dir_entry_t *next = list->next;
    vPortFree(list->name);
    vPortFree(list);
    list = next;
  }

#ifdef ESP_PLATFORM
  FF_DIR dir;
#else
  DIR dir;
#endif
  FILINFO fno;
  char path[sizeof(CORESEL_USB_DIR)+CORESEL_NAME_MAX+10];

  sdc_lock();
  if(f_opendir(&dir, CORESEL_USB_DIR) == FR_OK) {
    while(f_readdir(&dir, &fno) == FR_OK && fno.fname[0]) {
      if(!(fno.fattrib & AM_DIR) || (fno.fattrib & (AM_HID|AM_SYS)) || fno.fname[0] == '.') continue;
      if(!core_ok(fno.fname) || !core_file(path, sizeof(path), fno.fname)) continue;

      sdc_dir_entry_t *entry = pvPortMalloc(sizeof(sdc_dir_entry_t));
      entry->name = pvPortMalloc(strlen(fno.fname)+1);
      strcpy(entry->name, fno.fname);
      entry->len = 0;
      entry->is_dir = 1;

      // sorted insert
      sdc_dir_entry_t **pp = &list;
      while(*pp && strcasecmp((*pp)->name, entry->name) < 0) pp = &(*pp)->next;
      entry->next = *pp;
      *pp = entry;
    }
    f_closedir(&dir);
  }
  sdc_unlock();

  return list;
}

const char *coresel_load(const char *core) {
  char path[sizeof(CORESEL_USB_DIR)+CORESEL_NAME_MAX+10];

  if(!core_ok(core)) return "Invalid core name";

  sdc_lock();
  bool found = core_file(path, sizeof(path), core);
  sdc_unlock();
  if(!found) return "No core.bin or\ncore.fs found";

  // The selection must be stored, else the state would refer to
  // the previous core once the new one is running
  coresel_state_t st = { .pending = "1" };
  strcpy(st.core, core);
  if(!state_write(&st)) return "Unable to write\n" CORESEL_STATE;

  coresel_debugf("loading %s", path);
  mcu_hw_upload_core(path);   // doesn't return on success on the rp2040
  return NULL;
}

#else // ENABLE_JTAG

void coresel_init(void) { }
void coresel_autoboot(void) { }
void coresel_forget(void) { }
sdc_dir_entry_t *coresel_list(void) { return NULL; }
const char *coresel_load(__attribute__((unused)) const char *core) { return "No JTAG support"; }

#endif // ENABLE_JTAG
