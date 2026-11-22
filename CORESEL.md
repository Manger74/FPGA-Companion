# Core selector

On boards where the MCU can load the FPGA via JTAG (e.g. MSP20K and
MSP2_20K), the system menu offers **Select Core**. Every core has a
folder of its own:

```
USB stick                     SD card
/cores/Amiga/core.bin         /cores/Amiga/amiga.ini, adf/, hdf/, ...
/cores/AtariST/core.fs        /cores/AtariST/atarist.ini, ...
/cores/C64/core.bin           /cores/C64/...
```

- The bitstream is `core.bin` or `core.fs` in the core's folder on the
  USB stick. It can't be read from the SD card on these boards, since the
  SD card is accessed through the core that is being replaced.
- A core loaded through the selector only sees its folder on the SD card:
  file selectors, ini files and config selector all start there and can't
  go above it. Image paths in ini files outside the folder are ignored.
- `config.ini` stays in the root of the SD card, the FTP server still
  sees the whole card.

The selection is stored in `/cores/coresel.ini` on the SD card. The
companion tells which core is running by the name in the core's xml
config. If another core runs, e.g. the one from the FPGA's flash after a
power cycle, the core gets the whole SD card as before. With

```
[SYSTEM]
AUTOBOOT=on
```

in `config.ini` the selected core is loaded instead, once the USB stick
has been mounted. This also replaces a core that has been loaded by other
means, e.g. via USB-JTAG from a PC. Loading a core via **Load Core from
USB** (or SD) clears the selection.
