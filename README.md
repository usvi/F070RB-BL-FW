F070RB-BL-FW

# NUCLEO F070RB Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-F070RB evaluation board to boot position independent code (pic) firmware image. It is actual jump boot, not some blob function call thing.

Quick statuses

Bootloader: **Works. Automaticly jumps to first non-empty position in firmware area.**

Firmware_anywhere: **Works with all addresses sufficiently aligned (512/0x200 bytes) when booted by bootloader. Works also standalone.**

(Alignment is needed only for OTHER platforms than Cortex-M0, but using alignment so we can use the same functons.)

One can try for example these addresses, they should all work:

0x8005000 

0x8005200

0x8005400

0x8005600

0x8005800

0x8005A00

0x8005C00

0x8005E00

0x8006000

0x8006200

0x8006400

0x8006600

0x8006800

0x8006A00


