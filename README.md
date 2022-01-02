F070RB-BL-FW

# NUCLEO F070RB Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-F070RB evaluation board to boot position independent code (pic) firmware image. It is actual jump boot, not some blob function call thing.

Quick statuses

Bootloader: **Works. Automaticly jumps to first non-empty position in firmware area.**

Firmware_anywhere: **Works with all addresses sufficiently aligned when booted by bootloader. Works also standalone.**

One can try for example these addresses, they should all work: 0x8005000, 0x8010000, 0x8015000, 0x801A800, 0x801B800
