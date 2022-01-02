# NUCLEO F070RB Bootloader and PIC Firmware Image

This repository is for implementing how to get STM32 NUCLEO-F070RB evaluation board to boot position independent firmware image code from an *arbitrary* location using custom bootloader. So to boot the same firmware blob from whatever location. (Of course the bootloader must know the address.) And we are using real jumps, not calling the blob as library.

Quick statuses

Bootloader: **Works. Can jump to application firmware and to even itself :D**

Firmware_anywhere: **Works with all addresses (aligned to 4 bytes)!**

