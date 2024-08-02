*****************************************************************************
** ChibiOS/HAL - USB-HID driver demo for GD32.                             **
*****************************************************************************

** TARGET **

The demo runs on an GD32L235R-EVAL board with a GD32L235RBT6 MCU.

** The Demo **

The application demonstrates the use of the GD32L23x USB driver. A successful run of the test
should begin with the on-board LED blinking slowly, then faster when the USB driver initializes.
The host should recognize the board as a USB HID keyboard, When the user presses the Tamper key, 
the HID keyboard device begins to print letters, and when the key is released, the output stops.

** Board Setup **

- None

** Build Procedure **

The demo has been tested using the free Codesourcery GCC-based toolchain
and YAGARTO.
Just modify the TRGT line in the makefile in order to use different GCC ports.

** Notes **

Some files used by the demo are not part of ChibiOS/RT but are copyright of
GigaDevice Semiconductor and are licensed under a different license.
Also note that not all the files present in the GD library are distributed
with ChibiOS/RT, you can find the whole library on the GD32 MCU web site:

                       https://www.gd32mcu.com
