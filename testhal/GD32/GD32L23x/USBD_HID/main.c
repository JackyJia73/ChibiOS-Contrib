/*
    Copyright (C) 2024 Albert

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "hal.h"
#include "usbcfg.h"

#define USB_TX_TIMEOUT              100

/*
 * LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    systime_t time = USBD.state == USB_ACTIVE ? 200 : 800;
    palClearPad(PORT_LED1, PIN_LED1);
    chThdSleepMilliseconds(time);
    palSetPad(PORT_LED1, PIN_LED1);
    chThdSleepMilliseconds(time);
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Initializes a USB HID driver.
   */
  hidObjectInit(&UHD1);
  hidStart(&UHD1, &usbhidcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(usbhidcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(usbhidcfg.usbp, &usbcfg);
  chThdSleepMilliseconds(1000);
  usbConnectBus(usbhidcfg.usbp);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO, Thread1, NULL);

  while (true) {
    if (PAL_LOW == palReadLine(LINE_TAMPER)) {
      if (usbhidcfg.usbp->state == USB_ACTIVE) {
        hidGenReport(0);
        hidWriteReportt(&UHD1, &report[0], HID_REPORT_LEN, USB_TX_TIMEOUT);

        hidGenReport(1);
        hidWriteReportt(&UHD1, &report[0], HID_REPORT_LEN, USB_TX_TIMEOUT);
      }
    }

    chThdSleepMilliseconds(100);
  }
}
