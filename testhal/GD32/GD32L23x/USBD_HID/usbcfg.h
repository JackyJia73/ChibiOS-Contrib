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

/**
 * @file    usbcfg.h
 * @brief   USB driver config header.
 *
 * @addtogroup USB
 * @{
 */

#ifndef USBCFG_H
#define USBCFG_H

#include "hal_usb_lld.h"

/* HID device report descriptor length */
#define HID_REPORT_DESC_LEN          0x2EU

/* HID device report length */
#define HID_REPORT_LEN               0x08U

#define USB_DESC_STRING(...)                                                                       \
  {                                                                                                \
    USB_DESC_BYTE((sizeof((int[]){__VA_ARGS__}) / sizeof(int)) + 2),                               \
        USB_DESC_BYTE(USB_DESCRIPTOR_STRING), __VA_ARGS__                                          \
  }

#ifdef __cplusplus
extern "C" {
#endif
  void hidGenReport(uint8_t c_flag);
#ifdef __cplusplus
}
#endif

extern const USBConfig usbcfg;
extern const USBHIDConfig usbhidcfg;
extern USBHIDDriver UHD1;
extern uint8_t report[];

#endif  /* USBCFG_H */

/** @} */
