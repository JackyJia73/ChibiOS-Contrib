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
 * @file    USBD/gd32_usbd.h
 * @brief   GD32 USBD registers layout header.
 *
 * @addtogroup USB
 * @{
 */

#ifndef gd32_usb_H
#define gd32_usb_H

/**
 * @brief   Number of the available endpoints.
 * @details This value does not include the endpoint 0 which is always present.
 */
#define USB_ENDPOINTS_NUMBER            7

/**
 * @brief   Width of USB packet memory accesses.
 */
typedef uint32_t gd32_usb_pma_t;

/**
 * @brief   USB registers block.
 */
typedef struct {
  /**
   * @brief   endpoint registers.
   */
  volatile uint32_t             EPxCS[USB_ENDPOINTS_NUMBER + 1];
  /*
   * @brief   reserved space.
   */
  volatile uint32_t             Reserved20[8];
  /*
   * @brief   control register.
   */
  volatile uint32_t             CTL;
  /*
   * @brief   interrupt flag register.
   */
  volatile uint32_t             INTF;
  /*
   * @brief   status register.
   */
  volatile uint32_t             STAT;
  /*
   * @brief   device address register.
   */
  volatile uint32_t             DADDR;
  /*
   * @brief   buffer address register.
   */
  volatile uint32_t             BADDR;
  /*
   * @brief   LPM control and status register.
   */
  volatile uint32_t             LPMCS;

#if GD32_USB_HAS_DPCR
  /*
   * @brief   DP pull-up control Register
   */
  volatile uint32_t             DPC;
#endif
} gd32_usb_regs;

/**
 * @brief   USB descriptor registers block.
 */
typedef struct {
  /**
   * @brief   TX buffer offset register.
   */
  volatile gd32_usb_pma_t      tx_addr;
  /**
   * @brief   TX counter register 0.
   */
  volatile gd32_usb_pma_t      tx_count;
  /**
   * @brief   RX buffer offset register.
   */
  volatile gd32_usb_pma_t      rx_addr;
  /**
   * @brief   RX counter register 0.
   */
  volatile gd32_usb_pma_t      rx_count;
} gd32_usb_ep_ram;

/**
 * @name    Register aliases
 * @{
 */
#define RXCOUNT1                tx_count
#define TXCOUNT1                rx_count
#define RXADDR1                 tx_addr
#define TXADDR1                 rx_addr
/** @} */

/**
 * @brief USB registers block numeric address.
 */
#define GD32_USB_BASE          USBD_BASE

/**
 * @brief USB RAM numeric address.
 */
#define GD32_USBDRAM_BASE       USBD_RAM_BASE

/**
 * @brief Pointer to the USB registers block.
 */
#define GD32_USB               ((gd32_usb_regs *)GD32_USB_BASE)

/**
 * @brief   Pointer to the USB RAM.
 */
#define GD32_USBRAM            ((gd32_usb_pma_t *)GD32_USBDRAM_BASE)

/**
 * @brief   Mask of all the toggling bits in the EPxCS register.
 */
#define EPxCS_TOGGLE_MASK         (EPxCS_TX_STA_MASK | EPxCS_TX_DTG |           \
                                 EPxCS_RX_STA_MASK | EPxCS_RX_DTG |           \
                                 EPxCS_SETUP)

#define EPxCS_AR_MASK            0x000F
#define EPxCS_TX_STA_MASK        0x0030
#define EPxCS_TX_STA_DIS         0x0000
#define EPxCS_TX_STA_STALL       0x0010
#define EPxCS_TX_STA_NAK         0x0020
#define EPxCS_TX_STA_VALID       0x0030
#define EPxCS_TX_DTG             0x0040
#define EPxCS_SWBUF_RX           EPxCS_TX_DTG
#define EPxCS_TX_ST              0x0080
#define EPxCS_KCTL               0x0100
#define EPxCS_EP_DBL_BUF         EPxCS_KCTL
#define EPxCS_EP_STATUS_OUT      EPxCS_KCTL
#define EPxCS_CTL_MASK           0x0600
#define EPxCS_CTL_BULK           0x0000
#define EPxCS_CTL_CONTROL        0x0200
#define EPxCS_CTL_ISO            0x0400
#define EPxCS_CTL_INTERRUPT      0x0600
#define EPxCS_SETUP              0x0800
#define EPxCS_RX_STA_MASK        0x3000
#define EPxCS_RX_STA_DIS         0x0000
#define EPxCS_RX_STA_STALL       0x1000
#define EPxCS_RX_STA_NAK         0x2000
#define EPxCS_RX_STA_VALID       0x3000
#define EPxCS_RX_DTG             0x4000
#define EPxCS_SWBUF_TX           EPxCS_RX_DTG
#define EPxCS_RX_ST              0x8000

#define CTL_SETRST               0x0001
#define CTL_CLOSE                0x0002
#define CTL_LOWM                 0x0004
#define CTL_SETSPS               0x0008
#define CTL_RSREQ                0x0010
#define CTL_ESOFIE               0x0100
#define CTL_SOFIE                0x0200
#define CTL_RSTIE                0x0400
#define CTL_SPSIE                0x0800
#define CTL_WKUPIE               0x1000
#define CTL_ERRIE                0x2000
#define CTL_PMOUIE               0x4000
#define CTL_STIE                 0x8000

#define INTF_EPNUM               0x000F
#define INTF_DIR                 0x0010
#define INTF_ESOFIF              0x0100
#define INTF_SOFIF               0x0200
#define INTF_RSTIF               0x0400
#define INTF_SPSIF               0x0800
#define INTF_WKUPIF              0x1000
#define INTF_ERRIF               0x2000
#define INTF_PMOUIF              0x4000
#define INTF_STIF                0x8000

#define STAT_FCNT                0x07FF
#define STAT_SOFLN               0x1800
#define STAT_LOCK                0x2000
#define STAT_RXDM                0x4000
#define STAT_RXDP                0x8000

#define DADDR_USBADDR            0x007F
#define DADDR_USBEN              0x0080

#define DPC_DPUEN                0x8000

#define RXCOUNT_COUNT_MASK       0x03FF
#define TXCOUNT_COUNT_MASK       0x03FF

#define EPxCS_CTR_MASK            (EPxCS_TX_ST | EPxCS_RX_ST)

#define EPxCS_SET_STAT_RX(ep, epr)                                            \
  GD32_USB->EPxCS[ep] = ((GD32_USB->EPxCS[ep] &                               \
                        ~(EPxCS_TOGGLE_MASK & ~EPxCS_RX_STA_MASK)) ^          \
                       (epr)) | EPxCS_CTR_MASK

#define EPxCS_SET_STAT_TX(ep, epr)                                            \
  GD32_USB->EPxCS[ep] = ((GD32_USB->EPxCS[ep] &                               \
                        ~(EPxCS_TOGGLE_MASK & ~EPxCS_TX_STA_MASK)) ^          \
                       (epr)) | EPxCS_CTR_MASK

#define EPxCS_CLEAR_CTR_RX(ep)                                                    \
  GD32_USB->EPxCS[ep] = (GD32_USB->EPxCS[ep] & ~EPxCS_RX_ST & ~EPxCS_TOGGLE_MASK) \
                       | EPxCS_TX_ST

#define EPxCS_CLEAR_CTR_TX(ep)                                                    \
  GD32_USB->EPxCS[ep] = (GD32_USB->EPxCS[ep] & ~EPxCS_TX_ST & ~EPxCS_TOGGLE_MASK) \
                       | EPxCS_RX_ST

/**
 * @brief   Returns an endpoint descriptor pointer.
 */
#define USB_GET_EP_RAM(ep)                                              \
  ((gd32_usb_ep_ram *)((uint32_t)GD32_USBDRAM_BASE +                    \
                              (uint32_t)GD32_USB->BADDR +               \
                              (uint32_t)(ep) *                          \
                              sizeof(gd32_usb_ep_ram)))

/**
 * @brief   Converts from a PMA address to a physical address.
 */
#define USB_ADDR2PTR(addr)                                                 \
  ((gd32_usb_pma_t *)((addr) *                                             \
                       (sizeof(gd32_usb_pma_t) / 2) +                      \
                       GD32_USBDRAM_BASE))

#endif /* gd32_usb_H */

/** @} */
