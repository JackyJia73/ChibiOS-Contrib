/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio
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
 * @file    USBD/hal_usb_lld.c
 * @brief   GD32 USB subsystem low level driver source.
 *
 * @addtogroup USB
 * @{
 */

#include <string.h>
#include "hal.h"

#if HAL_USE_USB || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define BTABLE_ADDR     0x0000

#define EPxCS_CTL_IS_ISO(bits) ((bits & EPxCS_CTL_MASK) == EPxCS_CTL_ISO)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USB1 driver identifier.*/
#if GD32_USB_USE_USBD || defined(__DOXYGEN__)
USBDriver USBD;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   EP0 state.
 * @note    It is an union because IN and OUT endpoints are never used at the
 *          same time for EP0.
 */
static union {
  /**
   * @brief   IN EP0 state.
   */
  USBInEndpointState in;
  /**
   * @brief   OUT EP0 state.
   */
  USBOutEndpointState out;
} ep0_state;

/**
 * @brief   Buffer for the EP0 setup packets.
 */
static uint8_t ep0setup_buffer[8];

/**
 * @brief   EP0 initialization structure.
 */
static const USBEndpointConfig ep0config = {
  .ep_mode          = USB_EP_MODE_TYPE_CTRL,
  .setup_cb         = _usb_ep0setup,
  .in_cb            = _usb_ep0in,
  .out_cb           = _usb_ep0out,
  .in_maxsize       = 0x40U,
  .out_maxsize      = 0x40U,
  .in_state         = &ep0_state.in,
  .out_state        = &ep0_state.out,
  .ep_buffers       = 1U,
  .setup_buf        = ep0setup_buffer
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   Resets the packet memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 */
static void usb_pm_reset(USBDriver *usbp) {

  /* The first 64 bytes are reserved for the descriptors table. The effective
     available RAM for endpoint buffers is just 448 bytes.*/
  usbp->pmnext = 64U;
}

/**
 * @brief   Resets the packet memory allocator.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] size      size of the packet buffer to allocate
 * @return              The packet buffer address.
 */
static uint32_t usb_pm_alloc(USBDriver *usbp, size_t size) {
  uint32_t next;

  next = usbp->pmnext;
  usbp->pmnext += (size + 1U) & ~1U;

  osalDbgAssert(usbp->pmnext <= GD32_USB_PMA_SIZE, "PMA overflow");

  return next;
}

/**
 * @brief   Reads from a dedicated packet buffer.
 *
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 * @return              The size of the receivee packet.
 *
 * @notapi
 */
static size_t usb_packet_read_to_buffer(usbep_t ep, uint8_t *buf) {
  size_t i, n;
  gd32_usb_ep_ram *udp = USB_GET_EP_RAM(ep);
  gd32_usb_pma_t *pmap = USB_ADDR2PTR(udp->rx_addr);

#if 0
  __ALIGNED(4) uint8_t data[512];
  uint8_t* temp = data;

#if GD32_USB_USE_ISOCHRONOUS
  uint32_t epr = GD32_USB->EPxCS[ep];

  /* Double buffering is always enabled for isochronous endpoints, and
     although we overlap the two buffers for simplicity, we still need
     to read from the right counter. The DTOG_RX bit indicates the buffer
     that is currently in use by the USB peripheral, that is, the buffer
     in which the next received packet will be stored, so we need to
     read the counter of the OTHER buffer, which is where the last
     received packet was stored.*/
  if (EPxCS_CTL_IS_ISO(epr) && ((epr & EPxCS_RX_DTG) != 0U))
    n = (size_t)udp->RXCOUNT1 & RXCOUNT_COUNT_MASK;
  else
    n = (size_t)udp->rx_count & RXCOUNT_COUNT_MASK;
#else
  n = (size_t)udp->rx_count & RXCOUNT_COUNT_MASK;
#endif

  i = n;

#if GD32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w = *(pmap + 0);
    *(temp + 0) = (uint8_t)w;
    *(temp + 1) = (uint8_t)(w >> 8);
    w = *(pmap + 1);
    *(temp + 2) = (uint8_t)w;
    *(temp + 3) = (uint8_t)(w >> 8);
    w = *(pmap + 2);
    *(temp + 4) = (uint8_t)w;
    *(temp + 5) = (uint8_t)(w >> 8);
    w = *(pmap + 3);
    *(temp + 6) = (uint8_t)w;
    *(temp + 7) = (uint8_t)(w >> 8);
    w = *(pmap + 4);
    *(temp + 8) = (uint8_t)w;
    *(temp + 9) = (uint8_t)(w >> 8);
    w = *(pmap + 5);
    *(temp + 10) = (uint8_t)w;
    *(temp + 11) = (uint8_t)(w >> 8);
    w = *(pmap + 6);
    *(temp + 12) = (uint8_t)w;
    *(temp + 13) = (uint8_t)(w >> 8);
    w = *(pmap + 7);
    *(temp + 14) = (uint8_t)w;
    *(temp + 15) = (uint8_t)(w >> 8);

    i -= 16U;
    temp += 16U;
    pmap += 8U;
  }
#endif /* GD32_USB_USE_FAST_COPY */

  while (i >= 2U) {
    uint32_t w = *pmap++;
    *temp++ = (uint8_t)w;
    *temp++ = (uint8_t)(w >> 8);
    i -= 2U;
  }

  if (i >= 1U) {
    *temp = (uint8_t)*pmap;
  }

  memcpy(buf, data, n);

  return n;

#else

#if GD32_USB_USE_ISOCHRONOUS
  uint32_t epr = GD32_USB->EPxCS[ep];

  /* Double buffering is always enabled for isochronous endpoints, and
     although we overlap the two buffers for simplicity, we still need
     to read from the right counter. The DTOG_RX bit indicates the buffer
     that is currently in use by the USB peripheral, that is, the buffer
     in which the next received packet will be stored, so we need to
     read the counter of the OTHER buffer, which is where the last
     received packet was stored.*/
  if (EPxCS_CTL_IS_ISO(epr) && ((epr & EPxCS_RX_DTG) != 0U))
    n = (size_t)udp->RXCOUNT1 & RXCOUNT_COUNT_MASK;
  else
    n = (size_t)udp->rx_count & RXCOUNT_COUNT_MASK;
#else
  n = (size_t)udp->rx_count & RXCOUNT_COUNT_MASK;
#endif

  i = n;

#if GD32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w = *(pmap + 0);
    *(buf + 0) = (uint8_t)w;
    *(buf + 1) = (uint8_t)(w >> 8);
    w = *(pmap + 1);
    *(buf + 2) = (uint8_t)w;
    *(buf + 3) = (uint8_t)(w >> 8);
    w = *(pmap + 2);
    *(buf + 4) = (uint8_t)w;
    *(buf + 5) = (uint8_t)(w >> 8);
    w = *(pmap + 3);
    *(buf + 6) = (uint8_t)w;
    *(buf + 7) = (uint8_t)(w >> 8);
    w = *(pmap + 4);
    *(buf + 8) = (uint8_t)w;
    *(buf + 9) = (uint8_t)(w >> 8);
    w = *(pmap + 5);
    *(buf + 10) = (uint8_t)w;
    *(buf + 11) = (uint8_t)(w >> 8);
    w = *(pmap + 6);
    *(buf + 12) = (uint8_t)w;
    *(buf + 13) = (uint8_t)(w >> 8);
    w = *(pmap + 7);
    *(buf + 14) = (uint8_t)w;
    *(buf + 15) = (uint8_t)(w >> 8);

    i -= 16U;
    buf += 16U;
    pmap += 8U;
  }
#endif /* GD32_USB_USE_FAST_COPY */

  while (i >= 2U) {
    uint32_t w = *pmap++;
    *buf++ = (uint8_t)w;
    *buf++ = (uint8_t)(w >> 8);
    i -= 2U;
  }

  if (i >= 1U) {
    *buf = (uint8_t)*pmap;
  }

  return n;

#endif
}

/**
 * @brief   Writes to a dedicated packet buffer.
 *
 * @param[in] ep        endpoint number
 * @param[in] buf       buffer where to fetch the packet data
 * @param[in] n         maximum number of bytes to copy. This value must
 *                      not exceed the maximum packet size for this endpoint.
 *
 * @notapi
 */
static void usb_packet_write_from_buffer(usbep_t ep,
                                         const uint8_t *buf,
                                         size_t n) {
  gd32_usb_ep_ram *udp = USB_GET_EP_RAM(ep);
  gd32_usb_pma_t *pmap = USB_ADDR2PTR(udp->tx_addr);
  int i = (int)n;

#if 0
  __ALIGNED(4) uint8_t data[512];
  uint8_t* temp = data;
  memcpy(temp, buf, n);

#if GD32_USB_USE_ISOCHRONOUS
  uint32_t epr = GD32_USB->EPxCS[ep];

  /* Double buffering is always enabled for isochronous endpoints, and
     although we overlap the two buffers for simplicity, we still need
     to write to the right counter. The DTOG_TX bit indicates the buffer
     that is currently in use by the USB peripheral, that is, the buffer
     from which the next packet will be sent, so we need to write the
     counter of that buffer.*/
  if (EPxCS_CTL_IS_ISO(epr) && (epr & EPxCS_TX_DTG))
    udp->TXCOUNT1 = (gd32_usb_pma_t)n;
  else
    udp->tx_count = (gd32_usb_pma_t)n;
#else
  udp->tx_count = (gd32_usb_pma_t)n;
#endif

#if GD32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w  = *(temp + 0);
    w |= *(temp + 1) << 8;
    *(pmap + 0) = (gd32_usb_pma_t)w;
    w  = *(temp + 2);
    w |= *(temp + 3) << 8;
    *(pmap + 1) = (gd32_usb_pma_t)w;
    w  = *(temp + 4);
    w |= *(temp + 5) << 8;
    *(pmap + 2) = (gd32_usb_pma_t)w;
    w  = *(temp + 6);
    w |= *(temp + 7) << 8;
    *(pmap + 3) = (gd32_usb_pma_t)w;
    w  = *(temp + 8);
    w |= *(temp + 9) << 8;
    *(pmap + 4) = (gd32_usb_pma_t)w;
    w  = *(temp + 10);
    w |= *(temp + 11) << 8;
    *(pmap + 5) = (gd32_usb_pma_t)w;
    w  = *(temp + 12);
    w |= *(temp + 13) << 8;
    *(pmap + 6) = (gd32_usb_pma_t)w;
    w  = *(temp + 14);
    w |= *(temp + 15) << 8;
    *(pmap + 7) = (gd32_usb_pma_t)w;

    i -= 16;
    temp += 16U;
    pmap += 8U;
  }
#endif /* GD32_USB_USE_FAST_COPY */

  while (i > 0) {
    uint32_t w;

    w  = *temp++;
    w |= *temp++ << 8;
    *pmap++ = (gd32_usb_pma_t)w;
    i -= 2;
  }

#else

#if GD32_USB_USE_ISOCHRONOUS
  uint32_t epr = GD32_USB->EPxCS[ep];

  /* Double buffering is always enabled for isochronous endpoints, and
     although we overlap the two buffers for simplicity, we still need
     to write to the right counter. The DTOG_TX bit indicates the buffer
     that is currently in use by the USB peripheral, that is, the buffer
     from which the next packet will be sent, so we need to write the
     counter of that buffer.*/
  if (EPxCS_CTL_IS_ISO(epr) && (epr & EPxCS_TX_DTG))
    udp->TXCOUNT1 = (gd32_usb_pma_t)n;
  else
    udp->tx_count = (gd32_usb_pma_t)n;
#else
  udp->tx_count = (gd32_usb_pma_t)n;
#endif

uint32_t temp = (uint32_t)(buf);
if(0 != (temp % 2)){
  i=n;
}

#if GD32_USB_USE_FAST_COPY
  while (i >= 16) {
    uint32_t w;

    w  = *(buf + 0);
    w |= *(buf + 1) << 8;
    *(pmap + 0) = (gd32_usb_pma_t)w;
    w  = *(buf + 2);
    w |= *(buf + 3) << 8;
    *(pmap + 1) = (gd32_usb_pma_t)w;
    w  = *(buf + 4);
    w |= *(buf + 5) << 8;
    *(pmap + 2) = (gd32_usb_pma_t)w;
    w  = *(buf + 6);
    w |= *(buf + 7) << 8;
    *(pmap + 3) = (gd32_usb_pma_t)w;
    w  = *(buf + 8);
    w |= *(buf + 9) << 8;
    *(pmap + 4) = (gd32_usb_pma_t)w;
    w  = *(buf + 10);
    w |= *(buf + 11) << 8;
    *(pmap + 5) = (gd32_usb_pma_t)w;
    w  = *(buf + 12);
    w |= *(buf + 13) << 8;
    *(pmap + 6) = (gd32_usb_pma_t)w;
    w  = *(buf + 14);
    w |= *(buf + 15) << 8;
    *(pmap + 7) = (gd32_usb_pma_t)w;

    i -= 16;
    buf += 16U;
    pmap += 8U;
  }
#endif /* GD32_USB_USE_FAST_COPY */

  while (i > 0) {
    uint32_t w;

    w  = *buf++;
    w |= *buf++ << 8;
    *pmap++ = (gd32_usb_pma_t)w;
    i -= 2;
  }

#endif
}

/**
 * @brief   Common ISR code, serves the EP-related interrupts.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] istr      INTF register value to consider
 *
 * @notapi
 */
static void usb_serve_endpoints(USBDriver *usbp, uint32_t istr) {
  size_t n;
  uint32_t ep = istr & INTF_EPNUM;
  uint32_t epr = GD32_USB->EPxCS[ep];
  const USBEndpointConfig *epcp = usbp->epc[ep];

  if ((istr & INTF_DIR) == 0U) {
    /* IN endpoint, transmission.*/
    USBInEndpointState *isp = epcp->in_state;

    EPxCS_CLEAR_CTR_TX(ep);

    isp->txcnt += isp->txlast;
    n = isp->txsize - isp->txcnt;
    if (n > 0U) {
      /* Transfer not completed, there are more packets to send.*/
      if (n > epcp->in_maxsize)
        n = epcp->in_maxsize;

      /* Writes the packet from the defined buffer.*/
      isp->txbuf += isp->txlast;
      isp->txlast = n;

      usb_packet_write_from_buffer(ep, isp->txbuf, n);

      /* Starting IN operation.*/
      EPxCS_SET_STAT_TX(ep, EPxCS_TX_STA_VALID);
    }
    else {
      /* Transfer completed, invokes the callback.*/
      _usb_isr_invoke_in_cb(usbp, ep);
    }
  }
  else {
    /* OUT endpoint, receive.*/

    EPxCS_CLEAR_CTR_RX(ep);

    if (epr & EPxCS_SETUP) {
      /* Setup packets handling, setup packets are handled using a
         specific callback.*/
      _usb_isr_invoke_setup_cb(usbp, ep);
    }
    else {
      USBOutEndpointState *osp = epcp->out_state;

      /* Reads the packet into the defined buffer.*/
      n = usb_packet_read_to_buffer(ep, osp->rxbuf);
      osp->rxbuf += n;

      /* Transaction data updated.*/
      osp->rxcnt  += n;
      osp->rxsize -= n;
      osp->rxpkts -= 1U;

      /* The transaction is completed if the specified number of packets
         has been received or the current packet is a short packet.*/
      if ((n < epcp->out_maxsize) || (osp->rxpkts == 0)) {
        /* Transfer complete, invokes the callback.*/
        _usb_isr_invoke_out_cb(usbp, ep);
      }
      else {
        /* Transfer not complete, there are more packets to receive.*/
        EPxCS_SET_STAT_RX(ep, EPxCS_RX_STA_VALID);
      }
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if GD32_USB_USE_USBD || defined(__DOXYGEN__)
#if GD32_USBD_HP_CAN_TX_NUMBER != GD32_USBD_LP_CAN_RX0_NUMBER
#if GD32_USB_USE_ISOCHRONOUS
/**
 * @brief   USB high priority interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USBD_HP_CAN_TX_HANDLER) {
  uint32_t istr;
  USBDriver *usbp = &USBD;

  OSAL_IRQ_PROLOGUE();

  /* Endpoint events handling.*/
  istr = GD32_USB->INTF;
  while ((istr & INTF_STIF) != 0U) {
    usb_serve_endpoints(usbp, istr);
    istr = GD32_USB->INTF;
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_USB_USE_ISOCHRONOUS */
#endif /* GD32_USBD_LP_CAN_RX0_NUMBER != GD32_USBD_HP_CAN_TX_NUMBER */

/**
 * @brief   USB low priority interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(GD32_USBD_LP_CAN_RX0_HANDLER) {
  uint32_t istr;
  USBDriver *usbp = &USBD;

  OSAL_IRQ_PROLOGUE();

  /* Reading interrupt sources and atomically clearing them.*/
  istr = GD32_USB->INTF;
  GD32_USB->INTF = ~istr;

  /* USB bus reset condition handling.*/
  if ((istr & INTF_RSTIF) != 0U) {
    _usb_reset(usbp);
  }

  /* USB bus SUSPEND condition handling.*/
  if ((istr & INTF_SPSIF) != 0U) {
    GD32_USB->CTL |= CTL_SETSPS;
#if GD32_USB_LOW_POWER_ON_SUSPEND
    GD32_USB->CTL |= CTL_LOWM;
#endif
    _usb_suspend(usbp);
  }

  /* USB bus WAKEUP condition handling.*/
  if ((istr & INTF_WKUPIF) != 0U) {
    uint32_t fnr = GD32_USB->STAT;
    if ((fnr & STAT_RXDP) == 0U) {
      GD32_USB->CTL &= ~CTL_SETSPS;
      _usb_wakeup(usbp);
    }
#if GD32_USB_LOW_POWER_ON_SUSPEND
    else {
      /* Just noise, going back in SUSPEND mode, reference manual 22.4.5,
         table 169.*/
      GD32_USB->CTL |= CTL_LOWM;
    }
#endif
  }

  /* SOF handling.*/
  if ((istr & INTF_SOFIF) != 0U) {
    _usb_isr_invoke_sof_cb(usbp);
  }

  /* ERR handling.*/
  if ((istr & INTF_ERRIF) != 0U) {
    /* CHTODO */
  }

  /* Endpoint events handling.*/
  while ((istr & INTF_STIF) != 0U) {
    usb_serve_endpoints(usbp, istr);
    istr = GD32_USB->INTF;
  }

  OSAL_IRQ_EPILOGUE();
}
#endif /* GD32_USB_USE_USBD */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level USB driver initialization.
 *
 * @notapi
 */
void usb_lld_init(void) {

  /* Driver initialization.*/
  usbObjectInit(&USBD);
}

/**
 * @brief   Configures and activates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_start(USBDriver *usbp) {

  if (usbp->state == USB_STOP) {
    /* Clock activation.*/
#if GD32_USB_USE_USBD
    if (&USBD == usbp) {

      osalDbgAssert((GD32_USBCLK >= (48000000U - GD32_USB_48MHZ_DELTA)) &&
                    (GD32_USBCLK <= (48000000U + GD32_USB_48MHZ_DELTA)),
                    "invalid clock frequency");

      /* USB clock enabled.*/
      rcuEnableUSB(true);
      rcuResetUSB();

      /* Powers up the transceiver while holding the USB in reset state.*/
      GD32_USB->CTL = CTL_SETRST;

      /* Enabling the USB IRQ vectors, this also gives enough time to allow
         the transceiver power up (1uS).*/
#if GD32_USBD_HP_CAN_TX_NUMBER != GD32_USBD_LP_CAN_RX0_NUMBER
      nvicEnableVector(GD32_USBD_HP_CAN_TX_NUMBER, GD32_USB_USBD_HP_IRQ_PRIORITY);
#endif
      nvicEnableVector(GD32_USBD_LP_CAN_RX0_NUMBER, GD32_USB_USBD_LP_IRQ_PRIORITY);

      /* Releases the USB reset.*/
      GD32_USB->CTL = 0U;
    }
#endif
    /* Reset procedure enforced on driver start.*/
    usb_lld_reset(usbp);
  }
}

/**
 * @brief   Deactivates the USB peripheral.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_stop(USBDriver *usbp) {

  /* If in ready state then disables the USB clock.*/
  if (usbp->state != USB_STOP) {
#if GD32_USB_USE_USBD
    if (&USBD == usbp) {

#if GD32_USBD_HP_CAN_TX_NUMBER != GD32_USBD_LP_CAN_RX0_NUMBER
      nvicDisableVector(GD32_USBD_HP_CAN_TX_NUMBER);
#endif
      nvicDisableVector(GD32_USBD_LP_CAN_RX0_NUMBER);

      GD32_USB->CTL = CTL_CLOSE | CTL_SETRST;
      rcuDisableUSB();
    }
#endif
  }
}

/**
 * @brief   USB low level reset routine.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_reset(USBDriver *usbp) {
  uint32_t cntr;

  /* Post reset initialization.*/
  GD32_USB->BADDR  = BTABLE_ADDR;
  GD32_USB->INTF   = 0U;
  GD32_USB->DADDR  = DADDR_USBEN;
  cntr             = /* CTL_ESOFIE | */ CTL_RSTIE  | CTL_SPSIE |
                      CTL_WKUPIE | CTL_ERRIE |/* CTL_PMOUIE |*/ CTL_STIE;
  /* The SOF interrupt is only enabled if a callback is defined for
     this service because it is an high rate source.*/
  if (usbp->config->sof_cb != NULL)
    cntr |= CTL_SOFIE;
  GD32_USB->CTL = cntr;

  /* Resets the packet memory allocator.*/
  usb_pm_reset(usbp);

  /* EP0 initialization.*/
  usbp->epc[0] = &ep0config;
  usb_lld_init_endpoint(usbp, 0U);
}

/**
 * @brief   Sets the USB address.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_set_address(USBDriver *usbp) {

  GD32_USB->DADDR = (uint32_t)(usbp->address) | DADDR_USBEN;
}

/**
 * @brief   Enables an endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_init_endpoint(USBDriver *usbp, usbep_t ep) {
  uint16_t epr;
  gd32_usb_ep_ram *dp;
  const USBEndpointConfig *epcp = usbp->epc[ep];

  /* Setting the endpoint type. Note that isochronous endpoints cannot be
     bidirectional because it uses double buffering and both transmit and
     receive descriptor fields are used for either direction.*/
  switch (epcp->ep_mode & USB_EP_MODE_TYPE) {
  case USB_EP_MODE_TYPE_ISOC:
#if GD32_USB_USE_ISOCHRONOUS
    osalDbgAssert((epcp->in_state == NULL) || (epcp->out_state == NULL),
                  "isochronous EP cannot be IN and OUT");
    epr = EPxCS_CTL_ISO;
    break;
#else
    osalDbgAssert(false, "isochronous support disabled");
#endif
    /* Falls through.*/
  case USB_EP_MODE_TYPE_BULK:
    epr = EPxCS_CTL_BULK;
    break;
  case USB_EP_MODE_TYPE_INTR:
    epr = EPxCS_CTL_INTERRUPT;
    break;
  default:
    epr = EPxCS_CTL_CONTROL;
  }

  dp = USB_GET_EP_RAM(ep);

  /* IN endpoint handling.*/
  if (epcp->in_state != NULL) {
    dp->tx_count = 0U;
    dp->tx_addr  = usb_pm_alloc(usbp, epcp->in_maxsize);

#if GD32_USB_USE_ISOCHRONOUS
    if (epr == EPxCS_CTL_ISO) {
      epr |= EPxCS_TX_STA_VALID;
      dp->TXCOUNT1 = dp->tx_count;
      dp->TXADDR1  = dp->tx_addr;   /* Both buffers overlapped.*/
    }
    else {
      epr |= EPxCS_TX_STA_NAK;
    }
#else
    epr |= EPxCS_TX_STA_NAK;
#endif
  }

  /* OUT endpoint handling.*/
  if (epcp->out_state != NULL) {
    uint16_t nblocks;

    /* Endpoint size and address initialization.*/
    if (epcp->out_maxsize > 62) {
      nblocks = (((((uint32_t)epcp->out_maxsize - 1U) | 0x1FU) / 32U) << 10) |
                0x8000U;
    }
    else {
      nblocks = ((((uint32_t)(epcp->out_maxsize - 1U) | 1U) + 1U) / 2U) << 10;
    }
    dp->rx_count = nblocks;
    dp->rx_addr  = usb_pm_alloc(usbp, epcp->out_maxsize);

#if GD32_USB_USE_ISOCHRONOUS
    if (epr == EPxCS_CTL_ISO) {
      epr |= EPxCS_RX_STA_VALID;
      dp->RXCOUNT1 = dp->rx_count;
      dp->RXADDR1  = dp->rx_addr;   /* Both buffers overlapped.*/
    }
    else {
      epr |= EPxCS_RX_STA_NAK;
    }
#else
    epr |= EPxCS_RX_STA_NAK;
#endif
  }

  /* CHEPxR register cleared and initialized.*/
  GD32_USB->EPxCS[ep] = GD32_USB->EPxCS[ep];
  GD32_USB->EPxCS[ep] = epr | ep;
}

/**
 * @brief   Disables all the active endpoints except the endpoint zero.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 *
 * @notapi
 */
void usb_lld_disable_endpoints(USBDriver *usbp) {
  unsigned i;

  /* Resets the packet memory allocator.*/
  usb_pm_reset(usbp);

  /* Disabling all endpoints.*/
  for (i = 1; i <= USB_ENDPOINTS_NUMBER; i++) {

    /* Clearing all toggle bits then zeroing the rest.*/
    GD32_USB->EPxCS[i] = GD32_USB->EPxCS[i];
    GD32_USB->EPxCS[i] = 0U;
  }
}

/**
 * @brief   Returns the status of an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_out(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  switch (GD32_USB->EPxCS[ep] & EPxCS_RX_STA_MASK) {
  case EPxCS_RX_STA_DIS:
    return EP_STATUS_DISABLED;
  case EPxCS_RX_STA_STALL:
    return EP_STATUS_STALLED;
  default:
    return EP_STATUS_ACTIVE;
  }
}

/**
 * @brief   Returns the status of an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @return              The endpoint status.
 * @retval EP_STATUS_DISABLED The endpoint is not active.
 * @retval EP_STATUS_STALLED  The endpoint is stalled.
 * @retval EP_STATUS_ACTIVE   The endpoint is active.
 *
 * @notapi
 */
usbepstatus_t usb_lld_get_status_in(USBDriver *usbp, usbep_t ep) {

  (void)usbp;
  switch (GD32_USB->EPxCS[ep] & EPxCS_TX_STA_MASK) {
  case EPxCS_TX_STA_DIS:
    return EP_STATUS_DISABLED;
  case EPxCS_TX_STA_STALL:
    return EP_STATUS_STALLED;
  default:
    return EP_STATUS_ACTIVE;
  }
}

/**
 * @brief   Reads a setup packet from the dedicated packet buffer.
 * @details This function must be invoked in the context of the @p setup_cb
 *          callback in order to read the received setup packet.
 * @pre     In order to use this function the endpoint must have been
 *          initialized as a control endpoint.
 * @post    The endpoint is ready to accept another packet.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 * @param[out] buf      buffer where to copy the packet data
 *
 * @notapi
 */
void usb_lld_read_setup(USBDriver *usbp, usbep_t ep, uint8_t *buf) {
  gd32_usb_pma_t *pmap;
  gd32_usb_ep_ram *udp;
  uint32_t n;

  (void)usbp;

  udp = USB_GET_EP_RAM(ep);
  pmap = USB_ADDR2PTR(udp->rx_addr);
  for (n = 0; n < 4; n++) {
    *(uint16_t *)(void *)buf = (uint16_t)*pmap++;
    buf += 2;
  }
}

/**
 * @brief   Starts a receive operation on an OUT endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_out(USBDriver *usbp, usbep_t ep) {
  USBOutEndpointState *osp = usbp->epc[ep]->out_state;

  /* Transfer initialization.*/
  if (osp->rxsize == 0U) {       /* Special case for zero sized packets.*/
    osp->rxpkts = 1U;
  }
  else {
    osp->rxpkts = (uint16_t)((osp->rxsize + usbp->epc[ep]->out_maxsize - 1) /
                             usbp->epc[ep]->out_maxsize);
  }

  EPxCS_SET_STAT_RX(ep, EPxCS_RX_STA_VALID);
}

/**
 * @brief   Starts a transmit operation on an IN endpoint.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_start_in(USBDriver *usbp, usbep_t ep) {
  size_t n;
  USBInEndpointState *isp = usbp->epc[ep]->in_state;

  /* Transfer initialization.*/
  n = isp->txsize;
  if (n > (size_t)usbp->epc[ep]->in_maxsize) {
    n = (size_t)usbp->epc[ep]->in_maxsize;
  }

  isp->txlast = n;

  usb_packet_write_from_buffer(ep, isp->txbuf, n);

  EPxCS_SET_STAT_TX(ep, EPxCS_TX_STA_VALID);
}

/**
 * @brief   Brings an OUT endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_out(USBDriver *usbp, usbep_t ep) {

  (void)usbp;

  EPxCS_SET_STAT_RX(ep, EPxCS_RX_STA_STALL);
}

/**
 * @brief   Brings an IN endpoint in the stalled state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_stall_in(USBDriver *usbp, usbep_t ep) {

  (void)usbp;

  EPxCS_SET_STAT_TX(ep, EPxCS_TX_STA_STALL);
}

/**
 * @brief   Brings an OUT endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_out(USBDriver *usbp, usbep_t ep) {

  (void)usbp;

  /* Makes sure to not put to NAK an endpoint that is already
     transferring.*/
  if ((GD32_USB->EPxCS[ep] & EPxCS_RX_STA_MASK) != EPxCS_RX_STA_VALID) {
    EPxCS_SET_STAT_TX(ep, EPxCS_RX_STA_NAK);
  }
}

/**
 * @brief   Brings an IN endpoint in the active state.
 *
 * @param[in] usbp      pointer to the @p USBDriver object
 * @param[in] ep        endpoint number
 *
 * @notapi
 */
void usb_lld_clear_in(USBDriver *usbp, usbep_t ep) {

  (void)usbp;

  /* Makes sure to not put to NAK an endpoint that is already
     transferring.*/
  if ((GD32_USB->EPxCS[ep] & EPxCS_TX_STA_MASK) != EPxCS_TX_STA_VALID) {
    EPxCS_SET_STAT_TX(ep, EPxCS_TX_STA_NAK);
  }
}

#endif /* HAL_USE_USB */

/** @} */
