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
 * @file    GD32L23x/hal_lld.c
 * @brief   GD32L23x HAL subsystem low level driver source.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   NMSIS system core clock variable.
 * @note    It is declared in system_gd32vf103.h.
 */
uint32_t SystemCoreClock = GD32_HCLK;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 *
 * @notapi
 */
void hal_lld_init(void) {

  /* Reset of all peripherals.*/
  rcuResetAPB1(0xFFFFFFFF);
  rcuResetAPB2(0xFFFFFFFF);
  rcuResetAHB(0xFFFFFFFF);
  rcuResetAHB2(0xFFFFFFFF);

  /* PMU and BD clocks enabled.*/
  rcuEnablePMUInterface(true);
  rcuEnableBKPInterface(true);

  /* DMA subsystems initialization.*/
 #if defined(GD32_DMA_REQUIRED)
  dmaInit();
 #endif

  /* IRQ subsystem initialization.*/
  irqInit();

  /* Programmable voltage detector enable.*/
#if GD32_PVD_ENABLE
  PMU->CTL0 |= PMU_CTL0_LVDEN | (GD32_LVDT & PMU_CTL0_LVDT);
#endif /* GD32_PVD_ENABLE */
}

/**
 * @brief   GD32 clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */

/*
 * Clocks initialization for the CL sub-family.
 */
void gd32_clock_init(void) {

#if !GD32_NO_INIT
  /* IRC16M setup, it enforces the reset situation in order to handle possible
     problems with JTAG probes and re-initializations.*/
  RCU->CTL |= RCU_CTL_IRC16MEN;                  /* Make sure IRC16M is ON.         */
  while (!(RCU->CTL & RCU_CTL_IRC16MSTB))
    ;                                       /* Wait until IRC16M is stable.    */

  RCU->CFG0 &= ~RCU_CFG0_SCS;
#ifdef GD32L235
    RCU->CFG1 &= ~RCU_CFG1_SCS_2;
#endif /* GD32L235 */
    RCU->CTL &= ~(RCU_CTL_HXTALEN | RCU_CTL_CKMEN | RCU_CTL_PLLEN | RCU_CTL_HXTALBPS);
    /* reset RCU */
    RCU->CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC | \
                  RCU_CFG0_ADCPSC | RCU_CFG0_CKOUTSEL | RCU_CFG0_CKOUTDIV);
    RCU->CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_6 | RCU_CFG0_PLLDV);

#if GD32_HXTAL_ENABLED
#if defined(GD32_HXTAL_BYPASS)
  /* HXTAL Bypass.*/
  RCU->CTL |= RCU_CTL_HXTALBPS;
#endif
  /* HXTAL activation.*/
  RCU->CTL |= RCU_CTL_HXTALEN;
  while (!(RCU->CTL & RCU_CTL_HXTALSTB))
    ;                                       /* Waits until HXTAL is stable.   */
#endif

#if GD32_IRC32K_ENABLED
  /* IRC32K activation.*/
  RCU->RSTSCK |= RCU_RSTSCK_IRC32KEN;
  while ((RCU->RSTSCK & RCU_RSTSCK_IRC32KSTB) == 0)
    ;                                       /* Waits until IRC32K is stable.   */
#endif

#if GD32_IRC48M_ENABLED
  RCU->CTL |= RCU_CTL_IRC48MEN;             /* Make sure IRC48M is ON.         */
  while (!(RCU->CTL & RCU_CTL_IRC48MSTB))
    ;                                       /* Wait until IRC48M is stable.    */
#endif

  /* Flash setup and final clock selection.   */
  FLASH->WS &= ~FMC_WS_WSCNT;
  FLASH->WS |= GD32_FLASHBITS; /* Flash wait states depending on clock.    */
  while ((FLASH->WS & FMC_WS_WSCNT) !=
         (GD32_FLASHBITS & FMC_WS_WSCNT)) {
  }

  /* AHB prescaler selection */
  RCU->CFG0 |= GD32_AHBPSC;
  /* APB2 prescaler selection */
  RCU->CFG0 |= GD32_APB2PSC;
  /* APB1 prescaler selection */
  RCU->CFG0 |= GD32_APB1PSC;

  /* PLL setup, if activated.*/
#if GD32_ACTIVATE_PLL
  RCU_CFG1 &= ~(RCU_CFG1_PREDV);
  RCU_CFG1 |= GD32_PREDV_VALUE;
  /* PLL = GD32_PLLSEL * GD32_PLLMF_VALUE */
  RCU->CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLDV);
  RCU->CFG0 |= (GD32_PLLSEL | GD32_PLLMF_VALUE);
  /* enable PLL */
  RCU->CTL |= RCU_CTL_PLLEN;
  /* wait until PLL is stable */
  while(0U == (RCU->CTL & RCU_CTL_PLLSTB)) {
  }
#endif

  /* USBD Clock settings.*/
#if GD32_HAS_USBD && GD32_USBD_CLOCK_REQUIRED
  /* reset the USBDSEL bits and set according to ck_usbd */
  RCU->CFG2 &= ~RCU_CFG2_USBDSEL;
  RCU->CFG2 |= GD32_USBDCLK_SEL;
#endif

  /* Switching to the configured clock source if it is different from IRC16M.*/
#if (GD32_SCS != RCU_SCSS_IRC16M)
  RCU->CFG0 |= GD32_SCS;        /* Switches on the selected clock source.   */
  while (((RCU->CFG0 & RCU_CFG0_SCSS) != ((GD32_SCS & RCU_CFG0_SCS) << 2)) || \
         ((RCU->CFG1 & RCU_CFG1_SCSS_2) != ((GD32_SCS & RCU_CFG1_SCS_2) << 1)))
    ;
#endif

#if !GD32_IRC16M_ENABLED
  RCU->CTL &= ~RCU_CTL_IRC16MEN;
#endif
#endif /* !GD32_NO_INIT */
}

/** @} */
