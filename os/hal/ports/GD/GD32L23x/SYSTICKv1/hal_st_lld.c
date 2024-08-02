/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

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
 * @file    SYSTICKv1/hal_st_lld.c
 * @brief   ST Driver subsystem low level driver code.
 *
 * @addtogroup ST
 * @{
 */

#include "hal.h"

#if (OSAL_ST_MODE != OSAL_ST_MODE_NONE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

#if (OSAL_ST_RESOLUTION != 16) && (OSAL_ST_RESOLUTION != 32)
#error "unsupported ST resolution"
#endif

#if (OSAL_ST_RESOLUTION == 32)
#define ST_ARR_INIT                         0xFFFFFFFFU
#else
#define ST_ARR_INIT                         0x0000FFFFU
#endif


#if GD32_ST_USE_TIMER == 0

#if !GD32_HAS_TIM0
#error "TIM0 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM0_IS_32BITS
#error "TIM0 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM0_CC_HANDLER
#define ST_NUMBER                           GD32_TIM0_CC_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM0(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM0_STOP

#elif GD32_ST_USE_TIMER == 1

#if !GD32_HAS_TIM1
#error "TIM1 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM1_IS_32BITS
#error "TIM1 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM1_CC_HANDLER
#define ST_NUMBER                           GD32_TIM1_CC_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM1(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM1_STOP

#elif GD32_ST_USE_TIMER == 2

#if !GD32_HAS_TIM2
#error "TIM2 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM2_IS_32BITS
#error "TIM2 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM2_HANDLER
#define ST_NUMBER                           GD32_TIM2_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM2(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM2_STOP

#elif GD32_ST_USE_TIMER == 5

#if !GD32_HAS_TIM5
#error "TIM5 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM5_IS_32BITS
#error "TIM5 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM5_HANDLER
#define ST_NUMBER                           GD32_TIM5_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM5(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM5_STOP

#elif GD32_ST_USE_TIMER == 6

#if !GD32_HAS_TIM6
#error "TIM6 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM6_IS_32BITS
#error "TIM6 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM6_HANDLER
#define ST_NUMBER                           GD32_TIM6_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM6(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM6_STOP

#elif GD32_ST_USE_TIMER == 8

#if !GD32_HAS_TIM8
#error "TIM8 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM2_IS_32BITS
#error "TIM8 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM8_CC_HANDLER
#define ST_NUMBER                           GD32_TIM8_CC_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM8(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM8_STOP

#elif GD32_ST_USE_TIMER == 11

#if !GD32_HAS_TIM11
#error "TIM11 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM9_IS_32BITS
#error "TIM11 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM11_HANDLER
#define ST_NUMBER                           GD32_TIM11_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK2
#define ST_ENABLE_CLOCK()                   rccEnableTIM11(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM11_STOP

#elif GD32_ST_USE_TIMER == 14

#if !GD32_HAS_TIM14
#error "TIM14 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM14_IS_32BITS
#error "TIM14 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM14_HANDLER
#define ST_NUMBER                           GD32_TIM14_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK1
#define ST_ENABLE_CLOCK()                   rccEnableTIM14(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM14_STOP

#if !GD32_HAS_TIM40
#error "TIM40 not present in the selected device"
#endif

#if (OSAL_ST_RESOLUTION == 32) && !GD32_TIM40_IS_32BITS
#error "TIM40 is not a 32bits timer"
#endif

#define ST_HANDLER                          GD32_TIM40_HANDLER
#define ST_NUMBER                           GD32_TIM40_NUMBER
#define ST_CLOCK_SRC                        GD32_TIMCLK2
#define ST_ENABLE_CLOCK()                   rccEnableTIM40(true)
#define ST_ENABLE_STOP()                    DBGMCU->CR |= DBGMCU_CR_DBG_TIM40_STOP

#else
#error "GD32_ST_USE_TIMER specifies an unsupported timer"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#define ST_HANDLER                          SysTick_Handler

#if defined(GD32_CORE_CK)
#define SYSTICK_CK                          GD32_CORE_CK
#else
#define SYSTICK_CK                          GD32_HCLK
#endif

#if SYSTICK_CK % OSAL_ST_FREQUENCY != 0
#error "the selected ST frequency is not obtainable because integer rounding"
#endif

#if (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1 > 0xFFFFFF
#error "the selected ST frequency is not obtainable because SysTick timer counter limits"
#endif

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if !defined(GD32_SYSTICK_SUPPRESS_ISR)
/**
 * @brief   Interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(ST_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  st_lld_serve_interrupt();

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ST driver initialization.
 *
 * @notapi
 */
void st_lld_init(void) {

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  /* Free running counter mode.*/
  osalDbgAssert((ST_CLOCK_SRC % OSAL_ST_FREQUENCY) == 0U,
                "clock rounding error");
  osalDbgAssert(((ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1U) < 0x10000,
                "clock prescaler overflow");

  /* Enabling timer clock.*/
  ST_ENABLE_CLOCK();

  /* Enabling the stop mode during debug for this timer.*/
  ST_ENABLE_STOP();

  /* Initializing the counter in free running mode.*/
  GD32_ST_TIM->PSC    = (ST_CLOCK_SRC / OSAL_ST_FREQUENCY) - 1;
  GD32_ST_TIM->CAR    = ST_ARR_INIT;
  GD32_ST_TIM->CHCTL0  = 0;
  GD32_ST_TIM->CHCV[0] = 0;
#if ST_LLD_NUM_ALARMS > 1
  GD32_ST_TIM->CHCV[1] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 2
  GD32_ST_TIM->CHCV[2] = 0;
#endif
#if ST_LLD_NUM_ALARMS > 3
  GD32_ST_TIM->CHCV[3] = 0;
#endif
  GD32_ST_TIM->DMAINTEN   = 0;
  GD32_ST_TIM->CTL1    = 0;
  GD32_ST_TIM->SWEVG    = TIM_EGR_UG;
  GD32_ST_TIM->CTL0    = TIM_CR1_CEN;

#if !defined(GD32_SYSTICK_SUPPRESS_ISR)
  /* IRQ enabled.*/
  nvicEnableVector(ST_NUMBER, GD32_ST_IRQ_PRIORITY);
#endif
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#if OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC
  /* Periodic systick mode, the Cortex-Mx internal systick timer is used
     in this mode.*/
  SysTick->LOAD = (SYSTICK_CK / OSAL_ST_FREQUENCY) - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_ENABLE_Msk |
                  SysTick_CTRL_TICKINT_Msk;

  /* IRQ enabled.*/
  nvicSetSystemHandlerPriority(HANDLER_SYSTICK, GD32_ST_IRQ_PRIORITY);
#endif /* OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC */
}

/**
 * @brief   IRQ handling code.
 */
void st_lld_serve_interrupt(void) {
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
  uint32_t sr;
  gd32_tim_t *timp = GD32_ST_TIM;

  sr  = timp->SR;
  sr &= timp->DIER & GD32_TIM_DIER_IRQ_MASK;
  timp->SR = ~sr;

  if ((sr & TIM_SR_CC1IF) != 0U)
#endif
  {
    osalSysLockFromISR();
    osalOsTimerHandlerI();
    osalSysUnlockFromISR();
  }
#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING
#if ST_LLD_NUM_ALARMS > 1
  if ((sr & TIM_SR_CC2IF) != 0U) {
    if (st_callbacks[1] != NULL) {
      st_callbacks[1](1U);
    }
  }
#endif
#if ST_LLD_NUM_ALARMS > 2
  if ((sr & TIM_SR_CC3IF) != 0U) {
    if (st_callbacks[2] != NULL) {
      st_callbacks[2](2U);
    }
  }
#endif
#if ST_LLD_NUM_ALARMS > 3
  if ((sr & TIM_SR_CC4IF) != 0U) {
    if (st_callbacks[3] != NULL) {
      st_callbacks[3](3U);
    }
  }
#endif
#endif
}

#endif /* OSAL_ST_MODE != OSAL_ST_MODE_NONE */

/** @} */
