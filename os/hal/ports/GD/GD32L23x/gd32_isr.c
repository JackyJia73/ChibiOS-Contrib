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
 * @file    GD32L23x/gd32_isr.c
 * @brief   GD32L23x ISR handler code.
 *
 * @addtogroup GD32L23x_ISR
 * @{
 */

#include "hal.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

#define exti_serve_irq(pd, channel)                                            \
  {                                                                            \
                                                                               \
    if ((pd) & (1U << (channel))) {                                            \
      _pal_isr_code(channel);                                                  \
    }                                                                          \
  }

  OSAL_IRQ_HANDLER(HardFault_Handler) {

  OSAL_IRQ_PROLOGUE();
  OSAL_IRQ_EPILOGUE();
}


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if (HAL_USE_PAL && (PAL_USE_WAIT || PAL_USE_CALLBACKS)) || defined(__DOXYGEN__)
#if !defined(GD32_DISABLE_EXTI0_HANDLER)
/**
 * @brief   EXTI[0] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector58) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & (1U << 0);
  EXTI->PD = pd;

  exti_serve_irq(pd, 0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(GD32_DISABLE_EXTI1_HANDLER)
/**
 * @brief   EXTI[1] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector5C) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & (1U << 1);
  EXTI->PD = pd;

  exti_serve_irq(pd, 1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(GD32_DISABLE_EXTI2_HANDLER)
/**
 * @brief   EXTI[2] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector60) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & (1U << 2);
  EXTI->PD = pd;

  exti_serve_irq(pd, 2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(GD32_DISABLE_EXTI3_HANDLER)
/**
 * @brief   EXTI[3] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector64) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & (1U << 3);
  EXTI->PD = pd;

  exti_serve_irq(pd, 3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(GD32_DISABLE_EXTI4_HANDLER)
/**
 * @brief   EXTI[4] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(Vector68) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & (1U << 4);
  EXTI->PD = pd;

  exti_serve_irq(pd, 4);

  OSAL_IRQ_EPILOGUE();
}
#endif


#if !defined(GD32_DISABLE_EXTI5_9_HANDLER)
/**
 * @brief   EXTI[5]...EXTI[9] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(VectorEC) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & ((1U << 5) | (1U << 6) | (1U << 7) | (1U << 8) | (1U << 9));
  EXTI->PD = pd;

  exti_serve_irq(pd, 5);
  exti_serve_irq(pd, 6);
  exti_serve_irq(pd, 7);
  exti_serve_irq(pd, 8);
  exti_serve_irq(pd, 9);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(GD32_DISABLE_EXTI10_15_HANDLER)
/**
 * @brief   EXTI[10]...EXTI[15] interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(vectorFC) {
  uint32_t pd;

  OSAL_IRQ_PROLOGUE();

  pd = EXTI->PD;
  pd &= EXTI->INTEN & ((1U << 10) | (1U << 11) | (1U << 12) | (1U << 13) |
                     (1U << 14) | (1U << 15));
  EXTI->PD = pd;

  exti_serve_irq(pd, 10);
  exti_serve_irq(pd, 11);
  exti_serve_irq(pd, 12);
  exti_serve_irq(pd, 13);
  exti_serve_irq(pd, 14);
  exti_serve_irq(pd, 15);

  OSAL_IRQ_EPILOGUE();
}
#endif

#endif /* HAL_USE_PAL */

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Enables IRQ sources.
 *
 * @notapi
 */
void irqInit(void) {
#if HAL_USE_PAL
  nvicEnableVector(EXTI0_IRQn, GD32_IRQ_EXTI0_PRIORITY);
  nvicEnableVector(EXTI1_IRQn, GD32_IRQ_EXTI1_PRIORITY);
  nvicEnableVector(EXTI2_IRQn, GD32_IRQ_EXTI2_PRIORITY);
  nvicEnableVector(EXTI3_IRQn, GD32_IRQ_EXTI3_PRIORITY);
  nvicEnableVector(EXTI4_IRQn, GD32_IRQ_EXTI4_PRIORITY);
  nvicEnableVector(EXTI5_9_IRQn, GD32_IRQ_EXTI5_9_PRIORITY);
  nvicEnableVector(EXTI10_15_IRQn, GD32_IRQ_EXTI10_15_PRIORITY);
#endif
}

/**
 * @brief   Disables IRQ sources.
 *
 * @notapi
 */
void irqDeinit(void) {
#if HAL_USE_PAL
  nvicDisableVector(EXTI0_IRQn);
  nvicDisableVector(EXTI1_IRQn);
  nvicDisableVector(EXTI2_IRQn);
  nvicDisableVector(EXTI3_IRQn);
  nvicDisableVector(EXTI4_IRQn);
  nvicDisableVector(EXTI5_9_IRQn);
  nvicDisableVector(EXTI10_15_IRQn);
#endif
}

/** @} */
