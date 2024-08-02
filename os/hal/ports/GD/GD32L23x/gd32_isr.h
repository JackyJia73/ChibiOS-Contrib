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
 * @file    GD32L23x/gd32_isr.h
 * @brief   GD32L23x ISR handler header.
 *
 * @addtogroup GD32L23x_ISR
 * @{
 */

#ifndef GD32_ISR_H
#define GD32_ISR_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    ISR names and numbers remapping
 * @{
 */

/*
 * I2C units.
 */
#define GD32_I2C0_EVENT_HANDLER    VectorBC
#define GD32_I2C0_ERROR_HANDLER    VectorC0
#define GD32_I2C0_EVENT_NUMBER     31
#define GD32_I2C0_ERROR_NUMBER     32

#define GD32_I2C1_EVENT_HANDLER    VectorC4
#define GD32_I2C1_ERROR_HANDLER    VectorC8
#define GD32_I2C1_EVENT_NUMBER     33
#define GD32_I2C1_ERROR_NUMBER     34

/*
 * TIMER units.
 */
#define GD32_TIMER0_UP_HANDLER       VectorF0
#define GD32_TIMER0_CC_HANDLER       VectorF4
#define GD32_TIMER1_HANDLER          Vector94
#define GD32_TIMER2_HANDLER          Vector98
#define GD32_TIMER5_HANDLER          VectorA4
#define GD32_TIMER6_HANDLER          VectorA8
#define GD32_TIMER8_HANDLER          Vector9C
#define GD32_TIMER11_HANDLER         VectorA0
#define GD32_TIMER14_HANDLER         VectorF8
#define GD32_TIMER40_HANDLER         Vector100

#define GD32_TIMER0_UP_NUMBER        44
#define GD32_TIMER0_CC_NUMBER        45
#define GD32_TIMER1_NUMBER           21
#define GD32_TIMER2_NUMBER           22
#define GD32_TIMER5_NUMBER           25
#define GD32_TIMER6_NUMBER           26
#define GD32_TIMER8_NUMBER           23
#define GD32_TIMER11_NUMBER          24
#define GD32_TIMER14_NUMBER          46
#define GD32_TIMER40_NUMBER          48

/*
 * USART units.
 */
#define GD32_USART0_HANDLER        VectorAC
#define GD32_USART1_HANDLER        VectorB0
#define GD32_UART3_HANDLER         VectorB4
#define GD32_UART4_HANDLER         VectorB8

#define GD32_USART0_NUMBER         27
#define GD32_USART1_NUMBER         28
#define GD32_UART3_NUMBER          29
#define GD32_UART4_NUMBER          30

/*
 * LPUART units.
 */
#define GD32_LPUART0_HANDLER        Vector134
#define GD32_LPUART1_HANDLER        Vector15C

#define GD32_LPUART0_NUMBER         61
#define GD32_LPUART1_NUMBER         71

/*
 * SPI units.
 */
#define GD32_SPI0_HANDLER           VectorCC
#define GD32_SPI1_HANDLER           VectorD0

#define GD32_SPI0_NUMBER            35
#define GD32_SPI1_NUMBER            36

/*
 * ADC units.
 */
#define GD32_ADC_HANDLER            Vector88

#define GD32_ADC_NUMBER             18


/*
 * USB units.
 */
#define GD32_USBD_HP_CAN_TX_HANDLER               Vector8C
#define GD32_USBD_LP_CAN_RX0_HANDLER              Vector90
#define GD32_USBD_HP_CAN_TX_NUMBER                19
#define GD32_USBD_LP_CAN_RX0_NUMBER               20


/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   EXTI0 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI0_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI0_PRIORITY            3
#endif

/**
 * @brief   EXTI1 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI1_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI1_PRIORITY            3
#endif

/**
 * @brief   EXTI2 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI2_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI2_PRIORITY            3
#endif

/**
 * @brief   EXTI3 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI3_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI3_PRIORITY            3
#endif

/**
 * @brief   EXTI4 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI4_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI4_PRIORITY            3
#endif

/**
 * @brief   EXTI9..5 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI5_9_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI5_9_PRIORITY          3
#endif

/**
 * @brief   EXTI15..10 interrupt priority level setting.
 */
#if !defined(GD32_IRQ_EXTI10_15_PRIORITY) || defined(__DOXYGEN__)
#define GD32_IRQ_EXTI10_15_PRIORITY        3
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  void irqInit(void);
  void irqDeinit(void);
#ifdef __cplusplus
}
#endif

#endif /* GD32_ISR_H */

/** @} */
