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
 * @file    GD32L23x/gd32_registry.h
 * @brief   GD32L23xxx capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef GD32_REGISTRY_H
#define GD32_REGISTRY_H

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

#if defined(GD32L235) || defined(__DOXYGEN__)
/**
 * @name    GD32L23x family capabilities
 * @{
 */
/* ADC attributes.*/
#define GD32_HAS_ADC                        TRUE

/* CAN attributes.*/
#define GD32_HAS_CAN                        TRUE
#define GD32_CAN_MAX_FILTERS                28

/* CRC attributes.*/
#define GD32_HAS_CRC                        TRUE
#define GD32_CRC_PROGRAMMABLE               TRUE

/* DAC attributes.*/
#define GD32_HAS_DAC                        TRUE

/* DMA attributes.*/
#define GD32_DMA_NUM_CHANNELS               7
#define GD32_DMA_CH0_HANDLER                Vector6C
#define GD32_DMA_CH1_HANDLER                Vector70
#define GD32_DMA_CH2_HANDLER                Vector74
#define GD32_DMA_CH3_HANDLER                Vector78
#define GD32_DMA_CH4_HANDLER                Vector7C
#define GD32_DMA_CH5_HANDLER                Vector80
#define GD32_DMA_CH6_HANDLER                Vector84
#define GD32_DMA_CH0_NUMBER                 11
#define GD32_DMA_CH1_NUMBER                 12
#define GD32_DMA_CH2_NUMBER                 13
#define GD32_DMA_CH3_NUMBER                 14
#define GD32_DMA_CH4_NUMBER                 15
#define GD32_DMA_CH5_NUMBER                 16
#define GD32_DMA_CH6_NUMBER                 17

/* EXTI attributes.*/
#define GD32_EXTI_NUM_LINES                  15
#define GD32_EXTI_IMR_MASK                   0x00000000U

/* Flash attributes.*/
#define GD32_FLASH_NUMBER_OF_BANKS           1
#define GD32_FLASH_SECTOR_SIZE               1024U
#define GD32_FLASH_SECTORS_PER_BANK          128

/* GPIO attributes.*/
#define GD32_HAS_GPIOA                       TRUE
#define GD32_HAS_GPIOB                       TRUE
#define GD32_HAS_GPIOC                       TRUE
#define GD32_HAS_GPIOD                       TRUE
#define GD32_HAS_GPIOF                       TRUE

/* I2C attributes.*/
#define GD32_HAS_I2C0                        TRUE
#define GD32_HAS_I2C1                        TRUE

/* RTC attributes.*/
#define GD32_HAS_RTC                         TRUE
#define GD32_RTC_HAS_SUBSECONDS              TRUE
#define GD32_RTC_IS_CALENDAR                 FALSE

/* SPI attributes.*/
#define GD32_HAS_SPI0                        TRUE
#define GD32_SPI0_SUPPORTS_I2S               FALSE

#define GD32_HAS_SPI1                        TRUE
#define GD32_SPI1_SUPPORTS_I2S               TRUE
#define GD32_SPI1_I2S_FULLDUPLEX             FALSE

/* TIMER attributes.*/
#define GD32_TIMER_MAX_CHANNELS              4

#define GD32_HAS_TIMER0                      TRUE
#define GD32_TIMER0_IS_32BITS                FALSE
#define GD32_TIMER0_CHANNELS                 4

#define GD32_HAS_TIMER1                      TRUE
#define GD32_TIMER1_IS_32BITS                FALSE
#define GD32_TIMER1_CHANNELS                 4

#define GD32_HAS_TIMER2                      TRUE
#define GD32_TIMER2_IS_32BITS                FALSE
#define GD32_TIMER2_CHANNELS                 4

#define GD32_HAS_TIMER5                      TRUE
#define GD32_TIMER5_IS_32BITS                FALSE
#define GD32_TIMER5_CHANNELS                 0

#define GD32_HAS_TIMER6                      TRUE
#define GD32_TIMER6_IS_32BITS                FALSE
#define GD32_TIMER6_CHANNELS                 0

#define GD32_HAS_TIMER8                      TRUE
#define GD32_TIMER8_IS_32BITS                FALSE
#define GD32_TIMER8_CHANNELS                 2

#define GD32_HAS_TIMER11                     TRUE
#define GD32_TIMER11_IS_32BITS               FALSE
#define GD32_TIMER11_CHANNELS                2

#define GD32_HAS_TIMER14                     TRUE
#define GD32_TIMER14_IS_32BITS               FALSE
#define GD32_TIMER14_CHANNELS                2

#define GD32_HAS_TIMER40                     TRUE
#define GD32_TIMER40_IS_32BITS               FALSE
#define GD32_TIMER40_CHANNELS                2

/* USART attributes.*/
#define GD32_HAS_USART0                      TRUE
#define GD32_HAS_USART1                      TRUE
#define GD32_HAS_UART3                       TRUE
#define GD32_HAS_UART4                       TRUE

/* LPUART attributes.*/
#define GD32_HAS_LPUART0                     TRUE
#define GD32_HAS_LPUART1                     TRUE

/* USB attributes.*/
#define GD32_HAS_USB                         TRUE
#define GD32_HAS_USBD                        TRUE
#define GD32_USB_PMA_SIZE                    512
#define GD32_USB_HAS_DPCR                    TRUE
#define GD32_USBD_ENDPOINTS                  8

/* FWDGT attributes.*/
#define GD32_HAS_FWDGT                       TRUE

#endif

/** @} */

#endif /* GD32_REGISTRY_H */

/** @} */
