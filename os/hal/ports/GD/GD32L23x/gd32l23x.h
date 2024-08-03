/*!
    \file    gd32l23x.h
    \brief   general definitions for GD32L23x

    \version 2024-03-25, V2.1.2, firmware for GD32L23x, add support for GD32L235
*/

/*
 * Copyright (c) 2009-2018 Arm Limited. All rights reserved.
 * Copyright (c) 2024, GigaDevice Semiconductor Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* This file refers the CMSIS standard, some adjustments are made according to GigaDevice chips */

#ifndef GD32L23X_H
#define GD32L23X_H

#ifdef __cplusplus
extern "C" {
#endif

/* define GD32L23x */
#if !defined (GD32L23x)
#define GD32L23x
#endif /* define GD32L23x */
#if !defined (GD32L23x)
#error "Please select the target GD32L23x device used in your application (in gd32l23x.h file)"
#endif /* undefine GD32L23x tip */


#define CPU_CLOCK   (64000000)

/* define value of high speed crystal oscillator (HXTAL) in Hz */
#if !defined  (HXTAL_VALUE)
#define HXTAL_VALUE              8000000
#endif /* high speed crystal oscillator value */

/* define startup timeout value of high speed crystal oscillator (HXTAL) */
#if !defined  (HXTAL_STARTUP_TIMEOUT)
#define HXTAL_STARTUP_TIMEOUT    0x0FFFF
#endif /* high speed crystal oscillator startup timeout */

/* define value of internal 16MHz RC oscillator (IRC16M) in Hz */
#if !defined  (IRC16M_VALUE)
#define IRC16M_VALUE             16000000
#endif /* internal 16MHz RC oscillator value */

/* define startup timeout value of internal 16MHz RC oscillator (IRC16M) */
#if !defined  (IRC16M_STARTUP_TIMEOUT)
#define IRC16M_STARTUP_TIMEOUT   0x0500
#endif /* internal 16MHz RC oscillator startup timeout */

#if !defined  (IRC48M_VALUE)
#define IRC48M_VALUE             48000000
#endif /* IRC48M_VALUE */

/* define value of internal 32KHz RC oscillator(IRC32K) in Hz */
#if !defined  (IRC32K_VALUE)
#define IRC32K_VALUE             32000
#endif /* internal 32KHz RC oscillator value */

/* define value of low speed crystal oscillator (LXTAL)in Hz */
#if !defined  (LXTAL_VALUE)
#define LXTAL_VALUE              32768
#endif /* low speed crystal oscillator value */

/* GD32L23x firmware library version number V1.0 */
#define __GD32L23X_STDPERIPH_VERSION_MAIN   (0x01) /*!< [31:24] main version     */
#define __GD32L23X_STDPERIPH_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version     */
#define __GD32L23X_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version     */
#define __GD32L23X_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __GD32L23X_STDPERIPH_VERSION        ((__GD32L23X_STDPERIPH_VERSION_MAIN << 24)\
        |(__GD32L23X_STDPERIPH_VERSION_SUB1 << 16)\
        |(__GD32L23X_STDPERIPH_VERSION_SUB2 << 8)\
        |(__GD32L23X_STDPERIPH_VERSION_RC))

/* configuration of the Cortex-M23 processor and core peripherals                                        */
#define __CM23_REV                0x0100U   /*!< Core revision r1p0                                      */
#define __SAUREGION_PRESENT       0U        /*!< SAU regions are not present                             */
#define __MPU_PRESENT             0U        /*!< MPU is present                                          */
#define __VTOR_PRESENT            1U        /*!< VTOR is present                                         */
#define __NVIC_PRIO_BITS          2U        /*!< Number of Bits used for Priority Levels                 */
#define __Vendor_SysTickConfig    0U        /*!< Set to 1 if different SysTick Config is used            */

/* define interrupt number */
typedef enum IRQn {
    /* Cortex-M23 processor exceptions numbers */
    NonMaskableInt_IRQn          = -14,    /*!< non maskable interrupt                                   */
    HardFault_IRQn               = -13,    /*!< hardfault interrupt                                      */
    SVCall_IRQn                  = -5,     /*!< sv call interrupt                                        */
    PendSV_IRQn                  = -2,     /*!< pend sv interrupt                                        */
    SysTick_IRQn                 = -1,     /*!< system tick interrupt                                    */
    /* interruput numbers */
    WWDGT_IRQn                   = 0,      /*!< window watchdog timer interrupt                          */
    LVD_IRQn                     = 1,      /*!< LVD through EXTI line detect interrupt                   */
    TAMPER_STAMP_IRQn            = 2,      /*!< RTC Tamper and TimeStamp interrupt                       */
    RTC_WKUP_IRQn                = 3,      /*!< RTC Wakeup interrupt                                     */
    FMC_IRQn                     = 4,      /*!< FMC interrupt                                            */
    RCU_CTC_IRQn                 = 5,      /*!< RCU and CTC interrupt                                    */
    EXTI0_IRQn                   = 6,      /*!< EXTI line 0 interrupts                                   */
    EXTI1_IRQn                   = 7,      /*!< EXTI line 1 interrupts                                   */
    EXTI2_IRQn                   = 8,      /*!< EXTI line 2 interrupts                                   */
    EXTI3_IRQn                   = 9,      /*!< EXTI line 3 interrupts                                   */
    EXTI4_IRQn                   = 10,     /*!< EXTI line 4 interrupts                                   */
    DMA_Channel0_IRQn            = 11,     /*!< DMA channel 0 interrupt                                  */
    DMA_Channel1_IRQn            = 12,     /*!< DMA channel 1 interrupt                                  */
    DMA_Channel2_IRQn            = 13,     /*!< DMA channel 2 interrupt                                  */
    DMA_Channel3_IRQn            = 14,     /*!< DMA channel 3 interrupt                                  */
    DMA_Channel4_IRQn            = 15,     /*!< DMA channel 4 interrupt                                  */
    DMA_Channel5_IRQn            = 16,     /*!< DMA channel 5 interrupt                                  */
    DMA_Channel6_IRQn            = 17,     /*!< DMA channel 6 interrupt                                  */
    ADC_IRQn                     = 18,     /*!< ADC interrupts                                           */
#ifdef GD32L235
    USBD_HP_CAN_TX_IRQn          = 19,     /*!< USBD High Priority or CAN TX                             */
    USBD_LP_CAN_RX0_IRQn         = 20,     /*!< USBD Low Priority or CAN RX0                             */
#else
    USBD_HP_IRQn                 = 19,     /*!< USBD High Priority                                       */
    USBD_LP_IRQn                 = 20,     /*!< USBD Low Priority                                        */
# endif
    TIMER1_IRQn                  = 21,     /*!< TIMER1 interrupt                                         */
    TIMER2_IRQn                  = 22,     /*!< TIMER2 interrupt                                         */
    TIMER8_IRQn                  = 23,     /*!< TIMER8 interrupt                                         */
    TIMER11_IRQn                 = 24,     /*!< TIMER11 interrupt                                        */
    TIMER5_IRQn                  = 25,     /*!< TIMER5 interrupt                                         */
    TIMER6_IRQn                  = 26,     /*!< TIMER6 interrupt                                         */
    USART0_IRQn                  = 27,     /*!< USART0 interrupt                                         */
    USART1_IRQn                  = 28,     /*!< USART1 interrupt                                         */
    UART3_IRQn                   = 29,     /*!< UART3 interrupt                                          */
    UART4_IRQn                   = 30,     /*!< UART4 interrupt                                          */
    I2C0_EV_IRQn                 = 31,     /*!< I2C0 event interrupt                                     */
    I2C0_ER_IRQn                 = 32,     /*!< I2C0 error interrupt                                     */
    I2C1_EV_IRQn                 = 33,     /*!< I2C1 event interrupt                                     */
    I2C1_ER_IRQn                 = 34,     /*!< I2C1 error interrupt                                     */
    SPI0_IRQn                    = 35,     /*!< SPI0 interrupt                                           */
    SPI1_IRQn                    = 36,     /*!< SPI1 interrupt                                           */
    DAC_IRQn                     = 37,     /*!< DAC interrupt                                            */
    I2C2_EV_IRQn                 = 39,     /*!< I2C2 event interrupt                                     */
    I2C2_ER_IRQn                 = 40,     /*!< I2C2 error interrupt                                     */
    RTC_Alarm_IRQn               = 41,     /*!< RTC Alarm interrupt                                      */
    USBD_WKUP_IRQn               = 42,     /*!< USBD Wakeup interrupt                                    */
    EXTI5_9_IRQn                 = 43,     /*!< EXTI line 5 to 9 interrupts                              */
#ifdef GD32L235  
    TIMER0_TRG_CMT_UP_BRK_IRQn   = 44,     /*!< TIMER0 trigger and Channel commutation or update or break interrupt */
    TIMER0_Channel_IRQn          = 45,     /*!< TIMER0 capture compare interrupt                         */
    TIMER14_IRQn                 = 46,     /*!< TIMER14 interrupt                                        */
#endif /* GD32L235 */ 
    EXTI10_15_IRQn               = 47,     /*!< EXTI line 10 to 15 interrupts                            */
#ifdef GD32L235 
    TIMER40_IRQn                 = 48,     /*!< TIMER40 interrupt                                        */
    CAN_RX1_IRQn                 = 49,     /*!< CAN RX1 interrupt                                        */
    CAN_EWMC_IRQn                = 50,     /*!< CAN EWMC interrupt                                        */
#endif /* GD32L235 */    
    DMAMUX_IRQn                  = 55,     /*!< DMAMUX interrupt                                         */
    CMP0_IRQn                    = 56,     /*!< Comparator 0 interrupt                                   */
    CMP1_IRQn                    = 57,     /*!< Comparator 1 interrupt                                   */
    I2C0_WKUP_IRQn               = 58,     /*!< I2C0 Wakeup interrupt                                    */
    I2C2_WKUP_IRQn               = 59,     /*!< I2C2 Wakeup interrupt                                    */
    USART0_WKUP_IRQn             = 60,     /*!< USART0 Wakeup interrupt                                  */
#ifdef GD32L235    
    LPUART0_IRQn                 = 61,     /*!< LPUART0 global interrupt                                 */
#else    
    LPUART_IRQn                  = 61,     /*!< LPUART global interrupt                                  */
#endif    
    CAU_IRQn                     = 62,     /*!< CAU interrupt                                            */
    TRNG_IRQn                    = 63,     /*!< TRNG interrupt                                           */
    SLCD_IRQn                    = 64,     /*!< SLCD interrupt                                           */
    USART1_WKUP_IRQn             = 65,     /*!< USART1 Wakeup interrupt                                  */
    I2C1_WKUP_IRQn               = 66,     /*!< I2C1 Wakeup interrupt                                    */
#ifdef GD32L235
    LPUART0_WKUP_IRQn            = 67,     /*!< LPUART0 Wakeup interrupt                                 */
    LPTIMER0_IRQn                = 68,     /*!< LPTIMER0 interrupt                                       */
    LPUART1_WKUP_IRQn            = 69,     /*!< LPUART1 Wakeup interrupt                                 */
    LPTIMER1_IRQn                = 70,     /*!< LPTIMER1 interrupt                                       */   
    LPUART1_IRQn                 = 71      /*!< LPUART1 global interrupt                                */  
#else    
    LPUART_WKUP_IRQn             = 67,     /*!< LPUART Wakeup interrupt                                  */
    LPTIMER_IRQn                 = 68      /*!< LPTIMER interrupt                                        */
#endif /* GD32L235 */ 
} IRQn_Type;

/* includes */
//#include "core_cm23.h"//?????GD
#include "core_cm0.h"
#include "system_gd32l23x.h"
#include <stdint.h>



/** 
  * @brief Analog to Digital Converter  
  */

typedef struct
{
  __IO uint32_t STAT;
  __IO uint32_t CTL0;
  __IO uint32_t CTL1;
  __IO uint32_t SAMPT0;
  __IO uint32_t SAMPT1;
  __IO uint32_t IOFF0;
  __IO uint32_t IOFF1;
  __IO uint32_t IOFF2;
  __IO uint32_t IOFF3;
  __IO uint32_t WDHT;
  __IO uint32_t WDLT;
  __IO uint32_t RSQ0;
  __IO uint32_t RSQ1;
  __IO uint32_t RSQ2;
  __IO uint32_t ISQ;
  __IO uint32_t IDATA0;
  __IO uint32_t IDATA1;
  __IO uint32_t IDATA2;
  __IO uint32_t IDATA3;
  __IO uint32_t RDATA;
  uint32_t RESERVED[12];
  __IO uint32_t OVSAMPCTL;
#if defined(GD32L235) 
  __IO uint32_t DIFCTL;
#endif /* GD32L235 */
} ADC_TypeDef;

typedef struct
{
  __IO uint32_t STAT;               /*!< ADC status register,    used for ADC multimode (bits common to several ADC instances). Address offset: ADC0 base address         */
  __IO uint32_t CTL0;              /*!< ADC control register 1, used for ADC multimode (bits common to several ADC instances). Address offset: ADC0 base address + 0x04  */
  __IO uint32_t CTL1;              /*!< ADC control register 2, used for ADC multimode (bits common to several ADC instances). Address offset: ADC0 base address + 0x08  */
  uint32_t  RESERVED[16];
  __IO uint32_t RDATA;               /*!< ADC data register,      used for ADC multimode (bits common to several ADC instances). Address offset: ADC0 base address + 0x4C  */
} ADC_Common_TypeDef;

/** 
  * @brief Backup Registers  
  */

typedef struct
{
  uint32_t  RESERVED0;
  __IO uint32_t DATA1;
  __IO uint32_t DATA2;
  __IO uint32_t DATA3;
  __IO uint32_t DATA4;
  __IO uint32_t DATA5;
  __IO uint32_t DATA6;
  __IO uint32_t DATA7;
  __IO uint32_t DATA8;
  __IO uint32_t DATA9;
  __IO uint32_t DATA10;
  __IO uint32_t OCTL;
  __IO uint32_t TPCTL;
  __IO uint32_t TPCS;
  uint32_t  RESERVED13[2];
  __IO uint32_t DATA11;
  __IO uint32_t DATA12;
  __IO uint32_t DATA13;
  __IO uint32_t DATA14;
  __IO uint32_t DATA15;
  __IO uint32_t DATA16;
  __IO uint32_t DATA17;
  __IO uint32_t DATA18;
  __IO uint32_t DATA19;
  __IO uint32_t DATA20;
  __IO uint32_t DATA21;
  __IO uint32_t DATA22;
  __IO uint32_t DATA23;
  __IO uint32_t DATA24;
  __IO uint32_t DATA25;
  __IO uint32_t DATA26;
  __IO uint32_t DATA27;
  __IO uint32_t DATA28;
  __IO uint32_t DATA29;
  __IO uint32_t DATA30;
  __IO uint32_t DATA31;
  __IO uint32_t DATA32;
  __IO uint32_t DATA33;
  __IO uint32_t DATA34;
  __IO uint32_t DATA35;
  __IO uint32_t DATA36;
  __IO uint32_t DATA37;
  __IO uint32_t DATA38;
  __IO uint32_t DATA39;
  __IO uint32_t DATA40;
  __IO uint32_t DATA41;
  __IO uint32_t DATA42;
} BKP_TypeDef;
  
/** 
  * @brief Controller Area Network TxMailBox 
  */

typedef struct
{
  __IO uint32_t TMI;
  __IO uint32_t TMP;
  __IO uint32_t TMDATA0;
  __IO uint32_t TMDATA1;
} CAN_TxMailBox_TypeDef;

/** 
  * @brief Controller Area Network FIFOMailBox 
  */
  
typedef struct
{
  __IO uint32_t RFIFOMI;
  __IO uint32_t RFIFOMP;
  __IO uint32_t RFIFOMDATA0;
  __IO uint32_t RFIFOMDATA1;
} CAN_FIFOMailBox_TypeDef;

/** 
  * @brief Controller Area Network FilterRegister 
  */
typedef struct
{
  __IO uint32_t FR1;
  __IO uint32_t FR2;
} CAN_FilterRegister_TypeDef;

/** 
  * @brief Controller Area Network 
  */
typedef struct
{
  __IO uint32_t CTL;
  __IO uint32_t STAT;
  __IO uint32_t TSTAT;
  __IO uint32_t RFIFO0;
  __IO uint32_t RFIFO1;
  __IO uint32_t INTEN;
  __IO uint32_t ERR;
  __IO uint32_t BT;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t RESERVED1[12];
  __IO uint32_t FCTL;
  __IO uint32_t FMCFG;
  uint32_t  RESERVED2;
  __IO uint32_t FSCFG;
  uint32_t  RESERVED3;
  __IO uint32_t FAFIFO;
  uint32_t  RESERVED4;
  __IO uint32_t FW;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[28];
} CAN_TypeDef;

/** 
  * @brief CRC calculation unit 
  */
typedef struct
{
  __IO uint32_t DATA;           /*!< CRC data register,                    Address offset: 0x00 */
  __IO uint32_t FDATA;          /*!< CRC free data register,               Address offset: 0x04 */
  __IO uint32_t CTL;            /*!< CRC control register,                 Address offset: 0x08 */
  uint32_t RESERVED;            /*!< Reserved,                             Address offset: 0x0C */
  __IO uint32_t IDATA;          /*!< CRC initialization register,          Address offset: 0x10 */
  __IO uint32_t POLY;           /*!< CRC Control register,                 Address offset: 0x14 */ 
} CRC_TypeDef;

/** 
  * @brief Digital to Analog Converter
  */
typedef struct
{
  __IO uint32_t CTL0;
  __IO uint32_t SWT;
  __IO uint32_t OUT_R12DH;
  __IO uint32_t OUT_L12DH;
  __IO uint32_t OUT_R8DH;
  __IO uint32_t OUT_DO;
  __IO uint32_t STAT0;
} DAC_TypeDef;

/** 
  * @brief Debug MCU
  */
typedef struct
{
  __IO uint32_t IDCODE;
  __IO uint32_t CR;
} DBGMCU_TypeDef;

/** 
  * @brief DMA Controller
  */
typedef struct
{
  __IO uint32_t CTL;
  __IO uint32_t CNT;
  __IO uint32_t PADDR;
  __IO uint32_t MADDR;
} DMA_Channel_TypeDef;

typedef struct
{
  __IO uint32_t INTF;
  __IO uint32_t INTC;
} DMA_TypeDef;

/** 
  * @brief DMAMUX Controller
  */
typedef struct
{
  __IO uint32_t RM_CH0CFG;
  __IO uint32_t RM_CH1CFG;
  __IO uint32_t RM_CH2CFG;
  __IO uint32_t RM_CH3CFG;
  __IO uint32_t RM_CH4CFG;
  __IO uint32_t RM_CH5CFG;
  __IO uint32_t RM_CH6CFG;
  uint32_t RESERVED0[25];
  __IO uint32_t RM_INTF;
  __IO uint32_t RM_INTC;
  uint32_t RESERVED1[30];
  __IO uint32_t RG_CH0CFG;
  __IO uint32_t RG_CH1CFG;
  __IO uint32_t RG_CH2CFG;
  __IO uint32_t RG_CH3CFG;
  uint32_t RESERVED2[12];
  __IO uint32_t RG_INTF;
  __IO uint32_t RG_INTC;
} DMAMUX_TypeDef;

/** 
  * @brief External Interrupt/Event Controller
  */
typedef struct
{
  __IO uint32_t INTEN;
  __IO uint32_t EVEN;
  __IO uint32_t RTEN;
  __IO uint32_t FTEN;
  __IO uint32_t SWIEV;
  __IO uint32_t PD;
} EXTI_TypeDef;

/** 
  * @brief FLASH Registers
  */
typedef struct
{
  __IO uint32_t WS;
  __IO uint32_t KEY;
  __IO uint32_t OBKEY;
  __IO uint32_t STAT;
  __IO uint32_t CTL;
  __IO uint32_t ADDR;
#ifdef GD32L235
  __IO uint32_t ECCCS
#else
  __IO uint32_t RESERVED0;
#endif /* GD32L235 */
  __IO uint32_t OBSTAT;
  __IO uint32_t WP;
  __IO uint32_t SLPKEY;
  uint32_t RESERVED1[54];
  __IO  uint32_t PID;
} FLASH_TypeDef;

/** 
  * @brief Option Bytes Registers
  */
typedef struct
{
  __IO uint16_t SPC;
  __IO uint16_t USER;
  __IO uint16_t Data0;
  __IO uint16_t Data1;
  __IO uint16_t WP0;
  __IO uint16_t WP1;
  __IO uint16_t WP2;
  __IO uint16_t WP3;
} OB_TypeDef;

/** 
  * @brief General Purpose I/O
  */
typedef struct
{
  __IO uint32_t CTL;
  __IO uint32_t OMODE;
  __IO uint32_t OSPD;
  __IO uint32_t PUD;
  __IO uint32_t ISTAT;
  __IO uint32_t OCTL;
  __IO uint32_t BOP;
  __IO uint32_t LOCK;
  __IO uint32_t AFSEL0;
  __IO uint32_t AFSEL1;
  __IO uint32_t BC;
  __IO uint32_t TG;
} GPIO_TypeDef;

/** 
  * @brief Power Control
  */
typedef struct
{
  __IO uint32_t CTL0;
  __IO uint32_t CS;
  __IO uint32_t CTL1;
  __IO uint32_t STAT;
  __IO uint32_t PAR;
} PMU_TypeDef;

/**
  * @brief Reset and Clock Control
  */
typedef struct
{
  __IO uint32_t CTL;
  __IO uint32_t CFG0;
  __IO uint32_t INT;
  __IO uint32_t APB2RST;
  __IO uint32_t APB1RST;
  __IO uint32_t AHBEN;
  __IO uint32_t APB2EN;
  __IO uint32_t APB1EN;
  __IO uint32_t BDCTL;
  __IO uint32_t RSTSCK;
  __IO uint32_t AHBRST;
  __IO uint32_t CFG1;
  __IO uint32_t CFG2;
  __IO uint32_t AHB2EN;
  __IO uint32_t AHB2RST;
  uint32_t RESERVE0[49];
  __IO uint32_t VKEY;
  uint32_t RESERVE1[10];
  __IO uint32_t LPB;
} RCU_TypeDef;

/** 
  * @brief Serial Peripheral Interface
  */

typedef struct
{
  __IO uint32_t CTL0;
  __IO uint32_t CTL1;
  __IO uint32_t STAT;
  __IO uint32_t DATA;
  __IO uint32_t CRCPOLY;
  __IO uint32_t RCRC;
  __IO uint32_t TCRC;
  __IO uint32_t I2SCTL;
  __IO uint32_t I2SPSC;
  uint32_t RESERVE[23];
  __IO uint32_t QCTL;
} SPI_TypeDef;

/** 
  * @brief Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
  __IO uint32_t CTL0;        /*!< USART Control register 0,                Address offset: 0x00 */
  __IO uint32_t CTL1;        /*!< USART Control register 1,                Address offset: 0x04 */
  __IO uint32_t CTL2;        /*!< USART Control register 2,                Address offset: 0x08 */
  __IO uint32_t BAUD;        /*!< USART Baud rate register,                Address offset: 0x0C */
  __IO uint32_t GP;          /*!< USART Guard time and prescaler register, Address offset: 0x10 */
  __IO uint32_t RT;          /*!< USART receiver timeout register,         Address offset: 0x14 */
  __IO uint32_t CMD;         /*!< USART command register,                  Address offset: 0x18 */
  __IO uint32_t STAT;        /*!< USART status register,                   Address offset: 0x1C */
  __IO uint32_t INTC;        /*!< USART status clear register,             Address offset: 0x20 */
  __IO uint32_t RDATA;       /*!< USART receive data register,             Address offset: 0x24 */
  __IO uint32_t TDATA;        /*!< USART transmit data register,           Address offset: 0x28 */
  uint32_t RESERVE0[37];
  __IO uint32_t CHC;         /*!< USART coherence control register,        Address offset: 0xC0 */
  uint32_t RESERVE1[3];
  __IO uint32_t RFCS;        /*!< USART receive FIFO control and status  register, Address offset: 0xD0 */
} USART_TypeDef;

/** 
  * @brief Low-power Universal Synchronous Asynchronous Receiver Transmitter
  */
typedef struct
{
  __IO uint32_t CTL0;        /*!< USART Control register 0,                Address offset: 0x00 */
  __IO uint32_t CTL1;        /*!< USART Control register 1,                Address offset: 0x04 */
  __IO uint32_t CTL2;        /*!< USART Control register 2,                Address offset: 0x08 */
  __IO uint32_t BAUD;        /*!< USART Baud rate register,                Address offset: 0x0C */
  uint32_t RESERVE0[2];
  __IO uint32_t CMD;         /*!< USART command register,                  Address offset: 0x18 */
  __IO uint32_t STAT;        /*!< USART status register,                   Address offset: 0x1C */
  __IO uint32_t INTC;        /*!< USART status clear register,             Address offset: 0x20 */
  __IO uint32_t RDATA;       /*!< USART receive data register,             Address offset: 0x24 */
  __IO uint32_t TDATA;        /*!< USART transmit data register,           Address offset: 0x28 */
  uint32_t RESERVE1[37];
  __IO uint32_t CHC;         /*!< USART coherence control register,        Address offset: 0xC0 */
} LPUART_TypeDef;

/** 
  * @brief Inter Integrated Circuit Interface
  */
typedef struct
{
  __IO uint32_t CTL0;
  __IO uint32_t CTL1;
  __IO uint32_t SADDR0;
  __IO uint32_t SADDR1;
  __IO uint32_t TIMING;
  __IO uint32_t TIMEOUT;
  __IO uint32_t STAT;
  __IO uint32_t STATC;
  __IO uint32_t PEC;
  __IO uint32_t RDATA;
  __IO uint32_t TDATA;
  uint32_t RESERVED[25];
  __IO uint32_t CTL2;
} I2C_TypeDef;

/** 
  * @brief Window WATCHDOG
  */
typedef struct
{
  __IO uint32_t CTL;   /*!< WWDGT Control register, Address offset: 0x00 */
  __IO uint32_t CFG;   /*!< WWDGT Configuration register, Address offset: 0x04 */
  __IO uint32_t STAT;  /*!< WWDGT Status register, Address offset: 0x08 */
} WWDGT_TypeDef;

/** 
  * @brief Free WATCHDOG
  */
typedef struct
{
  __IO uint32_t CTL;           /*!< Key register,                                Address offset: 0x00 */
  __IO uint32_t PSC;           /*!< Prescaler register,                          Address offset: 0x04 */
  __IO uint32_t RLD;           /*!< Reload register,                             Address offset: 0x08 */
  __IO uint32_t STAT;          /*!< Status register,                             Address offset: 0x0C */
  __IO uint32_t WND;           /*!< Window register,                             Address offset: 0x10 */
} FWDGT_TypeDef;

/* enum definitions */
typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
typedef enum {RESET = 0, SET = !RESET} FlagStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;

/* bit operations */
#define REG64(addr)                  (*(volatile uint64_t *)(uint32_t)(addr))
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#define BIT(x)                       ((0x01U<<(x)))
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (end)))) 
#define GET_BITS(regval, start, end) (((uint32_t)(regval) & BITS((uint8_t)(start),(uint8_t)(end))) >> (uint8_t)(start))

/* main flash and SRAM memory map */
#define FLASH_BASE            ((uint32_t)0x08000000U)       /*!< main FLASH base address          */
#define SRAM_BASE             ((uint32_t)0x20000000U)       /*!< SRAM  base address               */
/* peripheral memory map */
#define APB1_BUS_BASE         ((uint32_t)0x40000000U)       /*!< apb1 base address                */
#define APB2_BUS_BASE         ((uint32_t)0x40010000U)       /*!< apb2 base address                */
#define AHB1_BUS_BASE         ((uint32_t)0x40020000U)       /*!< ahb1 base address                */
#define AHB2_BUS_BASE         ((uint32_t)0x48000000U)       /*!< ahb2 base address                */
/* advanced peripheral bus 1 memory map */
#define TIMER_BASE            (APB1_BUS_BASE + 0x00000000U) /*!< TIMER base address               */
#if defined(GD32L235)
#define TIMER0_BASE           (TIMER_BASE + 0x00012C00U)
#endif /* GD32L235 */
#define TIMER1_BASE           (TIMER_BASE + 0x00000000U)
#define TIMER2_BASE           (TIMER_BASE + 0x00000400U)
#define TIMER5_BASE           (TIMER_BASE + 0x00001000U)
#define TIMER6_BASE           (TIMER_BASE + 0x00001400U)
#define TIMER8_BASE           (TIMER_BASE + 0x00014C00U)
#define TIMER11_BASE          (TIMER_BASE + 0x00001800U)
#if defined(GD32L235) 
#define TIMER14_BASE          (TIMER_BASE + 0x00014000U)
#define TIMER40_BASE          (TIMER_BASE + 0x0001D000U)
#endif /* GD32L235 */
#define SLCD_BASE             (APB1_BUS_BASE + 0x00002400U) /*!< LCD base address                 */
#define RTC_BASE              (APB1_BUS_BASE + 0x00002800U) /*!< RTC base address                 */
#define WWDGT_BASE            (APB1_BUS_BASE + 0x00002C00U) /*!< WWDGT base address               */
#define FWDGT_BASE            (APB1_BUS_BASE + 0x00003000U) /*!< FWDGT base address               */
#define SPI_BASE              (APB1_BUS_BASE + 0x00003800U) /*!< SPI base address                 */
#define SPI0_BASE             (SPI_BASE + 0x0000F800U)
#define SPI1_BASE             (SPI_BASE + 0x00000000U)
#define USART_BASE            (APB1_BUS_BASE + 0x00004400U) /*!< USART base address               */
#define USART0_BASE           (USART_BASE + 0x0000F400U)
#define USART1_BASE           (USART_BASE + 0x00000000U)
#define UART3_BASE            (USART_BASE + 0x00000800U)
#define UART4_BASE            (USART_BASE + 0x00000C00U)
#define I2C_BASE              (APB1_BUS_BASE + 0x00005400U) /*!< I2C base address                 */
#define I2C0_BASE             (I2C_BASE + 0x00000000U)      /*!< I2C0 base address */
#define I2C1_BASE             (I2C_BASE + 0x00000400U)      /*!< I2C1 base address */
#define I2C2_BASE             (I2C_BASE + 0x00006C00U)      /*!< I2C2 base address */
#define USBD_BASE             (APB1_BUS_BASE + 0x00005C00U) /*!< USBD base address                */
#define USBD_RAM_BASE         (APB1_BUS_BASE + 0x00006000U) /*!< USBD RAM base address            */
#if defined(GD32L235) 
#define CAN_BASE              (APB1_BUS_BASE + 0x00006400U) /*!< CAN base address                 */
#endif /* GD32L235 */
#define PMU_BASE              (APB1_BUS_BASE + 0x00007000U) /*!< PMU base address                 */
#define DAC_BASE              (APB1_BUS_BASE + 0x00007400U) /*!< DAC base address                 */
#define LPUART_BASE           (APB1_BUS_BASE + 0x00008000U) /*!< LPUART base address              */
#define LPUART0_BASE          (LPUART_BASE + 0x00000000U)
#ifdef GD32L235
#define LPUART1_BASE          (LPUART_BASE - 0x00003800U)
#endif
#define LPTIMER_BASE          (APB1_BUS_BASE + 0x00009400U) /*!< LPUART base address              */
#ifdef GD32L233
#define LPTIMER_BASE          (LPTIMER_BASE + 0x00000000U)
#else
#define LPTIMER0_BASE         (LPTIMER_BASE + 0x00000000U)
#define LPTIMER1_BASE         (LPTIMER_BASE - 0x00001800U)
#endif /* GD32L233 */
#define CTC_BASE              (APB1_BUS_BASE + 0x0000C800U) /*!< LPUART base address              */
/* advanced peripheral bus 2 memory map */
#define SYSCFG_BASE           (APB2_BUS_BASE + 0x00000000U) /*!< SYSCFG base address              */
#define EXTI_BASE             (APB2_BUS_BASE + 0x00000400U) /*!< EXTI base address                */
#define ADC_BASE              (APB2_BUS_BASE + 0x00002400U) /*!< ADC base address                 */
#define CMP_BASE              (APB2_BUS_BASE + 0x00007C00U) /*!< CMP base address                 */
/* advanced high performance bus 1 memory map */
#define DMA_BASE              (AHB1_BUS_BASE + 0x00000000U) /*!< DMA base address                 */
#define DMA_CHANNEL_BASE      (DMA_BASE + 0x00000008U)      /*!< DMA channel base address         */
#define DMA_Channel0_BASE     (DMA_BASE + 0x00000008U)
#define DMA_Channel1_BASE     (DMA_BASE + 0x0000001CU)
#define DMA_Channel2_BASE     (DMA_BASE + 0x00000030U)
#define DMA_Channel3_BASE     (DMA_BASE + 0x00000044U)
#define DMA_Channel4_BASE     (DMA_BASE + 0x00000058U)
#define DMA_Channel5_BASE     (DMA_BASE + 0x0000006CU)
#define DMA_Channel6_BASE     (DMA_BASE + 0x00000080U)
#define DMAMUX_BASE           (AHB1_BUS_BASE + 0x00000800U) /*!< DMA base address                 */
#define RCU_BASE              (AHB1_BUS_BASE + 0x00001000U) /*!< RCU base address                 */
#define FMC_BASE              (AHB1_BUS_BASE + 0x00002000U) /*!< FMC base address                 */
#define CRC_BASE              (AHB1_BUS_BASE + 0x00003000U) /*!< CRC base address                 */
/* advanced high performance bus 2 memory map */
#define GPIO_BASE             (AHB2_BUS_BASE + 0x00000000U) /*!< GPIO base address                */
#define GPIOA_BASE            (GPIO_BASE + 0x00000000U)     /*!< GPIOA base address */
#define GPIOB_BASE            (GPIO_BASE + 0x00000400U)     /*!< GPIOB base address */
#define GPIOC_BASE            (GPIO_BASE + 0x00000800U)     /*!< GPIOC base address */
#define GPIOD_BASE            (GPIO_BASE + 0x00000C00U)     /*!< GPIOD base address */
#define GPIOF_BASE            (GPIO_BASE + 0x00001400U)     /*!< GPIOF base address */
#define CAU_BASE              (AHB2_BUS_BASE + 0x08060000U) /*!< CAU base address                 */
#define TRNG_BASE             (AHB2_BUS_BASE + 0x08060800U) /*!< TRNG base address                */
/* option byte and debug memory map */
#define OB_BASE               ((uint32_t)0x1FFFF800U)       /*!< OB base address                  */
#define DBG_BASE              ((uint32_t)0x40015800U)       /*!< DBG base address                 */

#define VREF_BASE             ((uint32_t)0x40010030U)       /*!< VREF base address                */

/* peripheral declaration */
// #define RTC                   ((RTC_TypeDef *)RTC_BASE)
#define WWDGT                 ((WWDGT_TypeDef *)WWDGT_BASE)
#define FWDGT                 ((FWDGT_TypeDef *)FWDGT_BASE)
#define SPI0                  ((SPI_TypeDef *)SPI0_BASE)
#define SPI1                  ((SPI_TypeDef *)SPI1_BASE)
#define USART0                ((USART_TypeDef *)USART0_BASE)
#define USART1                ((USART_TypeDef *)USART1_BASE)
#define UART3                 ((USART_TypeDef *)UART3_BASE)
#define UART4                 ((USART_TypeDef *)UART4_BASE)
#define I2C0                  ((I2C_TypeDef *)I2C0_BASE)
#define I2C1                  ((I2C_TypeDef *)I2C1_BASE)
#define I2C2                  ((I2C_TypeDef *)I2C2_BASE)
#if defined(GD32L235)
#define CAN                   ((CAN_TypeDef *)CAN0_BASE)
#endif /* GD32L235 */
#define PMU                   ((PMU_TypeDef *)PMU_BASE)
#define DAC                   ((DAC_TypeDef *)DAC_BASE)
#define LPUART0               ((LPUART_TypeDef *)LPUART0_BASE)
#if defined(GD32L235)
#define LPUART1               ((LPUART_TypeDef *)LPUART1_BASE)
#endif /* GD32L235 */
#define LPTIMER0              ((LPTIMER_TypeDef *)LPTIMER0_BASE)
#if defined(GD32L235)
#define LPTIMER1              ((LPTIMER_TypeDef *)LPTIMER1_BASE)
#endif /* GD32L235 */
// #define CTC                   ((CTC_TypeDef *)CTC_BASE)
// #define SYSCFG                ((SYSCFG_TypeDef *)SYSCFG_BASE)
#define EXTI                  ((EXTI_TypeDef *)EXTI_BASE)
#define ADC                   ((ADC_TypeDef *)ADC_BASE)
// #define CMP                   ((CMP_TypeDef *)CMP_BASE)
#define DMA                   ((DMA_TypeDef *)DMA_BASE)
#define DMA_Channel0          ((DMA_Channel_TypeDef *)DMA_Channel0_BASE)
#define DMA_Channel1          ((DMA_Channel_TypeDef *)DMA_Channel1_BASE)
#define DMA_Channel2          ((DMA_Channel_TypeDef *)DMA_Channel2_BASE)
#define DMA_Channel3          ((DMA_Channel_TypeDef *)DMA_Channel3_BASE)
#define DMA_Channel4          ((DMA_Channel_TypeDef *)DMA_Channel4_BASE)
#define DMA_Channel5          ((DMA_Channel_TypeDef *)DMA_Channel5_BASE)
#define DMA_Channel6          ((DMA_Channel_TypeDef *)DMA_Channel6_BASE)
#define DMAMUX                ((DMAMUX_TypeDef *)DMAMUX_BASE)
#define RCU                   ((RCU_TypeDef *)RCU_BASE)
#define FLASH                 ((FLASH_TypeDef *)FMC_BASE)
#define CRC                   ((CRC_TypeDef *)CRC_BASE)
#define GPIOA                 ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                 ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                 ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD                 ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOF                 ((GPIO_TypeDef *)GPIOF_BASE)
// #define CAU                   ((CAU_TypeDef *)CAU_BASE)
// #define TRNG                  ((TRNG_TypeDef *)TRNG_BASE)
#define OB                    ((OB_TypeDef *)OB_BASE)
#define DBGMCU                ((DBGMCU_TypeDef *)DBG_BASE)
// #define VREF                  ((VREF_TypeDef *)VREF_BASE)

#ifdef __cplusplus
}
#endif

#endif /* GD32L23X_H */
