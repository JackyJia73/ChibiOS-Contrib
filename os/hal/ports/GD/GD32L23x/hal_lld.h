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
 * @file    GD32L23x/hal_lld.h
 * @brief   GD32L23x HAL subsystem low level driver header.
 * @pre     This module requires the following macros to be defined in the
 *          @p board.h file:
 *          - GD32_LXTALCLK.
 *          - GD32_LXTAL_BYPASS (optionally).
 *          - GD32_HXTALCLK.
 *          - GD32_HXTAL_BYPASS (optionally).
 *
 * @addtogroup HAL
 * @{
 */

#ifndef HAL_LLD_H
#define HAL_LLD_H

#include "nvic.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    Platform identification
 * @{
 */
#if defined(__DOXYGEN__) || \
    defined(GD32L235RB) || defined(GD32L235CB) || defined(GD32L235KB) || defined(GD32L235EB)
  #define PLATFORM_NAME           "GigaDevice GD32L235 ARM M23"
  #define GD32L235
#else
  #error "unsupported or unrecognized GD32L23x member"
#endif

/**
 * @brief   Sub-family identifier.
 */
#if !defined(GD32L23x) || defined(__DOXYGEN__)
#define GD32L23x
#endif
/** @} */

/**
 * @name    Absolute Maximum Ratings
 * @{
 */
/**
 * @brief   Maximum system clock frequency.
 */
#define GD32_SYSCLK_MAX          64000000

/**
 * @brief   Maximum HXTAL clock frequency.
 */
#define GD32_HXTALCLK_MAX        32000000

/**
 * @brief   Minimum HXTAL clock frequency.
 */
#define GD32_HXTALCLK_MIN         4000000

/**
 * @brief   Maximum LXTAL clock frequency.
 */
#define GD32_LXTALCLK_MAX        1000000

/**
 * @brief   Minimum LXTAL clock frequency.
 */
#define GD32_LXTALCLK_MIN        32768

/**
 * @brief   Maximum PLL input clock frequency.
 */
#define GD32_PLLIN_MAX           48000000

/**
 * @brief   Minimum PLL input clock frequency.
 */
#define GD32_PLLIN_MIN            4000000

/**
 * @brief   Maximum PLL output clock frequency.
 */
#define GD32_PLLOUT_MAX           64000000

/**
 * @brief   Minimum PLL output clock frequency.
 */
#define GD32_PLLOUT_MIN           16000000

/**
 * @brief   Maximum APB1 clock frequency.
 */
#define GD32_PCLK1_MAX            32000000

/**
 * @brief   Maximum APB2 clock frequency.
 */
#define GD32_PCLK2_MAX            64000000

/**
 * @brief   Maximum ADC clock frequency.
 */
#define GD32_ADCCLK_MAX           14000000

/**
 * @name    RCU_CFG0 register bits definitions
 * @{
 */
// #define GD32_SCS_IRC16M          RCU_CKSYSSRC_IRC16M   /**< SYSCLK source is IRC16M.    */
// #define GD32_SCS_HXTAL           RCU_CKSYSSRC_HXTAL    /**< SYSCLK source is HXTAL.     */
// #define GD32_SCS_PLL             RCU_CKSYSSRC_PLL      /**< SYSCLK source is PLL.       */
// #define GD32_SCS_IRC48M          RCU_CKSYSSRC_IRC48M   /**< SYSCLK source is IRC48M.    */

// #define GD32_AHBPSC_DIV1         RCU_AHB_CKSYS_DIV1    /**< AHB prescaler select CK_SYS.       */
// #define GD32_AHBPSC_DIV2         RCU_AHB_CKSYS_DIV2    /**< AHB prescaler select CK_SYS/2.     */
// #define GD32_AHBPSC_DIV4         RCU_AHB_CKSYS_DIV4    /**< AHB prescaler select CK_SYS/4.     */
// #define GD32_AHBPSC_DIV8         RCU_AHB_CKSYS_DIV8    /**< AHB prescaler select CK_SYS/8.     */
// #define GD32_AHBPSC_DIV16        RCU_AHB_CKSYS_DIV16   /**< AHB prescaler select CK_SYS/16.    */
// #define GD32_AHBPSC_DIV64        RCU_AHB_CKSYS_DIV64   /**< AHB prescaler select CK_SYS/64.    */
// #define GD32_AHBPSC_DIV128       RCU_AHB_CKSYS_DIV128  /**< AHB prescaler select CK_SYS/128.   */
// #define GD32_AHBPSC_DIV256       RCU_AHB_CKSYS_DIV256  /**< AHB prescaler select CK_SYS/256.   */
// #define GD32_AHBPSC_DIV512       RCU_AHB_CKSYS_DIV512  /**< AHB prescaler select CK_SYS/512.    */

// #define GD32_APB1PSC_DIV1        RCU_APB1_CKAHB_DIV1    /**< APB1 prescaler select CK_AHB.         */
// #define GD32_APB1PSC_DIV2        RCU_APB1_CKAHB_DIV2    /**< APB1 prescaler select CK_AHB/2.       */
// #define GD32_APB1PSC_DIV4        RCU_APB1_CKAHB_DIV4    /**< APB1 prescaler select CK_AHB/4.       */
// #define GD32_APB1PSC_DIV8        RCU_APB1_CKAHB_DIV8    /**< APB1 prescaler select CK_AHB/8.       */
// #define GD32_APB1PSC_DIV16       RCU_APB1_CKAHB_DIV16   /**< APB1 prescaler select CK_AHB/16.      */

// #define GD32_APB2PSC_DIV1        RCU_APB2_CKAHB_DIV1    /**< APB2 prescaler select CK_AHB.         */
// #define GD32_APB2PSC_DIV2        RCU_APB2_CKAHB_DIV2    /**< APB2 prescaler select CK_AHB/2.       */
// #define GD32_APB2PSC_DIV4        RCU_APB2_CKAHB_DIV4    /**< APB2 prescaler select CK_AHB/4.       */
// #define GD32_APB2PSC_DIV8        RCU_APB2_CKAHB_DIV8    /**< APB2 prescaler select CK_AHB/8.       */
// #define GD32_APB2PSC_DIV16       RCU_APB2_CKAHB_DIV16   /**< APB2 prescaler select CK_AHB/16.      */

// #define GD32_ADCPSC_DIV2       (0 << 14)   /**< PPRE2 divided by 2.        */
// #define GD32_ADCPSC_DIV4       (1 << 14)   /**< PPRE2 divided by 4.        */
// #define GD32_ADCPSC_DIV6       (2 << 14)   /**< PPRE2 divided by 6.        */
// #define GD32_ADCPSC_DIV8       (3 << 14)   /**< PPRE2 divided by 8.        */
// #define GD32_ADCPSC_DIV12      ((1 << 28) | (1 << 14))  /**< PPRE2 divided by 12.        */
// #define GD32_ADCPSC_DIV16      ((1 << 28) | (3 << 14))  /**< PPRE2 divided by 16.        */

// #define GD32_PLLSEL_IRC16M     (0 << 16)   /**< PLL clock source is IRC16M.   */
// #define GD32_PLLSEL_PREDV0    (1 << 16)   /**< PLL clock source is PREDV0.  */

// #define GD32_USBDSEL_IRC48M    (0 << 13)      /**< USBD clock source is IRC48M.   */
// #define GD32_USBDSEL_PLL       (1 << 13)      /**< USBD clock source is CK_PLL.  */

// #define GD32_CKOUTSEL_NOCLOCK    (0 << 24)   /**< No clock on MCO pin.       */
// #define GD32_CKOUTSEL_SYSCLK     (4 << 24)   /**< SYSCLK on MCO pin.         */
// #define GD32_CKOUTSEL_IRC16M     (5 << 24)   /**< IRC16M clock on MCO pin.      */
// #define GD32_CKOUTSEL_HXTAL      (6 << 24)   /**< HXTAL clock on MCO pin.      */
// #define RCU_CKOUTSRC_CKPLL_DIV2    (7 << 24)   /**< PLL/2 clock on MCO pin.    */
// #define GD32_CKOUTSEL_XT1        (10 << 24)  /**< XT1 clock on MCO pin.      */

// /** @} */

// /**
//  * @name    RCU_BDCTL register bits definitions
//  * @{
//  */
// #define GD32_RTCSRC_MASK       (3 << 8)    /**< RTC clock source mask.     */
// #define GD32_RTCSRC_NOCLOCK    (0 << 8)    /**< No clock.                  */
// #define GD32_RTCSRC_LXTAL      (1 << 8)    /**< LXTAL used as RTC clock.     */
// #define GD32_RTCSRC_IRC32K     (2 << 8)    /**< IRC32K used as RTC clock.     */
// #define GD32_RTCSRC_HXTALDIV   (3 << 8)    /**< HXTAL divided by 32 used as RTC clock. */
// /** @} */

// /**
//  * @name    RCU_CFG02 register bits definitions
//  * @{
//  */
// #define GD32_PREDV0SEL_HXTAL    (0 << 16)   /**< PREDV0 source is HXTAL.     */
// /** @} */

// #define GD32_PLLMF_VALUE_6P5 65

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/*===========================================================================*/
/* a 48MHz system clock from
 *          a 8MHz crystal using PLL..                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Disables the PMU/RCU initialization in the HAL.
 */
#if !defined(GD32_NO_INIT) || defined(__DOXYGEN__)
#define GD32_NO_INIT               FALSE
#endif

/**
 * @brief   Enables or disables the programmable voltage detector.
 */
#if !defined(GD32_PVD_ENABLE) || defined(__DOXYGEN__)
#define GD32_PVD_ENABLE            FALSE
#endif

/**
 * @brief   Sets voltage level for programmable voltage detector.
 */
#if !defined(GD32_LVDT) || defined(__DOXYGEN__)
#define GD32_LVDT                   PMU_LVDT_0
#endif

/**
 * @brief   Enables or disables the IRC16M clock source.
 */
#if !defined(GD32_IRC16M_ENABLED) || defined(__DOXYGEN__)
#define GD32_IRC16M_ENABLED           TRUE
#endif

/**
 * @brief   Enables or disables the IRC16M clock source.
 */
#if !defined(GD32_IRC48M_ENABLED) || defined(__DOXYGEN__)
#define GD32_IRC48M_ENABLED           FALSE
#endif

/**
 * @brief   Enables or disables the IRC32K clock source.
 */
#if !defined(GD32_IRC32K_ENABLED) || defined(__DOXYGEN__)
#define GD32_IRC32K_ENABLED           FALSE
#endif

/**
 * @brief   Enables or disables the HXTAL clock source.
 */
#if !defined(GD32_HXTAL_ENABLED) || defined(__DOXYGEN__)
#define GD32_HXTAL_ENABLED           TRUE
#endif

/**
 * @brief   Enables or disables the LXTAL clock source.
 */
#if !defined(GD32_LXTAL_ENABLED) || defined(__DOXYGEN__)
#define GD32_LXTAL_ENABLED           FALSE
#endif
/** @} */

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   Main clock source selection.
 * @note    The default value is calculated for a 48MHz system clock from
 *          a 8MHz crystal using PLL.
 */
#if !defined(GD32_SCS) || defined(__DOXYGEN__)
#define GD32_SCS                    RCU_CKSYSSRC_PLL
#endif

/**
 * @brief   Clock source for the PLL.
 * @note    The default value is calculated for a 48MHz system clock from
 *          a 8MHz crystal using PLL.
 */
#if !defined(GD32_PLLSEL) || defined(__DOXYGEN__)
#define GD32_PLLSEL                RCU_PLLSRC_HXTAL
#endif

/**
 * @brief   PREDV0 division factor.
 * @note    The allowed range is 1...16.
 * @note    The default value is calculated for a 48MHz system clock from
 *          a 8MHz crystal using PLL.
 */
#if !defined(GD32_PREDV_VALUE) || defined(__DOXYGEN__)
#define GD32_PREDV_VALUE            RCU_PLL_PREDV2
#endif

/**
 * @brief   PLL multiplier value.
 * @note    The allowed range is 4...64.
 * @note    The default value is calculated for a 48MHz system clock from
 *          a 8MHz crystal using PLL.
 */
#if !defined(GD32_PLLMF_VALUE) || defined(__DOXYGEN__)
#define GD32_PLLMF_VALUE             RCU_PLL_MUL6
#endif

/**
 * @brief   AHB prescaler value.
 * @note    The default value is calculated for a 48MHz system clock from
 *          a 8MHz crystal using PLL.
 */
#if !defined(GD32_AHBPSC) || defined(__DOXYGEN__)
#define GD32_AHBPSC                  RCU_AHB_CKSYS_DIV1
#endif

/**
 * @brief   APB1 prescaler value.
 */
#if !defined(GD32_APB1PSC) || defined(__DOXYGEN__)
#define GD32_APB1PSC                  RCU_APB1_CKAHB_DIV1
#endif

/**
 * @brief   APB2 prescaler value.
 */
#if !defined(GD32_APB2PSC) || defined(__DOXYGEN__)
#define GD32_APB2PSC                  RCU_APB2_CKAHB_DIV1
#endif

/**
 * @brief   ADC prescaler value.
 */
#if !defined(GD32_ADCPSC) || defined(__DOXYGEN__)
#define GD32_ADCPSC                   RCU_ADCCK_APB2_DIV16
#endif

/**
 * @brief   USBD clock setting.
 */
#if !defined(GD32_USBD_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define GD32_USBD_CLOCK_REQUIRED      TRUE
#endif

/**
 * @brief   USBD clock setting.
 */
#if !defined(GD32_USBDCLK_SEL) || defined(__DOXYGEN__)
#define GD32_USBDCLK_SEL               RCU_USBDSRC_PLL
#endif

/**
 * @brief   Dedicated I2S clock setting.
 */
#if !defined(GD32_I2S_CLOCK_REQUIRED) || defined(__DOXYGEN__)
#define GD32_I2S_CLOCK_REQUIRED         FALSE
#endif

/**
 * @brief   Clock OUT pin setting.
 */
#if !defined(GD32_CKOUTSEL) || defined(__DOXYGEN__)
#define GD32_CKOUTSEL                  RCU_CKOUTSRC_NONE
#endif

/**
 * @brief   RTC clock source.
 */
#if !defined(GD32_RTCSRC) || defined(__DOXYGEN__)
#define GD32_RTCSRC                     RCU_RTCSRC_NONE
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*
 * Configuration-related checks.
 */
#if !defined(GD32L23x_MCUCONF)
#error "Using a wrong mcuconf.h file, GD32L23x_MCUCONF not defined"
#endif

/*
 * IRC16M related checks.
 */
#if GD32_IRC16M_ENABLED
#else /* !GD32_IRC16M_ENABLED */

#if GD32_SCS == RCU_CKSYSSRC_IRC16M
#error "IRC16M not enabled, required by GD32_SCS"
#endif

#if (GD32_SCS == RCU_CKSYSSRC_PLL) && (GD32_PLLSEL == RCU_PLLSRC_IRC16M)
#error "IRC16M not enabled, required by GD32_SCS and GD32_PLLSEL"
#endif

#if (GD32_CKOUTSEL == RCU_CKOUTSRC_IRC16M) ||                                   \
    ((GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV1) &&                              \
     (GD32_PLLSEL == RCU_PLLSRC_IRC16M)) ||
     ((GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV2) &&                              \
     (GD32_PLLSEL == RCU_PLLSRC_IRC16M))
#error "IRC16M not enabled, required by GD32_CKOUTSEL"
#endif

#endif /* !GD32_IRC16M_ENABLED */

/*
 * IRC48M related checks.
 */
#if GD32_IRC48M_ENABLED
#else /* !GD32_IRC16M_ENABLED */

#if GD32_SCS == RCU_CKSYSSRC_IRC48M
#error "IRC48M not enabled, required by GD32_SCS"
#endif

#if GD32_USBDCLK_SEL == RCU_USBDSRC_IRC48M
#error "IRC48M not enabled, required by GD32_USBD"
#endif

#if (GD32_SCS == RCU_CKSYSSRC_PLL) && (GD32_PLLSEL == RCU_PLLSRC_IRC48M)
#error "IRC48M not enabled, required by GD32_SCS and GD32_PLLSEL"
#endif

#if (GD32_CKOUTSEL == RCU_CKOUTSRC_IRC48M) ||                                   \
    ((GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV1) &&                              \
     (GD32_PLLSEL == RCU_PLLSRC_IRC48M)) ||
     ((GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV2) &&                              \
     (GD32_PLLSEL == RCU_PLLSRC_IRC48M))
#error "IRC48M not enabled, required by GD32_CKOUTSEL"
#endif

#endif /* !GD32_IRC48M_ENABLED */

/*
 * HXTAL related checks.
 */
#if GD32_HXTAL_ENABLED

#if GD32_HXTALCLK == 0
#error "HXTAL frequency not defined"
#elif (GD32_HXTALCLK < GD32_HXTALCLK_MIN) || (GD32_HXTALCLK > GD32_HXTALCLK_MAX)
#error "GD32_HXTALCLK outside acceptable range (GD32_HXTALCLK_MIN...GD32_HXTALCLK_MAX)"
#endif

#else /* !GD32_HXTAL_ENABLED */

#if GD32_SCS == RCU_CKSYSSRC_HXTAL
#error "HXTAL not enabled, required by GD32_SCS"
#endif

#if (GD32_CKOUTSEL == RCU_CKOUTSRC_HXTAL) ||                                   \
    ((GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV1) &&                              \
     (GD32_PLLSEL == RCU_PLLSRC_HXTAL)) ||
     ((GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV2) &&                              \
     (GD32_PLLSEL == RCU_PLLSRC_HXTAL))
#error "HXTAL not enabled, required by GD32_CKOUTSEL"
#endif

#if GD32_RTCSRC == RCU_RTCSRC_HXTAL_DIV32
#error "HXTAL not enabled, required by GD32_RTCSRC"
#endif

#endif /* !GD32_HXTAL_ENABLED */

/*
 * IRC32K related checks.
 */
#if GD32_IRC32K_ENABLED
#else /* !GD32_IRC32K_ENABLED */

#if GD32_RTCSRC == RCU_RTCSRC_IRC32K
#error "IRC32K not enabled, required by GD32_RTCSRC"
#endif

#endif /* !GD32_IRC32K_ENABLED */

/*
 * LXTAL related checks.
 */
#if GD32_LXTAL_ENABLED

#if (GD32_LXTALCLK == 0)
#error "LXTAL frequency not defined"
#endif

#if (GD32_LXTALCLK < GD32_LXTALCLK_MIN) || (GD32_LXTALCLK > GD32_LXTALCLK_MAX)
#error "GD32_LXTALCLK outside acceptable range (GD32_LXTALCLK_MIN...GD32_LXTALCLK_MAX)"
#endif

#else /* !GD32_LXTAL_ENABLED */

#if GD32_RTCSRC == RCU_RTCSRC_LXTAL
#error "LXTAL not enabled, required by GD32_RTCSRC"
#endif

#endif /* !GD32_LXTAL_ENABLED */

/* PLL activation conditions.*/
#if (GD32_USBD_CLOCK_REQUIRED == RCU_CKSYSSRC_PLL) ||                                             \
    (GD32_SCS == RCU_CKSYSSRC_PLL) ||                                           \
    (GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV1) ||                               \
    (GD32_CKOUTSEL == RCU_CKOUTSRC_CKPLL_DIV2) ||                               \
    defined(__DOXYGEN__)
/**
 * @brief   PLL activation flag.
 */
#define GD32_ACTIVATE_PLL         TRUE
#else
#define GD32_ACTIVATE_PLL         FALSE
#endif

/**
 * @brief   PLL input clock frequency.
 */
#if (GD32_PLLSEL == RCU_PLLSRC_IRC16M) || defined(__DOXYGEN__)
#define GD32_PLLCLKIN              IRC16M_VALUE
#elif GD32_PLLSEL == RCU_PLLSRC_IRC48M
#define GD32_PLLCLKIN              IRC48M_VALUE
#elif GD32_PLLSEL == RCU_PLLSRC_HXTAL
#define GD32_PLLCLKIN              GD32_HXTALCLK
#else
#error "invalid GD32_PLLSEL value specified"
#endif

/* PLL input frequency range check.*/
#if (GD32_PLLCLKIN < GD32_PLLIN_MIN) || (GD32_PLLCLKIN > GD32_PLLIN_MAX)
#error "GD32_PLLCLKIN outside acceptable range (GD32_PLLIN_MIN...GD32_PLLIN_MAX)"
#endif

/**
 * @brief   PLL output clock frequency.
 */
#if (GD32_PLLMF_VALUE < RCU_PLL_MUL16)
#define GD32_PLLCLKOUT             (GD32_PLLCLKIN / (GD32_PREDV_VALUE + 1) * ((GD32_PLLMF_VALUE >> 18) + 2) )
#elif (GD32_PLLMF_VALUE < RCU_PLL_MUL65)
#define GD32_PLLCLKOUT             (GD32_PLLCLKIN / (GD32_PREDV_VALUE + 1) * ((GD32_PLLMF_VALUE >> 18) + 1) )
#else
#define GD32_PLLCLKOUT             (GD32_PLLCLKIN / (GD32_PREDV_VALUE + 1) * ((((GD32_PLLMF_VALUE & RCU_CFG0_PLLMF) >> 18) |
                                   ((GD32_PLLMF_VALUE & RCU_CFG0_PLLMF_6) >> 21)) + 1))
#endif

/* PLL output frequency range check.*/
#if (GD32_PLLCLKOUT < GD32_PLLOUT_MIN) || (GD32_PLLCLKOUT > GD32_PLLOUT_MAX)
#error "GD32_PLLCLKOUT outside acceptable range (GD32_PLLOUT_MIN...GD32_PLLOUT_MAX)"
#endif

/**
 * @brief   System clock source.
 */
#if (GD32_SCS == RCU_CKSYSSRC_PLL) || defined(__DOXYGEN__)
#define GD32_SYSCLK                GD32_PLLCLKOUT
#elif (GD32_SCS == RCU_CKSYSSRC_IRC16M)
#define GD32_SYSCLK                IRC16M_VALUE
#elif (GD32_SCS == RCU_CKSYSSRC_IRC48M)
#define GD32_SYSCLK                IRC48M_VALUE
#elif (GD32_SCS == GD32_SCS_HXTAL)
#define GD32_SYSCLK                HXTAL_VALUE
#else
#error "invalid GD32_SCS value specified"
#endif

/* Check on the system clock.*/
#if GD32_SYSCLK > GD32_SYSCLK_MAX
#error "GD32_SYSCLK above maximum rated frequency (GD32_SYSCLK_MAX)"
#endif

/**
 * @brief   AHB frequency.
 */
#if (GD32_AHBPSC == RCU_AHB_CKSYS_DIV1) || defined(__DOXYGEN__)
#define GD32_HCLK                  (GD32_SYSCLK / 1)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV2
#define GD32_HCLK                  (GD32_SYSCLK / 2)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV4
#define GD32_HCLK                  (GD32_SYSCLK / 4)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV8
#define GD32_HCLK                  (GD32_SYSCLK / 8)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV16
#define GD32_HCLK                  (GD32_SYSCLK / 16)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV64
#define GD32_HCLK                  (GD32_SYSCLK / 64)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV128
#define GD32_HCLK                  (GD32_SYSCLK / 128)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV256
#define GD32_HCLK                  (GD32_SYSCLK / 256)
#elif GD32_AHBPSC == RCU_AHB_CKSYS_DIV512
#define GD32_HCLK                  (GD32_SYSCLK / 512)
#else
#error "invalid GD32_AHBPSC value specified"
#endif

/* AHB frequency check.*/
#if GD32_HCLK > GD32_SYSCLK_MAX
#error "GD32_HCLK exceeding maximum frequency (GD32_SYSCLK_MAX)"
#endif

/**
 * @brief   APB1 frequency.
 */
#if (GD32_APB1PSC == RCU_APB1_CKAHB_DIV1) || defined(__DOXYGEN__)
#define GD32_PCLK1                 (GD32_HCLK / 1)
#elif GD32_APB1PSC == RCU_APB1_CKAHB_DIV2
#define GD32_PCLK1                 (GD32_HCLK / 2)
#elif GD32_APB1PSC == RCU_APB1_CKAHB_DIV4
#define GD32_PCLK1                 (GD32_HCLK / 4)
#elif GD32_APB1PSC == RCU_APB1_CKAHB_DIV8
#define GD32_PCLK1                 (GD32_HCLK / 8)
#elif GD32_APB1PSC == RCU_APB1_CKAHB_DIV16
#define GD32_PCLK1                 (GD32_HCLK / 16)
#else
#error "invalid GD32_APB1PSC value specified"
#endif

/* APB1 frequency check.*/
#if GD32_PCLK1 > GD32_PCLK1_MAX
#error "GD32_PCLK1 exceeding maximum frequency (GD32_PCLK1_MAX)"
#endif

/**
 * @brief   APB2 frequency.
 */
#if (GD32_APB2PSC == RCU_APB2_CKAHB_DIV1) || defined(__DOXYGEN__)
#define GD32_PCLK2                 (GD32_HCLK / 1)
#elif GD32_APB2PSC == RCU_APB2_CKAHB_DIV2
#define GD32_PCLK2                 (GD32_HCLK / 2)
#elif GD32_APB2PSC == RCU_APB2_CKAHB_DIV4
#define GD32_PCLK2                 (GD32_HCLK / 4)
#elif GD32_APB2PSC == RCU_APB2_CKAHB_DIV8
#define GD32_PCLK2                 (GD32_HCLK / 8)
#elif GD32_APB2PSC == RCU_APB2_CKAHB_DIV16
#define GD32_PCLK2                 (GD32_HCLK / 16)
#else
#error "invalid GD32_APB2PSC value specified"
#endif

/* APB2 frequency check.*/
#if GD32_PCLK2 > GD32_PCLK2_MAX
#error "GD32_PCLK2 exceeding maximum frequency (GD32_PCLK2_MAX)"
#endif

/**
 * @brief   RTC clock.
 */
#if (GD32_RTCSRC == RCU_RTCSRC_LXTAL) || defined(__DOXYGEN__)
#define GD32_RTCCLK                GD32_LXTALCLK
#elif GD32_RTCSRC == RCU_RTCSRC_IRC32K
#define GD32_RTCCLK                IRC32K_VALUE
#elif GD32_RTCSRC == RCU_RTCSRC_HXTAL_DIV32
#define GD32_RTCCLK                (GD32_HXTALCLK / 32)
#elif GD32_RTCSRC == RCU_RTCSRC_NONE
#define GD32_RTCCLK                0
#else
#error "invalid source selected for RTC clock"
#endif

/**
 * @brief   ADC frequency.
 */
#if (GD32_ADCPSC == RCU_ADCCK_APB2_DIV2) || defined(__DOXYGEN__)
#define GD32_ADCCLK                (GD32_PCLK2 / 2)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV4
#define GD32_ADCCLK                (GD32_PCLK2 / 4)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV6
#define GD32_ADCCLK                (GD32_PCLK2 / 6)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV8
#define GD32_ADCCLK                (GD32_PCLK2 / 8)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV10
#define GD32_ADCCLK                (GD32_PCLK2 / 10)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV12
#define GD32_ADCCLK                (GD32_PCLK2 / 12)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV14
#define GD32_ADCCLK                (GD32_PCLK2 / 14)
#elif GD32_ADCPSC == RCU_ADCCK_APB2_DIV16
#define GD32_ADCCLK                (GD32_PCLK2 / 16)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV3
#define GD32_ADCCLK                (GD32_HCLK / 3)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV5
#define GD32_ADCCLK                (GD32_HCLK / 5)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV7
#define GD32_ADCCLK                (GD32_HCLK / 7)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV9
#define GD32_ADCCLK                (GD32_HCLK / 9)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV11
#define GD32_ADCCLK                (GD32_HCLK / 11)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV13
#define GD32_ADCCLK                (GD32_HCLK / 13)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV15
#define GD32_ADCCLK                (GD32_HCLK / 15)
#elif GD32_ADCPSC == RCU_ADCCK_AHB_DIV17
#define GD32_ADCCLK                (GD32_HCLK / 17)
#else
#error "invalid GD32_ADCPSC value specified"
#endif

/* ADC frequency check.*/
#if GD32_ADCCLK > GD32_ADCCLK_MAX
#error "GD32_ADCCLK exceeding maximum frequency (GD32_ADCCLK_MAX)"
#endif

/**
 * @brief   USBD frequency.
 */
#if (GD32_USBDCLK_SEL == RCU_USBDSRC_IRC48M) || defined(__DOXYGEN__)
#define GD32_USBDCLK                IRC48M_VALUE
#elif (GD32_USBDCLK_SEL == RCU_USBDSRC_PLL)
#define GD32_USBDCLK                GD32_PLLCLKOUT
#endif

/**
 * @brief   Timers 1, 2, 5, 6, 11 clock.
 */
#if (GD32_APB1PSC == RCU_APB1_CKAHB_DIV1) || defined(__DOXYGEN__)
#define GD32_TIMCLK1               (GD32_HCLK * 1)
#else
#define GD32_TIMCLK1               (GD32_HCLK * 2 / (GD32_APB1PSC >> 8U))
#endif

/**
 * @brief   Timers 0, 8, 14, 40 clock.
 */
#if (GD32_APB2PSC == RCU_APB2_CKAHB_DIV1) || defined(__DOXYGEN__)
#define GD32_TIMCLK2               (GD32_HCLK * 1)
#else
#define GD32_TIMCLK2               (GD32_HCLK * 2 / (GD32_APB2PSC >> 11U))
#endif

/**
 * @brief   Flash settings.
 */
#if (GD32_HCLK <= 21000000) || defined(__DOXYGEN__)
#define GD32_FLASHBITS             FMC_WAIT_STATE_0
#elif GD32_HCLK <= 42000000
#define GD32_FLASHBITS             FMC_WAIT_STATE_1
#else
#define GD32_FLASHBITS             FMC_WAIT_STATE_2
#endif
/** @} */

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#include "gd32_registry.h"

/* Various helpers.*/
#include "gd32_isr.h"
#include "gd32_rcu.h"
#include "gd32l23x_fmc.h"
#include "gd32l23x_gpio.h"
#include "gd32l23x_rcu.h"

#ifdef __cplusplus
extern "C" {
#endif
  void hal_lld_init(void);
  void gd32_clock_init(void);
#ifdef __cplusplus
}
#endif

#endif /* HAL_LLD_H */

/** @} */
