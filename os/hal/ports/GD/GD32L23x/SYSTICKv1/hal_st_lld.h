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
 * @file    SYSTICKv1/hal_st_lld.h
 * @brief   ST Driver subsystem low level driver header.
 * @details This header is designed to be include-able without having to
 *          include other files from the HAL.
 *
 * @addtogroup ST
 * @{
 */

#ifndef HAL_ST_LLD_H
#define HAL_ST_LLD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   SysTick timer IRQ priority.
 */
#if !defined(GD32_ST_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define GD32_ST_IRQ_PRIORITY               3
#endif

/**
 * @brief   TIMx unit (by number) to be used for free running operations.
 * @note    You must select a 32 bits timer if a 32 bits @p systick_t type
 *          is required.
 * @note    Timers 0, 1, 2, 5, 6, 8, 11, 14, 40
 *          21 and 22 are supported.
 */
#if !defined(GD32_ST_USE_TIMER) || defined(__DOXYGEN__)
#define GD32_ST_USE_TIMER                  2
#endif

/**
 * @brief   Overrides the number of supported alarms.
 * @note    The default number of alarms is equal to the number of
 *          comparators in the timer, overriding it to one makes
 *          the driver a little faster and smaller. The kernel itself
 *          only needs one alarm, additional features could need more.
 * @note    Zero means do not override.
 * @note    This setting is only meaningful in free running mode, in
 *          tick mode there are no alarms.
 */
#if !defined(GD32_ST_OVERRIDE_ALARMS) || defined(__DOXYGEN__)
#define GD32_ST_OVERRIDE_ALARMS            1
#endif
/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/* This has to go after transition to shared handlers is complete for all
   platforms.*/
#if !defined(GD32_HAS_TIM0)
#define GD32_HAS_TIM0                      FALSE
#endif

#if !defined(GD32_HAS_TIM1)
#define GD32_HAS_TIM1                      FALSE
#endif

#if !defined(GD32_HAS_TIM2)
#define GD32_HAS_TIM2                      FALSE
#endif

#if !defined(GD32_HAS_TIM5)
#define GD32_HAS_TIM5                      FALSE
#endif

#if !defined(GD32_HAS_TIM6)
#define GD32_HAS_TIM6                      FALSE
#endif

#if !defined(GD32_HAS_TIM8)
#define GD32_HAS_TIM8                      FALSE
#endif

#if !defined(GD32_HAS_TIM11)
#define GD32_HAS_TIM11                     FALSE
#endif

#if !defined(GD32_HAS_TIM14)
#define GD32_HAS_TIM14                     FALSE
#endif

#if !defined(GD32_HAS_TIM40)
#define GD32_HAS_TIM40                     FALSE
#endif

/* End of checks to be removed.*/

#if OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING

#if GD32_ST_USE_TIMER == 0

#if defined(GD32_TIM0_IS_USED)
#error "ST requires TIM0 but the timer is already used"
#else
#define GD32_TIM0_IS_USED
#endif

#if defined(GD32_TIM0_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM0
#define ST_LLD_NUM_ALARMS                  GD32_TIM0_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   TRUE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 2

#if defined(GD32_TIM1_IS_USED)
#error "ST requires TIM1 but the timer is already used"
#else
#define GD32_TIM1_IS_USED
#endif

#if defined(GD32_TIM1_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM1
#define ST_LLD_NUM_ALARMS                   GD32_TIM1_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   TRUE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 2

#if defined(GD32_TIM2_IS_USED)
#error "ST requires TIM2 but the timer is already used"
#else
#define GD32_TIM2_IS_USED
#endif

#if defined(GD32_TIM2_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM2
#define ST_LLD_NUM_ALARMS                   GD32_TIM2_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   TRUE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 5

#if defined(GD32_TIM5_IS_USED)
#error "ST requires TIM5 but the timer is already used"
#else
#define GD32_TIM5_IS_USED
#endif

#if defined(GD32_TIM5_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM5
#define ST_LLD_NUM_ALARMS                   GD32_TIM5_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   TRUE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 6

#if defined(GD32_TIM6_IS_USED)
#error "ST requires TIM6 but the timer is already used"
#else
#define GD32_TIM6_IS_USED
#endif

#if defined(GD32_TIM6_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM6
#define ST_LLD_NUM_ALARMS                   GD32_TIM6_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   TRUE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 8

#if defined(GD32_TIM8_IS_USED)
#error "ST requires TIM8 but the timer is already used"
#else
#define GD32_TIM8_IS_USED
#endif

#if defined(GD32_TIM8_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM8
#define ST_LLD_NUM_ALARMS                   GD32_TIM8_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   TRUE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 11

#if defined(GD32_TIM11_IS_USED)
#error "ST requires TIM11 but the timer is already used"
#else
#define GD32_TIM11_IS_USED
#endif

#if defined(GD32_TIM11_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM11
#define ST_LLD_NUM_ALARMS                   GD32_TIM11_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  TRUE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 14

#if defined(GD32_TIM14_IS_USED)
#error "ST requires TIM14 but the timer is already used"
#else
#define GD32_TIM14_IS_USED
#endif

#if defined(GD32_TIM14_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM14
#define ST_LLD_NUM_ALARMS                   GD32_TIM14_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  TRUE
#define GD32_ST_USE_TIM40                  FALSE

#elif GD32_ST_USE_TIMER == 40

#if defined(GD32_TIM40_IS_USED)
#error "ST requires TIM40 but the timer is already used"
#else
#define GD32_TIM40_IS_USED
#endif

#if defined(GD32_TIM40_SUPPRESS_ISR)
#define GD32_SYSTICK_SUPPRESS_ISR
#endif

#define GD32_ST_TIM                        GD32_TIM40
#define ST_LLD_NUM_ALARMS                  GD32_TIM40_CHANNELS
#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  TRUE

#else
#error "GD32_ST_USE_TIMER specifies an unsupported timer"
#endif

#if (GD32_ST_OVERRIDE_ALARMS < 0) ||                                       \
    (GD32_ST_OVERRIDE_ALARMS > ST_LLD_NUM_ALARMS)
#error "invalid GD32_ST_OVERRIDE_ALARMS value"
#endif

#if GD32_ST_OVERRIDE_ALARMS > 0
#undef ST_LLD_NUM_ALARMS
#define ST_LLD_NUM_ALARMS                   GD32_ST_OVERRIDE_ALARMS
#endif

#elif OSAL_ST_MODE == OSAL_ST_MODE_PERIODIC

#define GD32_ST_USE_SYSTICK                TRUE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#else

#define GD32_ST_USE_SYSTICK                FALSE
#define GD32_ST_USE_TIM0                   FALSE
#define GD32_ST_USE_TIM1                   FALSE
#define GD32_ST_USE_TIM2                   FALSE
#define GD32_ST_USE_TIM5                   FALSE
#define GD32_ST_USE_TIM6                   FALSE
#define GD32_ST_USE_TIM8                   FALSE
#define GD32_ST_USE_TIM11                  FALSE
#define GD32_ST_USE_TIM14                  FALSE
#define GD32_ST_USE_TIM40                  FALSE

#endif

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
  void st_lld_init(void);
  void st_lld_serve_interrupt(void);
#ifdef __cplusplus
}
#endif

/*===========================================================================*/
/* Driver inline functions.                                                  */
/*===========================================================================*/

#if (OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING) || defined(__DOXYGEN__)

/**
 * @brief   Returns the time counter value.
 *
 * @return              The counter value.
 *
 * @notapi
 */
static inline systime_t st_lld_get_counter(void) {

  return (systime_t)GD32_ST_TIM->CNT;
}

/**
 * @brief   Starts the alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 *
 * @param[in] abstime   the time to be set for the first alarm
 *
 * @notapi
 */
static inline void st_lld_start_alarm(systime_t abstime) {

  GD32_ST_TIM->CHCV[0] = (uint32_t)abstime;
  GD32_ST_TIM->INTF     = 0;
#if ST_LLD_NUM_ALARMS == 1
  GD32_ST_TIM->DMAINTEN   = GD32_TIM_DIER_CC1IE;
#else
  GD32_ST_TIM->DMAINTEN  |= GD32_TIM_DIER_CC1IE;
#endif
}

/**
 * @brief   Stops the alarm interrupt.
 *
 * @notapi
 */
static inline void st_lld_stop_alarm(void) {

#if ST_LLD_NUM_ALARMS == 1
  GD32_ST_TIM->DMAINTEN = 0U;
#else
 GD32_ST_TIM->DMAINTEN &= ~GD32_TIM_DIER_CC1IE;
#endif
}

/**
 * @brief   Sets the alarm time.
 *
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm(systime_t abstime) {

  GD32_ST_TIM->CHCV[0] = (uint32_t)abstime;
}

/**
 * @brief   Returns the current alarm time.
 *
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm(void) {

  return (systime_t)GD32_ST_TIM->CHCV[0];
}

/**
 * @brief   Determines if the alarm is active.
 *
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active(void) {

  return (bool)((GD32_ST_TIM->DMAINTEN & GD32_TIM_DIER_CC1IE) != 0);
}

#if (ST_LLD_NUM_ALARMS > 1) || defined(__DOXYGEN__)
/**
 * @brief   Starts an alarm.
 * @note    Makes sure that no spurious alarms are triggered after
 *          this call.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] abstime   the time to be set for the first alarm
 * @param[in] alarm     alarm channel number
 *
 * @notapi
 */
static inline void st_lld_start_alarm_n(unsigned alarm, systime_t abstime) {

  GD32_ST_TIM->CHCV[alarm] = (uint32_t)abstime;
  GD32_ST_TIM->INTF         = 0;
  GD32_ST_TIM->DMAINTEN      |= (GD32_TIM_DIER_CC1IE << alarm);
}

/**
 * @brief   Stops an alarm interrupt.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 *
 * @notapi
 */
static inline void st_lld_stop_alarm_n(unsigned alarm) {

  GD32_ST_TIM->DMAINTEN &= ~(GD32_TIM_DIER_CC1IE << alarm);
}

/**
 * @brief   Sets an alarm time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 * @param[in] abstime   the time to be set for the next alarm
 *
 * @notapi
 */
static inline void st_lld_set_alarm_n(unsigned alarm, systime_t abstime) {

  GD32_ST_TIM->CHCV[alarm] = (uint32_t)abstime;
}

/**
 * @brief   Returns an alarm current time.
 * @note    This functionality is only available in free running mode, the
 *          behavior in periodic mode is undefined.
 *
 * @param[in] alarm     alarm channel number
 * @return              The currently set alarm time.
 *
 * @notapi
 */
static inline systime_t st_lld_get_alarm_n(unsigned alarm) {

  return (systime_t)GD32_ST_TIM->CHCV[alarm];
}

/**
 * @brief   Determines if an alarm is active.
 *
 * @param[in] alarm     alarm channel number
 * @return              The alarm status.
 * @retval false        if the alarm is not active.
 * @retval true         is the alarm is active
 *
 * @notapi
 */
static inline bool st_lld_is_alarm_active_n(unsigned alarm) {

  return (bool)((GD32_ST_TIM->DMAINTEN & (GD32_TIM_DIER_CC1IE << alarm)) != 0);
}
#endif /* ST_LLD_NUM_ALARMS > 1 */

#endif /* OSAL_ST_MODE == OSAL_ST_MODE_FREERUNNING */

#endif /* HAL_ST_LLD_H */

/** @} */
