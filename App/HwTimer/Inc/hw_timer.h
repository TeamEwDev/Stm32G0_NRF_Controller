/*
 * hw_timer.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Akash
 */

#ifndef HW_TIMER_H_
#define HW_TIMER_H_

#include "stdint.h"
#include "stm32g0xx_hal.h"

// Define hardware timer return type as HAL_StatusTypeDef
#define HW_TIMER_RET_TYPE              HAL_StatusTypeDef

// Define hardware timer structure type as TIM_HandleTypeDef
#define HW_TIMER_STRUCT_TYPE           TIM_HandleTypeDef

// Define system clock frequency in Hz
#define HW_TIMER_SYSTEM_CLOCK_HZ       SystemCoreClock

// Macro to calculate prescaler value based on desired timer frequency
#define HW_TIMER_CALCULATE_PRESCALER(desiredFreqHz)   (uint32_t)(HW_TIMER_SYSTEM_CLOCK_HZ/desiredFreqHz) - 1

// Define maximum prescaler value for timer
#define HW_TIMER_MAX_PRESCALER_VAL     TIM_ICPSC_DIV8

// Define maximum number of steps for timer configuration
#define HW_TIMER_MAX_STEPS             100

// Macro to calculate step size based on period value
#define HW_TIMER_STEP_SIZE(periodValue) (float)(periodValue/HW_TIMER_MAX_STEPS)

// Define structure type for PWM timer output configuration
#define HW_TIMER_PWM_OUT_STRUCT_TYPE   TIM_OC_InitTypeDef

// Define maximum period size for the timer
#define HW_TIMER_MAX_PERIOD_SIZE       65535

// Define default timer frequency in Hz
#define HW_TIMER_FREQ_HZ               500000

// Structure to hold hardware timer configuration for PWM control
typedef struct
{
    HW_TIMER_STRUCT_TYPE *hTim;     // Pointer to hardware timer handle
    HW_TIMER_PWM_OUT_STRUCT_TYPE *hPWMTimOut;  // Pointer to PWM timer output handle
    uint32_t clockFreqHz;           // Desired clock frequency in Hz
    uint32_t channel;               // Timer channel for PWM output
    uint32_t dutyCycle;             // Desired PWM duty cycle
    uint32_t dutyScale;             // Scale factor for duty cycle calculation
    uint32_t dutyOffset;            // Offset value to adjust duty cycle
    uint32_t dutyRefScale;          // Reference scale for duty cycle calculation
} HW_TIMER_TYPE;

// Function prototypes
HW_TIMER_RET_TYPE HW_Timer_Init(HW_TIMER_STRUCT_TYPE *hTim, uint32_t clockFreqHZ, uint32_t timChannel);
HW_TIMER_RET_TYPE HW_Timer_PWM_Duty_Cycle(HW_TIMER_TYPE *pwmTimer);

#endif /* HW_TIMER_H_ */
