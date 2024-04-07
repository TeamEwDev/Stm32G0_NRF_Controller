/**
  ******************************************************************************
  * @file    hw_timer.c
  * @brief   Source file for Hardware Timer
  * @author  EW Dev
  ******************************************************************************
  */


#include "hw_timer.h"

/**
  * @brief  Initialize the hardware timer with specified parameters.
  * @param  hTim: Pointer to the hardware timer handle
  * @param  clockFreqHZ: Desired timer clock frequency in Hz
  * @param  timChannel: Timer channel for PWM configuration
  * @retval HW_TIMER_RET_TYPE: Status of the timer initialization
  */
HW_TIMER_RET_TYPE HW_Timer_Init(HW_TIMER_STRUCT_TYPE *hTim, uint32_t clockFreqHZ, uint32_t timChannel)
{
    HW_TIMER_RET_TYPE ret = HAL_OK;
    uint32_t systemClockFreqHz = SystemCoreClock;
    uint32_t prescaler = 0;
    uint32_t period = 0;

    // Check if the timer handle is valid and clock frequency is within limits
    if (hTim == NULL || clockFreqHZ > HW_TIMER_FREQ_HZ)
    {
        return HAL_ERROR;
    }

    // Determine prescaler and period values based on clock frequency
    if (clockFreqHZ < (systemClockFreqHz / HW_TIMER_MAX_PERIOD_SIZE))
    {
        prescaler = HW_TIMER_CALCULATE_PRESCALER(HW_TIMER_FREQ_HZ);
        period = HW_TIMER_FREQ_HZ / clockFreqHZ;
    }
    else
    {
        prescaler = 0;
        period = systemClockFreqHz / clockFreqHZ;
    }

    // Configure timer initialization parameters
    hTim->Init.Prescaler = prescaler;
    hTim->Init.Period = period;

    // Initialize the timer in base and PWM mode
    if (HAL_TIM_Base_Init(hTim) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    if (HAL_TIM_PWM_Init(hTim) != HAL_OK)
    {
        ret = HAL_ERROR;
    }

    // Stop PWM on the specified channel
    HAL_TIM_PWM_Stop(hTim, timChannel);

    return ret;
}

/**
  * @brief  Set PWM duty cycle for the specified PWM timer.
  * @param  pwmTimer: Pointer to the PWM timer configuration structure
  * @retval HW_TIMER_RET_TYPE: Status of the PWM duty cycle configuration
  */
HW_TIMER_RET_TYPE HW_Timer_PWM_Duty_Cycle(HW_TIMER_TYPE *pwmTimer)
{
    HW_TIMER_RET_TYPE ret = HAL_OK;
    uint32_t scaledDuty = 0;
    uint32_t dutyWithOffset = 0;

    // Check if the timer and PWM timer output handle are valid
    if ((pwmTimer->hTim == NULL) || (pwmTimer->hPWMTimOut == NULL))
    {
        return HAL_ERROR;
    }

    // Calculate scaled duty cycle and duty with offset
    dutyWithOffset = pwmTimer->dutyCycle + pwmTimer->dutyOffset;
    scaledDuty = (float)pwmTimer->hTim->Init.Period * ((float)pwmTimer->dutyScale / (float)100);
    scaledDuty = scaledDuty * ((float)dutyWithOffset / (float)pwmTimer->dutyRefScale);

    // Calculate PWM pulse based on scaled duty and duty reference scale
    pwmTimer->hPWMTimOut->Pulse = scaledDuty;

    // Configure PWM channel on the timer
    ret = HAL_TIM_PWM_ConfigChannel(pwmTimer->hTim, pwmTimer->hPWMTimOut, pwmTimer->channel);

    // Start PWM output on the specified channel
    if (ret == HAL_OK)
    {
        HAL_TIM_PWM_Start(pwmTimer->hTim, pwmTimer->channel);
    }

    return ret;
}
