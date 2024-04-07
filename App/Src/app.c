/*
 * app.c
 *
 *      Author: EW Dev
 */
/* Includes -------------------------------------------------------------------*/

#include "app.h"
#include "hw_timer.h"
#include "stm32g0xx_hal.h"
#include "nrf24.h"
#include <string.h>

HW_TIMER_TYPE DriveMotorTimer;
HW_TIMER_TYPE SteeringMotorTimer;

extern TIM_OC_InitTypeDef ConfigOcM0;
extern TIM_OC_InitTypeDef ConfigOcM1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim17;

APP_Radio_Packet_Type AppRadioPacket;

uint8_t TxAddress[] = {0xD7, 0xD7, 0xD7, 0xD7, 0xD7};
uint8_t RxAddress[] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};

void App_Init(void)
{
    if (App_Init_Drive_Motor_Timer(&DriveMotorTimer) != HAL_OK)
    {
        return;
    }

    if (App_Init_Steering_Motor_Timer(&SteeringMotorTimer) != HAL_OK)
    {
        return;
    }

    AppRadioPacket.motorDirection = APP_DRIVE_MOTOR_STOP;
    AppRadioPacket.motorSpeed = 0;
    AppRadioPacket.steering = 0;
    AppRadioPacket.headlight = false;

    NRF24_Init();

    NRF24_Config(APP_RADIO_CHANNEL, APP_PAYLOAD_SIZE);

    /* Set the device addresses */
    NRF24_Rx_Address(TxAddress);
    NRF24_Rx_Address(RxAddress);
}

HAL_StatusTypeDef App_Init_Drive_Motor_Timer(HW_TIMER_TYPE *driveMotorTimer)
{
    HAL_StatusTypeDef ret = HAL_OK;
    driveMotorTimer->hTim = &APP_DRIVER_MOTOR_TIMER;
    driveMotorTimer->hPWMTimOut = &APP_DRIVER_MOTOR_TIMER_PWM;
    driveMotorTimer->channel = APP_DRIVER_MOTOR_PWM_0_CHANNEL;
    driveMotorTimer->clockFreqHz = APP_DRIVER_MOTOR_TIMER_CLK_FREQ_HZ;
    driveMotorTimer->dutyCycle = 0;
    driveMotorTimer->dutyScale = 100;
    driveMotorTimer->dutyOffset = 0;
    driveMotorTimer->dutyRefScale = 100;

    ret = HW_Timer_Init(driveMotorTimer->hTim, driveMotorTimer->clockFreqHz, driveMotorTimer->channel);
    return ret;
}

HAL_StatusTypeDef App_Set_Drive_Motor_Speed(uint8_t dutyCyle, uint8_t channel)
{
    HAL_StatusTypeDef ret = HAL_OK;
    DriveMotorTimer.dutyCycle = dutyCyle;
    DriveMotorTimer.channel = channel;
    ret = HW_Timer_PWM_Duty_Cycle(&DriveMotorTimer);
    return ret;
}

HAL_StatusTypeDef App_Init_Steering_Motor_Timer(HW_TIMER_TYPE *steeringMotorTimer)
{
    HAL_StatusTypeDef ret = HAL_OK;
    steeringMotorTimer->hTim = &APP_STEERING_MOTOR_TIMER;
    steeringMotorTimer->hPWMTimOut = &APP_STERRING_MOTOR_TIMER_PWM;
    steeringMotorTimer->channel = APP_STEERING_MOTOR_PWM_CHANNEL;
    steeringMotorTimer->clockFreqHz = APP_STEERING_MOTOR_TIMER_CLK_FREQ_HZ;
    steeringMotorTimer->dutyCycle = 0;
    steeringMotorTimer->dutyScale = 10;
    steeringMotorTimer->dutyOffset = 48;
    steeringMotorTimer->dutyRefScale = 180;

    ret = HW_Timer_Init(steeringMotorTimer->hTim, steeringMotorTimer->clockFreqHz, steeringMotorTimer->channel);
    return ret;
}

HAL_StatusTypeDef App_Set_Steering_Motor_Speed(uint8_t dutyCyle)
{
    HAL_StatusTypeDef ret = HAL_OK;
    SteeringMotorTimer.dutyCycle = dutyCyle;

    ret = HW_Timer_PWM_Duty_Cycle(&SteeringMotorTimer);
    return ret;
}

void App_Main(void)
{
    APP_Radio_Packet_Type appRadioPacketPrevious = AppRadioPacket;
    while (1)
    {
        //HAL_SuspendTick();
        //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
        //HAL_ResumeTick();
        if (APP_CMP_PKT(appRadioPacketPrevious, AppRadioPacket))
        {
            appRadioPacketPrevious = AppRadioPacket;
            App_Car_Function(&AppRadioPacket);
        }
    }
}

void App_Car_Function(APP_Radio_Packet_Type *packet)
{
    if (packet->motorDirection == APP_DRIVE_MOTOR_FORWARED_DIRECTION)
    {
        App_Set_Drive_Motor_Speed(0, APP_DRIVER_MOTOR_PWM_0_CHANNEL);
        App_Set_Drive_Motor_Speed(packet->motorSpeed, APP_DRIVER_MOTOR_PWM_1_CHANNEL);
    }
    else if (packet->motorDirection == APP_DRIVE_MOTOR_BACKWARD_DIRECTION)
    {
        App_Set_Drive_Motor_Speed(packet->motorSpeed, APP_DRIVER_MOTOR_PWM_0_CHANNEL);
        App_Set_Drive_Motor_Speed(0, APP_DRIVER_MOTOR_PWM_1_CHANNEL);
    }
    else
    {
        App_Set_Drive_Motor_Speed(0, APP_DRIVER_MOTOR_PWM_0_CHANNEL);
        App_Set_Drive_Motor_Speed(0, APP_DRIVER_MOTOR_PWM_1_CHANNEL);
    }

    App_Set_Steering_Motor_Speed(packet->steering);
    APP_TURN_ON_OFF_HEAD_LIGHT(AppRadioPacket.headlight & 0x01);
    APP_TURN_ON_OFF_BREAK_LIGHT(AppRadioPacket.headlight & 0x02);
}

HAL_StatusTypeDef App_Get_Rx_Packet(APP_Radio_Packet_Type *packet)
{
    uint8_t rxBuffer[sizeof(APP_Radio_Packet_Type)];
    if (NRF24_Data_Ready())
    {
        NRF24_Get_Data(rxBuffer);
        packet->motorDirection = rxBuffer[0];
        packet->motorSpeed = rxBuffer[1];
        packet->steering = rxBuffer[2];
        packet->headlight = rxBuffer[3] & 0x01;
        return HAL_OK;
    }

    return HAL_TIMEOUT;
}

void App_Get_Rx_Packet_Callback(void)
{
    uint8_t rxBuffer[sizeof(APP_Radio_Packet_Type)];
    NRF24_Get_Data(rxBuffer);
    AppRadioPacket.motorDirection = rxBuffer[0];
    AppRadioPacket.motorSpeed = rxBuffer[1];
    AppRadioPacket.steering = rxBuffer[2];
    AppRadioPacket.headlight = rxBuffer[3] & 0x01;
}
