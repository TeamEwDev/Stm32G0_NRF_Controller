/*
 * app.h
 *
 *  Created on: Apr 6, 2024
 *      Author: Akash
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "hw_timer.h"
#include "stdbool.h"

#define APP_DRIVER_MOTOR_TIMER                   htim3
#define APP_DRIVER_MOTOR_PWM_0_CHANNEL           TIM_CHANNEL_1
#define APP_DRIVER_MOTOR_PWM_1_CHANNEL           TIM_CHANNEL_3

#define APP_DRIVER_MOTOR_TIMER_PWM               ConfigOcM0
#define APP_DRIVER_MOTOR_TIMER_CLK_FREQ_HZ       350

#define APP_STEERING_MOTOR_TIMER                 htim17
#define APP_STEERING_MOTOR_PWM_CHANNEL           TIM_CHANNEL_1
#define APP_STERRING_MOTOR_TIMER_PWM             ConfigOcM1
#define APP_STEERING_MOTOR_TIMER_CLK_FREQ_HZ     50

#define APP_PAYLOAD_SIZE                         sizeof(APP_Radio_Packet_Type)
#define APP_RADIO_CHANNEL                        2

#define APP_DRIVE_MOTOR_STOP                     0
#define APP_DRIVE_MOTOR_FORWARED_DIRECTION       1
#define APP_DRIVE_MOTOR_BACKWARD_DIRECTION       2

#define APP_TURN_ON_OFF_HEAD_LIGHT(state)        HAL_GPIO_WritePin(CAR_HEADLIGHT_GPIO_Port, CAR_HEADLIGHT_Pin, state)
#define APP_TURN_ON_OFF_BREAK_LIGHT(state)       HAL_GPIO_WritePin(CAR_BREAKLIGHT_GPIO_Port, CAR_BREAKLIGHT_Pin, state)

#define APP_CMP_PKT(data0, data1)                                   \
    (data0.motorDirection != data1.motorDirection)  ||             \
    (data0.motorSpeed != data1.motorSpeed)         ||             \
    (data0.steering != data1.steering)            ||             \
    (data0.headlight != data1.headlight)


typedef struct
{
    uint8_t motorDirection;
    uint8_t motorSpeed;
    uint8_t steering;
    bool headlight;
} APP_Radio_Packet_Type;

void App_Init(void);
void App_Main(void);
void App_Get_Rx_Packet_Callback(void);
void App_Car_Function(APP_Radio_Packet_Type *packet);
HAL_StatusTypeDef App_Get_Rx_Packet(APP_Radio_Packet_Type *packet);
HAL_StatusTypeDef App_Init_Drive_Motor_Timer(HW_TIMER_TYPE *driveMotorTimer);
HAL_StatusTypeDef App_Set_Drive_Motor_Speed(uint8_t dutyCyle, uint8_t channel);
HAL_StatusTypeDef App_Init_Steering_Motor_Timer(HW_TIMER_TYPE *steeringMotorTimer);
HAL_StatusTypeDef App_Set_Steering_Motor_Speed(uint8_t dutyCyle);



#endif /* INC_APP_H_ */
