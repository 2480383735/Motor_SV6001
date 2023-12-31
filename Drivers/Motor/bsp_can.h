/*
 * @Author: Yangyujian 2480383735@qq.com
 * @Date: 2023-11-22 18:37:35
 * @LastEditors: Yangyujian 2480383735@qq.com
 * @LastEditTime: 2023-11-27 10:07:55
 * @FilePath: \Motor_SV6001\Drivers\Motor\bsp_can.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __BSP_CAN__H
#define __BSP_CAN__H

#include "main.h"
#include "motor.h"
#include "can.h"

#define	LEFT_SERVO_ID	    0x06
#define	RIGHT_SERVO_ID	    0x05
#define DEFLECTION_ANFLE    0x800

void CanFeedback_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle);
void servoAngle_Init(void);
void setServoAngle(int16_t left_servo_angle, uint16_t left_vel, int16_t right_servo_angle, uint16_t right_vel);

void SV601_run(CAN_HandleTypeDef* hcan, uint16_t id, uint16_t pos, uint16_t vel);
void SV601_setZerPos(CAN_HandleTypeDef* hcan, uint16_t id, int16_t pos);
void SV601_setAcceleration(CAN_HandleTypeDef* hcan, uint16_t id, int16_t acc);
void SV601_setFeedback(CAN_HandleTypeDef* hcan, uint16_t id, uint16_t time);
#endif // !__BSP_CAN__H