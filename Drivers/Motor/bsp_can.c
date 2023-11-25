/*
 * @Author: Yangyujian 2480383735@qq.com
 * @Date: 2023-11-22 18:37:35
 * @LastEditors: Yangyujian 2480383735@qq.com
 * @LastEditTime: 2023-11-25 17:28:40
 * @FilePath: \Motor_SV6001\Drivers\Motor\bsp_can.c
 * @Description: 
 */

#include "bsp_can.h"
#include "main.h"
 

typedef struct bsp_can
{
    /* data */
    uint8_t		REPLE;			//反馈字节
    uint8_t		ANGLE_L;		//角度低位
    uint8_t		ANGLE_H;		//角度高位
    uint8_t		VEL_L;			//速度低位
    uint8_t		VEL_H;			//速度高位
    uint8_t		CURRENT_L;		//电流低位
    uint8_t		CURRENT_H;		//电流高位
    uint8_t		STATUS;			//状态
}CAN_RxDataTypeDef;

CAN_RxDataTypeDef		rx_can_left_servo_data,rx_can_right_servo_data;				//舵机反馈数据
uint16_t				left_servo_feedback_angle, right_servo_feedback_angle;		//反馈角度
uint16_t				left_servo_feedback_vel, right_servo_feedback_vel;			//反馈速度
uint16_t				left_servo_send_angle, right_servo_send_angle;				//下发角度
int16_t					left_servo_difference_angle, right_servo_difference_angle;	//反馈角度与下发角度的差值
uint16_t				servo_allowable_error, servo_adjust_value;
float					servo_error_scale;
 /**
  * @name			HAL_CAN_RxFifo0MsgPendingCallback
  * @brief			Rx Fifo 0 message pending callback in non blocking mode
  * @param  		CanHandle: pointer to a CAN_HandleTypeDef structure that contains the configuration information for the specified CAN.
  * @retval 		无
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef   	RxHeader;
  	uint8_t					RxData[8] = {0};
  /*获取接收CAN1数据*/
  if(hcan -> Instance == CAN1)
  {
  	HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &RxHeader, RxData);
	if(RxHeader.StdId	==  LEFT_SERVO_ID && RxHeader.DLC	==	8)							//接收左舵机的反馈
	{
		rx_can_left_servo_data.REPLE		=	RxData[0];
		rx_can_left_servo_data.ANGLE_L		=	RxData[1];
		rx_can_left_servo_data.ANGLE_H		=	RxData[2];
		rx_can_left_servo_data.VEL_L		=	RxData[3];
		rx_can_left_servo_data.VEL_H		=	RxData[4];
		rx_can_left_servo_data.CURRENT_L	=	RxData[5];
		rx_can_left_servo_data.CURRENT_H	=	RxData[6];
		rx_can_left_servo_data.STATUS		=	RxData[7];
	}
	if(RxHeader.StdId	==	RIGHT_SERVO_ID && RxHeader.DLC	==	8)							//接收来自右舵机的反馈
	{
		rx_can_right_servo_data.REPLE		=	RxData[0];
		rx_can_right_servo_data.ANGLE_L		=	RxData[1];
		rx_can_right_servo_data.ANGLE_H		=	RxData[2];
		rx_can_right_servo_data.VEL_L		=	RxData[3];
		rx_can_right_servo_data.VEL_H		=	RxData[4];
		rx_can_right_servo_data.CURRENT_L	=	RxData[5];
		rx_can_right_servo_data.CURRENT_H	=	RxData[6];
		rx_can_right_servo_data.STATUS		=	RxData[7];
	}
  }
}
/**
 * @brief	舵机控制值初始化
 * 
 */
void servoAngle_Init(void)
{
	left_servo_send_angle 		=		DEFLECTION_ANFLE;			//左舵机下发值
	right_servo_send_angle		=		DEFLECTION_ANFLE;			//有舵机下发值
	servo_allowable_error		=		2;							//设置允许最大误差值
	servo_adjust_value			=		0x30;						//设置需要调整误差范围
	servo_error_scale  			=		2.0;						//设置误差比例

	SV601_setFeedback(&hcan1, LEFT_SERVO_ID, 50);      				//设置电机的反馈
  	SV601_setFeedback(&hcan1, RIGHT_SERVO_ID, 50);
}

/**
 * @brief	控制舵机的偏转角度以及速度
 * 
 * @param left_servo_angle		左舵机偏转角度
 * @param left_servo_vel		左舵机速度
 * @param right_servo_angle 	右舵机偏转角度
 * @param right_servo_vel 		右舵机速度
 */
void setServoAngle(int16_t left_servo_angle, uint16_t left_servo_vel, int16_t right_servo_angle, uint16_t right_servo_vel)
{
	left_servo_angle 	+=	DEFLECTION_ANFLE;
	left_servo_feedback_angle	=	(rx_can_left_servo_data.ANGLE_H << 8)  +	rx_can_left_servo_data.ANGLE_L;		//获得左舵机的反馈值
	left_servo_feedback_vel	=	(rx_can_left_servo_data.VEL_H << 8)	+	rx_can_left_servo_data.VEL_L;				//获得右舵机速度的反馈值
	left_servo_difference_angle =	left_servo_angle	-	left_servo_feedback_angle;								//获得左舵机当前值与下发值的差值
    if(left_servo_feedback_angle	>	left_servo_angle + servo_allowable_error || left_servo_feedback_angle < left_servo_angle - servo_allowable_error)	    //如果反馈的结果不等于下发值,则进入计算并再次下发指令
    {
      if((left_servo_feedback_angle >= left_servo_angle - servo_adjust_value) && (left_servo_feedback_angle <= left_servo_angle + servo_adjust_value)/* && (left_servo_feedback_vel < left_servo_vel / 2)*/)		
      {
		if(left_servo_difference_angle > 0)
		{
        	left_servo_send_angle	+=	ceil(left_servo_difference_angle /  servo_error_scale);					//大于零时向上取整
		}
		else
		{
        	left_servo_send_angle	+=	floor(left_servo_difference_angle /  servo_error_scale);					//小于零时向下取整
		}
      } 
      else
      {
        left_servo_send_angle	=	left_servo_angle;
      }
	    SV601_run(&hcan1, LEFT_SERVO_ID,left_servo_send_angle, left_servo_vel);
    }


	right_servo_angle	+=	DEFLECTION_ANFLE;
    right_servo_feedback_angle	=	(rx_can_right_servo_data.ANGLE_H << 8)  +	rx_can_right_servo_data.ANGLE_L;	//获得右舵机的反馈值
	right_servo_feedback_vel	=	(rx_can_right_servo_data.VEL_H << 8)	+	rx_can_right_servo_data.VEL_L;		//获得右舵机速度的反馈值
	right_servo_difference_angle	=	right_servo_angle - right_servo_feedback_angle;								//获得右舵机角度当前值与下发值的差值
    if(right_servo_feedback_angle >	right_servo_angle + servo_allowable_error || right_servo_feedback_angle < right_servo_angle - servo_allowable_error)
	{
      if((right_servo_feedback_angle >= right_servo_angle - servo_adjust_value) && (right_servo_feedback_angle <= right_servo_angle + servo_adjust_value)/* && (right_servo_feedback_vel < right_servo_vel / 2)*/)
      {
        if(right_servo_difference_angle > 0)
		{
        	right_servo_send_angle	+=	ceil(right_servo_difference_angle /  servo_error_scale);
		}
		else
		{
        	right_servo_send_angle	+=	floor(right_servo_difference_angle /  servo_error_scale);
		}
      } 
      else
      {
        right_servo_send_angle	=	right_servo_angle;
      }
	  SV601_run(&hcan1, RIGHT_SERVO_ID,right_servo_send_angle, right_servo_vel);
    }
}

/**************************舵机控制命令************************************/

/**
 * @brief SV601电机位置速度控制
 * 
 * @param hcan 
 * @param id 
 * @param pos 电机转动的位置, 0 ~ 0xFFF 
 * @param vel 电机转动的速度, 0 ~ 0xFFFF
 */
void SV601_run(CAN_HandleTypeDef* hcan, uint16_t id, uint16_t pos, uint16_t vel)
{
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t Data[5];
	Data[0] = 0x03;
	Data[1] = pos;
	Data[2] = pos >> 8;
	Data[3] = vel;
	Data[4] = vel >> 8;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 5;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}
/**
 * @brief 设置舵机零点
 * 
 * @param hcan 
 * @param id 
 * @param pos 位置, 范围0 ~ 0xFFF
 */
void SV601_setZerPos(CAN_HandleTypeDef* hcan, uint16_t id, int16_t pos)
{
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t Data[3];
	Data[0] = 0x0B;
	Data[1] = pos;
	Data[2] = pos >> 8;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 3;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}
/**
 * @brief 设置舵机加速度
 * 
 * @param hcan 
 * @param id 舵机id
 * @param acc 加速度, 范围0 ~ 20
 */
void SV601_setAcceleration(CAN_HandleTypeDef* hcan, uint16_t id, int16_t acc)
{
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t Data[2];
	Data[0] = 0x23;
	Data[1] = acc;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 2;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}
/**
 * @brief 设置舵机反馈
 * 
 * @param hcan 
 * @param id 
 * @param time 反馈周期
 */
void SV601_setFeedback(CAN_HandleTypeDef* hcan, uint16_t id, uint16_t time)
{
	uint32_t  TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	uint8_t Data[3];
	Data[0] = 0x19;
	Data[1] = time;
	Data[2] = time >> 8;
	TxHeader.StdId = id;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 3;
	HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &TxMailbox);
}

/**************************END OF FILE************************************/