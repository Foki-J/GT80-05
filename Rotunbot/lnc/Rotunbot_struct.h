/*
* 文件名：Rotunbot_struct.h
* 作者：决策控制组
* 描述：包含整个工程需要的结构体定义
*/

#ifndef __ROTUNBOT_STRUCT_H_
#define __ROTUNBOT_STRUCT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal_can.h"

/* 位置式PID结构体 */
typedef struct 
{
	float Kp;			// 比例系数Proportional
	float Ki;			// 积分系数Integral
	float Kd;			// 微分系数Derivative
	
	float Ek;			// 当前误差
	float Ek1;			// 前一次误差 e(k-1)
	float Ek2;			// 再前一次误差 e(k-2)
	float LocSum;		// 累计积分位置
	
	int MaxSum;     //积分饱和值
	int MaxResult;  //输出最大值
}PID_LocTypeDef;

/* 通信-控制消息 */
struct Control_Message_TypeDef
{
	unsigned int ord;			// 程控模式中控制方式变量
	unsigned int vel_1;			// 4个字节表示目标速度，(这里应该指传递消息时候四分变量)
	unsigned int vel_2;
	unsigned int vel_3;	
	unsigned int vel_4;
	unsigned int pos_1;			// 4个字节表示目标位置，(这里应该指传递消息时候四分变量)
	unsigned int pos_2;
	unsigned int pos_3;
	unsigned int pos_4;
	unsigned int acc_1; 		//2个字节表示加速度，(这里应该指传递消息时候四分变量)
	unsigned int acc_2;
	unsigned short int pid_switcher;		// pid选择变量
	unsigned short int pid_switcher_last;	// 上一个pid选择变量
};

/* 通信-IMU消息 */
struct IMU_Message_TypeDef
{
	float angle_roll;
	float omega_roll;
};

/* CAN-IMU消息 */
struct IMU_CAN_Message_TypeDef
{
	int16_t Acc_x;
	int16_t Acc_y;
	int16_t Acc_z;
	int16_t GyroI_Align_x;
	int16_t GyroI_Align_y;
	int16_t GyroI_Align_z;
	int16_t Mag_x;
	int16_t Mag_y;
	int16_t Mag_z;
	int16_t Euler_x;
	int16_t Euler_y;
	int16_t Euler_z;
	int16_t Quat_w;
	int16_t Quat_x;
	int16_t Quat_y;
	int16_t Quat_z;
};

/* 封装数据 */
typedef struct __Tx_Packge
{
	CAN_TxHeaderTypeDef *TxHeader;
	uint8_t Payload[8];
}Tx_Packge;


#endif /* __ROTUNBOT_STRUCT_H_ */
