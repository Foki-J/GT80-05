/**
  *****************************************************************************
  * @file    Rotunbot_Pwm.h
  * @author  决策控制组
  * @version V1.0.0
  * @date    2022-04-28
  * @brief   PWM输出功能（主要应用于螺旋桨），PWM捕获功能（主要应用于遥控器）
  *****************************************************************************
  * @history
  *
  * 1. Date:2022-04-28
  *    Author:林柏羽
  *    Modification:创建文件
  *
  *****************************************************************************
  */
  
#ifndef ROTUNBOT_PWM_H_
#define ROTUNBOT_PWM_H_

#include "tim.h"
#include "usart.h"
#include "Rotunbot_vary.h"
#include "Rotunbot_Can.h"
#include "Rotunbot_Comm.h"
#define TIM_IC_BUFFSIZE 10
extern unsigned int Signal_1_High, Signal_2_High, Signal_3_High;
extern unsigned int Signal_3_State;

void pwmInit(void);
void RemoteRx(void);

#endif /* ROTUNBOT_PWM_H_ */
