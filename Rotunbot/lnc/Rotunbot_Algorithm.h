/**
  *****************************************************************************
  * @file    Rotunbot_Algorithm.h
  * @author  ���߿�����
  * @version V1.0.0
  * @date    2022-05-09
  * @brief   ������п�������㷨���롢����һ�������ĳ�ʼ��
  *****************************************************************************
  * @history
  *
  * 1. Date:2022-05-09
  *    Author:�ְ���
  *    Modification:�����ļ�
  *
  *****************************************************************************
  */
  
#ifndef __ROTUNBOT_ALGORITHM_H_
#define __ROTUNBOT_ALGORITHM_H_


#include "Rotunbot_vary.h"
#include "Rotunbot_Comm.h"
#include "Rotunbot_Can.h"
#include "math.h"

void programControl(void);

float PID_Loc(float set_value, float actual_value, float value_gradient, PID_LocTypeDef *PID);
void firstAxis_PID(void);
void secondAxis_PID(void);
void PID_Switch(unsigned int switch_order);

static void linearQuantization(float set_value, float actual_value,float angular_velocity,float *qValue);
static void calcMembership(float *ms, float qv, int *index);
static void fuzzyComputation(PID_LocTypeDef *PID);
double courseEstimate(float course_speed, double course_roll, double course_wheel);

double mySin(double x);
double myCos(double y);
void paramInit(void);
void odomfeedback(void);



#endif /* __ROTUNBOT_ALGORITHM_H_ */
