/*
 * 文件名：Rotunbot_vary.h
 * 作者：决策控制组
 * 描述：包含整个工程需要的全局变量、常量以及宏定义
 *****************************************************************************
 * 			宏定义：			全大写字母，单词间下划线连接
 *			系统函数名：		来源字母全大写，函数名驼峰法
 *			自定义函数名：	首小写字母，函数名驼峰法
 *			全局变量：		首字母大写，单词间下划线连接，角标字母全小写
 *			局部变量：		全小写字母，单词间用下划线连接
 *****************************************************************************
 */

#ifndef __ROTUNBOT_VARY_H_
#define __ROTUNBOT_VARY_H_

/* 头文件 */
#include "Rotunbot_vary.h"
#include "Rotunbot_struct.h"

/* 常量 */
const float PI = 3.1415926;
const float FirstAxis_Ratio = 0.002018*7/5; // 公式为FirstAxis_Ratio = 2*pi/减速比/60
const int SecondAxis_Ratio = 3770;      // 公式为SecondAxis_Ratio = 2*pi/减速比/60
const int ThirdAxis_Ratio = 15;         // 公式为ThirdAxis_Ratio = 2*pi/减速比/60
// const float R40 0.2021
const float R60 = 0.3;
const float R80 = 0.405;
const float m_pendulum = 71;    // 摆重量(含摆)/kg
const float m_ball = 155;       // 球重量/kg
const float l_pendulum = 0.276; // 摆长度/m
// #define COUNTER 15

/* 控制相关变量 */
unsigned int Remote_Control_Counter = 200;                                   // 程控频率最大5Hz，每10ms++，进入UDP中断清空，0~20表示处在程控状态
int Remote_Controllor_Mode = 2, Remote_Controllor_Mode_last = 0;             // 操控器模式变量，即原来的SRC：=1表示程控，=2表示遥控
int Velocity_Hope = 0, Position_Hope = 0, Roll_Hope = 0, W_Hope = 0, V_Hope; // Velocity_Hope：目标速度；Position_Hope：目标位置
int Left_Pulse_Hope, Right_Pulse_Hope;                                       // 左右螺旋桨期望转速
float PID_Remote_Controllor = 0;                                             // 遥控PID目标位置

/* 消息帧 */
struct Control_Message_TypeDef Control_Message;
struct IMU_Message_TypeDef IMU_Message;
struct IMU_CAN_Message_TypeDef IMU_CAN_Message;
char UDP_MessagetoSend[40]; // 待上传数据缓存数组
uint8_t Mesh_gun = 0x80;
/* 控制相关标志位 */
char Flag_PID_FirstAxis = 0, Flag_PID_SecondAxis = 0, Flag_PID_ThirdAxis = 0; // 是否开启主、副轴PID控制标志位+动量轮PID控制标志位
char Flag_Feedback_Timer = 0;                                                 // Flag_Feedback_Timer：反馈标志位，置1表示向上位机上传里程信息
char Flag_PID_Control_Timer = 0;                                              // Flag_PID_Control_Timer：控制标志位，置1表示进行pid控制
char Flag_Remote_Controllor_Timer = 0;                                        // Flag_Remote_Controllor_Timer：遥控PID标志位，置1表示根据滑窗计算一次目标位置
char Flag_LED_Toggle_Timer = 0;                                               // LED闪烁标志位，置1表示LED电位需要翻转
char Flag_Feedback_IMU = 0;
char FLag_Camera_Trigger = 0;
/* 机器人状态变量 */
int Velocity_FirstAxis, Velocity_SecondAxis, Velocity_ThirdAxis, Position_FirstAxis, Position_SecondAxis, Position_ThirdAxis; // 主副轴及动量轮实际转速
short Current_FirstAxis, Current_SecondAxis, Current_ThirdAxis;
float Omega_ball, Velocity_ball, Angle_roll, Omega_roll, Alpha_roll, Omega_wheel; // 依次表示：球壳角速度、机器人线速度、机器人横滚角度以及横滚角速度/横滚角加速度、动量轮角速度
float Odom_all, Odom_now, Odom_last, Arc_now, Delta_roll;                         // 依次表示：总里程、当前帧里程、上一帧里程、两帧里程和以及航向预估值
// 控制器输出
short first, sec, third, left, right, v_force, w_force; // 主轴力矩，副轴力矩，推进器前向/共模推力，差模推力
/* 传感器相关变量 */
int Humidity, Temperature; // 温湿度传感器测量值，Humidity表示湿度，Temperature表示温度

/* PID相关常量 */
PID_LocTypeDef PID_FirstAxis = {2, 3, 0, 0, 0, 0, 0, 400, 1000};         // 主轴PID
PID_LocTypeDef PID_SecondAxis = {0.08, 0.07, 0, 0, 0, 0, 0, 500, 1000}; // 副轴PID
PID_LocTypeDef PID_ThirdAxis = {1.3, 0.00, 0.6, 0, 0, 0, 0, 0, 900};     // 动量轮PID
PID_LocTypeDef PID_W = {0.043, 0.002, 0.15, 0, 0, 0, 0, 20, 250};
float PIDBox_ThirdAxis[4][4] =
			{{1.0 , 1.2, 1.4, 2.2},
			 {0.0, 0.00, 0.00, 0},
			 {0.4 , 0.5, 0.6, 1.1},
			 {		0,    3,    6, 10}};
/****
  PID速度分段参数
*****/
float PIDBox_SecondAxis[4][5] =
    {{0.12, 0.1, 0.08, 0.06, 0}, // Kp
     {0.02, 0.02, 0.01, 0, 0},   // Ki
     {0.0, 0.0, 0.0, 0.0, 0},    // Kd
     {0.0, 1.0, 2.0, 3.0, 6.0}}; // V
//	{Kp0, Kp1, Kp2...},
//	{Ki0, Ki1, Ki2...},
//	{Kd0, Kd1, Kd2...},
//	{V1, V2, V3...}
// 当Vn<|V|<V(n+1)，参数为Kn~K(n+1)的线性插值，其中V0=0。
/**************************************************************************************/
/* 以下定义模糊集对应的论域 */
const int NB = -6;
const int NM = -4;
const int NS = -2;
const int ZO = 0;
const int PS = 2;
const int PM = 4;
const int PB = 6;

/* 模糊PID相关常量 */
float qp = 0.02, qi = 0, qd = 0.01;          // 影响因子
const float qValueMin = -50, qValueMax = 50; // PID增量的上下限
float deltaK[3];
const double PIDBox_P[7][7] = {{ZO, ZO, ZO, PB, PM, PS, ZO}, // Kp模糊规则表
                               {ZO, ZO, ZO, PM, PS, ZO, NS},
                               {ZO, ZO, ZO, PS, ZO, NS, NM},
                               {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
                               {NM, NS, ZO, PS, ZO, ZO, ZO},
                               {NS, ZO, PS, PM, ZO, ZO, ZO},
                               {ZO, PS, PM, PB, ZO, ZO, ZO}};
const double PIDBox_I[7][7] = {{NB, NB, NM, NM, NS, ZO, ZO}, // Ki模糊规则表
                               {NB, NB, NM, NS, NS, ZO, ZO},
                               {NB, NM, NS, NS, ZO, PS, PS},
                               {NM, NM, NS, ZO, PS, PM, PM},
                               {NM, NS, ZO, PS, PS, PM, PB},
                               {ZO, ZO, PS, PS, PM, PB, PB},
                               {ZO, ZO, PS, PM, PM, PB, PB}};
const double PIDBox_D[7][7] = {{ZO, ZO, ZO, ZO, NM, NS, ZO}, // Kd模糊规则表
                               {ZO, ZO, ZO, ZO, NS, ZO, PS},
                               {ZO, ZO, ZO, ZO, ZO, PS, PM},
                               {PB, PM, PS, ZO, PS, PM, PB},
                               {PM, PS, ZO, ZO, ZO, ZO, ZO},
                               {PS, ZO, NS, ZO, ZO, ZO, ZO},
                               {ZO, NS, NM, ZO, ZO, ZO, ZO}};

/********************************************分 隔 符*****************************************/

/* CAN通信相关变量 */
uint32_t CAN_Flag = 0;        // 用于标志是否接收到数据，在中断函数中赋值
uint8_t Can_Rx_Buff[8];       // CAN消息接收暂存
CAN_TxHeaderTypeDef TxHeader; // 发送头
CAN_RxHeaderTypeDef RxHeader; // 接收头

uint8_t Can_Msg_Temp[8];               // 准备加载到Payload里面的数据暂存
unsigned int Can_Msg_MDH, Can_Msg_MDL; // 两个32bit的寄存器，存放准备加载到Can_Msg_Temp里面的数据/指令

/* 封装数据 */
Tx_Packge my_Packge =
    {
        .TxHeader = &TxHeader,
};

/********************************************分 隔 符*****************************************/

#endif
