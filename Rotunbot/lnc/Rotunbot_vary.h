/*
* 文件名：Rotunbot_vary.h
* 作者：决策控制组
* 描述：包含整个工程需要的全局变量、常量以及宏定义
*****************************************************************************
* 			宏定义：		全大写字母，单词间下划线连接
*			系统函数名：	来源字母全大写，函数名驼峰法
*			自定义函数名：	全小写字母，单词间下划线连接
*			全局变量：		首字母大写，单词间下划线连接，角标字母全小写
*			局部变量：		全小写字母，单词间用下划线连接
*****************************************************************************
*/

#ifndef __ROTUNBOT_VARY_H_
#define __ROTUNBOT_VARY_H_

/* 头文件 */
#include "Rotunbot_struct.h"

//常量
extern const float PI;
extern const float FirstAxis_Ratio;	 					// 公式为FirstAxis_Ratio = 2*pi/减速比/60
extern const int SecondAxis_Ratio;						// 公式为SecondAxis_Ratio = 2*pi/减速比/60
extern const int ThirdAxis_Ratio;							// 公式为ThirdAxis_Ratio = 2*pi/减速比/60
extern const float R60;
extern const float R80;
extern const float m_pendulum;								// 摆重量
extern const float m_ball;										// 球重量
extern const float l_pendulum;								// 摆长度
	
/* 控制相关变量 */
extern unsigned int Remote_Control_Counter;
extern int Remote_Controllor_Mode, Remote_Controllor_Mode_last;		// 操控器模式变量，即原来的SRC：=1表示程控，=2表示遥控
extern int Velocity_Hope, Position_Hope, Roll_Hope, W_Hope, V_Hope;							// Velocity_Hope：目标速度；Position_Hope：目标位置
extern int Left_Pulse_Hope, Right_Pulse_Hope;						// 左右螺旋桨期望转速
extern float PID_Remote_Controllor;								// 遥控PID目标位置

/* 消息帧 */
extern struct Control_Message_TypeDef Control_Message;
extern struct IMU_Message_TypeDef IMU_Message;
extern struct IMU_CAN_Message_TypeDef IMU_CAN_Message;
extern char UDP_MessagetoSend[40];
extern uint8_t Mesh_gun;
/* 控制相关标志位 */
extern char Flag_PID_FirstAxis, Flag_PID_SecondAxis, Flag_PID_ThirdAxis;				// 是否开启主、副轴PID控制标志位
extern char Flag_Feedback_Timer;									// Flag_Feedback_Timer：反馈标志位，置1表示向上位机上传里程信息
extern char Flag_PID_Control_Timer;									// Flag_PID_Control_Timer：控制标志位，置1表示进行pid控制
extern char Flag_Remote_Controllor_Timer;							// Flag_Remote_Controllor_Timer：遥控PID标志位，置1表示根据滑窗计算一次目标位置
extern char Flag_Feedback_IMU;
extern char Flag_LED_Toggle_Timer;
extern char FLag_Camera_Trigger;
/* 机器人状态变量 */
extern int Velocity_FirstAxis, Velocity_SecondAxis, Velocity_ThirdAxis, Position_FirstAxis, Position_SecondAxis, Position_ThirdAxis;					// 主副轴及动量轮实际转速
extern short Current_FirstAxis, Current_SecondAxis, Current_ThirdAxis;
extern float Omega_ball, Velocity_ball, Angle_roll, Omega_roll, Alpha_roll, Omega_wheel;		// 依次表示：球壳角速度、机器人线速度、机器人横滚角度以及横滚角速度/横滚角加速度、动量轮角速度
extern float Odom_all, Odom_now, Odom_last, Arc_now, Delta_roll;	// 依次表示：总里程、当前帧里程、上一帧里程、两帧里程和以及航向预估值
//控制器输出
extern short first, sec, third, left, right, v_force, w_force; //主轴力矩，副轴力矩，推进器前向/共模推力，差模推力
/* 传感器相关变量 */
extern int Humidity, Temperature;									// 温湿度传感器测量值，Humidity表示湿度，Temperature表示温度

/* PID相关常量 */
extern PID_LocTypeDef PID_FirstAxis;								// 主轴PID
extern PID_LocTypeDef PID_SecondAxis;								// 副轴PID
extern PID_LocTypeDef PID_ThirdAxis;								// 动量轮PID
extern PID_LocTypeDef PID_W;												// 推进器偏航角角速度PID
extern float PIDBox_SecondAxis[4][5];						// pid值格表，用于记录不同速度下的副轴pid
extern float PIDBox_ThirdAxis[4][5];						// pid值格表，用于记录不同速度下的副轴pid

/**************************************************************************************/

/* 以下定义模糊集对应的论域 */
extern const int NB;
extern const int NM;
extern const int NS;
extern const int ZO;
extern const int PS;
extern const int PM;
extern const int PB;

/* 模糊PID相关常量 */
extern float qp, qi, qd;  						// 影响因子
extern const float qValueMin, qValueMax;  		// PID增量的上下限
extern float deltaK[3];
extern const double PIDBox_P[7][7];				// Kp模糊规则表
extern const double PIDBox_I[7][7];				// Ki模糊规则表
extern const double PIDBox_D[7][7];				// Kd模糊规则表

/**************************************************************************************/

/* CAN通信相关变量 */
extern __IO uint32_t CAN_Flag;		 			// 用于标志是否接收到数据，在中断函数中赋值			
extern uint8_t Can_Rx_Buff[8];					// CAN消息接收暂存
extern CAN_TxHeaderTypeDef TxHeader;			// 发送头
extern CAN_RxHeaderTypeDef RxHeader;			// 接收头				

extern uint8_t Can_Msg_Temp[8];					// 准备加载到Payload里面的数据暂存	
extern unsigned int Can_Msg_MDH, Can_Msg_MDL;	// 两个32bit的寄存器，存放准备加载到Can_Msg_Temp里面的数据/指令

/* 封装数据 */
extern Tx_Packge my_Packge;

/**************************************************************************************/

#endif
