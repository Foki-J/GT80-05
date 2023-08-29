/*
* �ļ�����Rotunbot_vary.h
* ���ߣ����߿�����
* ��������������������Ҫ��ȫ�ֱ����������Լ��궨��
*****************************************************************************
* 			�궨�壺		ȫ��д��ĸ�����ʼ��»�������
*			ϵͳ��������	��Դ��ĸȫ��д���������շ巨
*			�Զ��庯������	ȫСд��ĸ�����ʼ��»�������
*			ȫ�ֱ�����		����ĸ��д�����ʼ��»������ӣ��Ǳ���ĸȫСд
*			�ֲ�������		ȫСд��ĸ�����ʼ����»�������
*****************************************************************************
*/

#ifndef __ROTUNBOT_VARY_H_
#define __ROTUNBOT_VARY_H_

/* ͷ�ļ� */
#include "Rotunbot_struct.h"

//����
extern const float PI;
extern const float FirstAxis_Ratio;	 					// ��ʽΪFirstAxis_Ratio = 2*pi/���ٱ�/60
extern const int SecondAxis_Ratio;						// ��ʽΪSecondAxis_Ratio = 2*pi/���ٱ�/60
extern const int ThirdAxis_Ratio;							// ��ʽΪThirdAxis_Ratio = 2*pi/���ٱ�/60
extern const float R60;
extern const float R80;
extern const float m_pendulum;								// ������
extern const float m_ball;										// ������
extern const float l_pendulum;								// �ڳ���
	
/* ������ر��� */
extern unsigned int Remote_Control_Counter;
extern int Remote_Controllor_Mode, Remote_Controllor_Mode_last;		// �ٿ���ģʽ��������ԭ����SRC��=1��ʾ�̿أ�=2��ʾң��
extern int Velocity_Hope, Position_Hope, Roll_Hope, W_Hope, V_Hope;							// Velocity_Hope��Ŀ���ٶȣ�Position_Hope��Ŀ��λ��
extern int Left_Pulse_Hope, Right_Pulse_Hope;						// ��������������ת��
extern float PID_Remote_Controllor;								// ң��PIDĿ��λ��

/* ��Ϣ֡ */
extern struct Control_Message_TypeDef Control_Message;
extern struct IMU_Message_TypeDef IMU_Message;
extern struct IMU_CAN_Message_TypeDef IMU_CAN_Message;
extern char UDP_MessagetoSend[40];
extern uint8_t Mesh_gun;
/* ������ر�־λ */
extern char Flag_PID_FirstAxis, Flag_PID_SecondAxis, Flag_PID_ThirdAxis;				// �Ƿ�����������PID���Ʊ�־λ
extern char Flag_Feedback_Timer;									// Flag_Feedback_Timer��������־λ����1��ʾ����λ���ϴ������Ϣ
extern char Flag_PID_Control_Timer;									// Flag_PID_Control_Timer�����Ʊ�־λ����1��ʾ����pid����
extern char Flag_Remote_Controllor_Timer;							// Flag_Remote_Controllor_Timer��ң��PID��־λ����1��ʾ���ݻ�������һ��Ŀ��λ��
extern char Flag_Feedback_IMU;
extern char Flag_LED_Toggle_Timer;
extern char FLag_Camera_Trigger;
/* ������״̬���� */
extern int Velocity_FirstAxis, Velocity_SecondAxis, Velocity_ThirdAxis, Position_FirstAxis, Position_SecondAxis, Position_ThirdAxis;					// �����ἰ������ʵ��ת��
extern short Current_FirstAxis, Current_SecondAxis, Current_ThirdAxis;
extern float Omega_ball, Velocity_ball, Angle_roll, Omega_roll, Alpha_roll, Omega_wheel;		// ���α�ʾ����ǽ��ٶȡ����������ٶȡ������˺���Ƕ��Լ�������ٶ�/����Ǽ��ٶȡ������ֽ��ٶ�
extern float Odom_all, Odom_now, Odom_last, Arc_now, Delta_roll;	// ���α�ʾ������̡���ǰ֡��̡���һ֡��̡���֡��̺��Լ�����Ԥ��ֵ
//���������
extern short first, sec, third, left, right, v_force, w_force; //�������أ��������أ��ƽ���ǰ��/��ģ��������ģ����
/* ��������ر��� */
extern int Humidity, Temperature;									// ��ʪ�ȴ���������ֵ��Humidity��ʾʪ�ȣ�Temperature��ʾ�¶�

/* PID��س��� */
extern PID_LocTypeDef PID_FirstAxis;								// ����PID
extern PID_LocTypeDef PID_SecondAxis;								// ����PID
extern PID_LocTypeDef PID_ThirdAxis;								// ������PID
extern PID_LocTypeDef PID_W;												// �ƽ���ƫ���ǽ��ٶ�PID
extern float PIDBox_SecondAxis[4][5];						// pidֵ������ڼ�¼��ͬ�ٶ��µĸ���pid
extern float PIDBox_ThirdAxis[4][5];						// pidֵ������ڼ�¼��ͬ�ٶ��µĸ���pid

/**************************************************************************************/

/* ���¶���ģ������Ӧ������ */
extern const int NB;
extern const int NM;
extern const int NS;
extern const int ZO;
extern const int PS;
extern const int PM;
extern const int PB;

/* ģ��PID��س��� */
extern float qp, qi, qd;  						// Ӱ������
extern const float qValueMin, qValueMax;  		// PID������������
extern float deltaK[3];
extern const double PIDBox_P[7][7];				// Kpģ�������
extern const double PIDBox_I[7][7];				// Kiģ�������
extern const double PIDBox_D[7][7];				// Kdģ�������

/**************************************************************************************/

/* CANͨ����ر��� */
extern __IO uint32_t CAN_Flag;		 			// ���ڱ�־�Ƿ���յ����ݣ����жϺ����и�ֵ			
extern uint8_t Can_Rx_Buff[8];					// CAN��Ϣ�����ݴ�
extern CAN_TxHeaderTypeDef TxHeader;			// ����ͷ
extern CAN_RxHeaderTypeDef RxHeader;			// ����ͷ				

extern uint8_t Can_Msg_Temp[8];					// ׼�����ص�Payload����������ݴ�	
extern unsigned int Can_Msg_MDH, Can_Msg_MDL;	// ����32bit�ļĴ��������׼�����ص�Can_Msg_Temp���������/ָ��

/* ��װ���� */
extern Tx_Packge my_Packge;

/**************************************************************************************/

#endif
