/*
 * �ļ�����Rotunbot_vary.h
 * ���ߣ����߿�����
 * ��������������������Ҫ��ȫ�ֱ����������Լ��궨��
 *****************************************************************************
 * 			�궨�壺			ȫ��д��ĸ�����ʼ��»�������
 *			ϵͳ��������		��Դ��ĸȫ��д���������շ巨
 *			�Զ��庯������	��Сд��ĸ���������շ巨
 *			ȫ�ֱ�����		����ĸ��д�����ʼ��»������ӣ��Ǳ���ĸȫСд
 *			�ֲ�������		ȫСд��ĸ�����ʼ����»�������
 *****************************************************************************
 */

#ifndef __ROTUNBOT_VARY_H_
#define __ROTUNBOT_VARY_H_

/* ͷ�ļ� */
#include "Rotunbot_vary.h"
#include "Rotunbot_struct.h"

/* ���� */
const float PI = 3.1415926;
const float FirstAxis_Ratio = 0.002018*7/5; // ��ʽΪFirstAxis_Ratio = 2*pi/���ٱ�/60
const int SecondAxis_Ratio = 3770;      // ��ʽΪSecondAxis_Ratio = 2*pi/���ٱ�/60
const int ThirdAxis_Ratio = 15;         // ��ʽΪThirdAxis_Ratio = 2*pi/���ٱ�/60
// const float R40 0.2021
const float R60 = 0.3;
const float R80 = 0.405;
const float m_pendulum = 71;    // ������(����)/kg
const float m_ball = 155;       // ������/kg
const float l_pendulum = 0.276; // �ڳ���/m
// #define COUNTER 15

/* ������ر��� */
unsigned int Remote_Control_Counter = 200;                                   // �̿�Ƶ�����5Hz��ÿ10ms++������UDP�ж���գ�0~20��ʾ���ڳ̿�״̬
int Remote_Controllor_Mode = 2, Remote_Controllor_Mode_last = 0;             // �ٿ���ģʽ��������ԭ����SRC��=1��ʾ�̿أ�=2��ʾң��
int Velocity_Hope = 0, Position_Hope = 0, Roll_Hope = 0, W_Hope = 0, V_Hope; // Velocity_Hope��Ŀ���ٶȣ�Position_Hope��Ŀ��λ��
int Left_Pulse_Hope, Right_Pulse_Hope;                                       // ��������������ת��
float PID_Remote_Controllor = 0;                                             // ң��PIDĿ��λ��

/* ��Ϣ֡ */
struct Control_Message_TypeDef Control_Message;
struct IMU_Message_TypeDef IMU_Message;
struct IMU_CAN_Message_TypeDef IMU_CAN_Message;
char UDP_MessagetoSend[40]; // ���ϴ����ݻ�������
uint8_t Mesh_gun = 0x80;
/* ������ر�־λ */
char Flag_PID_FirstAxis = 0, Flag_PID_SecondAxis = 0, Flag_PID_ThirdAxis = 0; // �Ƿ�����������PID���Ʊ�־λ+������PID���Ʊ�־λ
char Flag_Feedback_Timer = 0;                                                 // Flag_Feedback_Timer��������־λ����1��ʾ����λ���ϴ������Ϣ
char Flag_PID_Control_Timer = 0;                                              // Flag_PID_Control_Timer�����Ʊ�־λ����1��ʾ����pid����
char Flag_Remote_Controllor_Timer = 0;                                        // Flag_Remote_Controllor_Timer��ң��PID��־λ����1��ʾ���ݻ�������һ��Ŀ��λ��
char Flag_LED_Toggle_Timer = 0;                                               // LED��˸��־λ����1��ʾLED��λ��Ҫ��ת
char Flag_Feedback_IMU = 0;
char FLag_Camera_Trigger = 0;
/* ������״̬���� */
int Velocity_FirstAxis, Velocity_SecondAxis, Velocity_ThirdAxis, Position_FirstAxis, Position_SecondAxis, Position_ThirdAxis; // �����ἰ������ʵ��ת��
short Current_FirstAxis, Current_SecondAxis, Current_ThirdAxis;
float Omega_ball, Velocity_ball, Angle_roll, Omega_roll, Alpha_roll, Omega_wheel; // ���α�ʾ����ǽ��ٶȡ����������ٶȡ������˺���Ƕ��Լ�������ٶ�/����Ǽ��ٶȡ������ֽ��ٶ�
float Odom_all, Odom_now, Odom_last, Arc_now, Delta_roll;                         // ���α�ʾ������̡���ǰ֡��̡���һ֡��̡���֡��̺��Լ�����Ԥ��ֵ
// ���������
short first, sec, third, left, right, v_force, w_force; // �������أ��������أ��ƽ���ǰ��/��ģ��������ģ����
/* ��������ر��� */
int Humidity, Temperature; // ��ʪ�ȴ���������ֵ��Humidity��ʾʪ�ȣ�Temperature��ʾ�¶�

/* PID��س��� */
PID_LocTypeDef PID_FirstAxis = {2, 3, 0, 0, 0, 0, 0, 400, 1000};         // ����PID
PID_LocTypeDef PID_SecondAxis = {0.08, 0.07, 0, 0, 0, 0, 0, 500, 1000}; // ����PID
PID_LocTypeDef PID_ThirdAxis = {1.3, 0.00, 0.6, 0, 0, 0, 0, 0, 900};     // ������PID
PID_LocTypeDef PID_W = {0.043, 0.002, 0.15, 0, 0, 0, 0, 20, 250};
float PIDBox_ThirdAxis[4][4] =
			{{1.0 , 1.2, 1.4, 2.2},
			 {0.0, 0.00, 0.00, 0},
			 {0.4 , 0.5, 0.6, 1.1},
			 {		0,    3,    6, 10}};
/****
  PID�ٶȷֶβ���
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
// ��Vn<|V|<V(n+1)������ΪKn~K(n+1)�����Բ�ֵ������V0=0��
/**************************************************************************************/
/* ���¶���ģ������Ӧ������ */
const int NB = -6;
const int NM = -4;
const int NS = -2;
const int ZO = 0;
const int PS = 2;
const int PM = 4;
const int PB = 6;

/* ģ��PID��س��� */
float qp = 0.02, qi = 0, qd = 0.01;          // Ӱ������
const float qValueMin = -50, qValueMax = 50; // PID������������
float deltaK[3];
const double PIDBox_P[7][7] = {{ZO, ZO, ZO, PB, PM, PS, ZO}, // Kpģ�������
                               {ZO, ZO, ZO, PM, PS, ZO, NS},
                               {ZO, ZO, ZO, PS, ZO, NS, NM},
                               {ZO, ZO, ZO, ZO, ZO, ZO, ZO},
                               {NM, NS, ZO, PS, ZO, ZO, ZO},
                               {NS, ZO, PS, PM, ZO, ZO, ZO},
                               {ZO, PS, PM, PB, ZO, ZO, ZO}};
const double PIDBox_I[7][7] = {{NB, NB, NM, NM, NS, ZO, ZO}, // Kiģ�������
                               {NB, NB, NM, NS, NS, ZO, ZO},
                               {NB, NM, NS, NS, ZO, PS, PS},
                               {NM, NM, NS, ZO, PS, PM, PM},
                               {NM, NS, ZO, PS, PS, PM, PB},
                               {ZO, ZO, PS, PS, PM, PB, PB},
                               {ZO, ZO, PS, PM, PM, PB, PB}};
const double PIDBox_D[7][7] = {{ZO, ZO, ZO, ZO, NM, NS, ZO}, // Kdģ�������
                               {ZO, ZO, ZO, ZO, NS, ZO, PS},
                               {ZO, ZO, ZO, ZO, ZO, PS, PM},
                               {PB, PM, PS, ZO, PS, PM, PB},
                               {PM, PS, ZO, ZO, ZO, ZO, ZO},
                               {PS, ZO, NS, ZO, ZO, ZO, ZO},
                               {ZO, NS, NM, ZO, ZO, ZO, ZO}};

/********************************************�� �� ��*****************************************/

/* CANͨ����ر��� */
uint32_t CAN_Flag = 0;        // ���ڱ�־�Ƿ���յ����ݣ����жϺ����и�ֵ
uint8_t Can_Rx_Buff[8];       // CAN��Ϣ�����ݴ�
CAN_TxHeaderTypeDef TxHeader; // ����ͷ
CAN_RxHeaderTypeDef RxHeader; // ����ͷ

uint8_t Can_Msg_Temp[8];               // ׼�����ص�Payload����������ݴ�
unsigned int Can_Msg_MDH, Can_Msg_MDL; // ����32bit�ļĴ��������׼�����ص�Can_Msg_Temp���������/ָ��

/* ��װ���� */
Tx_Packge my_Packge =
    {
        .TxHeader = &TxHeader,
};

/********************************************�� �� ��*****************************************/

#endif
