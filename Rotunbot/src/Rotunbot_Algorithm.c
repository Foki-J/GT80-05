/**
 *****************************************************************************
 * @file    Rotunbot_Algorithm.c
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

#include "Rotunbot_Algorithm.h"

/* �ڲ����� */

/***********************************************
�������ƣ�PID_Loc
��	�ܣ�λ��ʽPID����
��	����set_value-----�趨ֵ
        actual_value-----ʵ��ֵ
        value_gradient-----ʵ��ֵ�仯��
        PID-----PID���ݽṹ
��	�أ�PID_Loc ------PIDλ��ʽ������
��	ע����
************************************************/
float PID_Loc(float set_value, float actual_value, float value_gradient, PID_LocTypeDef *PID)
{
    float PIDLoc;
    PID->Ek2 = PID->Ek1;
    PID->Ek1 = PID->Ek;
    PID->Ek = set_value - actual_value;
    PID->LocSum += (PID->Ki * PID->Ek * 0.01f);
    PID->LocSum = ((PID->LocSum) > (PID->MaxSum)) ? (PID->MaxSum) : (PID->LocSum);
    PID->LocSum = ((PID->LocSum) < (-PID->MaxSum)) ? (-(PID->MaxSum)) : (PID->LocSum);
    PIDLoc = (PID->Kp * PID->Ek) + (PID->LocSum) + (PID->Kd * value_gradient);
    PIDLoc = (PIDLoc > (PID->MaxResult)) ? (PID->MaxResult) : (PIDLoc);
    PIDLoc = (PIDLoc < (-(PID->MaxResult))) ? (-(PID->MaxResult)) : (PIDLoc);
    return PIDLoc;
}

/***********************************************
�������ƣ�PIDSwitch
��	�ܣ��������ٶȲ�ͬʱ���������ָ�����PID����
��	����switch_order-----PID����ָ��(0x01��ʾʹ�õ�һ��PID�������Դ�����)
��	�أ���
��	ע��PID�����ı�ʱ����������ҲҪ����Ӧ�任�Ա�֤Ki*LocSum��ֵ����
************************************************/

/***********************************************
�������ƣ�linearQuantization
��	�ܣ�ģ��PID���Ӻ���֮һ������Ϊ��������������ͨ������ӳ�佫������������Լ����ٶȣ�ӳ�䵽-6��6֮��
��	����set_value-----Ŀ��ֵ
        actual_value-----��ǰֵ
        angular_velocity-----��ǰ���ٶ�
        qValue-----���ڴ�����������������ֵ
��	�أ���
��	ע����
************************************************/
static void linearQuantization(float set_value, float actual_value, float angular_velocity, float *qValue)
{
    float thisError, deltaError;

    thisError = set_value - actual_value; // ��ȡ��ǰ���
    deltaError = angular_velocity;        // ��ȡƫ������

    qValue[0] = 6.0f * thisError / 20.0f; // 20��ʾ����������֮��
    if (qValue[0] < -6)
    {
        qValue[0] = -6;
    }
    if (qValue[0] > 6)
    {
        qValue[0] = 6;
    }

    qValue[1] = 6.0f * deltaError / 0.9f; // 0.9��ʾ���ٶȵ�������֮��
    if (qValue[1] < -6)
    {
        qValue[1] = -6;
    }
    if (qValue[1] > 6)
    {
        qValue[1] = 6;
    }
}

/***********************************************
�������ƣ�calcMembership
��	�ܣ�ģ��PID���Ӻ���֮һ������Ϊ���������ȣ������������������ֵ�����������������������Լ���Ӧ�������ȣ����������Ⱥ���Ϊ���������Ⱥ�����
��	����ms-----���ڴ����Ӧ��������
        qv-----���������������ֵ
        index-----���ڴ��������������������
��	�أ���
��	ע����
************************************************/
static void calcMembership(float *ms, float qv, int *index)
{
    if ((qv >= NB) && (qv < NM))
    {
        index[0] = 0;
        index[1] = 1;
        ms[0] = -0.5f * qv - 2.0f; // y=-0.5x-2.0
        ms[1] = 0.5f * qv + 3.0f;  // y=0.5x+3.0
    }
    else if ((qv >= NM) && (qv < NS))
    {
        index[0] = 1;
        index[1] = 2;
        ms[0] = -0.5f * qv - 1.0f; // y=-0.5x-1.0
        ms[1] = 0.5f * qv + 2.0f;  // y=0.5x+2.0
    }
    else if ((qv >= NS) && (qv < ZO))
    {
        index[0] = 2;
        index[1] = 3;
        ms[0] = -0.5f * qv;       // y=-0.5x
        ms[1] = 0.5f * qv + 1.0f; // y=0.5x+1.0
    }
    else if ((qv >= ZO) && (qv < PS))
    {
        index[0] = 3;
        index[1] = 4;
        ms[0] = -0.5f * qv + 1.0f; // y=-0.5x+1.0
        ms[1] = 0.5f * qv;         // y=0.5x
    }
    else if ((qv >= PS) && (qv < PM))
    {
        index[0] = 4;
        index[1] = 5;
        ms[0] = -0.5f * qv + 2.0f; // y=-0.5x+2.0
        ms[1] = 0.5f * qv - 1.0f;  // y=0.5x-1.0
    }
    else if ((qv >= PM) && (qv <= PB))
    {
        index[0] = 5;
        index[1] = 6;
        ms[0] = -0.5f * qv + 3.0f; // y=-0.5x+3.0
        ms[1] = 0.5f * qv - 2.0f;  // y=0.5x-2.0
    }
}

/***********************************************
�������ƣ�fuzzyComputation
��	�ܣ�ģ��PID���㺯������������Ϊ�����Ƚ�����Լ����ٶȽ���������ģ��������Ȼ���������Լ����ٶ��������Ӧ�������������Լ������ȣ�
        Ȼ����������ȼ����������������ģ��������
��	����PID-----��Ҫ����ģ�������PID����
��	�أ���
��	ע����
************************************************/
static void fuzzyComputation(PID_LocTypeDef *PID)
{
    float qValue[2] = {0, 0}; // ƫ�������������ֵ
    int indexE[2] = {0, 0};   // ƫ������������
    float msE[2] = {0, 0};    // ƫ��������
    int indexEC[2] = {0, 0};  // ƫ����������������
    float msEC[2] = {0, 0};   // ƫ������������
    float qValueK[3];         // �����ķ������ֵ

    linearQuantization(Position_Hope / 3770, Angle_roll, -Omega_roll, qValue);

    calcMembership(msE, qValue[0], indexE);
    calcMembership(msEC, qValue[1], indexEC);

    qValueK[0] = msE[0] * (msEC[0] * PIDBox_P[indexE[0]][indexEC[0]] + msEC[1] * PIDBox_P[indexE[0]][indexEC[1]]) + msE[1] * (msEC[0] * PIDBox_P[indexE[1]][indexEC[0]] + msEC[1] * PIDBox_P[indexE[1]][indexEC[1]]);
    qValueK[1] = msE[0] * (msEC[0] * PIDBox_I[indexE[0]][indexEC[0]] + msEC[1] * PIDBox_I[indexE[0]][indexEC[1]]) + msE[1] * (msEC[0] * PIDBox_I[indexE[1]][indexEC[0]] + msEC[1] * PIDBox_I[indexE[1]][indexEC[1]]);
    qValueK[2] = msE[0] * (msEC[0] * PIDBox_D[indexE[0]][indexEC[0]] + msEC[1] * PIDBox_D[indexE[0]][indexEC[1]]) + msE[1] * (msEC[0] * PIDBox_D[indexE[1]][indexEC[0]] + msEC[1] * PIDBox_D[indexE[1]][indexEC[1]]);

    PID->Kp = PID->Kp + qp * qValueK[0];
    PID->Ki = PID->Ki + qi * qValueK[1];
    PID->Kd = PID->Kd + qd * qValueK[2];
}

/***********************************************
�������ƣ�courseEstimate
��	�ܣ�����Ԥ���㷨
��	����course_speed-----��ǰ�ٶ�
        course_roll-----��ǰ�����
        course_wheel-----�����֡���֮��
��	�أ�course_estimate-----����Ԥ��ֵ
��	ע����
************************************************/
double courseEstimate(float course_speed, double course_roll, double course_wheel)
{
    double course_cos, course_estimate;
    course_cos = myCos((Angle_roll * PI / 360));
    if (course_wheel == 0)
        course_estimate = 0;
    else
        course_estimate = (course_speed * 0.2f * course_cos * 3.14159f * course_roll) / (180 * course_wheel);
    return course_estimate;
}

//**************************���������Ժ�����ʼ****************************

/***********************************************
�������ƣ�mySin
��	�ܣ�sin��������̩��չ����x�㴦��sinֵ
��	����x-----�Ƕ�(������)
��	�أ�sin(x)
��	ע����
************************************************/
double mySin(double x)
{
    double p = 0.000001, sum = 0, t = x; // ���徫������p��չ��ʽ����֮��sum��չ��ʽ��ǰ��t
    int n = 0;                           // �����������n
    do
    {
        sum += t;                               // ��ǰsumֵΪ��ǰ���ֵt����֮ǰ�ĸ����sum
        n++;                                    // �����������
        t = -t * x * x / (2 * n + 1) / (2 * n); // ����̩��չ��ʽ���㵱ǰ��t
    } while (fabs(t) >= p);                     // ���㵱ǰ�����ֵfabs(t)������С�ھ�������pʱֹͣѭ������ʱ�����Ѿ��㹻С�����Ժ����ˡ�
    return sum;
}

/***********************************************
�������ƣ�myCos
��	�ܣ�cos��������y��ֵ����1/4�����ڣ����ɰ���sin��������ֵ
��	����y-----�Ƕ�(������)
��	�أ�cos(y)
��	ע����
************************************************/
double myCos(double y)
{
    const float Q = 1.5707963268; // Q=pi/2
    y += Q;                       // ��y��ֵ����1/4������
    if (y > PI)
        y -= 2 * PI;   // ��y����һ�����ڣ����ȥ2pi
    return (mySin(y)); // ����sin������ֵ������������
}

/***********************************************
�������ƣ�paramInit
��	�ܣ�ȫ�ֱ�����ʼ��
��	������
��	�أ���
��	ע����
************************************************/
void paramInit(void)
{
    uint16_t i;

    for (i = 0; i < 19; i++)
    {
        UDP_MessagetoSend[i] = 0;
    }

    Remote_Controllor_Mode = 2;
    Velocity_FirstAxis = Omega_ball = Velocity_ball = 0.0;
    Omega_roll = Angle_roll = 0.0;
    Velocity_Hope = Position_Hope = 0;

    Control_Message.vel_1 = Control_Message.vel_2 = Control_Message.vel_3 = Control_Message.vel_4 = 0;
    Control_Message.pos_1 = Control_Message.pos_2 = Control_Message.pos_3 = Control_Message.pos_4 = 0;
    Control_Message.ord = 0xff;

    Flag_Feedback_Timer = Flag_PID_Control_Timer = Flag_Remote_Controllor_Timer = 0;
    Odom_all = Odom_now = Odom_last = Arc_now = Delta_roll = 0.0;

    Flag_PID_FirstAxis = Flag_PID_SecondAxis = 0;

    Control_Message.pid_switcher = Control_Message.pid_switcher_last = 0;

    deltaK[0] = 0;
    deltaK[1] = 0;
    deltaK[2] = 0;

    Humidity = Temperature = 0;
}

/***********************************************
�������ƣ�odomfeedback
��	�ܣ��������������ڼ�����̣����Ϸ�����λ��
��	������
��	�أ���
��	ע����
************************************************/
void odomfeedback(void)
{

    int odom_to_send;
    odom_to_send = (Velocity_FirstAxis + IMU_CAN_Message.GyroI_Align_y) * 0.069813;
    /* �������Ϣ���͸���λ�� */
    UDP_MessagetoSend[0] = 0xff;                          // ֡ͷ
    UDP_MessagetoSend[1] = 0xbb;                          // ֡ͷ
    UDP_MessagetoSend[2] = ((odom_to_send >> 24)) & 0xff; // ��̼���Ϣodom_to_send
    UDP_MessagetoSend[3] = ((odom_to_send >> 16)) & 0xff;
    UDP_MessagetoSend[4] = ((odom_to_send >> 8)) & 0xff;

    UDP_MessagetoSend[5] = (odom_to_send)&0xff;
    UDP_MessagetoSend[6] = 0xdd; // ֡β������Ϊ0xdd��ʾ��һ�������Ѿ�������ɣ�����ִ�лش���

    if ((unsigned char)UDP_MessagetoSend[6] == 0xdd) // ���Ѿ�����ִ�лش�
    {
        udpClientSend(UDP_MessagetoSend, 7); // ��UDP_MessagetoSend���鴫��scib_SendMsg����
        UDP_MessagetoSend[6] = 0xff;         // ֡β����Ϊ0xff����ʾĿǰ����ִ�лش�
    }
}
