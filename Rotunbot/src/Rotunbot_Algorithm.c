/**
 *****************************************************************************
 * @file    Rotunbot_Algorithm.c
 * @author  决策控制组
 * @version V1.0.0
 * @date    2022-05-09
 * @brief   存放所有控制相关算法代码、包括一定数量的初始化
 *****************************************************************************
 * @history
 *
 * 1. Date:2022-05-09
 *    Author:林柏羽
 *    Modification:创建文件
 *
 *****************************************************************************
 */

#include "Rotunbot_Algorithm.h"

/* 内部变量 */

/***********************************************
函数名称：PID_Loc
功	能：位置式PID计算
参	数：set_value-----设定值
        actual_value-----实际值
        value_gradient-----实际值变化率
        PID-----PID数据结构
返	回：PID_Loc ------PID位置式计算结果
备	注：无
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
函数名称：PIDSwitch
功	能：机器人速度不同时，根据相关指令更换PID参数
参	数：switch_order-----PID更换指令(0x01表示使用第一组PID参数，以此类推)
返	回：无
备	注：PID参数改变时，误差积分项也要做相应变换以保证Ki*LocSum的值不变
************************************************/

/***********************************************
函数名称：linearQuantization
功	能：模糊PID的子函数之一，作用为将输入量量化，通过线性映射将输入量（误差以及角速度）映射到-6到6之间
参	数：set_value-----目标值
        actual_value-----当前值
        angular_velocity-----当前角速度
        qValue-----用于存放输入量被量化后的值
返	回：无
备	注：无
************************************************/
static void linearQuantization(float set_value, float actual_value, float angular_velocity, float *qValue)
{
    float thisError, deltaError;

    thisError = set_value - actual_value; // 求取当前误差
    deltaError = angular_velocity;        // 求取偏差增量

    qValue[0] = 6.0f * thisError / 20.0f; // 20表示误差的上下限之差
    if (qValue[0] < -6)
    {
        qValue[0] = -6;
    }
    if (qValue[0] > 6)
    {
        qValue[0] = 6;
    }

    qValue[1] = 6.0f * deltaError / 0.9f; // 0.9表示角速度的上下限之差
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
函数名称：calcMembership
功	能：模糊PID的子函数之一，作用为计算隶属度，根据输入量量化后的值计算输入量的隶属度索引以及相应的隶属度（其中隶属度函数为三角隶属度函数）
参	数：ms-----用于存放相应的隶属度
        qv-----输入量被量化后的值
        index-----用于存放输入量的隶属度索引
返	回：无
备	注：无
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
函数名称：fuzzyComputation
功	能：模糊PID计算函数，计算流程为：首先将误差以及角速度进行量化（模糊化）；然后计算误差以及角速度量化后对应的隶属度索引以及隶属度；
        然后根据隶属度计算输出；最后将输出解模糊化即可
参	数：PID-----将要进行模糊计算的PID参数
返	回：无
备	注：无
************************************************/
static void fuzzyComputation(PID_LocTypeDef *PID)
{
    float qValue[2] = {0, 0}; // 偏差及其增量的量化值
    int indexE[2] = {0, 0};   // 偏差隶属度索引
    float msE[2] = {0, 0};    // 偏差隶属度
    int indexEC[2] = {0, 0};  // 偏差增量隶属度索引
    float msEC[2] = {0, 0};   // 偏差增量隶属度
    float qValueK[3];         // 反重心法求出的值

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
函数名称：courseEstimate
功	能：航向预估算法
参	数：course_speed-----当前速度
        course_roll-----当前横滚角
        course_wheel-----最近两帧里程之和
返	回：course_estimate-----航向预估值
备	注：无
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

//**************************辅助功能性函数开始****************************

/***********************************************
函数名称：mySin
功	能：sin函数，用泰勒展开求x点处的sin值
参	数：x-----角度(弧度制)
返	回：sin(x)
备	注：无
************************************************/
double mySin(double x)
{
    double p = 0.000001, sum = 0, t = x; // 定义精度限制p、展开式各项之和sum、展开式当前项t
    int n = 0;                           // 定义项数序号n
    do
    {
        sum += t;                               // 当前sum值为当前项的值t加上之前的各项和sum
        n++;                                    // 项数序号自增
        t = -t * x * x / (2 * n + 1) / (2 * n); // 根据泰勒展开式计算当前项t
    } while (fabs(t) >= p);                     // 计算当前项绝对值fabs(t)，当它小于精度限制p时停止循环，此时增量已经足够小，可以忽略了。
    return sum;
}

/***********************************************
函数名称：myCos
功	能：cos函数，将y的值加上1/4个周期，即可按照sin函数来求值
参	数：y-----角度(弧度制)
返	回：cos(y)
备	注：无
************************************************/
double myCos(double y)
{
    const float Q = 1.5707963268; // Q=pi/2
    y += Q;                       // 将y的值加上1/4个周期
    if (y > PI)
        y -= 2 * PI;   // 若y超出一个周期，则减去2pi
    return (mySin(y)); // 利用sin函数求值，并返回数据
}

/***********************************************
函数名称：paramInit
功	能：全局变量初始化
参	数：无
返	回：无
备	注：无
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
函数名称：odomfeedback
功	能：反馈函数，用于计算里程，并上发给上位机
参	数：无
返	回：无
备	注：无
************************************************/
void odomfeedback(void)
{

    int odom_to_send;
    odom_to_send = (Velocity_FirstAxis + IMU_CAN_Message.GyroI_Align_y) * 0.069813;
    /* 将里程信息发送给上位机 */
    UDP_MessagetoSend[0] = 0xff;                          // 帧头
    UDP_MessagetoSend[1] = 0xbb;                          // 帧头
    UDP_MessagetoSend[2] = ((odom_to_send >> 24)) & 0xff; // 里程计信息odom_to_send
    UDP_MessagetoSend[3] = ((odom_to_send >> 16)) & 0xff;
    UDP_MessagetoSend[4] = ((odom_to_send >> 8)) & 0xff;

    UDP_MessagetoSend[5] = (odom_to_send)&0xff;
    UDP_MessagetoSend[6] = 0xdd; // 帧尾，设置为0xdd表示新一批数据已经更新完成，可以执行回传了

    if ((unsigned char)UDP_MessagetoSend[6] == 0xdd) // 若已经可以执行回传
    {
        udpClientSend(UDP_MessagetoSend, 7); // 将UDP_MessagetoSend数组传给scib_SendMsg函数
        UDP_MessagetoSend[6] = 0xff;         // 帧尾设置为0xff，表示目前不能执行回传
    }
}
