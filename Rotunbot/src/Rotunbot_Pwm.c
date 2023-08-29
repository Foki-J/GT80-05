/*
 *ZBX for Wheel Spherical
 *****************************************************************************
 * @file    Rotunbot_Pwm.c
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

#include "Rotunbot_Pwm.h"

/* 定义PWM相关定时器*/
#define PWM_OUTPUT_TIMER &htim4           // PWM输出功能定义在TIM4上面
#define PWM_LEFT_PROPELLER TIM_CHANNEL_1  // Channel_1用于输出给左螺旋桨
#define PWM_RIGHT_PROPELLER TIM_CHANNEL_2 // Channel_2用于输出给右螺旋桨

/* 局部变量 */
unsigned int Signal_1_Period, Signal_1_High;                 // 第一个PWM捕获信号：前后摇杆	>>
unsigned int Signal_2_Period, Signal_2_High;                 // 第二个PWM捕获信号：左右摇杆	>>
unsigned int Signal_3_Period, Signal_3_High, Signal_3_State; // 第三个PWM捕获信号：模式切换	>>

/* 螺旋桨用局部变量 */
int left_controller_zero_position = 1500;  // 左螺旋桨的零位
int right_controller_zero_position = 1500; // 右螺旋桨的零位
int left_right_difference = 0;
short surface_vx = 0; // 水面x方向速度
short surface_w = 0;  // 水面角速度

/* 摆用局部变量 */
const int first_axis_zero_position = 1500;  // 主摆的零位
const int first_axis_max_width = 400;       // 遥控器摇杆对应的最大脉宽（以zero_position作为中位，上下此宽度）
const int first_axis_dead_zone = 30;        // 主摆遥控零位附近的死区
const int first_axis_max = 900;             // 主摆电机最大速度
const int second_axis_zero_position = 1500; // 副摆的零位
const int second_axis_max_width = 300;      // 遥控器摇杆对应的最大脉宽（以zero_position作为中位，上下此宽度）
const int second_axis_dead_zone = 20;       // 副摆遥控零位附近的死区
const int second_axis_max = 3000;           // 副摆电机最大输出角度（对应摆起的角度，上下此角度）
const int w_max = 2000;
/***********************************************
函数名称：pwmInit
功	能：PWM输出功能（主要用于螺旋桨驱动），初始化函数
参	数：无
返	回：无
备	注：无
************************************************/
unsigned short TIM8_CH2_buff[TIM_IC_BUFFSIZE] = {first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position, first_axis_zero_position};
unsigned short TIM8_CH4_buff[TIM_IC_BUFFSIZE] = {second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position, second_axis_zero_position};
unsigned short TIM1_CH2_buff[TIM_IC_BUFFSIZE] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
void pwmInit(void)
{
    HAL_TIM_PWM_Start(PWM_OUTPUT_TIMER, PWM_LEFT_PROPELLER);  // 左螺旋桨初始化
    HAL_TIM_PWM_Start(PWM_OUTPUT_TIMER, PWM_RIGHT_PROPELLER); // 右螺旋桨初始化
    HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_2, (uint32_t *)TIM8_CH2_buff, TIM_IC_BUFFSIZE);
    HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_4, (uint32_t *)TIM8_CH4_buff, TIM_IC_BUFFSIZE);
    HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_2, (uint32_t *)TIM1_CH2_buff, TIM_IC_BUFFSIZE);
}
// Sort the array in ascending order
short calculate_average(unsigned short *buff, int size)
{

    // Calculate the standard deviation of the array
    int buff_sum = 0;
    for (int i = 0; i < size; i++)
    {
        buff_sum += buff[i];
    }
    short mean = buff_sum / size;
    float variance = 0;
    for (int i = 0; i < size; i++)
    {
        variance += ((buff[i] - mean) * (buff[i] - mean));
    }
    variance /= (size - 1);

    // Remove outliers and Calculate the average of the remaining values in the array
    int new_size = 0;
    int sum = 0;
    for (int i = 0; i < size; i++)
    {
        if (((buff[i] - mean) * (buff[i] - mean)) <= 4 * variance)
        {
            sum += buff[i];
            new_size++;
        }
    }
    return (short)(sum / new_size);
}

/* brief: 由定时器DMA写入的数组获得遥控器通道变量，并将其赋值给Velocity_Hope, Roll_Hope及W_hope
 * 无参数，无返回
 *
 */
void RemoteRx(void)
{
    int t = 0;
    Signal_1_High = calculate_average(TIM8_CH2_buff, TIM_IC_BUFFSIZE);
    Signal_2_High = calculate_average(TIM8_CH4_buff, TIM_IC_BUFFSIZE);
    Signal_3_High = calculate_average(TIM1_CH2_buff, TIM_IC_BUFFSIZE);
    Signal_3_State = (char)((Signal_3_High - 800) / 467);

    // if (Signal_1_High >= 950 && Signal_1_High <= 2050)							// 主轴通道
    {
        t = Signal_1_High - first_axis_zero_position;
        Left_Pulse_Hope = t;
        if (t < first_axis_dead_zone && t > -first_axis_dead_zone)
        { // 零点死区
            Velocity_Hope = 0;
        }
        else if (t >= first_axis_dead_zone && t <= first_axis_max_width)
        { // 线性增长区
            Velocity_Hope = first_axis_max * (t - first_axis_dead_zone) / (first_axis_max_width - first_axis_dead_zone);
        }
        else if (t <= -first_axis_dead_zone && t >= -first_axis_max_width)
        {
            Velocity_Hope = first_axis_max * (t + first_axis_dead_zone) / (first_axis_max_width - first_axis_dead_zone);
        }
        else if (t > first_axis_max_width)
        { // 饱和区
            Velocity_Hope = first_axis_max;
        }
        else
        {
            Velocity_Hope = -first_axis_max;
        }
    }
    // if (Signal_2_High >= 950 && Signal_2_High <= 2050)							// 副轴通道
    {
        t = Signal_2_High - second_axis_zero_position;
        Right_Pulse_Hope = t;
        if (-second_axis_dead_zone < t && t < second_axis_dead_zone)
        { // 零点死区
            Roll_Hope = 0;
        }
        else if (second_axis_dead_zone <= t && t <= second_axis_max_width)
        { // 线性增长区
            Roll_Hope = (second_axis_max * (t - second_axis_dead_zone) * 1.0 / (second_axis_max_width - second_axis_dead_zone));
        }
        else if (-second_axis_max_width <= t && t <= -second_axis_dead_zone)
        {
            Roll_Hope = (second_axis_max * (t + second_axis_dead_zone) * 1.0 / (second_axis_max_width - second_axis_dead_zone));
        }
        else if (t > second_axis_max_width)
        { // 饱和区
            Roll_Hope = second_axis_max;
        }
        else if (t < -second_axis_max_width)
        {
            Roll_Hope = -second_axis_max;
        }
    }
    if (Signal_3_State == 0) // 陆地
    {
        V_Hope = 0;
        W_Hope = 0;
    }
    else if (Signal_3_State == 1) // 搁浅
    {
        V_Hope = 1000 * Velocity_Hope / first_axis_max;
        W_Hope = w_max * Roll_Hope / second_axis_max;
    }
    else if (Signal_3_State == 2) // 水面
    {
        V_Hope = 1000 * Velocity_Hope / first_axis_max;
        W_Hope = w_max * Roll_Hope / second_axis_max;
        Velocity_Hope = 0;
        Roll_Hope = 0;
    }
//    	short msg[8];
//    	msg[0] = Signal_1_High;
//    	msg[1] = Velocity_Hope;
//    	msg[2] = TIM8_CH2_buff[0];
//    	msg[3] = Roll_Hope;
//    	msg[4] = Signal_2_High;
//    	send_ANO_msg(msg);
}
