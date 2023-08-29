/**
 *****************************************************************************
 * @file    Rotunbot_Can.c
 * @author  ZBX for Wheel Spherical
 * @version V1.0.0
 * @date    2022-04-28
 * @brief   与电机驱动器通信以控制电机
 *****************************************************************************
 * @history
 *
 * 1. Date:2022-04-28
 *    Author:张璧萱
 *    Modification:加入动量轮驱动
 *
 *****************************************************************************
 */

#include "Rotunbot_Can.h"

// 内部局部变量

// CAN_ID
const unsigned short int CAN_ID_0 = 0x0601;  // 1号驱动板发送CAN_ID
const unsigned short int CAN_ID_1 = 0x0581;  // 1号驱动板接收CAN_ID
const unsigned short int CAN_ID_2 = 0x0602;  // 2号驱动板发送CAN_ID
const unsigned short int CAN_ID_3 = 0x0582;  // 2号驱动板接收CAN_ID
const unsigned short int CAN_ID_21 = 0x0603; // 3号驱动板发送CAN_ID
const unsigned short int CAN_ID_31 = 0x0583; // 3号驱动板接收CAN_ID

const unsigned short int CAN_ID_4 = 0x0181;  // IMU CANopen模式：通道  1 |  2 |  3 |  4
const unsigned short int CAN_ID_5 = 0x0281;  // IMU CANopen模式：通道  5 |  6 |  7 |  8
const unsigned short int CAN_ID_6 = 0x0381;  // IMU CANopen模式：通道  9 | 10 | 11 | 12
const unsigned short int CAN_ID_7 = 0x0481;  // IMU CANopen模式：通道 13 | 14 | 15 | 16
const unsigned short int CAN_ID_8 = 0x0701;  // 驱动器 CANopen模式： 主轴驱动器上电
const unsigned short int CAN_ID_9 = 0X0702;  // 驱动器 CANopen模式： 副轴驱动器上电
const unsigned short int CAN_ID_91 = 0X0703; // 驱动器 CANopen模式： 动量轮驱动器上电
const unsigned short int CAN_ID_10 = 0x081;  // 驱动器 CANopen模式： 主轴驱动器下电
const unsigned short int CAN_ID_11 = 0X082;  // 驱动器 CANopen模式： 副轴驱动器下电
const unsigned short int CAN_ID_22 = 0X083;  // 驱动器 CANopen模式： 动量轮驱动器下电

const unsigned short int CAN_ID_12 = 0X0281; // 无线充电发
const unsigned short int CAN_ID_13 = 0X0280; // 无线充电收

const unsigned short int CAN_ID_14 = 0x0100; // 主轴TPDO1 位置，速度
const unsigned short int CAN_ID_15 = 0x0101; // 主轴TPDO2 工作模式
const unsigned short int CAN_ID_16 = 0x0102; // 副轴TPDO1 位置，速度
const unsigned short int CAN_ID_17 = 0x0103; // 副轴TPDO2 工作模式
const unsigned short int CAN_ID_18 = 0x0104; // 动量轮TPDO1 位置，速度
const unsigned short int CAN_ID_19 = 0x0105; // 动量轮TPDO2 工作模式
// 电机控制指令(32bit指令，4个字节)
const unsigned int enable_set = 0x2b406000; // 使能设置指令
const unsigned int servo_on = 0x0f000000;   // 电机使能
// const unsigned int enable_set2 = 0x2b406010;					// 位置模式使能设置指令
const unsigned int servo_on2 = 0x3f100000;             // 电机位置模式使能
const unsigned int servo_off = 0x06000000;             // 电机非使能
const unsigned int mode_select = 0x2f606000;           // 模式选择指令
const unsigned int current_mode = 0x04000000;          // 进入电流模式
const unsigned int current_set = 0x2b716000;           // 电流设置指令
const unsigned int velocity_mode = 0xFD000000;         // 进入速度模式
const unsigned int velocity_set = 0x23FF6000;          // 速度设置指令
const unsigned int position_mode = 0x01000000;         // 进入pro位置模式
const unsigned int pro_velocity_set = 0x23816000;      // pro位置模式中速度设置
const unsigned int operation_mode_select = 0x2b406000; // pro位置模式的运行模式选择
const unsigned int absolute_position_mode1 = 0x2F000000;
const unsigned int absolute_position_mode2 = 0x3F000000; // 2F->3F表示以绝对位置模式执行
const unsigned int relative_position_mode1 = 0x4F000000;
const unsigned int relative_position_mode2 = 0x5F000000; // 4F->5F表示以相对位置模式执行
const unsigned int position_set = 0x237A6000;            // 位置设置指令
const unsigned int acceleration_set = 0x23836000;        // 加速度设置指令
const unsigned int deceleration_set = 0x23846000;        // 减速度设置指令
const unsigned int mode_query = 0x40616000;              // 电机所处模式查询指令
const unsigned int velocity_query = 0x406C6000;          // 电机转速查询指令
const unsigned int position_query = 0x40636000;          // 电机位置查询指令
const unsigned int current_query = 0x406C6000;           // 电机电流查询命令

// 其他变量
const float dec2rpm = 0.0003662109375;
char firAxisState = 0, secAxisState = 0, thiAxisState = 0; // 用于记录主副轴电机所处工作模式+动量轮电机模式
char flag_IMU_Received_1 = 0;                              // 用于IMU消息接收的标志位 1
char flag_IMU_Received_2 = 0;                              // 用于IMU消息接收的标志位 2
char flag_IMU_Received_3 = 0;                              // 用于IMU消息接收的标志位 3
unsigned int IMU_Data[4][8];                               // 记录IMU的数据，当一帧信息全部接收时，将此数据通过 Rotunbot_Comm 全部发送到上位机
char driver_power = 0;                                     // 驱动器是否上电
char driver_power_last = 0;
short motor_orig_angle = 0;           // 副轴电机初始角度，与主轴垂直为0
unsigned short driver_state_word = 0;  // 9bits状态字，从左到右表示 动量轮上电动量轮使能|动量轮模式|主轴电机是否上电|主轴电机是否使能|主轴电机是否进入模式|副轴电机是否上电|副轴电机是否使能|副轴电机是否进入模式
short driver_PDO_timer = 0;           // 每次收到驱动器消息，计时器清零，进入主循环，计时加1。当计时达到阈值，认为驱动器已下电。
char flag_jam = 0;

uint16_t Battery_Current = 0;        // 电池电流    单位：mA
uint16_t Battery_Capacity = 0;       // 电池容量    单位：mAH
uint16_t Battery_Voltage = 0;        // 电池电压    单位：mV
uint8_t Battery_Soc = 0;             // 电池SOC     单位：%
uint16_t Power_Nominal_Capacity = 0; // 电池标称电量  单位：WH
/***********************************************
函数名称：CAN_sendXxxxAxis
功	能：主（副）轴Can通信消息发送函数
参	数：无
返	回：无
备	注：通过 jam_count 变量记录Can通信阻塞时间，以实现电机下电后相关变量清零的功能
************************************************/
void CAN_sendFirstAxis(void)
{
    /* 阻塞变量与标志位 */
    uint32_t jam_count = 0;

    /* 发送邮箱号（事实上会自动获得这个Mailbox的真实数值，这里仅作标注） */
    uint32_t TxMailbox = 0;
    /* 装载一帧数据 */
    CAN_SetMsg(CAN_ID_0);
    /* 开始发送数据 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        jam_count++;
        if (jam_count > 4000) // 通信阻塞变量，实现电机下电后的相关变量清零的功能（似乎不可行，因为电机下电时IMU未能下电，导致消息还可以正常发送）
            break;
    }
    HAL_CAN_AddTxMessage(&hcan1, my_Packge.TxHeader, my_Packge.Payload, &TxMailbox);
}

void CAN_sendSecondAxis(void)
{
    /* 阻塞变量与标志位 */
    uint32_t jam_count = 0;
    /* 发送邮箱号（事实上会自动获得这个Mailbox的真实数值，这里仅作标注） */
    uint32_t TxMailbox = 2;
    /* 装载一帧数据 */
    CAN_SetMsg(CAN_ID_2);
    /* 开始发送数据 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        jam_count++;
        if (jam_count > 4000) // 通信阻塞变量，实现电机下电后的相关变量清零的功能（似乎不可行，因为电机下电时IMU未能下电，导致消息还可以正常发送）
            break;
    }
    HAL_CAN_AddTxMessage(&hcan1, my_Packge.TxHeader, my_Packge.Payload, &TxMailbox);
}

void CAN_sendThirdAxis(void)
{
    /* 阻塞变量与标志位 */
    uint32_t jam_count = 0;
    /* 发送邮箱号（事实上会自动获得这个Mailbox的真实数值，这里仅作标注） */
    uint32_t TxMailbox = 3;
    /* 装载一帧数据 */
    CAN_SetMsg(CAN_ID_21);
    /* 开始发送数据 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        jam_count++;
        if (jam_count > 4000) // 通信阻塞变量，实现电机下电后的相关变量清零的功能（似乎不可行，因为电机下电时IMU未能下电，导致消息还可以正常发送）
            break;
    }

    HAL_CAN_AddTxMessage(&hcan1, my_Packge.TxHeader, my_Packge.Payload, &TxMailbox);
}

/***********************************************
函数名称：CAN_SetMsg_battery
功	能：CAN通信报文内容:设置数据包
参	数：发送的CAN_ID
返	回：无
备	注：无
************************************************/
void CAN_SetMsg_battery(void)
{
    my_Packge.TxHeader->StdId = 0x00;       // 使用的标准ID：主轴ID
    my_Packge.TxHeader->ExtId = 0x182C1860; // 使用的拓展ID
    my_Packge.TxHeader->IDE = CAN_ID_EXT;   // 扩展模式
    my_Packge.TxHeader->RTR = CAN_RTR_DATA; // 发送的是数据
    my_Packge.TxHeader->DLC = 6;            // 数据长度为8字节

    /* 装载暂存数据 */
    for (int i = 0; i < 4; i++)
        Can_Msg_Temp[i] = Can_Msg_MDL >> (24 - i * 8);
    for (int i = 4; i < 8; i++)
        Can_Msg_Temp[i] = Can_Msg_MDH >> (24 - (i - 4) * 8);

    /* 装载数据 */
    for (uint8_t i = 0; i < 8; i++)
        my_Packge.Payload[i] = Can_Msg_Temp[i];

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

/***********************************************
函数名称：Battery_send
功	能：电池Can通信消息发送函数
参	数：无
返	回：无
备	注：通过 jam_count 变量记录Can通信阻塞时间，以实现电机下电后相关变量清零的功能
************************************************/
void Battery_send(void)
{
    /* 阻塞变量与标志位 */
    uint32_t jam_count = 0;
    char flag_jam = 0;
    /* 发送邮箱号（事实上会自动获得这个Mailbox的真实数值，这里仅作标注） */
    uint32_t TxMailbox = 0;
    /* 装载一帧数据 */
    CAN_SetMsg_battery();

    /* 开始发送数据 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        jam_count++;
        if (jam_count > 4000) // 通信阻塞变量，实现电机下电后的相关变量清零的功能（似乎不可行，因为电机下电时IMU未能下电，导致消息还可以正常发送）
            break;
    }
    if (flag_jam == 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, my_Packge.TxHeader, my_Packge.Payload, &TxMailbox);
    }
}
/***********************************************
函数名称：firstAxisModeSelect
功	能：选择主轴电机工作模式
参	数：mode-----电机工作模式（01：电流模式，02：速度模式，03：定速度pro位置模式，04：自定义速度pro位置模式）
返	回：无
备	注：电机控制方式为先置低位（MDL）为控制命令，再置高位（MDH）为控制数据
************************************************/
void firstAxisModeSelect(int mode)
{
    Can_Msg_MDL = mode_select; // 置低位为模式选择命令
    if (mode == 1)             // 01-电流模式
    {
        Can_Msg_MDH = current_mode; // 置高位为电流模式命令
    }
    else if (mode == 2) // 02-pro速度模式
    {
        Can_Msg_MDH = velocity_mode; // 置高位为pro速度模式命令
    }
    else if (mode == 3) // 03-定速度pro位置模式
    {
        Can_Msg_MDH = position_mode; // 置高位为pro位置模式命令
    }
    else if (mode == 4) // 04-自定义速度pro位置模式
    {
        Can_Msg_MDH = position_mode; // 置高位为pro位置模式命令
    }
    firAxisState = mode;
    CAN_sendFirstAxis();

    if (mode == 3) // 在固定速度pro位置模式中需设定额定转速，这里设置为3000r/min
    {
        Can_Msg_MDL = pro_velocity_set; // 置低位为速度设置命令
        Can_Msg_MDH = 0x00000D00;       // 置高位为pro速度设置数值：固定数值 0x00007D00 （意为3000r/min）
        CAN_sendFirstAxis();
    }
}

/***********************************************
函数名称：firstAxisxxxMode
功	能：在电机相应工作模式中，向电机发送指定的控制数据
参	数：firAxisCurrentMode：cc1-----指定电流的8-16位; cc2-----指定电流的0-8位
        firAxisVelocityMode:vv1-----指定速度的24-32位; vv2-----指定速度的16-24位; vv3-----指定速度的8-16位; vv4-----指定速度的0-8位
        firAxisPositionMode:pvp1-----指定位置的24-32位; pvp2-----指定位置的16-24位; pvp3-----指定位置的8-16位; pvp4-----指定位置的0-8位
返	回：无
备	注：firAxisState用于记录主轴电机工作模式，当电机不在指定工作模式时，需先切换到指定工作模式
        传输协议中，电流、速度、位置数据均为4个字节反序排列（低位在前，高位在后）
        速度模式发送目标速度后需再使能电机
        位置模式需先发送目标位置再发送绝对位置执行指令
************************************************/
void firstAxisCurrentMode(int cc1, int cc2)
{
    int cc;

    if (firAxisState != 1)
        firstAxisModeSelect(1); // 改变记录的电机工作模式 - 1

    cc = (cc2 << 24) + (cc1 << 16) + 0x00000000; // 由于指定电流值只有两个字节，其高位默认为0
    Can_Msg_MDL = current_set;                   // 置低位为电流设置命令
    Can_Msg_MDH = cc;                            // 置高位为电流设置数值
    CAN_sendFirstAxis();
}

void firstAxisVelMode(int vv1, int vv2, int vv3, int vv4)
{
    int vv;

//		Can_Msg_MDL = enable_set; // 主轴使能
//    Can_Msg_MDH = servo_on;
//    CAN_sendFirstAxis();
//    HAL_Delay(1);
    if (firAxisState != 2)
    firstAxisModeSelect(2); // 改变记录的电机工作模式 - 2

    vv = (vv4 << 24) + (vv3 << 16) + (vv2 << 8) + vv1;
    Can_Msg_MDL = velocity_set; // 置低位为速度设置命令
    Can_Msg_MDH = vv;           // 置高位为速度设置数值
    CAN_sendFirstAxis();
}

void firstAxisPosMode(int pvp1, int pvp2, int pvp3, int pvp4)
{
    int pvp;

    if (firAxisState != 3)
        firstAxisModeSelect(3); // 改变记录的电机工作模式 - 3

    pvp = (pvp4 << 24) + (pvp3 << 16) + (pvp2 << 8) + pvp1;
    Can_Msg_MDL = position_set; // 置低位为pro位置设置命令
    Can_Msg_MDH = pvp;          // 置高位为pro位置设置数值
    CAN_sendFirstAxis();

    Can_Msg_MDL = operation_mode_select; // 以绝对位置模式执行
    Can_Msg_MDH = absolute_position_mode1;
    CAN_sendFirstAxis();

    Can_Msg_MDL = operation_mode_select;
    Can_Msg_MDH = absolute_position_mode2;
    CAN_sendFirstAxis();
}

void firstAxisVelPosMode(int vvp1, int vvp2, int vvp3, int vvp4, int pvp1, int pvp2, int pvp3, int pvp4)
{
    int vvp, pvp;

    if (firAxisState != 4)
        firstAxisModeSelect(4); // 改变记录的电机工作模式 - 4

    vvp = (vvp4 << 24) + (vvp3 << 16) + (vvp2 << 8) + vvp1;
    Can_Msg_MDL = pro_velocity_set; // 置低位为pro速度设置命令
    Can_Msg_MDH = vvp;              // 置高位为pro速度设置数值
    CAN_sendFirstAxis();

    pvp = (pvp4 << 24) + (pvp3 << 16) + (pvp2 << 8) + pvp1;
    Can_Msg_MDL = position_set; // 置低位为pro位置设置命令
    Can_Msg_MDH = pvp;          // 置高位为pro位置设置数值
    CAN_sendFirstAxis();

    Can_Msg_MDL = operation_mode_select; // 以绝对位置模式执行
    Can_Msg_MDH = absolute_position_mode1;
    CAN_sendFirstAxis();

    Can_Msg_MDL = operation_mode_select;
    Can_Msg_MDH = absolute_position_mode2;
    CAN_sendFirstAxis();
}

/***********************************************
函数名称：secAxisModeSelect
功	能：选择副轴电机工作模式
参	数：mode-----电机工作模式（01：电流模式，02：速度模式，03：定速度pro位置模式，04：自定义速度pro位置模式，05：松摆模式）
返	回：无
备	注：电机控制方式为先置低位（MBOX3.MDL.all）为控制命令，再置高位（MBOX3.MDH.all）为控制数据
************************************************/
void secondAxisModeSelect(int mode)
{
    Can_Msg_MDL = mode_select; // 置低位为模式选择命令
    if (mode == 1)             // 01-电流模式
    {
        Can_Msg_MDH = current_mode; // 置高位为电流模式命令
    }
    else if (mode == 2) // 02-pro速度模式
    {
        Can_Msg_MDH = velocity_mode; // 置高位为pro速度模式命令
    }
    else if (mode == 3) // 03-定速度pro位置模式
    {
        Can_Msg_MDH = position_mode; // 置高位为pro位置模式命令
    }
    else if (mode == 4) // 04-自定义速度pro位置模式
    {
        Can_Msg_MDH = position_mode; // 置高位为pro位置模式命令
    }
    CAN_sendSecondAxis();

    if (mode == 5) // 05：松摆模式（电机非使能）
    {
        Can_Msg_MDL = enable_set;
        Can_Msg_MDH = servo_off;
        CAN_sendSecondAxis();
    }
    secAxisState = mode;

    if (mode == 3) // 在固定速度pro位置模式中需设定额定转速，这里设置为3000r/min
    {
        Can_Msg_MDL = pro_velocity_set; // 置低位为pro速度设置命令
        Can_Msg_MDH = 0x00000D00;       // 置高位为pro速度设置数值：固定数值 0x00007D00 （意为3000r/min）
        CAN_sendSecondAxis();
    }
}

/***********************************************
函数名称：secondAxisxxxMode
功	能：在电机相应工作模式中，向电机发送指定的控制数据
参	数：secAxisCurrentMode：cc1-----指定电流的8-16位; cc2-----指定电流的0-8位
        secAxisVelocityMode:vv1-----指定速度的24-32位; vv2-----指定速度的16-24位; vv3-----指定速度的8-16位; vv4-----指定速度的0-8位
        secAxisPositionMode:pvp1-----指定位置的24-32位; pvp2-----指定位置的16-24位; pvp3-----指定位置的8-16位; pvp4-----指定位置的0-8位
返	回：无
备	注：secAxisState用于记录副轴电机工作模式，当电机不在指定工作模式时，需先切换到指定工作模式
        传输协议中，电流、速度、位置数据均为4个字节反序排列（低位在前，高位在后）
        速度模式发送目标速度后需再使能电机
        位置模式需先发送目标位置再发送绝对位置执行指令
************************************************/
void secondAxisCurrentMode(int cc1, int cc2)
{
    int cc;

    if (secAxisState != 1) // 改变记录的电机工作模式 - 1
        secondAxisModeSelect(1);

    cc = (cc2 << 24) + (cc1 << 16) + 0x00000000;
    Can_Msg_MDL = current_set; // 置低位为电流设置命令
    Can_Msg_MDH = cc;          // 置高位为电流设置数值
    CAN_sendSecondAxis();
}

void secondAxisVelMode(int vv1, int vv2, int vv3, int vv4)
{
    unsigned int vv;

    if (secAxisState != 2)
        secondAxisModeSelect(2); // 改变记录的电机工作模式 - 2

    vv = (vv4 << 24) + (vv3 << 16) + (vv2 << 8) + vv1;
    Can_Msg_MDL = velocity_set; // 置低位为速度设置命令
    Can_Msg_MDH = vv;           // 置高位为速度设置数值
    CAN_sendSecondAxis();
}

void secondAxisPosMode(unsigned char pvp1, unsigned char pvp2, unsigned char pvp3, unsigned char pvp4)
{
    int pvp;
		
//		Can_Msg_MDL = enable_set; // 副轴使能
//    Can_Msg_MDH = servo_on2;
//    CAN_sendSecondAxis();
//		HAL_Delay(1);
    if (secAxisState != 3)
    secondAxisModeSelect(3); // 改变记录的电机工作模式 - 3

    Can_Msg_MDL = pro_velocity_set; // 置低位为pro速度设置命令
    Can_Msg_MDH = 0x00000D00;       // 置高位为pro速度设置数值：固定数值 0x00007D00 （意为3000r/min）
    CAN_sendSecondAxis();
    HAL_Delay(1);
    pvp = (pvp4 << 24) | (pvp3 << 16) | (pvp2 << 8) | pvp1;
    Can_Msg_MDL = position_set; // 置低位为pro位置设置命令
    Can_Msg_MDH = pvp;          // 置高位为pro位置设置数值
    CAN_sendSecondAxis();
    //	HAL_Delay(1);
    //	Can_Msg_MDL = enable_set;//重新上电
    //	Can_Msg_MDH = servo_on;
    //	CAN_sendSecondAxis();
    //	HAL_Delay(1);
    //	Can_Msg_MDL = operation_mode_select;					// 以绝对位置模式执行
    //	Can_Msg_MDH = relative_position_mode1;
    //	CAN_sendSecondAxis();
    //	HAL_Delay(1);
    //	Can_Msg_MDL = operation_mode_select;
    //	Can_Msg_MDH = relative_position_mode2;				// 2F->3F
    //	CAN_sendSecondAxis();
}

void secondAxisVelPosMode(int vvp1, int vvp2, int vvp3, int vvp4, int pvp1, int pvp2, int pvp3, int pvp4)
{
    int vvp, pvp;

    if (secAxisState != 4)
        secondAxisModeSelect(4); // 改变记录的电机工作模式 - 4

    vvp = (vvp4 << 24) + (vvp3 << 16) + (vvp2 << 8) + vvp1;
    Can_Msg_MDL = pro_velocity_set; // 置低位为pro速度设置命令
    Can_Msg_MDH = vvp;              // 置高位为pro速度设置数值
    CAN_sendSecondAxis();

    pvp = (pvp4 << 24) + (pvp3 << 16) + (pvp2 << 8) + pvp1;
    Can_Msg_MDL = position_mode; // 置低位为pro位置设置命令
    Can_Msg_MDH = pvp;           // 置高位为pro位置设置数值
    CAN_sendSecondAxis();

    Can_Msg_MDL = operation_mode_select; // 以绝对位置模式执行
    Can_Msg_MDH = absolute_position_mode1;
    CAN_sendSecondAxis();

    Can_Msg_MDL = operation_mode_select;
    Can_Msg_MDH = absolute_position_mode2;
    CAN_sendSecondAxis();
}

void secondAxisUnlockMode(void)
{
    if (secAxisState != 5)
        secondAxisModeSelect(5); // 改变记录的电机工作模式 - 5
}
/***********************************************
函数名称：thiAxisModeSelect
功	能：选择副轴电机工作模式
参	数：mode-----电机工作模式（01：电流模式，02：速度模式，03：定速度pro位置模式，04：自定义速度pro位置模式，05：松摆模式）
返	回：无
备	注：电机控制方式为先置低位（MBOX3.MDL.all）为控制命令，再置高位（MBOX3.MDH.all）为控制数据
************************************************/
void thirdAxisModeSelect(int mode)
{
    Can_Msg_MDL = mode_select; // 置低位为模式选择命令
    if (mode == 1)             // 01-电流模式
    {
        Can_Msg_MDH = current_mode; // 置高位为电流模式命令
    }
    else if (mode == 2) // 02-pro速度模式
    {
        Can_Msg_MDH = velocity_mode; // 置高位为pro速度模式命令
    }
    else if (mode == 3) // 03-定速度pro位置模式
    {
        Can_Msg_MDH = position_mode; // 置高位为pro位置模式命令
    }
    else if (mode == 4) // 04-自定义速度pro位置模式
    {
        Can_Msg_MDH = position_mode; // 置高位为pro位置模式命令
    }
    CAN_sendThirdAxis();

    if (mode == 5) // 05：松摆模式（电机非使能）
    {
        Can_Msg_MDL = enable_set;
        Can_Msg_MDH = servo_off;
        CAN_sendThirdAxis();
    }
    thiAxisState = mode;

    if (mode == 3) // 在固定速度pro位置模式中需设定额定转速，这里设置为3000r/min
    {
        Can_Msg_MDL = pro_velocity_set; // 置低位为pro速度设置命令
        Can_Msg_MDH = 0x00000D00;       // 置高位为pro速度设置数值：固定数值 0x00007D00 （意为3000r/min）
        CAN_sendThirdAxis();
    }
}

/***********************************************
函数名称：thirdAxisxxxMode
功	能：在电机相应工作模式中，向电机发送指定的控制数据
参	数：thiAxisCurrentMode：cc1-----指定电流的8-16位; cc2-----指定电流的0-8位
        thiAxisVelocityMode:vv1-----指定速度的24-32位; vv2-----指定速度的16-24位; vv3-----指定速度的8-16位; vv4-----指定速度的0-8位
        thiAxisPositionMode:pvp1-----指定位置的24-32位; pvp2-----指定位置的16-24位; pvp3-----指定位置的8-16位; pvp4-----指定位置的0-8位
返	回：无
备	注：thiAxisState用于记录副轴电机工作模式，当电机不在指定工作模式时，需先切换到指定工作模式
        传输协议中，电流、速度、位置数据均为4个字节反序排列（低位在前，高位在后）
        速度模式发送目标速度后需再使能电机
        位置模式需先发送目标位置再发送绝对位置执行指令
************************************************/
void thirdAxisCurrentMode(int cc1, int cc2)
{
    int cc;
//		Can_Msg_MDL = enable_set; // 动量轮使能
//    Can_Msg_MDH = servo_on;
//    CAN_sendThirdAxis();
//		HAL_Delay(1);
    if (thiAxisState != 1) // 改变记录的电机工作模式 - 1
    thirdAxisModeSelect(1);

    cc = (cc2 << 24) + (cc1 << 16) + 0x00000000;
    Can_Msg_MDL = current_set; // 置低位为电流设置命令
    Can_Msg_MDH = cc;          // 置高位为电流设置数值
    CAN_sendThirdAxis();
}

void thirdAxisVelMode(int vv1, int vv2, int vv3, int vv4)
{
    unsigned int vv;

    if (thiAxisState != 2)
        thirdAxisModeSelect(2); // 改变记录的电机工作模式 - 2

    vv = (vv4 << 24) + (vv3 << 16) + (vv2 << 8) + vv1;
    Can_Msg_MDL = velocity_set; // 置低位为速度设置命令
    Can_Msg_MDH = vv;           // 置高位为速度设置数值
    CAN_sendThirdAxis();
}

void thirdAxisPosMode(int pvp1, int pvp2, int pvp3, int pvp4)
{
    int pvp;

    if (thiAxisState != 3)
        thirdAxisModeSelect(3); // 改变记录的电机工作模式 - 3

    pvp = (pvp4 << 24) + (pvp3 << 16) + (pvp2 << 8) + pvp1;
    Can_Msg_MDL = position_set; // 置低位为pro位置设置命令
    Can_Msg_MDH = pvp;          // 置高位为pro位置设置数值
    CAN_sendThirdAxis();

    Can_Msg_MDL = operation_mode_select; // 以绝对位置模式执行
    Can_Msg_MDH = absolute_position_mode1;
    CAN_sendThirdAxis();

    Can_Msg_MDL = operation_mode_select;
    Can_Msg_MDH = absolute_position_mode2;
    CAN_sendThirdAxis();
}

void thirdAxisVelPosMode(int vvp1, int vvp2, int vvp3, int vvp4, int pvp1, int pvp2, int pvp3, int pvp4)
{
    int vvp, pvp;

    if (thiAxisState != 4)
        thirdAxisModeSelect(4); // 改变记录的电机工作模式 - 4

    vvp = (vvp4 << 24) + (vvp3 << 16) + (vvp2 << 8) + vvp1;
    Can_Msg_MDL = pro_velocity_set; // 置低位为pro速度设置命令
    Can_Msg_MDH = vvp;              // 置高位为pro速度设置数值
    CAN_sendThirdAxis();

    pvp = (pvp4 << 24) + (pvp3 << 16) + (pvp2 << 8) + pvp1;
    Can_Msg_MDL = position_mode; // 置低位为pro位置设置命令
    Can_Msg_MDH = pvp;           // 置高位为pro位置设置数值
    CAN_sendThirdAxis();
    Can_Msg_MDL = operation_mode_select; // 以绝对位置模式执行
    Can_Msg_MDH = absolute_position_mode1;
    CAN_sendThirdAxis();
    Can_Msg_MDL = operation_mode_select;
    Can_Msg_MDH = absolute_position_mode2;
    CAN_sendThirdAxis();
}

void thirdAxisUnlockMode(void)
{
    if (thiAxisState != 5)
        thirdAxisModeSelect(5); // 改变记录的电机工作模式 - 5
}

/***********************************************
函数名称：BatteryFeedback
功	能：电池反馈函数，用于查询电池相关状态
参	数：无
返	回：无
备	注：由于均为查询指令，控制数据写入0即可（即MBOX1.MDH.all = 0x00000000）
************************************************/
void BatteryFeedback(void)
{
    Can_Msg_MDL = 0x00041600;
    Can_Msg_MDH = 0x4F850000;
    Battery_send();
}

/***********************************************
函数名称：motorStateFeedback
功	能：电机反馈函数，用于查询电机相关状态
参	数：无
返	回：无
************************************************/
void motorStateFeedback(void)
{
    Can_Msg_MDL = velocity_query; // 询问主轴当前速度
    Can_Msg_MDH = 0x00000000;
    CAN_sendFirstAxis();

    CAN_sendSecondAxis();
    CAN_sendThirdAxis();
}
/***********************************************
driverStateFeedback
功	能：驱动器反馈函数，用于查询驱动器相关状态
参	数：无
返	回：无
************************************************/
void driverStateFeedback(void)
{
    Can_Msg_MDL = mode_query; // 询问主轴当前所处模式
    Can_Msg_MDH = 0x00000000;
    CAN_sendFirstAxis();
    HAL_Delay(1);
    CAN_sendSecondAxis(); // 询问副轴当前所处模式/
    CAN_sendThirdAxis();  // 询问动量轮当前所处模式/
}

void TPDO_Enable(uint8_t id)
{
    uint8_t data[2] = {0x01, id};
    CAN_TxHeaderTypeDef TPDO_TxHeader;
    TPDO_TxHeader.StdId = 0x00;
    TPDO_TxHeader.ExtId = 0x0000;     // 未使用的拓展ID
    TPDO_TxHeader.IDE = CAN_ID_STD;   // 标准模式
    TPDO_TxHeader.RTR = CAN_RTR_DATA; // 发送的是数据
    TPDO_TxHeader.DLC = 2;            // 数据长度为2字节
    uint32_t TxMailbox = 0;
    /* 阻塞变量与标志位 */
    uint32_t jam_count = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        jam_count++;
        if (jam_count > 4000) // 通信阻塞变量，实现电机下电后的相关变量清零的功能（似乎不可行，因为电机下电时IMU未能下电，导致消息还可以正常发送）
            break;
    }
    HAL_CAN_AddTxMessage(&hcan1, &TPDO_TxHeader, data, &TxMailbox);
}
/***********************************************
函数名称：DriverEnable
功	能：使能电机，令主轴电机进入速度模式，副轴电机进入位置模式，动量轮电机进入力矩模式。
参	数：无
返	回：无
************************************************/
void DriverEnable(void)
{
    if ((driver_state_word & 0x20) != 0) // 主轴上电
    {
        TPDO_Enable(0x00);
        if ((driver_state_word & 0x10) == 0) // 主轴未使能
        {
            Can_Msg_MDL = enable_set; // 主轴使能
            Can_Msg_MDH = servo_on;
            CAN_sendFirstAxis();
            HAL_Delay(1);
        }
        else if ((driver_state_word & 0x08) == 0) // 主轴未进入模式
        {
            firstAxisModeSelect(2);
            HAL_Delay(1);
        }
    }
    if ((driver_state_word & 0x04) != 0) // 副轴上电
    {
        TPDO_Enable(0x01);
        if ((driver_state_word & 0x02) == 0) // 副轴未使能
        {
            Can_Msg_MDL = enable_set; // 副轴使能？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
            Can_Msg_MDH = servo_on2;
            CAN_sendSecondAxis();
            PID_SecondAxis.LocSum = 0;
            HAL_Delay(1);
            secondAxisPosMode(0, 0, 0, 0);
        }

        else if ((driver_state_word & 0x01) == 0) // 副轴未进入模式
        {
            secondAxisModeSelect(3); // ？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？
        }
    }
    if ((driver_state_word & 0x100) != 0) // 动量轮上电
    {
        TPDO_Enable(0x02);
        if ((driver_state_word & 0x80) == 0) // 动量轮未使能
        {
            Can_Msg_MDL = enable_set; // 动量轮使能
            Can_Msg_MDH = servo_on;
            CAN_sendThirdAxis();
            PID_ThirdAxis.LocSum = 0;
            HAL_Delay(1);
        }
        else if ((driver_state_word & 0x40) == 0) // 动量轮未进入模式
        {
            thirdAxisModeSelect(1);
        }
    }
}
/***********************************************
函数名称：imuFeedback
功	能：IMU反馈函数。
参	数：无
返	回：无
备	注：一帧IMU数据分为四个消息帧。帧头的第二个字节，分别为：0xAA，0xAB,0xAC,0xAD。
************************************************/
void imuFeedback(void)
{
    for (int i = 0; i < 4; i++)
    {
        /* 将IMU消息发送给上位机 —— 每两字节（16位）一个数据，低位在前 */
        UDP_MessagetoSend[0] = 0xFF;                 // 帧头
        UDP_MessagetoSend[1] = 0xAA + i;             // 帧头
        UDP_MessagetoSend[2] = (char)IMU_Data[i][0]; // IMU消息反馈到上位机
        UDP_MessagetoSend[3] = (char)IMU_Data[i][1];
        UDP_MessagetoSend[4] = (char)IMU_Data[i][2];
        UDP_MessagetoSend[5] = (char)IMU_Data[i][3];
        UDP_MessagetoSend[6] = (char)IMU_Data[i][4];
        UDP_MessagetoSend[7] = (char)IMU_Data[i][5];
        UDP_MessagetoSend[8] = (char)IMU_Data[i][6];
        UDP_MessagetoSend[9] = (char)IMU_Data[i][7];
        UDP_MessagetoSend[10] = 0xDD; // 帧尾，设置为0xDD表示新一批数据已经更新完成，可以执行回传了

        if ((unsigned char)UDP_MessagetoSend[10] == 0xDD) // 若已经可以执行回传
        {
            udpClientSend(UDP_MessagetoSend, 11); // 将UDP_MessagetoSend数组传给 udpClientSend 函数
            UDP_MessagetoSend[10] = 0xFF;         // 帧尾设置为0xFF，表示目前不能执行回传
        }
    }
}

void internalStateFeedback(void)
{
    UDP_MessagetoSend[0] = 0xFF; // 帧头
    UDP_MessagetoSend[1] = 0xEE;
    UDP_MessagetoSend[2] = Battery_Voltage >> 8;      // 电池电压高八位，单位0.1V
    UDP_MessagetoSend[3] = Battery_Voltage & 0xff;    // 电池电压低八位，单位0.1V
    UDP_MessagetoSend[4] = Battery_Current >> 8;      // 电池电流高八位，单位0.1A
    UDP_MessagetoSend[5] = Battery_Current & 0xff;    // 电池电流低八位，单位0.1A
    UDP_MessagetoSend[6] = Battery_Soc;               // 电池电量信息，%
    UDP_MessagetoSend[7] = 0;                         // 电池内部温度，
    UDP_MessagetoSend[8] = 0;                         // 电池充电状态
    UDP_MessagetoSend[9] = 0;                         // 未定义
    UDP_MessagetoSend[10] = 0;                        // 未定义
    UDP_MessagetoSend[11] = 0;                        // 未定义
    UDP_MessagetoSend[12] = 0;                        // 未定义
    UDP_MessagetoSend[13] = 0;                        // 未定义
    UDP_MessagetoSend[14] = 0;                        // 未定义
    UDP_MessagetoSend[15] = 0xEE;                     // 帧尾
    if ((unsigned char)UDP_MessagetoSend[15] == 0xEE) // 若已经可以执行回传
    {
        udpClientSend(UDP_MessagetoSend, 16); // 将UDP_MessagetoSend数组传给scib_SendMsg函数
        UDP_MessagetoSend[15] = 0xDD;         // 帧尾设置为0xEE，表示目前不能执行回传
    }
}

void motorFeedback(void)
{
    UDP_MessagetoSend[0] = 0xFF; // 帧头
    UDP_MessagetoSend[1] = 0xDD;
    UDP_MessagetoSend[2] = Current_FirstAxis >> 8; // 主轴力矩 16位
    UDP_MessagetoSend[3] = Current_FirstAxis & 0xff;
    UDP_MessagetoSend[4] = (Velocity_FirstAxis >> 24) & 0xff; // 主轴转速 32位
    UDP_MessagetoSend[5] = (Velocity_FirstAxis >> 16) & 0xff;
    UDP_MessagetoSend[6] = (Velocity_FirstAxis >> 8) & 0xff;
    UDP_MessagetoSend[7] = Velocity_FirstAxis & 0xff;
    UDP_MessagetoSend[8] = Current_SecondAxis >> 8; // 副轴力矩 16位
    UDP_MessagetoSend[9] = Current_SecondAxis & 0xff;
    UDP_MessagetoSend[10] = (Velocity_SecondAxis >> 24) & 0xff; // 副轴转速 32位
    UDP_MessagetoSend[11] = (Velocity_SecondAxis >> 16) & 0xff;
    UDP_MessagetoSend[12] = (Velocity_SecondAxis >> 8) & 0xff;
    UDP_MessagetoSend[13] = Velocity_SecondAxis & 0xff;
    UDP_MessagetoSend[14] = ((Position_SecondAxis + motor_orig_angle) >> 24) & 0xff; // 副轴位置 32位
    UDP_MessagetoSend[15] = ((Position_SecondAxis + motor_orig_angle) >> 16) & 0xff;
    UDP_MessagetoSend[16] = ((Position_SecondAxis + motor_orig_angle) >> 8) & 0xff;
    UDP_MessagetoSend[17] = (Position_SecondAxis + motor_orig_angle) & 0xff;
    UDP_MessagetoSend[18] = Current_ThirdAxis >> 8; // 动量轮力矩 16位
    UDP_MessagetoSend[19] = Current_ThirdAxis & 0xff;
    UDP_MessagetoSend[20] = (Velocity_ThirdAxis >> 24) & 0xff; // 动量轮转速 32位
    UDP_MessagetoSend[21] = (Velocity_ThirdAxis >> 16) & 0xff;
    UDP_MessagetoSend[22] = (Velocity_ThirdAxis >> 8) & 0xff;
    UDP_MessagetoSend[23] = Velocity_ThirdAxis & 0xff;
    UDP_MessagetoSend[24] = (left >> 8) & 0xff; // 左推进器转速 16位
    UDP_MessagetoSend[25] = left & 0xff;
    UDP_MessagetoSend[26] = (right >> 8) & 0xff; // 右推进器转速 16位
    UDP_MessagetoSend[27] = right & 0xff;
    UDP_MessagetoSend[28] = 0xEE;
    if ((unsigned char)UDP_MessagetoSend[28] == 0xEE) // 若已经可以执行回传
    {
        udpClientSend(UDP_MessagetoSend, 29); // 将UDP_MessagetoSend数组传给scib_SendMsg函数
        UDP_MessagetoSend[28] = 0xDD;         // 帧尾设置为0xDD，表示目前不能执行回传
    }
}
/********************************************分 隔 符*****************************************/

/***********************************************
函数名称：CAN_Config
功	能：完整配置CAN的功能
参	数：无
返	回：无
备	注：一帧IMU数据分为四个消息帧。帧头的第二个字节，分别为：0xAA，0xAB,0xAC,0xAD。
************************************************/
void CAN_Config(void)
{
    CAN_Filter_Config();   // CAN过滤器设置
    Init_RxMes();          // 初始化 Rx_Message数据结构体
    HAL_CAN_Start(&hcan1); // CAN1使能
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/***********************************************
函数名称：Init_RxMes
功	能：初始化 Rx_Message数据结构体
参	数：无
返	回：无
备	注：RxMessage：指向要初始化的数据结构体。
************************************************/
void Init_RxMes(void)
{
    /*把接收结构体清零*/
    RxHeader.StdId = 0x00;
    RxHeader.ExtId = 0x0000;
    RxHeader.IDE = CAN_ID_STD | CAN_ID_EXT;
    RxHeader.DLC = 0;
    RxHeader.FilterMatchIndex = 0;
}

/***********************************************
函数名称：CAN_SetMsg
功	能：CAN通信报文内容:设置数据包
参	数：发送的CAN_ID
返	回：无
备	注：无
************************************************/
void CAN_SetMsg(unsigned short int id)
{
    my_Packge.TxHeader->StdId = id;         // 使用的标准ID：主轴ID
    my_Packge.TxHeader->ExtId = 0x0000;     // 未使用的拓展ID
    my_Packge.TxHeader->IDE = CAN_ID_STD;   // 标准模式
    my_Packge.TxHeader->RTR = CAN_RTR_DATA; // 发送的是数据
    my_Packge.TxHeader->DLC = 8;            // 数据长度为8字节

    /* 装载暂存数据 */
    for (int i = 0; i < 4; i++)
        Can_Msg_Temp[i] = Can_Msg_MDL >> (24 - i * 8);
    for (int i = 4; i < 8; i++)
        Can_Msg_Temp[i] = Can_Msg_MDH >> (24 - (i - 4) * 8);

    /* 装载数据 */
    for (uint8_t i = 0; i < 8; i++)
        my_Packge.Payload[i] = Can_Msg_Temp[i];

    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

/********************************************分 隔 符*****************************************/

/***********************************************
函数名称：HAL_CAN_ErrorCallback
功	能：CAN通信错误回调函数
参	数：hcan: CAN句柄指针
返	回：无
备	注：会向调试串口打印ErrorCode
************************************************/
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    printf("***CAN callback error***\n");
    printf("  ErrorCode = %x\n", hcan->ErrorCode); // 向调试串口打印 ErrorCode (具体Code代表意义参考 stm32f4xx_hal_can.h 的285行前后)
}

/***********************************************
函数名称：HAL_CAN_RxFifo0MsgPendingCallback
功	能：CAN回调函数
参	数：hcan: CAN句柄指针
返	回：无
备	注：无
************************************************/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    unsigned char Can_Rx_Buff[8];

    /* 比较ID是否为CAN_ID中的一个 */
    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0)
    {
    };
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, Can_Rx_Buff);
    if (hcan->Instance == CAN1)
    {
        if (((RxHeader.StdId == CAN_ID_4) || (RxHeader.StdId == CAN_ID_5) || (RxHeader.StdId == CAN_ID_6) || (RxHeader.StdId == CAN_ID_7)) && (RxHeader.IDE == CAN_ID_STD) && (RxHeader.DLC == 8))
        {
            CAN_Flag = 1; // CAN消息接收成功

            if (RxHeader.StdId == CAN_ID_4) // IMU_ID1
            {
                if (flag_IMU_Received_1 == 0) // 该系列flag的功能见变量声明
                {
                    flag_IMU_Received_1 = 1;
                    for (int i = 0; i < 8; i++)
                        IMU_Data[0][i] = Can_Rx_Buff[i];

                    IMU_CAN_Message.Acc_x = (Can_Rx_Buff[1] << 8) + Can_Rx_Buff[0];
                    IMU_CAN_Message.Acc_y = (Can_Rx_Buff[3] << 8) + Can_Rx_Buff[2];
                    IMU_CAN_Message.Acc_z = (Can_Rx_Buff[5] << 8) + Can_Rx_Buff[4];
                    IMU_CAN_Message.GyroI_Align_x = (Can_Rx_Buff[7] << 8) + Can_Rx_Buff[6];
                    IMU_Message.omega_roll = (float)IMU_CAN_Message.GyroI_Align_x; // IMU以Degree的形式发送，下位机进行接收，这里转换成Radius的形式
                    Omega_roll = IMU_Message.omega_roll / 100 * PI / 180;          // 100是我们内部定的因子
                }
                else
                {
                    flag_IMU_Received_1 = 0;
                    flag_IMU_Received_2 = 0;
                    flag_IMU_Received_3 = 0;
                }
            }
            else if (RxHeader.StdId == CAN_ID_5) // IMU_ID2
            {
                if (flag_IMU_Received_1 == 1)
                {
                    flag_IMU_Received_1 = 0;
                    flag_IMU_Received_2 = 1;
                    for (int i = 0; i < 8; i++)
                        IMU_Data[1][i] = Can_Rx_Buff[i];

                    IMU_CAN_Message.GyroI_Align_y = (Can_Rx_Buff[1] << 8) + Can_Rx_Buff[0];
                    IMU_CAN_Message.GyroI_Align_z = (Can_Rx_Buff[3] << 8) + Can_Rx_Buff[2];
                    IMU_CAN_Message.Mag_x = (Can_Rx_Buff[5] << 8) + Can_Rx_Buff[4];
                    IMU_CAN_Message.Mag_y = (Can_Rx_Buff[7] << 8) + Can_Rx_Buff[6];
                }
                else
                {
                    flag_IMU_Received_1 = 0;
                    flag_IMU_Received_2 = 0;
                    flag_IMU_Received_3 = 0;
                }
            }
            else if (RxHeader.StdId == CAN_ID_6) // IMU_ID3
            {
                if (flag_IMU_Received_2 == 1)
                {
                    flag_IMU_Received_2 = 0;
                    flag_IMU_Received_3 = 1;
                    for (int i = 0; i < 8; i++)
                        IMU_Data[2][i] = Can_Rx_Buff[i];

                    IMU_CAN_Message.Mag_z = (Can_Rx_Buff[1] << 8) + Can_Rx_Buff[0];
                    IMU_CAN_Message.Euler_x = (Can_Rx_Buff[3] << 8) + Can_Rx_Buff[2];
                    IMU_CAN_Message.Euler_y = (Can_Rx_Buff[5] << 8) + Can_Rx_Buff[4];
                    IMU_CAN_Message.Euler_z = (Can_Rx_Buff[7] << 8) + Can_Rx_Buff[6];
                    IMU_Message.angle_roll = (float)IMU_CAN_Message.Euler_x; // IMU以Degree的形式发送，下位机进行接收
                    Angle_roll = IMU_Message.angle_roll;                     // 100是我们内部定的因子
                }
                else
                {
                    flag_IMU_Received_1 = 0;
                    flag_IMU_Received_2 = 0;
                    flag_IMU_Received_3 = 0;
                }
            }
            else if (RxHeader.StdId == CAN_ID_7) // IMU_ID4
            {
                if (flag_IMU_Received_3 == 1)
                {
                    Flag_Feedback_IMU = 1;
                    flag_IMU_Received_1 = 0;
                    flag_IMU_Received_2 = 0;
                    flag_IMU_Received_3 = 0;

                    for (int i = 0; i < 8; i++)
                        IMU_Data[3][i] = Can_Rx_Buff[i];

                    IMU_CAN_Message.Quat_w = (Can_Rx_Buff[1] << 8) + Can_Rx_Buff[0];
                    IMU_CAN_Message.Quat_x = (Can_Rx_Buff[3] << 8) + Can_Rx_Buff[2];
                    IMU_CAN_Message.Quat_y = (Can_Rx_Buff[5] << 8) + Can_Rx_Buff[4];
                    IMU_CAN_Message.Quat_z = (Can_Rx_Buff[7] << 8) + Can_Rx_Buff[6];
                }
                else
                {
                    flag_IMU_Received_1 = 0;
                    flag_IMU_Received_2 = 0;
                    flag_IMU_Received_3 = 0;
                }
            }
        }
        else if (RxHeader.StdId == CAN_ID_14)
        {
            Velocity_FirstAxis = ((Can_Rx_Buff[7] << 24) + (Can_Rx_Buff[6] << 16) + (Can_Rx_Buff[5] << 8) + Can_Rx_Buff[4]) * 0.0092386; // 0.0006451;
            Position_FirstAxis = ((Can_Rx_Buff[3] << 24) + (Can_Rx_Buff[2] << 16) + (Can_Rx_Buff[1] << 8) + Can_Rx_Buff[0]) * 0.1517288;
        }
        else if (RxHeader.StdId == CAN_ID_16)
        {
            Velocity_SecondAxis = ((short)((Can_Rx_Buff[7] << 15) + (Can_Rx_Buff[6] << 7) + (Can_Rx_Buff[5] >> 1)) * (-0.13));
            Position_SecondAxis = -((short)((((Can_Rx_Buff[2] << 16) + (Can_Rx_Buff[1] << 8) + (Can_Rx_Buff[0]))) >> 2) / 5.735);
        }
        else if (RxHeader.StdId == CAN_ID_18)
        {
            Velocity_ThirdAxis = ((short)((Can_Rx_Buff[7] << 15) + (Can_Rx_Buff[6] << 7) + (Can_Rx_Buff[5] >> 1)) * (-0.13));
            Position_ThirdAxis = -((short)((((Can_Rx_Buff[2] << 16) + (Can_Rx_Buff[1] << 8) + (Can_Rx_Buff[0]))) >> 2) / 5.735);
        }
        else if (RxHeader.StdId == CAN_ID_15)
        {
            if (Can_Rx_Buff[0] == 0x00) // 主轴处于非使能状态
            {
                driver_state_word &= 0xFFE7; // 主轴使能位reset
            }
            else if (Can_Rx_Buff[0] == 0xFD) // 主轴当前处于速度模式
            {
                driver_state_word |= 0x10; // 主轴使能位set
                driver_state_word |= 0x08; // 主轴模式位set
            }
            else // 主轴使能但处于其他模式
            {
                driver_state_word |= 0x10; // 主轴使能位set
                driver_state_word &= 0xFFF7; // 主轴模式位reset
            }
            Current_FirstAxis = (short)(Can_Rx_Buff[2] << 8) | (Can_Rx_Buff[1]);
        }
        else if (RxHeader.StdId == CAN_ID_17)
        {
            driver_PDO_timer = 0;
            if (Can_Rx_Buff[0] == 0x00) // 副轴处于非使能状态
            {
                driver_state_word &= 0xFFFC; // 副轴使能位reset
            }
            else if (Can_Rx_Buff[0] == 0x01) // 副轴当前处于位置模式？？？？？？？？？？？？？？？？？？？？？
            {
                //			secAxisState = 1;
                driver_state_word |= 0x02; // 副轴使能位set
                driver_state_word |= 0x01; // 副轴模式位set
            }
            else // 副轴使能但处于其他模式
            {
                driver_state_word |= 0x02; // 副轴使能位set
                driver_state_word &= 0xFFFE; // 副轴模式位reset
            }
            Current_SecondAxis = (short)(Can_Rx_Buff[2] << 8) | (Can_Rx_Buff[1]);
        }
        else if (RxHeader.StdId == CAN_ID_19)
        {
            driver_PDO_timer = 0;
            if(Can_Rx_Buff[0] == 0x00)								//动量轮处于非使能状态
            	{
            		driver_state_word &= 0xFF3F;      //动量轮使能位reset
            	}
            	else if(Can_Rx_Buff[0] == 0x04)					//动量轮当前处于力矩模式
            	{
            		driver_state_word |= 0x80;      //动量轮使能位set
            		driver_state_word |= 0x40;      //动量轮模式位set
            	}
            	else                              //动量轮使能但处于其他模式
            	{
            		driver_state_word |= 0x80;      //动量轮使能位set
            		driver_state_word &= 0xFFBF;      //动量轮模式位reset
            	}
            Current_ThirdAxis = (short)(Can_Rx_Buff[2] << 8) | (Can_Rx_Buff[1]);
        }
        else if (RxHeader.StdId == CAN_ID_10) // 主轴驱动器下电
        {
            driver_power = 0;
            driver_state_word &= 0xFFC7; // 主轴所有位reset
        }
        else if (RxHeader.StdId == CAN_ID_11) // 副轴驱动器下电
        {
            driver_power = 0;
            driver_state_word &= 0xFFF8; // 副轴所有位reset
        }
        else if (RxHeader.StdId == CAN_ID_22) // 动量轮驱动器下电
        {
            driver_power = 0;
            driver_state_word &= 0xFE3F; // 动量轮所有位reset
        }
        else if (RxHeader.StdId == CAN_ID_8)
        {
            if (Can_Rx_Buff[0] == 0x05)
                return;
            driver_power = 1;
            driver_state_word |= 0x20; // 主轴上电位set
        }
        else if (RxHeader.StdId == CAN_ID_9)
        {
            driver_power = 1;
            driver_state_word |= 0x04; // 副轴上电位set
            motor_orig_angle = -IMU_CAN_Message.Euler_x;
        }
        else if (RxHeader.StdId == CAN_ID_91)
        {
            driver_power = 1;
            driver_state_word |= 0x100; // 动量轮上电位set
        }
        else if (((RxHeader.ExtId == 0x1a0c5860) || (RxHeader.ExtId == 0x1a0c5861) || (RxHeader.ExtId == 0x1a0c5862) ||
                  (RxHeader.ExtId == 0x1a0c5863) || (RxHeader.ExtId == 0x1a0c5864)) &&
                 (RxHeader.IDE == CAN_ID_EXT) && (RxHeader.DLC == 8))
        {
            CAN_Flag = 1; // 接收成功
            if (RxHeader.ExtId == 0x1a0c5860)
            {
                Battery_Current = ((Can_Rx_Buff[1] | Can_Rx_Buff[2] << 8)) / 100;  // 电流
                Battery_Capacity = ((Can_Rx_Buff[5] | Can_Rx_Buff[6] << 8)) / 100; // 实时容量
            }
            else if (RxHeader.ExtId == 0x1a0c5863)
            {
                Battery_Voltage = ((Can_Rx_Buff[1] | Can_Rx_Buff[2] << 8)) / 100; // 实时电压
            }
            else if (RxHeader.ExtId == 0x1a0c5864)
            {
                Battery_Soc = Can_Rx_Buff[3]; // 电量百分比
            }
            Power_Nominal_Capacity = Battery_Capacity / 1000 * Battery_Voltage / 1000; // 电池标称电量
        }
        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else
        CAN_Flag = 0; // CAN消息接收失败

    Init_RxMes(); // 清零 Rx_Message 数据结构体

    /* 准备中断接收 */
}

/***********************************************
函数名称：CAN_Filter_Config
功	能：CAN的过滤器配置
参	数：无
返	回：无
备	注：这里仅作了定义，没有过滤任何东西
************************************************/
static void CAN_Filter_Config(void)
{
    CAN_FilterTypeDef CAN_FilterInitStructure;

    /*CAN筛选器初始化*/
    CAN_FilterInitStructure.FilterBank = 14;
    // 工作在掩码模式
    CAN_FilterInitStructure.FilterMode = CAN_FILTERMODE_IDMASK;
    // 筛选器位宽为单个32位。
    CAN_FilterInitStructure.FilterScale = CAN_FILTERSCALE_32BIT;
    /* 使能筛选器，按照标志的内容进行比对筛选，标准ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
    // 要筛选的ID高位
    CAN_FilterInitStructure.FilterIdHigh = 0;
    // 要筛选的ID低位
    CAN_FilterInitStructure.FilterIdLow = 0;
    // 筛选器高16位每位必须匹配
    CAN_FilterInitStructure.FilterMaskIdHigh = 0;
    // 筛选器低16位每位必须匹配
    CAN_FilterInitStructure.FilterMaskIdLow = 0;
    // 筛选器被关联到FIFO0
    CAN_FilterInitStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    // 使能筛选器
    CAN_FilterInitStructure.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterInitStructure);
}
