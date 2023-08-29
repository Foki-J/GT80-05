/**
 *****************************************************************************
 * @file    Rotunbot_Comm.c
 * @author  决策控制组
 * @version V1.0.0
 * @date    2022-04-28
 * @brief   通信相关，主要包括蓝牙通信、上下位机通信相关函数
 *****************************************************************************
 * @history
 *
 * 1. Date:2022-04-28
 *    Author:林柏羽
 *    Modification:创建文件，完成UDP功能，从上位机接收信号保存在变量中
 *
 *****************************************************************************
 */

/* 头文件 */
#include "Rotunbot_Comm.h"

///* 定义远端IP */
#define UDP_REMOTE_IP4_0 192
#define UDP_REMOTE_IP4_1 168
#define UDP_REMOTE_IP4_2 2
#define UDP_REMOTE_IP4_3 2

/* 定义端口号 */
#define UDP_REMOTE_PORT 8881 // 远端端口
#define UDP_LOCAL_PORT 8880  // 本地端口

/* UDP控制块 */
static struct udp_pcb *upcb;

/***********************************************
 * 名称  ：udpReceiveCallback
 * 描述  : 接收回调函数
 * 参数  : -
 * 返回  : 无
 * 备注  ：上下位机通信消息格式为：共12个字节，2个字节表示消息头(0xff 0xbb)，1个字节表示消息类型(ord)，若ord=0x50，则表示这一帧消息为IMU数据，那么
           后面4个字节表示横滚角速度(omega_roll)，4个字节表示横滚角度(angle_roll)，最后1个字节暂时闲置；若ord!=0x50，则表示这一帧为控制数据，ord
           表示控制方式，后面2个字节表示主轴目标速度或电流，4个字节表示副轴目标位置，2个字节暂时闲置，最后1个字节表示PID基
************************************************/
static void udpReceiveCallback(void *arg, struct udp_pcb *upcb,
                               struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    /* 局部变量 */
    // unsigned long angle_roll_received, omega_roll_received;
    // uint32_t received_count = 0;

    /* 数据转存到结构体变量中 */
    if (p != NULL)
    {
        struct pbuf *ptmp = p;
        while (ptmp != NULL)
        {
            /* 数据帧操作 */
            if (*((unsigned char *)p->payload + 0) == 0xFF && *((unsigned char *)p->payload + 1) == 0xDD) // 判断消息帧头
            {
                // 存储收到的数据
                Remote_Control_Counter = 0;
                Velocity_Hope = (short)(((*((unsigned char *)p->payload + 3)) << 8) + (*((unsigned char *)p->payload + 4))) / 10;
                Roll_Hope = (short)(((*((unsigned char *)p->payload + 5)) << 8) + (*((unsigned char *)p->payload + 6))) / 10;
                char switch_cmd = *((unsigned char *)p->payload + 11);
                // 后三位：扬声器开关|前灯开关|后灯开关
                if ((switch_cmd & 0x01))
                    LIGHT_BACK_ON;
                else
                    LIGHT_BACK_OFF;

                if ((switch_cmd & 0x02))
                    LIGHT_FRONT_ON;
                else
                    LIGHT_FRONT_OFF;

                if ((switch_cmd & 0x04))
                    SPEAKER_ON;
                else
                    SPEAKER_OFF;
            }
            else
                printf("This is not a right Message\n");
            ptmp = p->next;
        }
    }
    printf("%d %d\n", Velocity_Hope, Position_Hope);

    /* 数据回传-UDP */
    /* （发现如果先数据回传再串口回传，会出现BUG） */
    // udp_send(upcb, p);

    /* 释放缓冲区数据 */
    pbuf_free(p);
}

/***********************************************
函数名称：udpClientSend
功	能：发送udp数据
参	数：pData 发送数据的指针，len 发送数据的长度
返	回：无
备	注：无
************************************************/
void udpClientSend(char *pData, int len)
{
    struct pbuf *p;

    /* 分配缓冲区空间 */
    p = pbuf_alloc(PBUF_TRANSPORT, len, PBUF_POOL);

    if (p != NULL)
    {
        /* 填充缓冲区数据 */
        pbuf_take(p, pData, len);
        /* 发送udp数据 */
        udp_send(upcb, p);
        /* 释放缓冲区空间 */
        pbuf_free(p);
    }
}

/***********************************************
函数名称：udpClientInit
功	能：初始化，创建UDP客户端
参	数：无
返	回：无
备	注：无
************************************************/
void udpClientInit(void)
{
    ip_addr_t serverIP;
    err_t err;

    IP4_ADDR(&serverIP, UDP_REMOTE_IP4_0, UDP_REMOTE_IP4_1, UDP_REMOTE_IP4_2, UDP_REMOTE_IP4_3);

    /* 创建udp控制块 */
    upcb = udp_new();

    if (upcb != NULL)
    {
        /* 配置本地端口 */
        upcb->local_port = UDP_LOCAL_PORT;
        /* 配置服务器IP和端口 */
        err = udp_connect(upcb, &serverIP, UDP_REMOTE_PORT);
        if (err == ERR_OK)
        {
            /* 注册接收回调函数 */
            udp_recv(upcb, udpReceiveCallback, NULL);
            /* 发送udp数据 */
            // udpClientSend("udp client connected");
            /* 向串口输入（调试用） */
            //            printf("udp client connected\r\n");
        }
        else
        {
            udp_remove(upcb);
            //            printf("can not connect udp pcb\r\n");
        }
    }
}
/* brief: 向匿名上位机发送最多包含5个16位数据的数组
 * param: 待发送数组
 */
#define UART5_LEN 16
char usart_frame[UART5_LEN + 5] = {0xAA, 0xAA, 0xF1, UART5_LEN};
void send_ANO_msg(short *msg)
{
    char sum = (char)(0xAA + 0xAA + 0xF1 + UART5_LEN);
    usart_frame[4] = msg[0] >> 8;
    usart_frame[5] = msg[0];
    usart_frame[6] = msg[1] >> 8;
    usart_frame[7] = msg[1];
    usart_frame[8] = msg[2] >> 8;
    usart_frame[9] = msg[2];
    usart_frame[10] = msg[3] >> 8;
    usart_frame[11] = msg[3];
    usart_frame[12] = msg[4] >> 8;
    usart_frame[13] = msg[4];
    usart_frame[14] = msg[5] >> 8;
    usart_frame[15] = msg[5];
    usart_frame[16] = msg[6] >> 8;
    usart_frame[17] = msg[6];
    usart_frame[18] = msg[7] >> 8;
    usart_frame[19] = msg[7];
    for (int i = 4; i < UART5_LEN + 4; i++)
        sum += usart_frame[i];
    usart_frame[UART5_LEN + 4] = sum;
    HAL_UART_Transmit(&huart5, (uint8_t *)usart_frame, UART5_LEN + 5, 10);
}
/******************************** END OF FILE ********************************/
