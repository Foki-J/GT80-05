/**
  *****************************************************************************
  * @file    Rotunbot_Comm.h
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

#ifndef __ROTUNBOT_COMM_H_
#define __ROTUNBOT_COMM_H_

/* 头文件 */
#include "stm32f4xx_hal.h"
#include "lwip.h"
#include "udp.h"
#include "string.h"
#include "Rotunbot_vary.h"
#include "usart.h"

/* 函数定义 */
// 前照明打开
#define LIGHT_BACK_ON   (HAL_GPIO_WritePin(LIGHT_BACK_GPIO_Port, LIGHT_BACK_Pin, GPIO_PIN_SET))
//前照明关闭
#define LIGHT_BACK_OFF  (HAL_GPIO_WritePin(LIGHT_BACK_GPIO_Port, LIGHT_BACK_Pin, GPIO_PIN_RESET))
// 后照明打开
#define LIGHT_FRONT_ON   (HAL_GPIO_WritePin(LIGHT_FRONT_GPIO_Port, LIGHT_FRONT_Pin, GPIO_PIN_SET))
//后照明关闭
#define LIGHT_FRONT_OFF  (HAL_GPIO_WritePin(LIGHT_FRONT_GPIO_Port, LIGHT_FRONT_Pin, GPIO_PIN_RESET))	
//扬声器开
#define SPEAKER_ON (HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_SET))	
//扬声器关
#define SPEAKER_OFF (HAL_GPIO_WritePin(SPEAKER_GPIO_Port, SPEAKER_Pin, GPIO_PIN_RESET))	
static void udpReceiveCallback(void *arg, struct udp_pcb *upcb,
    struct pbuf *p, const ip_addr_t *addr, u16_t port);

void udpClientSend(char *pData, int len);

void udpClientInit(void);

void send_ANO_msg(short* msg);
#endif /* ROTUNBOT_COMM_H_ */
