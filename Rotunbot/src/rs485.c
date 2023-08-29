#include "usart.h"
#include "main.h"
#include "can.h"
#include "dma.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "Rotunbot_Comm.h"
#include "rs485.h"
// RS485 发送使能
#define RS485_T (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET))
// RS485 接收使能
#define RS485_R (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET))
unsigned char USART1_MeBuf[20];  // 测量值寄存器
unsigned char USART1_TDBuf[20];  // 报警阈值寄存器
unsigned char USART1_RxBuft[40]; // 接受处理寄存器
unsigned char USART1_TxBuf[40];  // 发送寄存器
unsigned char USART1_FLME = 0;   // 测量值查询标志位
unsigned char USART1_FLTD = 0;   // 阈值查询标志位
unsigned char USART1_FLTM = 0;   // 上传时间更改标志位
unsigned short int crc;          // 16位crc校验值
unsigned short int w_times;      // 写入上传时间
unsigned short int crc1;
unsigned char RS485_rxflag;
uint8_t rDataBuffer[1]; //  RX Data buffer
uint8_t rDataCount_Me;  //  count Data bytes
uint8_t rDataCount_TD;
uint8_t rDataFlag = 0; //  waitting complete RX date having been send
uint8_t rxsize = 9;
int f, g, h;
void RS485_Receive_Data(uint8_t *buf, uint8_t *len);
void RS485_handle(void);
void RS485_send(void);
void TEST(void);
static unsigned short int getModbusCRC16(unsigned char *_pBuf, unsigned short int _usLen);

/**
 * @brief This function handles USART1 global interrupt.
 */

void USART1_IRQHandler(void)
{

    /* USER CODE BEGIN USART1_IRQn 0 */

    /* USER CODE END USART1_IRQn 0 */
    HAL_UART_IRQHandler(&huart1);
    /* USER CODE BEGIN USART1_IRQn 1 */

    /* USER CODE END USART1_IRQn 1 */
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        HAL_UART_Receive_IT(&huart1, USART1_RxBuft, 9);
        // USART1_RxBuft[rxsize]=rDataBuffer[0];
    }
}
/**
 * 功能：根据ModBus规则计算CRC16
 * 参数：
 *       _pBuf:待计算数据缓冲区,计算得到的结果存入_pBuf的最后两字节
 *       _usLen:待计算数据长度(字节数)
 * 返回值：16位校验值
 */
static unsigned short int getModbusCRC16(unsigned char *_pBuf, unsigned short int _usLen)
{
    unsigned short int CRCValue = 0xFFFF; // 初始化CRC变量各位为1
    unsigned char i, j;

    for (i = 0; i < _usLen; ++i)
    {
        CRCValue ^= *(_pBuf + i); // 当前数据异或CRC低字节
        for (j = 0; j < 8; ++j)   // 一个字节重复右移8次
        {
            if ((CRCValue & 0x01) == 0x01) // 判断右移前最低位是否为1
            {
                CRCValue = (CRCValue >> 1) ^ 0xA001; // 如果为1则右移并异或表达式
            }
            else
            {
                CRCValue >>= 1; // 否则直接右移一位
            }
        }
    }
    return CRCValue;
}
void RS485_Receive_Data(uint8_t *buf, uint8_t *len)

{

    uint8_t rxlen = rxsize;

    uint8_t i = 0;

    *len = 0;

    if (rxlen == rxsize && rxlen)

    {

        for (i = 0; i <= rxlen; i++)

        {

            buf[i + 2] = USART1_RxBuft[i];
        }

        *len = rxsize; // 记录本次数据长度

        // 清零
    }
}

void RS485_handle(void)
{
    crc = getModbusCRC16(USART1_RxBuft, rxsize - 2);                                         // 计算接收数组校验码
    crc1 = (USART1_RxBuft[rxsize - 1] & 0x00ff) << 8 | (USART1_RxBuft[rxsize - 2] & 0x00ff); // 取接收数组中的校验码
    if (crc == crc1)
    {
        if (USART1_RxBuft[1] == 0x03)
        {
            if (USART1_RxBuft[2] == 0x04)
            {
                rDataCount_Me = rxsize;
                for (int i = 0; i < 10; i++)
                {
                    USART1_MeBuf[i] = 0;
                }
                RS485_Receive_Data(USART1_MeBuf, &rDataFlag); // 记录测量数据
                USART1_MeBuf[0] = 0X55;
                USART1_MeBuf[1] = 0XAA;
                USART1_MeBuf[rDataCount_Me + 2] = 0XDD;
                for (int i = 0; i < rDataCount_Me; i++)
                {
                    USART1_RxBuft[i] = 0;
                }
            }
            else if (USART1_RxBuft[2] == 0x02)
            {
                rDataCount_TD = rxsize;
                for (int i = 0; i < 10; i++)
                {
                    USART1_TDBuf[i] = 0;
                }
                RS485_Receive_Data(USART1_TDBuf, &rDataFlag); // 记录报警阈值数据
                USART1_MeBuf[0] = 0X55;
                USART1_MeBuf[1] = 0XAA;
                USART1_MeBuf[rDataCount_TD + 2] = 0XDD;
                for (int i = 0; i < rDataCount_TD; i++)
                {
                    USART1_RxBuft[i] = 0;
                }
            }
        }
    }
    RS485_send();
}

void RS485_send(void)
{
    if (USART1_FLME != 0)
    {
        USART1_TxBuf[0] = 0x01;
        USART1_TxBuf[1] = 0x03;
        USART1_TxBuf[2] = 0x00;
        USART1_TxBuf[3] = 0xA4;
        USART1_TxBuf[4] = 0x00;
        USART1_TxBuf[5] = 0x02;
        crc = getModbusCRC16(USART1_TxBuf, 6);
        USART1_TxBuf[6] = crc & 0xff;
        USART1_TxBuf[7] = (crc >> 8) & 0xff;
        RS485_T;
        HAL_UART_Transmit(&huart1, (uint8_t *)&USART1_TxBuf, 8, 0xFF);
        RS485_R;
        USART1_FLME = 0;
    }
    else if (USART1_FLTD != 0)
    {
        USART1_TxBuf[0] = 0x01;
        USART1_TxBuf[1] = 0x03;
        USART1_TxBuf[2] = 0x00;
        USART1_TxBuf[3] = 0x9E;
        USART1_TxBuf[4] = 0x00;
        USART1_TxBuf[5] = 0x01;
        crc = getModbusCRC16(USART1_TxBuf, 6);
        USART1_TxBuf[6] = crc & 0xff;
        USART1_TxBuf[7] = (crc >> 8) & 0xff;
        RS485_T;
        HAL_UART_Transmit(&huart1, (uint8_t *)&USART1_TxBuf, 8, 0xFF);
        RS485_R;
        USART1_FLTD = 0;
    }
    else if (USART1_FLTM != 0)
    {
        USART1_TxBuf[0] = 0x01;
        USART1_TxBuf[1] = 0x06;
        USART1_TxBuf[2] = 0x00;
        USART1_TxBuf[3] = 0x12;
        USART1_TxBuf[4] = 0x00;
        USART1_TxBuf[5] = 0x01;
        crc = getModbusCRC16(USART1_TxBuf, 6);
        USART1_TxBuf[6] = crc & 0xff;
        USART1_TxBuf[7] = (crc >> 8) & 0xff;
        RS485_T;
        HAL_UART_Transmit(&huart1, (uint8_t *)&USART1_TxBuf, 8, 0xFF);
        RS485_R;
        USART1_FLTM = 0;
    }
}

void RS485_Transmit_Data(uint8_t *pData)
{
    RS485_T;
    HAL_UART_Transmit(&huart1, (uint8_t *)pData, 1, 0x10);
    RS485_R;
}

void exdevFeedback(void)
{
    if ((unsigned char)USART1_MeBuf[11] == 0xDD) // 若已经可以执行回传
    // uint8_t fake_value[20] = {0x55, 0xAA, 0x01, 0x03, 0x04, 0x3E, 0x59, 0xE8, 0x9B, 0x28, 0x63, 0xDD};
    {
        udpClientSend((char *)USART1_MeBuf, 12); // 将UDP_MessagetoSend数组传给 udpClientSend 函数
        USART1_MeBuf[11] = 0xFF;                 // 帧尾设置为0xFF，表示目前不能执行回传
    }
}
