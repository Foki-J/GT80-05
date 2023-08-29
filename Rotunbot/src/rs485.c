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
// RS485 ����ʹ��
#define RS485_T (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET))
// RS485 ����ʹ��
#define RS485_R (HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET))
unsigned char USART1_MeBuf[20];  // ����ֵ�Ĵ���
unsigned char USART1_TDBuf[20];  // ������ֵ�Ĵ���
unsigned char USART1_RxBuft[40]; // ���ܴ���Ĵ���
unsigned char USART1_TxBuf[40];  // ���ͼĴ���
unsigned char USART1_FLME = 0;   // ����ֵ��ѯ��־λ
unsigned char USART1_FLTD = 0;   // ��ֵ��ѯ��־λ
unsigned char USART1_FLTM = 0;   // �ϴ�ʱ����ı�־λ
unsigned short int crc;          // 16λcrcУ��ֵ
unsigned short int w_times;      // д���ϴ�ʱ��
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
 * ���ܣ�����ModBus�������CRC16
 * ������
 *       _pBuf:���������ݻ�����,����õ��Ľ������_pBuf��������ֽ�
 *       _usLen:���������ݳ���(�ֽ���)
 * ����ֵ��16λУ��ֵ
 */
static unsigned short int getModbusCRC16(unsigned char *_pBuf, unsigned short int _usLen)
{
    unsigned short int CRCValue = 0xFFFF; // ��ʼ��CRC������λΪ1
    unsigned char i, j;

    for (i = 0; i < _usLen; ++i)
    {
        CRCValue ^= *(_pBuf + i); // ��ǰ�������CRC���ֽ�
        for (j = 0; j < 8; ++j)   // һ���ֽ��ظ�����8��
        {
            if ((CRCValue & 0x01) == 0x01) // �ж�����ǰ���λ�Ƿ�Ϊ1
            {
                CRCValue = (CRCValue >> 1) ^ 0xA001; // ���Ϊ1�����Ʋ������ʽ
            }
            else
            {
                CRCValue >>= 1; // ����ֱ������һλ
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

        *len = rxsize; // ��¼�������ݳ���

        // ����
    }
}

void RS485_handle(void)
{
    crc = getModbusCRC16(USART1_RxBuft, rxsize - 2);                                         // �����������У����
    crc1 = (USART1_RxBuft[rxsize - 1] & 0x00ff) << 8 | (USART1_RxBuft[rxsize - 2] & 0x00ff); // ȡ���������е�У����
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
                RS485_Receive_Data(USART1_MeBuf, &rDataFlag); // ��¼��������
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
                RS485_Receive_Data(USART1_TDBuf, &rDataFlag); // ��¼������ֵ����
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
    if ((unsigned char)USART1_MeBuf[11] == 0xDD) // ���Ѿ�����ִ�лش�
    // uint8_t fake_value[20] = {0x55, 0xAA, 0x01, 0x03, 0x04, 0x3E, 0x59, 0xE8, 0x9B, 0x28, 0x63, 0xDD};
    {
        udpClientSend((char *)USART1_MeBuf, 12); // ��UDP_MessagetoSend���鴫�� udpClientSend ����
        USART1_MeBuf[11] = 0xFF;                 // ֡β����Ϊ0xFF����ʾĿǰ����ִ�лش�
    }
}
