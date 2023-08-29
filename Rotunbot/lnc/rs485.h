#ifndef __rs485_H__
#define __rs485_H__
void RS485_Transmit_Data(unsigned char *pData);
void exdevFeedback(void);
extern unsigned char rDataBuffer[1];
#endif
