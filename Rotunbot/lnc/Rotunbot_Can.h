/**
  *****************************************************************************
  * @file    Rotunbot_Can.h
  * @author  决策控制组
  * @version V1.0.0
  * @date    2022-04-28
  * @brief   与电机驱动器通信以控制电机
  *****************************************************************************
  * @history
  *
  * 1. Date:2022-04-28
  *    Author:林柏羽
  *    Modification:创建文件
  *
  *****************************************************************************
  */

#ifndef __ROTUNBOT_CAN_H_
#define __ROTUNBOT_CAN_H_

#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "usart.h"
#include "tim.h"
#include "Rotunbot_struct.h"
#include "Rotunbot_vary.h"
#include "Rotunbot_Comm.h"

/*debug*/
#define CAN_DEBUG_ON         1
#define CAN_DEBUG_ARRAY_ON   1
#define CAN_DEBUG_FUNC_ON    1

extern uint8_t Battery_Soc;			//电池SOC
extern uint16_t Battery_Capacity;//电池容量
extern uint16_t Battery_Voltage;//电池电压
extern uint16_t Battery_Current;//电池电流
extern uint16_t Power_Nominal_Capacity; //电池标称电压
   
// Log define
#define CAN_INFO(fmt,arg...)           printf("<<-CAN-INFO->> "fmt"\n",##arg)
#define CAN_ERROR(fmt,arg...)          printf("<<-CAN-ERROR->> "fmt"\n",##arg)
#define CAN_DEBUG(fmt,arg...)          do{\
                                         if(CAN_DEBUG_ON)\
                                         printf("<<-CAN-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#define CAN_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(CAN_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("<<-CAN-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define CAN_DEBUG_FUNC()               do{\
                                         if(CAN_DEBUG_FUNC_ON)\
                                         printf("<<-CAN-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)

void CAN_SetMsg_battery(void);
void BatteryFeedback(void);
void Battery_send(void);
void internalStateFeedback(void);
													
extern short motor_orig_angle;
extern char driver_power;			
extern char driver_power_last;	
extern unsigned short driver_state_word;		
extern short driver_PDO_timer;																			 
void CAN_sendFirstAxis(void);
void CAN_sendSecondAxis(void);						   
void CAN_sendThirdAxis(void);
void firstAxisModeSelect(int mode);									   
void firstAxisCurrentMode(int cc1, int cc2);
void firstAxisVelMode(int vv1, int vv2, int vv3, int vv4);
void firstAxisPosMode(int pvp1, int pvp2, int pvp3, int pvp4);
void firstAxisVelPosMode(int vvp1, int vvp2, int vvp3, int vvp4, int pvp1, int pvp2, int pvp3, int pvp4);

void secondAxisModeSelect(int mode);
void secondAxisCurrentMode(int cc1, int cc2);
void secondAxisVelMode(int vv1, int vv2, int vv3, int vv4);
void secondAxisPosMode(unsigned char pvp1, unsigned char pvp2, unsigned char pvp3, unsigned char pvp4);
void secondAxisVelPosMode(int vvp1, int vvp2, int vvp3, int vvp4, int pvp1, int pvp2, int pvp3, int pvp4);
void secondAxisUnlockMode(void);

void thirdAxisModeSelect(int mode);
	void thirdAxisCurrentMode(int cc1, int cc2);
	void thirdAxisVelMode(int vv1, int vv2, int vv3, int vv4);
	void thirdAxisPosMode(int pvp1, int pvp2, int pvp3, int pvp4);
	void thirdAxisVelPosMode(int vvp1, int vvp2, int vvp3, int vvp4, int pvp1, int pvp2, int pvp3, int pvp4);
	void thirdAxisUnlockMode(void);

void parameterSave(void);
void imuFeedback(void);
void driverStateFeedback(void);
void motorFeedback(void);
void DriverEnable(void);			
void CAN_Config(void);
void Init_RxMes(void);
void CAN_SetMsg(unsigned short int id);

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
								   
static void CAN_Filter_Config(void);

#endif /* __ROTUNBOT_CAN_H_ */
