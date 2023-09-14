/* ZBX for Wheel Spherical */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "lwip.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Rotunbot_vary.h"
#include "Rotunbot_Comm.h"
#include "Rotunbot_Pwm.h"
#include "Rotunbot_Can.h"
#include "Rotunbot_Algorithm.h"
#include "rs485.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SEV_BUFF_SIZE 10
#define _ABS(x) ((x > 0) ? (x) : (-x))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned int timer = 0;
short current_msg[4];
short sec_buff[SEV_BUFF_SIZE];
int buff_counter = 0;
int pos = 0;
int pre_GyroI_Align_x = 0, mo_wheel_out_last = 0, mo_wheel_out = 0; // �Ǽ��ٶȵ�ͨ�˲����
unsigned char Signal_3_State_last = 0;
unsigned char netgunCommand = 0;
unsigned char shootFlag = 0;
float v_inc = 0;
float ff_coef = 0.00; // ת��ǰ����ϵ��
float abs_velocity_ball = 0.0;
int shootCounter = 0; // ��ǹ������ʱ
short Pendulum_Ver_Angle = 0;
char Flag_Battery_Timer = 0;
int velocity_sum = 0;
const short MAX_ANGULER = 3000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM8_Init();
    MX_UART5_Init();
    MX_USART1_UART_Init();

    MX_LWIP_Init();
    MX_TIM7_Init();
    /* USER CODE BEGIN 2 */
    pwmInit();
    udpClientInit();
    paramInit();
    CAN_Config();
    //	LIGHT_FRONT_ON;
    //	LIGHT_BACK_ON;
    SPEAKER_ON;
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    while (1)
    {
        /* UDPͨ�ų��� */
        MX_LWIP_Process();
        if (!Flag_PID_Control_Timer)
            continue;
        Flag_PID_Control_Timer = 0;
        // �����������������������������������������������������������������vx,roll��������������������������������������������������������//
        // ��ң������ÿ�����
        // Roll_Hope�����������ǣ���Χ-1700~1700����Ӧ-17��~17��
        // Velocity_Hope�������ٶȣ���Χ-700~700����Ӧ-7m/s~7m/s��
        if (Remote_Control_Counter > 100)
            RemoteRx();

        // ����������������������������������������������������������������ᡢ�������롪����������������������������������������������������//
        //	������					������													����			����
        //  �ذڸ������ٶ� IMU_CAN_Message.GyroI_Align_y    0.01��/s	��
        //  �ذڸ����Ƕ�	 IMU_CAN_Message.Euler_y					0.01��		��
        //  �ذڷ������ٶ� Velocity_SecondAxis+							0.01��/s	ǰ
        //								IMU_CAN_Message.GyroI_Align_x;
        //  �ذڷ����Ƕ�	 Pendulum_Ver_Angle								0.01��		ǰ
        //  �򷭹����ٶ�	 IMU_CAN_Message.GyroI_Align_x	  0.01��/s  ǰ
        //  �򷭹��Ƕ�		 IMU_CAN_Message.Euler_x					0.01��		ǰ
        //  ���ǰ���ٶ�	 Velocity_FirstAxis+
        //								IMU_CAN_Message.GyroI_Align_y   0.01��/s  ��
        //  ŷ����				 IMU_CAN_Message.Euler_						0.01��		ǰ����
        //  ŷ�����ٶ�		 IMU_CAN_Message.GyroI_Align_			0.01��/s	ǰ����
        //  ������ת��	 Velocity_FirstAxis								0.01��/s  ��
        //  ������ת��	 Velocity_SecondAxis							0.01��/s 	ǰ
        //  �������Ƕ�	 Position_SecondAxis							0.01��		ǰ

        for (int i = 1; i < 5; i++)
        {
            if (abs_velocity_ball < PIDBox_ThirdAxis[3][i])
            { // ����PID����
                v_inc = (abs_velocity_ball - PIDBox_ThirdAxis[3][i - 1]) / (PIDBox_ThirdAxis[3][i] - PIDBox_ThirdAxis[3][i - 1]);
                PID_ThirdAxis.Kp = v_inc * (PIDBox_ThirdAxis[0][i] - PIDBox_ThirdAxis[0][i - 1]) + PIDBox_ThirdAxis[0][i - 1];
                PID_ThirdAxis.Ki = v_inc * (PIDBox_ThirdAxis[1][i] - PIDBox_ThirdAxis[1][i - 1]) + PIDBox_ThirdAxis[1][i - 1];
                PID_ThirdAxis.Kd = v_inc * (PIDBox_ThirdAxis[2][i] - PIDBox_ThirdAxis[2][i - 1]) + PIDBox_ThirdAxis[2][i - 1];
                break;
            }
        }
        //		//����ת��ʱ������������//��δ����----------------------------------------------------------------------------------------------------------------------------------------
        //		ff_coef = (_ABS(Velocity_Hope)<200)?(0.002*_ABS(Velocity_Hope)):(0.8-0.0027*_ABS(Velocity_Hope));
        //		ff_coef = (ff_coef<0)?0:ff_coef;
        ff_coef = asin(Velocity_Hope / 100 * Velocity_Hope / 100 * tan(Roll_Hope / 18000.0 * PI) * m_ball / m_pendulum / 9.8 / l_pendulum) * 18000.0 / PI + Roll_Hope;
        if (ff_coef > 2800)
            ff_coef = 2800; // ������
        else if (ff_coef < -2800)
            ff_coef = -2800; // ������
        // ���㸱���λ��
        pos = (PID_Loc(Roll_Hope, IMU_CAN_Message.Euler_x, -IMU_CAN_Message.GyroI_Align_x, &PID_SecondAxis)+motor_orig_angle + ff_coef) * 22.6f; //λ��ģʽ
        //sec = PID_Loc(Roll_Hope, IMU_CAN_Message.Euler_x, -IMU_CAN_Message.GyroI_Align_x, &PID_SecondAxis) + ff_coef * Roll_Hope;//����ģʽ
        // ���㶯���ֵ�����/���ٶȴ���ͨ�˲���
        float alpha = (2.0f * PI * 3.0f * 0.02f) / (2.0f * PI * 3.0f * 0.02f + 1.0f);
        mo_wheel_out = mo_wheel_out_last + alpha * (IMU_CAN_Message.GyroI_Align_x + pre_GyroI_Align_x - mo_wheel_out_last);
        third = PID_Loc(0, IMU_CAN_Message.GyroI_Align_x, mo_wheel_out_last - mo_wheel_out, &PID_ThirdAxis);
        pre_GyroI_Align_x = IMU_CAN_Message.GyroI_Align_x;
        mo_wheel_out_last = mo_wheel_out;
        // �����ƽ���
        v_force = V_Hope / 2;
        w_force = PID_Loc(W_Hope, IMU_CAN_Message.GyroI_Align_z, (PID_W.Ek - PID_W.Ek1), &PID_W);

        // �������������ٶȣ���ֹ����
				short Gyro_y_Hope = IMU_CAN_Message.GyroI_Align_y;
				if(IMU_CAN_Message.Euler_y>0)
				{
					if(IMU_CAN_Message.GyroI_Align_y>(-MAX_ANGULER*IMU_CAN_Message.Euler_y/9000+MAX_ANGULER)) //ɲ������
					{
						Gyro_y_Hope = -MAX_ANGULER*IMU_CAN_Message.Euler_y/9000+MAX_ANGULER;
						Velocity_Hope = (Velocity_FirstAxis+IMU_CAN_Message.GyroI_Align_y - Gyro_y_Hope)*0.007;
					}
					else if(IMU_CAN_Message.GyroI_Align_y<-MAX_ANGULER)
					{
						Gyro_y_Hope = -MAX_ANGULER;
						Velocity_Hope = (Velocity_FirstAxis+IMU_CAN_Message.GyroI_Align_y - Gyro_y_Hope)*0.007;
					}
				}
				else
				{
					if(IMU_CAN_Message.GyroI_Align_y<(-MAX_ANGULER*IMU_CAN_Message.Euler_y/9000-MAX_ANGULER))
					{
						Gyro_y_Hope = -MAX_ANGULER*IMU_CAN_Message.Euler_y/9000-MAX_ANGULER;
						Velocity_Hope = (Velocity_FirstAxis+IMU_CAN_Message.GyroI_Align_y - Gyro_y_Hope)*0.007;
					}
					else if(IMU_CAN_Message.GyroI_Align_y>MAX_ANGULER)
					{
						Gyro_y_Hope = MAX_ANGULER;
						Velocity_Hope = (Velocity_FirstAxis+IMU_CAN_Message.GyroI_Align_y - Gyro_y_Hope)*0.007;
					}
				}

        // �����������������������������������������������������������·����������������������������������������������������������������//
        //  ���
        int v_command = 0;
        v_command = Velocity_Hope * 15455.57 ;// 7 * 5;        				// ���ٶ�ֵ�Ŵ�������������
				if((driver_state_word&0x38) == 0x38)
					firstAxisVelMode(v_command >> 24, v_command >> 16, v_command >> 8, v_command); // �����ٶ�ģʽ
        if((driver_state_word&0x07) == 0x07)
					secondAxisPosMode(pos >> 24, pos >> 16, pos >> 8, pos); // ����λ��ģʽ
        if((driver_state_word&0x1C0) == 0x1C0)
					thirdAxisCurrentMode(third >> 8, third);
				
        // �ƽ���
        /**��ң��������**/
        right = ((v_force + w_force) > 500) ? 500 : (v_force + w_force);
        right = (left < (-500)) ? (-500) : right;
        left = right - 2 * w_force;
        if (left > 500)
        {
            left = 500;
            right = left + 2 * w_force;
        }
        else if (left < -500)
        {
            left = -500;
            right = left + 2 * w_force;
        }
        /***************/
        // �·��ƽ���ת��
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 20000 - 1500 - right);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 20000 - 1500 - left);

        // ������������������������������������������������������������������������������������������������������������������������//
        Remote_Control_Counter++;
        driver_PDO_timer++;
        if (Flag_LED_Toggle_Timer) // LED��˸
        {
            Flag_LED_Toggle_Timer = 0;
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            RS485_handle();
            if (driver_PDO_timer > 100)
            {
                driver_state_word &= 0x124;
            }
            DriverEnable(); // ������ʹ��

        }
        velocity_sum += (Velocity_FirstAxis + IMU_CAN_Message.GyroI_Align_y);
        short msg[8];
        msg[0] = IMU_CAN_Message.GyroI_Align_x;
        msg[1] = IMU_CAN_Message.Euler_x;
        msg[2] = Velocity_FirstAxis + IMU_CAN_Message.GyroI_Align_y;
        msg[3] = Velocity_Hope;
        msg[4] = Roll_Hope;
        msg[5] = Signal_1_High;
        msg[6] = driver_state_word;
        msg[7] = driver_state_word;
        //send_ANO_msg(msg);

        if (Flag_Feedback_Timer)
        {
            Flag_Feedback_Timer = 0;
            imuFeedback();
            if ((driver_state_word & 0x24) == 0x00)
            {
                first = 0;
                sec = 0;
                third = 0;
                Velocity_FirstAxis = 0;
                Velocity_SecondAxis = 0;
                Velocity_ThirdAxis = 0;
                left = 0;
                right = 0;
            }
            motorFeedback();
            HAL_UART_Receive_IT(&huart1, USART1_RxBuft, 9);
        }
        else
        {
            HAL_GPIO_TogglePin(CAMERA_TRIG_GPIO_Port, CAMERA_TRIG_Pin);
        }
        // ��������������������������������������������������������������ط���ָ�����������������������������������������������������������//
        if (Flag_Battery_Timer == 1) // ���
        {
            Flag_Battery_Timer = 0;
            BatteryFeedback(); // ��������
					  internalStateFeedback();
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 192;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/*	brief: ��ʱ��3����жϺ���������10ms���ڲ����з�Ƶ���õ�����20ms��200ms�ȱ�־λ
 *
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim3))
    {
        timer++;                    // ÿ10msһ�μ���
        Flag_PID_Control_Timer = 1; // ÿ10ms����һ��pid���� 100Hz
        if (timer % 2 == 0)
            Flag_Feedback_Timer = 1; // ÿ20ms����λ������һ������ 50Hz
        if (timer % 1000 == 0)
            Flag_Battery_Timer = 1; // ÿ10000ms�õ���ص�����
        if (timer % 100 == 0)
        {
            Flag_LED_Toggle_Timer = 1; // LEDÿ1000ms��תһ�� ��˸Ƶ��0.5Hz
            USART1_FLME = 1;
        }
    }
}
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart1)
//	{
//		HAL_UART_Receive_IT(&huart1, USART1_RxBuf, 59);
//	}
// }
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
