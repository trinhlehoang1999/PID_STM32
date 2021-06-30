/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define Ts_ 0.01

uint8_t time_tick_1ms = 0;
uint8_t time_tick_10ms = 0;

float get_enc_value = 0;
float get_vel_value = 0;

float Kp = 0, Ki = 0, Kd = 0;

float Vc = 0;

float Vref = 0;

float ek=0 ,ek_1 = 0,ek_2 = 0;
float uk = 0,uk1 = 0, uk_1 = 0;

float rev_angle_enc = 0;

uint8_t ON = 0;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float my_motorVel(void);
float my_position(void);
float Saturation(float _uk);
void process_PID(float _Vref, float _Vc);
void reset_PID(void);
void PID_init(float _Kp,float _Ki,float _Kd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float my_motorVel()
{
	static int16_t enc_current = 0, enc_pre = 0;
	float vel= 0;
	enc_current = __HAL_TIM_GetCounter(&htim2);
	
	float pulse_cnt = enc_current - enc_pre;
	
	if (pulse_cnt >= 32768)
	{
		pulse_cnt -= 65536;
	}
	else if (pulse_cnt <= -32768)
	{
		pulse_cnt += 65536;
	}
	
	vel = (pulse_cnt*100*60.0)/1600;
	enc_pre = enc_current;
	return vel;
}

float my_position()
{
	static int16_t enc_count = 0;
	float rev_angle = 0;
	
	enc_count = __HAL_TIM_GetCounter(&htim2);
	rev_angle = enc_count*360.0/1600;
	
	return rev_angle;
}

float Saturation(float _uk)
{
	if(_uk < 0) _uk =  abs(_uk);

	if(_uk < 15) _uk = 0;

	if(_uk > 90) _uk = 90;

	return _uk;
}

void process_PID(float _Vref, float _Vc)
{
	ek = _Vref - _Vc;
	uk = uk_1 + Kp*(ek-ek_1)
				+ Ki*Ts_*(ek + ek_1)*0.5
				+ Kd*(ek-2*ek_1+ek_2)/Ts_;
	uk_1 = uk;
	ek_2 = ek_1;
	ek_1 = ek;
	
	uk1 = uk;
	
	if(uk1 > 0){
		uk1 = Saturation(uk1);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,uk1);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	}else if (uk1 < 0){
		uk1 = Saturation(uk1);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,uk1);
	} 
	else if (uk == 0){
		uk1 = Saturation(uk1);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
	}

}

void reset_PID()
{
	ek = 0;
	ek_1 = 0;
	ek_2 = 0;
	uk = 0;
	uk_1 = 0;
	rev_angle_enc = 0;
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
}

void PID_init(float _Kp,float _Ki,float _Kd)
{
	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
	ek = 0;
	ek_1 = 0;
	ek_2 = 0;
	uk = 0;
	uk_1 = 0;	
}
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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	 HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1|TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start_IT(&htim4);
	
	
	Vref = 0;
	ON = 0;
	reset_PID();
	//NVIC_SystemReset();
	//HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (time_tick_1ms == 1)
		{
			get_enc_value = TIM2->CNT;
			time_tick_10ms ++;
			time_tick_1ms = 0;
		}
			if (time_tick_10ms == 10)
			{
				rev_angle_enc = my_position();
				//Vc = my_motorVel();
				if (ON == 1)
				{
					process_PID(Vref,rev_angle_enc);
				}
				else
				{
					reset_PID();
					PID_init(Kp,Ki,Kd);
					rev_angle_enc = 0;
				}
				time_tick_10ms = 0;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		if(htim->Instance == TIM4)
		{
				time_tick_1ms = 1;
		}
}
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
