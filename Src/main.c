
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile uint8_t WatchDog_Flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define TIME_OUT	2200
#define MAX 2000
#define MIN 200

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	GPIO_PinState old_pin_state = GPIO_PIN_RESET;
	GPIO_PinState pin_state = GPIO_PIN_RESET;
	__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim1);
	uint8_t sleep = 0;
	uint32_t i = 0;
	volatile uint32_t old_time = 0;
	volatile uint32_t now_time = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(WatchDog_Flag >= 4)
		{
			HAL_TIM_Base_Stop_IT(&htim1);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
			HAL_Delay(60000);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
			htim1.Instance->CNT = 0;
			__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim1);
			WatchDog_Flag = 0;
		}
		pin_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
		if(pin_state != old_pin_state)
		{
			//reset timer and watchdog_flag
			WatchDog_Flag = 0;
			HAL_TIM_Base_Stop_IT(&htim1);
			htim1.Instance->CNT = 0;
			__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
			HAL_TIM_Base_Start_IT(&htim1);
			old_pin_state = pin_state;
		}
		
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_SET)
		{
			htim3.Instance->CNT = 0;
			HAL_TIM_Base_Start(&htim3);
			sleep = 1;
			
			old_time = htim3.Instance->CNT;
			while((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_SET) && sleep)
			{
				now_time = htim3.Instance->CNT;
				if((now_time - old_time) > TIME_OUT)
				{
//					sleep = 0;
					break;
				}
			}
			if(((now_time - old_time) < MAX) && ((now_time - old_time) > MIN))
				sleep = 1;
			else
				sleep = 0;
			
			old_time = htim3.Instance->CNT;
			while((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_RESET) && sleep)
			{
				now_time = htim3.Instance->CNT;
				if((now_time - old_time) > TIME_OUT)
				{
//					sleep = 0;
					break;
				}
			}
			if(((now_time - old_time) < MAX) && ((now_time - old_time) > MIN))
				sleep = 1;
			else
				sleep = 0;
			
			old_time = htim3.Instance->CNT;
			while((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_SET) && sleep)
			{
				now_time = htim3.Instance->CNT;
				if((now_time - old_time) > TIME_OUT)
				{
//					sleep = 0;
					break;
				}
			}
			if(((now_time - old_time) < MAX) && ((now_time - old_time) > MIN))
				sleep = 1;
			else
				sleep = 0;
			
			old_time = htim3.Instance->CNT;
			while((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_RESET) && sleep)
			{
				now_time = htim3.Instance->CNT;
				if((now_time - old_time) > TIME_OUT)
				{
//					sleep = 0;
					break;
				}
			}
			if(((now_time - old_time) < MAX) && ((now_time - old_time) > MIN))
				sleep = 1;
			else
				sleep = 0;
			
			old_time = htim3.Instance->CNT;
			while((HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2) == GPIO_PIN_SET) && sleep)
			{
				now_time = htim3.Instance->CNT;
				if((now_time - old_time) > TIME_OUT)
				{
//					sleep = 0;
					break;
				}
			}
			if(((now_time - old_time) < MAX) && ((now_time - old_time) > MIN))
				sleep = 1;
			else
				sleep = 0;
			
			if(sleep == 1)
			{
				//stop timer1
				HAL_TIM_Base_Stop_IT(&htim1);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
				for(i = 0; i < 1200; i++)
					HAL_Delay(1000);
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
				//restart timer 1
				__HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_UPDATE);
				HAL_TIM_Base_Start_IT(&htim1);
				WatchDog_Flag = 0;
			}
			HAL_TIM_Base_Stop(&htim3);
		}

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim1.Instance)
		WatchDog_Flag++;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
