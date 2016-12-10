/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
float Simulator_dt;
uint32_t RobotState_status;
uint64_t RobotState_timestamp;
float RobotState_x;
float RobotState_v;
float RobotState_a;
uint8_t RobotState_light;
volatile uint8_t tick;
volatile uint8_t dataReady;
uint8_t inputStream[25];
uint32_t receivedState_status;
uint64_t receivedState_timestamp;
float receivedState_x;
float receivedState_v;
float receivedState_a;
uint8_t receivedState_light;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Simulator_start(float intervalsec);
void Simulator_tick();
void Simulator_dataReady();
void Uart_sendstate(UART_HandleTypeDef *huart, uint32_t status, uint64_t timestamp, float x, float v, float a, uint8_t light, uint32_t Timeout);
void Simulator_ReadFrom();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
  //HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  //HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim6);
  Simulator_start(90000000/1373/65501);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  if(tick == 1)
	  {
		  Simulator_tick();
		  tick = 0;
	  }
	  if(dataReady == 1)
	  {
		  Simulator_dataReady();
		  dataReady = 0;
	  }

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
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
void Simulator_start(float intervalsec)
{
	Simulator_dt = intervalsec;
	RobotState_status = ROBOTSTATE_STATUS_DEFAULT;
	RobotState_timestamp = 0;
	RobotState_x = 0.0F;
	RobotState_v = 0.0F;
	RobotState_a = 0.0F;
	RobotState_light = 0;
}
void Simulator_tick()
{
	RobotState_timestamp = RobotState_timestamp + Simulator_dt;
	RobotState_x = RobotState_x + RobotState_v * Simulator_dt;
	RobotState_v = RobotState_v + RobotState_a * Simulator_dt;
	if(RobotState_v < -10.0)
	{
		RobotState_v = -10.0F;
	}
	if(RobotState_v > 10.0)
	{
		RobotState_v = 10.0F;
	}

	RobotState_light = RobotState_v == 10.0F ? 1.0F : 0.0F;

	switch(RobotState_status)
	{
	case ROBOTSTATE_STATUS_DEFAULT:
		break;
	case ROBOTSTATE_STATUS_RESET:
		RobotState_status = ROBOTSTATE_STATUS_RESET;
		RobotState_x = 0.0F;
		RobotState_v = 0.0F;
		RobotState_a = 0.0F;
		RobotState_light = 0;
		break;
	case ROBOTSTATE_STATUS_STOPPING:
		if(RobotState_v > 1.5F)
		{
			RobotState_a = -1.0F;
		}
		else if(RobotState_v > 0.1F)
		{
			RobotState_a = -0.05F;
		}
		else if(RobotState_v < -1.5F)
		{
			RobotState_a = 1.0F;
		}
		else if(RobotState_v < -0.1F)
		{
			RobotState_a = 0.05F;
		}
		else
		{
			RobotState_status = ROBOTSTATE_STATUS_DEFAULT;
			RobotState_a = 0.0F;
		}
		break;
	case ROBOTSTATE_STATUS_ACCELERATE:
		break;
	default:
		break;
	}

	Uart_sendstate(&huart4, RobotState_status, RobotState_timestamp, RobotState_x, RobotState_v, RobotState_a, RobotState_light, 100000);
}
void Simulator_dataReady()
{
	Simulator_ReadFrom();

	switch(receivedState_status)
	{
	case ROBOTSTATE_STATUS_DEFAULT:
		break;
	case ROBOTSTATE_STATUS_RESET:
		RobotState_status = ROBOTSTATE_STATUS_RESET;
		break;
	case ROBOTSTATE_STATUS_STOPPING:
		RobotState_status = ROBOTSTATE_STATUS_STOPPING;
		break;
	case ROBOTSTATE_STATUS_ACCELERATE:
		RobotState_status = ROBOTSTATE_STATUS_DEFAULT;
		RobotState_a = receivedState_a;
		break;
	default:
		break;
	}
}
void Simulator_ReadFrom()
{
	receivedState_status = *(&(inputStream[0]));
	receivedState_timestamp = *(&(inputStream[4]));
	receivedState_x = *(&(inputStream[12]));
	receivedState_v = *(&(inputStream[16]));
	receivedState_a = *(&(inputStream[20]));
	receivedState_light = inputStream[24];
}
/*void szervoPszabalyozo(int16_t *vonalpozicio)
{
	  uint16_t servo_pulse;
	  servo_pulse = (-10*(*vonalpozicio))*KC+6707;
	  if (servo_pulse>7407)
		  servo_pulse=7407;
	  if (servo_pulse<6007)
		  servo_pulse=6007;
	  MX_TIM1_Init(servo_pulse);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		tick = 1;
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		HAL_UART_Receive_IT(huart, inputStream, sizeof(inputStream));
		dataReady = 1;
	}
}
void Uart_sendstate(UART_HandleTypeDef *huart, uint32_t status, uint64_t timestamp, float x, float v, float a, uint8_t light, uint32_t Timeout)
{
	uint8_t i;
	uint8_t state[25];
	for(i = 0; i < 4; ++i)
	{
		state[i] = *((&status)+i);
	}
	for(i = 0; i < 8; ++i)
	{
		state[i+4] = *((&timestamp)+i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+12] = *((&x)+i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+16] = *((&v)+i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+20] = *((&a)+i);
	}
	state[24] = light;
	HAL_UART_Transmit(huart, state, sizeof(state), Timeout);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
