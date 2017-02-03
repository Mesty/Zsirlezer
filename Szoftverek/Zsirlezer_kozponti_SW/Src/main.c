/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile bool dataReady = false;
volatile bool tick = false;
volatile bool SHARP_valid;
volatile uint32_t previousmotorinput = 0;
volatile uint32_t actualmotorinput = 0;
bool stop = false;
bool stop_deadman = false;
bool stop_drone = false;
int32_t motorpulsePWM;
volatile uint32_t timestamp = 0;
volatile uint32_t actualencoderval=0;
volatile bool dovelocitymeasurement=false;
/*Encoder vars*/
uint32_t encoder1;
HAL_TIM_StateTypeDef state;
uint32_t encoderprev=0;
int32_t encoderdiff;
uint32_t dir;
//int32_t velocity;
//int32_t filteredvelocity;
//int32_t filteredvelocitydiff;
//int32_t velocityarray[4]={0,0,0,0}; //Kis csalas, akkor jo, ha VELOCITY_FILTER_DEPTH = 4
//uint32_t velocityabs;
/* SHARP vars */
uint16_t SHARPData[3] = {0,0,0};
int32_t SHARP_F = 0;
int32_t SHARP_R = 0;
int32_t SHARP_L = 0;
int32_t SHARP_F_ARRAY[4] = {0,0,0,0};
int32_t SHARP_R_ARRAY[4] = {0,0,0,0};
int32_t SHARP_L_ARRAY[4] = {0,0,0,0};
/* UART-on kuldento string */
uint8_t string[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint32_t reverse_byte_order_32(uint32_t value);
void drone();
void WMAfilter(int32_t* filteredval, int32_t* newelement, int32_t* array, uint32_t filter_depth);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1)
		SHARP_valid = true;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM4)
	{
		previousmotorinput = actualmotorinput;
		actualmotorinput = HAL_TIM_ReadCapturedValue(&htim4,TIM_CHANNEL_1);
		__HAL_TIM_SET_COUNTER(&htim4,0);
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		tick = true;
	}
	if(htim->Instance == TIM7)
	{
		timestamp++;
		actualencoderval=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		dovelocitymeasurement=true;
	}
}
/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
		HAL_UART_Receive_IT(huart, (uint8_t *) inputStream, sizeof(inputStream));
		dataReady = true;
	}
}*/
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  HAL_Delay(5000);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  motorpulsePWM = 7332;

  while (1)
  {
	  /* Deadman emergency brake */
	  if(((actualmotorinput < previousmotorinput) ? actualmotorinput : previousmotorinput) < 4000)
		  stop_deadman = true;
	  else
		  stop_deadman = false;
	  if (stop_deadman || stop_drone)
		  stop = true;
	  else
		  stop = false;

	  if (!stop)
		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, motorpulsePWM);
	  else if (encoderdiff > 10)
		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 4000);
	  else
		  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 6932);
	  /* SHARP olvasas es kuldes */
	  if(tick)
	  {
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) SHARPData, 3);
		  while(!SHARP_valid);
		  WMAfilter(SHARP_F,SHARPData[0],SHARP_F_ARRAY,4);
		  WMAfilter(SHARP_R,SHARPData[1],SHARP_R_ARRAY,4);
		  WMAfilter(SHARP_L,SHARPData[2],SHARP_L_ARRAY,4);
		  sprintf(&string,"..................\r\n");
		  sprintf(&string,".%d.%d.%d.",SHARP_F,SHARP_R,SHARP_L);
		  HAL_UART_Transmit(&huart4, &string, sizeof(string)*sizeof(uint8_t), 10000);
		  tick = 0;
	  }
	  /* Sebessegmeres */
	  if(dovelocitymeasurement) //v=[mm/s], delta t = 10ms
	  {//valami szamitas hibas, wma filter furi.
		  state = HAL_TIM_Encoder_GetState(&htim2);
		  if(state==HAL_TIM_STATE_READY)
		  {
			  dir = __HAL_TIM_DIRECTION_STATUS(&htim2);
			  encoder1=actualencoderval;
			  encoderdiff=encoder1-encoderprev;
			  //velocity=((encoderdiff*10000)/743); //v[mm/s]
			  //filteredvelocitydiff=-filteredvelocity;
			  //WMAfilter(&filteredvelocity, &velocity, velocityarray, 4);
			  /*if(filteredvelocity>=0)
				  velocityabs = filteredvelocity;
			  else
				  velocityabs = -filteredvelocity;*/
			  encoderprev=encoder1;
			  //filteredvelocitydiff+=filteredvelocity;
		  }
		  else
		  {
			  //error handling...
		  }
		  dovelocitymeasurement=false;
	  }
	  //drone();

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
uint32_t reverse_byte_order_32(uint32_t value)
{
	uint8_t lolo = (value >> 0) & 0xFF;
	uint8_t lohi = (value >> 8) & 0xFF;
	uint8_t hilo = (value >> 16) & 0xFF;
	uint8_t hihi = (value >> 24) & 0xFF;
	return (hihi << 0) | (hilo << 8) | (lohi << 16) | (lolo << 24);
}
void drone()
{
	static uint32_t encoderatstop = 0;
	static uint32_t timestampatfly = 0;
	if (SHARP_F > 1552)
		stop_drone = true;
	if ((stop_drone) && (encoderdiff == 0))
	{
		if (encoderatstop == 0)
		{
			encoderatstop = encoder1;
			timestampatfly = timestamp;
		}
		if ((SHARP_F < 1241) && (timestamp - timestampatfly > 200))
			stop_drone = false;
	}
	//if (encoder1 - encoderatstop > 2100) // Feladat vege, allapotvaltas, a 2100 itt random szam, a lenyeg, hogy uthosszt figyelunk a dron elotti megallasi helytol
}
void WMAfilter(int32_t* filteredval, int32_t* newelement, int32_t* array, uint32_t filter_depth)
{
	*filteredval=0;
	int32_t coeffsum=0;

	for(int i=0; i<filter_depth-1; i++)
	{
		array[i]=array[i+1];
		*filteredval=*filteredval+array[i]*(i+1);
		coeffsum+=(i+1);
	}
	array[filter_depth-1]=*newelement;
	*filteredval=*filteredval+array[filter_depth-1]*(filter_depth);
	coeffsum+=(filter_depth);
	*filteredval=*filteredval/coeffsum;
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
