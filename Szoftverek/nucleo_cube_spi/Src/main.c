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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
 volatile bool adcvalid=false;
 uint16_t adcmeasuredvalues[3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t send8bit_to_uart(UART_HandleTypeDef* huart, uint8_t* data,uint32_t Timeout);
uint8_t send32bitdecimal_to_uart(UART_HandleTypeDef* huart, uint32_t* data,uint32_t Timeout);
uint8_t send16bitdecimal_to_uart(UART_HandleTypeDef* huart, uint16_t* data,uint32_t Timeout);
uint8_t sendadcvals_to_uart(UART_HandleTypeDef* huart, uint16_t* data1,uint16_t* data2, uint16_t* data3,uint32_t Timeout);
uint8_t sendlineregisters_to_uart(UART_HandleTypeDef* huart, uint8_t* data1,uint8_t* data2, uint8_t* data3,uint32_t Timeout);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcvalid = true;
}

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
  MX_USART2_UART_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
  /*Initialize variables for main()*/
  uint8_t spidata = 0b00000010;

  uint8_t endline=10;
  uint8_t CR=13;
  uint8_t tab=9;
  uint8_t space=32;
  uint8_t line_register1=0;
  uint8_t line_register2=0;
  uint8_t line_register3=0;
  uint16_t adcvalregister1[8];
  uint16_t adcvalregister2[8];
  uint16_t adcvalregister3[8];
  uint16_t threshold=1000;
  uint8_t i=0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

	  /*SPI communication*/
	  /*If only 3 sequence is sent, then something is wrong with the bits (not the good bits are received by the LED drivers.
	   Dont know, whats the problem.*/
	  for(int i=0; i<6; i++)
	  {
		  HAL_SPI_Transmit(&hspi3,&spidata,1,10);
	  }
	  /*LE signal output*/
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_SET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_RESET);
	  HAL_Delay(1);

	  adcvalid=false;
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcmeasuredvalues, 3);
	  while (!adcvalid);

	  /*ADC single value read in*/
	  /*HAL_ADC_Start(&hadc1);
	  if (HAL_ADC_PollForConversion(&hadc1,100)==HAL_OK)
		  adcmeasuredval=HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);*/

	  adcvalregister1[i]=adcmeasuredvalues[0];
	  adcvalregister2[i]=adcmeasuredvalues[1];
	  adcvalregister3[i]=adcmeasuredvalues[2];

	  if (adcvalregister1[i]>threshold)
		  line_register1+=spidata;
	  if (adcvalregister2[i]>threshold)
		  line_register2+=spidata;
	  if (adcvalregister3[i]>threshold)
			  line_register3+=spidata;
	  i++;

	  /*State machine (8 state) for SPI data and MUX signals*/
	  if (spidata==0b10000000)
	  {
		  sendlineregisters_to_uart(&huart2, &line_register1, &line_register2, &line_register3, 1000);
		 // sendadcvals_to_uart(&huart2, adcvalregister1, adcvalregister2, adcvalregister3, 1000);

		  line_register1=0;
		  line_register2=0;
		  line_register3=0;
		  spidata=0b00000001;
		  i=0;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_SET);

	  }
	  else if (spidata==0b00000001)
	  {
		  spidata=0b00000010;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_SET);

	  }
	  else if (spidata==0b00000010)
	  {
		  spidata=0b00000100;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_SET);

	  }
	  else if (spidata==0b00000100)
	  {
		  spidata=0b00001000;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_SET);

	  }
	  else if (spidata==0b00001000)
	  {
		  spidata=0b00010000;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_RESET);

	  }
	  else if (spidata==0b00010000)
	  {
		  spidata=0b00100000;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_RESET);

	  }
	  else if (spidata==0b00100000)
	  {
		  spidata=0b01000000;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_RESET);

	  }
	  else if (spidata==0b01000000)
	  {
		  spidata=0b10000000;
		  HAL_GPIO_WritePin(MUX1_GPIO_Port,MUX1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX2_GPIO_Port,MUX2_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX3_GPIO_Port,MUX3_Pin,GPIO_PIN_RESET);

	  }

	 // HAL_Delay(10);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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

  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

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

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/*Converts 8bit binary number to 8 character (e.g.: 00010001), and sends on UART*/
/*Warning! Reihenfolge ist umgekehrt!*/
/*Every current LSB bit is checked with %2, and whole number is shifted with /2 */
uint8_t send8bit_to_uart(UART_HandleTypeDef* huart, uint8_t* data,uint32_t Timeout)
{
	uint8_t data2 = *data;
	uint8_t txbyte;
	for (int i=0; i<8; i++)
	{
		if (data2%2)
			txbyte=49;
		else
			txbyte=48;
		HAL_UART_Transmit(huart, &txbyte, sizeof(txbyte), Timeout);
		data2=data2/2;
	}
	return 0;
}

/*Converts 32bit decimal number to 10 character and sends on UART*/
/*Character order now is right(good)*/
/*Checks every MSB, substarct it from the number and goes for the next digit*/
uint8_t send32bitdecimal_to_uart(UART_HandleTypeDef* huart, uint32_t* data,uint32_t Timeout)
{
	uint32_t data2 = *data;
	uint32_t datacopy;
	uint32_t decimals;
	uint8_t txbyte;
	uint8_t issent=0;
	for (int i=10; i>0; i--)
	{
		datacopy = data2;
		decimals=1;
		for (int j=i-1; j>0; j--)
		{
			datacopy=datacopy/10;
			decimals = decimals*10;
		}
		data2 -= datacopy*decimals;
		txbyte = (uint8_t) (datacopy+48);

		if(datacopy) //send only if not the zeros before the number
			issent=1;
		if(issent)
			HAL_UART_Transmit(huart, &txbyte, sizeof(txbyte), Timeout);
	}
	return 0;
}

uint8_t send16bitdecimal_to_uart(UART_HandleTypeDef* huart, uint16_t* data,uint32_t Timeout)
{
	uint16_t data2 = *data;
	uint16_t datacopy;
	uint32_t decimals;
	uint8_t txbyte;
	uint8_t issent=0;
	uint8_t space=32;
	for (int i=5; i>0; i--)
	{
		datacopy = data2;
		decimals=1;
		for (int j=i-1; j>0; j--)
		{
			datacopy=datacopy/10;
			decimals = decimals*10;
		}
		data2 -= datacopy*decimals;
		txbyte = (uint8_t) (datacopy+48);

		if(datacopy) //send spaces if zeros before the number
			issent=1;
		if(issent)
			HAL_UART_Transmit(huart, &txbyte, sizeof(txbyte), Timeout);
		else
			HAL_UART_Transmit(huart, &space, sizeof(txbyte), Timeout);
	}
	return 0;
}

uint8_t sendadcvals_to_uart(UART_HandleTypeDef* huart, uint16_t* data1,uint16_t* data2, uint16_t* data3,uint32_t Timeout)
{
	uint8_t space=32;
	uint16_t* pdata;
	uint8_t endline=10;
	uint8_t CR=13;
	for (int i=0; i<3; i++)
	{
		if(i==0)
			pdata=data1;
		else if (i==1)
			pdata=data2;
		else if(i==2)
			pdata=data3;
		for (int j=0; j<8; j++)
		{
			send16bitdecimal_to_uart(huart, &(pdata[j]), Timeout);
			HAL_UART_Transmit(huart, &space, sizeof(uint8_t), Timeout);
		}
	}
	HAL_UART_Transmit(&huart2,&endline, sizeof(uint8_t), 100000);
	HAL_UART_Transmit(&huart2,&CR, sizeof(uint8_t), 100000);
	return 0;
}
uint8_t sendlineregisters_to_uart(UART_HandleTypeDef* huart, uint8_t* data1,uint8_t* data2, uint8_t* data3,uint32_t Timeout)
{
	uint8_t endline=10;
	uint8_t CR=13;
	send8bit_to_uart(huart,data1,Timeout);
	send8bit_to_uart(huart,data2,Timeout);
	send8bit_to_uart(huart,data3,Timeout);
	HAL_UART_Transmit(&huart2,&endline, sizeof(uint8_t), 100000);
	HAL_UART_Transmit(&huart2,&CR, sizeof(uint8_t), 100000);
	return 0;
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
