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
#include "stm32f3xx_hal.h"
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
/* Defines */
#define WMAFILTERMELYSEG 4
#define UJVONALERZEKELES_ITERACIOUTAN 4

/* Global Variable definition */
//ADC
volatile bool adckeszelso = false;
volatile bool adckeszmasodik = false;
uint16_t adceredmenyelso[4] = {0,0,0,0};
uint16_t adceredmenymasodik[3] = {0,0,0};
volatile bool varakozas280usec = false;

//WMA filter
uint32_t wmafilterarray_elso[WMAFILTERMELYSEG];
uint32_t wmafilterarray_masodik[WMAFILTERMELYSEG];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MUXselectkuldes(uint8_t* infraLEDminta);
void WMAfilterkezeles(uint32_t* WMAfilterarray, uint32_t* filteredposition, uint32_t* newelement);
void szenzorertekatlagolas (uint16_t* forrastomb, uint8_t hossz, uint32_t* eredmeny);
//UART
void send16bitdecimal_to_uart(UART_HandleTypeDef* huart, uint16_t* data,uint32_t Timeout);
void sendadcvals_to_uart(UART_HandleTypeDef* huart, uint16_t* data1,uint16_t* data2, uint16_t* data3, uint16_t* data4,uint32_t Timeout);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/*Periferiak kezelofuggvenyei (weak callback-ek) */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
		adckeszelso=true;
	}

	else if(hadc->Instance == ADC2)
	{
		adckeszmasodik=true;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM7)
	{
		varakozas280usec = true;
	}

}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* Local Variable definition */
	//SPI
	uint8_t infraLEDminta=0b00000001;
	uint8_t visszajelzoLEDminta[4] = {0,0,0,0};
	uint8_t SPIIteracio=8;

	//ADC
	uint16_t szenzorertekek_elso[4][8];
	uint16_t szenzorertekek_thresholddal_elso[4][8];

	uint16_t szenzorertekek_masodik[3][8];
	uint16_t szenzorertekek_thresholddal_masodik[3][8];

	uint16_t threshold=400;

	//Szenzor adatok
	uint32_t pozicio_elso=1600;
	uint32_t pozicio_elozo_elso;
	uint32_t pozicioWMA_elso;
	bool vonal_elhagyva_elso=false;
	uint16_t miota_van_ujra_vonal_elso=0;

	uint32_t pozicio_masodik=1200;
	uint32_t pozicio_elozo_masodik;
	uint32_t pozicioWMA_masodik;
	bool vonal_elhagyva_masodik=false;
	uint16_t miota_van_ujra_vonal_masodik=0;


	//UART
	 uint8_t endline=10;
	 uint8_t CR=13;
	 uint8_t tab=9;

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //A TCRT-k infraLED-mintajanak kikuldese SPI-on (7 ugyanolyan byte mivel 7x8 TCRT van)
	  for(int q=0; q<7; q++)
	  {
		  HAL_SPI_Transmit(&hspi1,&infraLEDminta,1,1000);
	  }
	  //A visszajelzo LED-ekhez 4 byte kikuldese SPI-on
	  HAL_SPI_Transmit(&hspi1, visszajelzoLEDminta, 4, 1000);

	  //Latch enable impulzus kiadása
	  HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_RESET);

	  //Timer inditas, hogy a TCRT-k baziskapacitasa biztos feltoltodjon  - 280us -------------------------------------------------------------------------------------------
	  varakozas280usec=false;
	  __HAL_TIM_SET_COUNTER(&htim7, 0);
	  HAL_TIM_Base_Start_IT(&htim7);

	  //MUX kezeles
	  MUXselectkuldes(&infraLEDminta);

	  //szenzorertekek teljes feldolgozasa a 8. iteracioban + UART kuldes
	  if(infraLEDminta==0b00000010)
	  {

		  //ELSO szenzorsor -----------
		  szenzorertekatlagolas(szenzorertekek_thresholddal_elso,32,&pozicio_elso);
		  //Az elso szenzorsor forditva van bekotve, igy a poziciot at kell forditani a masik iranyba. De ha 0 a pozicio, akkor nem latunk vonalat, ekkor nem szabad valtoztatni. Ehhez ez kell:
		  if(pozicio_elso)
			  pozicio_elso=3300-pozicio_elso;

		  //Elmentjuk az utolso poziciot, amit lattunk. Amennyiben leterunk a vonalrol, az utolso latott poziciot tartjuk.
		  //Ha letertunk, de ujra latunk vonalat akkor csak akkor vesszuk be, ha legalabb UJVONALERZEKELES_ITERACIOUTAN szenzor-iteracioig valoban latunk nem 0 poziciot.
		  //A nulla pozicio valos jelentese az hogy nem latunk vonalat. Egyebkent az ertektartomanya: [jobb:100,3200:bal]
		  if (pozicio_elso==0)
		  {
			  pozicio_elso=pozicio_elozo_elso;
			  vonal_elhagyva_elso=true;
			  miota_van_ujra_vonal_elso=0;
		  }
		  else
		  {
			  if (vonal_elhagyva_elso)
			  {
				if(miota_van_ujra_vonal_elso >= UJVONALERZEKELES_ITERACIOUTAN)
				{
					vonal_elhagyva_elso=false;
					miota_van_ujra_vonal_elso=0;
					pozicio_elozo_elso=pozicio_elso;
				}
				else
				{
					miota_van_ujra_vonal_elso++;
					pozicio_elso=pozicio_elozo_elso;
				}
			  }
			  else
			  {
					pozicio_elozo_elso=pozicio_elso;
			  }
		  }


		  WMAfilterkezeles(wmafilterarray_elso, &pozicioWMA_elso, &pozicio_elso);

		  send16bitdecimal_to_uart(&huart1, (uint16_t*) &pozicioWMA_elso, 1000);
		  HAL_UART_Transmit(&huart1,&tab, sizeof(uint8_t), 100000);

		  //MASODIK szenzorsor ---------------------

		  szenzorertekatlagolas(szenzorertekek_thresholddal_masodik,24,&pozicio_masodik);

		  //Elmentjuk az utolso poziciot, amit lattunk. Amennyiben leterunk a vonalrol, az utolso latott poziciot tartjuk.
		  //Ha letertunk, de ujra latunk vonalat akkor csak akkor vesszuk be, ha legalabb UJVONALERZEKELES_ITERACIOUTAN szenzor-iteracioig valoban latunk nem 0 poziciot.
		  //A nulla pozicio valos jelentese az hogy nem latunk vonalat. Egyebkent az ertektartomanya: [jobb:100,2400:bal]
		  if (pozicio_masodik==0)
		  {
			  pozicio_masodik=pozicio_elozo_masodik;
			  vonal_elhagyva_masodik=true;
			  miota_van_ujra_vonal_masodik=0;
		  }
		  else
		  {
			  if (vonal_elhagyva_masodik)
			  {
				if(miota_van_ujra_vonal_masodik >= UJVONALERZEKELES_ITERACIOUTAN)
				{
					vonal_elhagyva_masodik=false;
					miota_van_ujra_vonal_masodik=0;
					pozicio_elozo_masodik=pozicio_masodik;
				}
				else
				{
					miota_van_ujra_vonal_masodik++;
					pozicio_masodik=pozicio_elozo_masodik;
				}
			  }
			  else
			  {
					pozicio_elozo_masodik=pozicio_masodik;
			  }
		  }


		  WMAfilterkezeles(wmafilterarray_masodik, &pozicioWMA_masodik, &pozicio_masodik);

		  send16bitdecimal_to_uart(&huart1, (uint16_t*) &pozicioWMA_masodik, 1000);
		  HAL_UART_Transmit(&huart1,&endline, sizeof(uint8_t), 100000);
		  HAL_UART_Transmit(&huart1,&CR, sizeof(uint8_t), 100000);

		  //ADC adatok elkuldese ----------------------
		  sendadcvals_to_uart(&huart1, szenzorertekek_elso[0], szenzorertekek_elso[1], szenzorertekek_elso[2], szenzorertekek_elso[3], 1000);

		  //NUCLEONAK UART-on pozicio es vonalszam elkuldese

	  }

	  //ADC adatok mentese, feldolgozasa
	  //HATSO
	  //ADC adatok mentese
	  {
	  for (int q=0; q<3; q++)
		szenzorertekek_masodik[q][SPIIteracio-1]=adceredmenymasodik[q];
	  }
	  //Thresholdozas
	  for (int q=0; q<3; q++)
	  {
		  if (adceredmenymasodik[q] > threshold)
		  {
			  szenzorertekek_thresholddal_masodik[q][SPIIteracio-1] = adceredmenymasodik[q];
		  }
		  else
		  {
			  szenzorertekek_thresholddal_masodik[q][SPIIteracio-1] = 0;
		  }
	  }

	  //ELSO
	  //Gyors kijelzes threshold alapjan
	  for (int q=0; q<4; q++)
	  {
		  if (adceredmenyelso[q] > threshold)
		  {
			  visszajelzoLEDminta[3-q] = visszajelzoLEDminta[3-q] | 1 << SPIIteracio-1; //infraLEDminta;
		  }
		  else
		  {
			  visszajelzoLEDminta[3-q] = visszajelzoLEDminta[3-q] & ~(1 << SPIIteracio-1); //~infraLEDminta;
		  }
	  }
	  //ADC adatok mentese
	  for (int q=0; q<4; q++)
	  {
		szenzorertekek_elso[q][SPIIteracio-1]=adceredmenyelso[q];
	  }
	  //Thresholdozas
	  for (int q=0; q<4; q++)
	  {
		  if (adceredmenyelso[q] > threshold)
		  {
			  szenzorertekek_thresholddal_elso[q][SPIIteracio-1] = adceredmenyelso[q];
		  }
		  else
		  {
			  szenzorertekek_thresholddal_elso[q][SPIIteracio-1] = 0;
		  }
	  }


	  //Vilagito TCRT LED-ek leptetese (minden 8-ik vilagit)
	  if (infraLEDminta==0b10000000)
	  {
		  infraLEDminta=0b00000001;
		  SPIIteracio=8;
	  }
	  else if (infraLEDminta==0b00000001)
	  {
		  infraLEDminta=0b00000010;
		  SPIIteracio=1;
	  }
	  else
	  {
		  infraLEDminta=infraLEDminta*2;
		  SPIIteracio+=1;
	  }



	  //280us timer vege ----------------------------------------------------------------------------------------------------------------------------------------
	  while(!varakozas280usec);
	  HAL_TIM_Base_Stop_IT(&htim7);

	  //ADC inditasa
	  //Hatso szenzorsorhoz
	  adckeszmasodik=false;
	  HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adceredmenymasodik,3);
	  //Elso szenzorsorhoz
	  adckeszelso=false;
	  HAL_ADC_Start_DMA(&hadc1,(uint32_t *)adceredmenyelso,4);

	  //Hatso szenzorsor ADC konverzio vegere varakozas
	  while (!adckeszmasodik);

	  //Elso szenzorsor ADC konverzio vegere valo varakozas
	  while (!adckeszelso);



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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

    /**Initializes the CPU, AHB and APB busses clocks 
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

void MUXselectkuldes(uint8_t* infraLEDminta)
{
	//Allapotgep: infra LED minta alapjan a megfelelo TCRT fototranzisztorok kivalasztasa a MUX-okkal
	//A jelenlegi infra LED minta alapjan a JELENLEGI MUX select jelek kikuldese
	  if (*infraLEDminta==0b00000001)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_SET);

	  }
	  else if (*infraLEDminta==0b00000010)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_SET);

	  }
	  else if (*infraLEDminta==0b00000100)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_SET);

	  }
	  else if (*infraLEDminta==0b00001000)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_SET);

	  }
	  else if (*infraLEDminta==0b00010000)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_RESET);

	  }
	  else if (*infraLEDminta==0b00100000)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_RESET);

	  }
	  else if (*infraLEDminta==0b01000000)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_RESET);

	  }
	  else if (*infraLEDminta==0b10000000)
	  {
		  HAL_GPIO_WritePin(MUX_SEL_0_GPIO_Port,MUX_SEL_0_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_1_GPIO_Port,MUX_SEL_1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(MUX_SEL_2_GPIO_Port,MUX_SEL_2_Pin,GPIO_PIN_RESET);
	  }
}

//uj elemet tesz a FIFOba es kiszamolja az atlagot is (WeightedMovingAverage)
void WMAfilterkezeles(uint32_t* WMAfilterarray, uint32_t* filteredposition, uint32_t* newelement)
{
	*filteredposition=0;
	uint32_t coeffsum=0;
	for(int i=0; i<WMAFILTERMELYSEG-1; i++)
	{
		WMAfilterarray[i]=WMAfilterarray[i+1];
		*filteredposition+=WMAfilterarray[i]*(i+1);
		coeffsum+=(i+1);
	}
	WMAfilterarray[WMAFILTERMELYSEG-1]=*newelement;
	*filteredposition+=WMAfilterarray[WMAFILTERMELYSEG-1]*(WMAFILTERMELYSEG);
	coeffsum+=(WMAFILTERMELYSEG);
	*filteredposition=*filteredposition/coeffsum;
}

void szenzorertekatlagolas (uint16_t* forrastomb, uint8_t hossz, uint32_t* eredmeny)
{
	*eredmeny = 0;
	uint32_t suly=0;

	//Atlagolas: a szenzorpoziciot(*100) sulyozzuk a filterezett szenzor ertekkel, es azt atlagoljuk
	for(int i=0; i<hossz; i++)
	{
		*eredmeny += forrastomb[i]*100*(i+1);
		suly += forrastomb[i];
	}

	//Ne osszunk 0-val
	if(suly)
	{
		*eredmeny=*eredmeny/suly;
	}
}

void send16bitdecimal_to_uart(UART_HandleTypeDef* huart, uint16_t* data,uint32_t Timeout)
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
}

void sendadcvals_to_uart(UART_HandleTypeDef* huart, uint16_t* data1,uint16_t* data2, uint16_t* data3, uint16_t* data4, uint32_t Timeout)
{
	uint8_t space=32;
	uint16_t* pdata;
	uint8_t endline=10;
	uint8_t CR=13;
	for (int i=0; i<4; i++)
	{
		if(i==0)
			pdata=data1;
		else if (i==1)
			pdata=data2;
		else if(i==2)
			pdata=data3;
		else if(i==3)
			pdata=data4;
		for (int j=0; j<8; j++)
		{
			send16bitdecimal_to_uart(huart, &(pdata[j]), Timeout);
			HAL_UART_Transmit(huart, &space, sizeof(uint8_t), Timeout);
		}
	}
	HAL_UART_Transmit(huart,&endline, sizeof(uint8_t), 100000);
	HAL_UART_Transmit(huart,&CR, sizeof(uint8_t), 100000);
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
