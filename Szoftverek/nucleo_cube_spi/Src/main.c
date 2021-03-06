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
#include "stm32f4xx_hal_tim.h"
#include "inttypes.h"
//MotorPWM
#define MOTOR_0MPERS 6932
#define MOTOR_1P7MPERS 7362
#define MOTOR_3MPERS 7578 //Nem ennyi
#define MOTOR_6MPERS 8224 //Nem biztos h ennyi

#define SEB_LASSU_DEFAULT 700//1100
#define SEB_GYORS_DEFAULT 1200//3300

//Szurok
#define FILTER_DEPTH 16
#define VELOCITY_FILTER_DEPTH 4
//Szervo
#define SERVO_KOZEP 6914
#define SERVO_BAL 5644//5944
#define SERVO_JOBB 7883
//P szab
#define KC 1

//Szervo ertekek:
/*Elozo meres: 6000, 7497, 9000
 Jelenlegi: 5829, 6766, 7638*/

//PD szab
#define L_FROM_ROTATION_AXIS 220 //Vonalszenzor tavolsaga a forgastengelytol [mm]
#define TD_COEFF_FAST 105		 //Coefficiens 10-szerese a TD-hez
#define TD_COEFF_SLOW 20
#define KD_FAST -2				//Kd 10-szerese a szabalyzohoz
#define KD_SLOW -13//-11
#define TSRECIP	333				//Ts mintavetelezesi ido reciproka

//Linetype defines
#define NOLINE 0
#define ONELINE 1
#define TWOLINE 2
#define THREELINE 3
#define LINERROR 10
//Lineobject observing state defines
#define NORMAL 0
#define UNKNOWN 1
#define END_FAST 2
#define ONESTRIPE 3
#define TWOSTRIPE 4
#define START_FAST 5
//PD parameter pair identifiers
#define PD_FAST 1
#define PD_SLOW 2
//velocity defines
#define FAST 10
#define SLOW 20


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
 volatile bool adcvalid=false;
 volatile bool adcwaitdone=false;
 volatile bool dovelocitymeasurement=false;
 volatile uint32_t actualencoderval=0;
 volatile uint32_t actualmotorinput=0;
 volatile uint32_t previousmotorinput=0;
 volatile bool endrxuart=false;
 volatile uint32_t pData[2]={0,0};
 uint16_t adcmeasuredvalues[3];
 uint32_t WMAfilterarray[FILTER_DEPTH]; //vonalpoziciohoz
 //PD parameterei
uint32_t SEB_LASSU=SEB_LASSU_DEFAULT;
uint32_t SEB_GYORS=SEB_GYORS_DEFAULT;
 int32_t KD=-11;
 int32_t TD_COEFF=20;
 uint8_t velocity_state;
 uint8_t space=32;
 uint8_t endline=10;
 uint8_t tab = 9;
 uint8_t CR = 13;
 uint32_t timestamp=0;


 /*Encoder vars*/
 uint32_t encoder1;
 HAL_TIM_StateTypeDef state;
 uint32_t encoderprev=0;
 int32_t encoderdiff;
 uint32_t dir;
 int32_t velocity;
 int32_t filteredvelocity;
 int32_t filteredvelocitydiff;
 int32_t velocityarray[VELOCITY_FILTER_DEPTH]={0,0,0,0}; //Kis csalas, akkor jo, ha VELOCITY_FILTER_DEPTH = 4
 uint32_t velocityabs;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t send8bit_to_uart(UART_HandleTypeDef* huart, uint8_t* data,uint32_t Timeout);
uint8_t send32bitdecimal_to_uart(UART_HandleTypeDef* huart, uint32_t* data,uint32_t Timeout);
uint8_t send16bitdecimal_to_uart(UART_HandleTypeDef* huart, uint16_t* data,uint32_t Timeout);
uint8_t send8bitdecimal_to_uart(UART_HandleTypeDef* huart, uint8_t* data,uint32_t Timeout);
uint8_t sendadcvals_to_uart(UART_HandleTypeDef* huart, uint16_t* data1,uint16_t* data2, uint16_t* data3,uint32_t Timeout);
uint8_t sendlineregisters_to_uart(UART_HandleTypeDef* huart, uint8_t* data1,uint8_t* data2, uint8_t* data3,uint32_t Timeout);
void medianfilterW3(uint16_t* data1,uint16_t* data2, uint16_t* data3);
uint32_t getposition(uint32_t* positionreg, uint16_t* adcfiltervals1, uint16_t* adcfiltervals2, uint16_t* adcfiltervals3);
uint8_t getlinetype(uint8_t* linetype, uint16_t* adcvals1, uint16_t* adcvals2, uint16_t* adcvals3, uint16_t* threshold);
void WMAfilter(int32_t* filteredval, int32_t* newelement, int32_t* array, uint32_t filter_depth);
void initWMAfilterarray();
uint8_t handleWMAfilter(uint32_t* filteredposition, uint32_t* newelement);
void szervoPszabalyozo(int16_t vonalpozicio);
void szervoPDszabalyozo(uint32_t vonalpozicio, int32_t sebesseg);
void sebessegto(int32_t mmpersec);
void setPD(uint32_t PD_type);
uint32_t reverse_byte_order_32(uint32_t value);
void dili_telemetria(UART_HandleTypeDef *huart, uint32_t linestatus, uint32_t velocitystatus, uint32_t position, uint32_t timestamp, int32_t x, int32_t v, int32_t a,  uint32_t Timeout);
void handle_QT_command(uint32_t* pData);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == UART4)
	{
		//HAL_UART_Transmit(&huart4,pData, sizeof(char), 1000 ); teszthez kellett
		endrxuart=true; //telemetria adatok fogadasahoz callback IT
	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	adcvalid = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	/*if (htim->Instance == TIM6)
	{
		adcwaitdone = true;
	}*/
	if(htim->Instance == TIM7)
	{
		timestamp++;
		actualencoderval=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		dovelocitymeasurement=true;
	}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_UART4_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */


  /*Initialize variables for main()*/
  uint8_t stop = 0;
  uint8_t stopm1 = 0;
  uint8_t spidata = 0b00000010;
  uint8_t endline=10;
  uint8_t CR=13;
  uint8_t tab=9;
  uint8_t space=32;
  uint8_t minusz=45;
  uint8_t line_register1=0;			//Tarolja hogy hol erzekelnek vonalat a TCRT-k
  uint8_t line_register2=0;
  uint8_t line_register3=0;
  uint8_t line_count=0;				//Szamolja hogy hany TCRT erzekel vonalat
  uint8_t linetype;
  uint16_t adcvalregister1[8];
  uint16_t adcvalregister2[8];
  uint16_t adcvalregister3[8];
  uint16_t adcfilterregister1[8] = {0,0,0,0,0,0,0,0};
  uint16_t adcfilterregister2[8] = {0,0,0,0,0,0,0,0};
  uint16_t adcfilterregister3[8] = {0,0,0,0,0,0,0,0};
  uint16_t threshold=1200; //TODO 1000-1100? //Vedekezni kell a koszok ellen, ezert nagyobb
  uint16_t thresholdforlinetype=800; //A vonalakat viszont eszre kell venni
  uint8_t i=0;
  uint32_t position=0;
  uint32_t filteredposition=0;
  uint32_t prevfilteredposition=0;
  int32_t Pszabalyozopozicio=0;
  //uint32_t timestamp=0;
  uint8_t string[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int32_t motorpulsePWM;
  initWMAfilterarray();
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim4,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);
  HAL_Delay(5000);
  //sebessegto((int32_t)(SEB_LASSU+600));
  //velocity_state=SLOW;
  //__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2 , SERVO_KOZEP);
  //HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_UART_Receive_DMA(&huart4, (uint8_t*)pData, (2*sizeof(uint32_t)));	//Telemetria parancsok fogadása - konvencio: 2db uint32-t kapunk, elso: command ID, masodik: adat, ha van

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  /*Deadman emergency brake*/
	  if(((actualmotorinput < previousmotorinput) ? actualmotorinput : previousmotorinput) < 4000)
	  {
		  stop = 1;
	  }
	  else
	  {
		  stop = 0;
	  }
	  /*Encoder test*/

	  /*HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
	  while(1)
	  {
		  state = HAL_TIM_Encoder_GetState(&htim2);

		  encoder1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		  dir = __HAL_TIM_DIRECTION_STATUS(&htim2);
		  encoderdiff=encoder1-encoderprev;
		  velocity=((encoderdiff*1000)/743); //v[mm/s]
		  encoderprev=encoder1;
		  sprintf(string, "%"PRId32 "\n\r", velocity);
		  HAL_UART_Transmit(&huart2, string, sizeof(string), 10000);
		  HAL_Delay(100);
	  }*/




	  /*SPI communication*/
	  /*If only 3 sequence is sent, then something is wrong with the bits (not the good bits are received by the LED drivers.
	   Dont know, whats the problem.*/

	  /*for(int q=0; q<3; q++)
	  {
		  HAL_SPI_Transmit(&hspi3,&spidata,1,100);
	  }*/

	  /*Valami nem oke a LE outputtal. Ha kiveszem az utolso 2 hal-delay(1)-et, akkor nem vilagit az uccso(elso) TCRT. Ha az egyik bent van a kodban, akkor hol vilaghit, hol nem. Ha mindketto akkor ok.*/
	  /*Elobbi megoldottnak tunik*/
	  /*LE signal output*/
	  //HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_SET);
	  //HAL_GPIO_WritePin(LE_GPIO_Port,LE_Pin,GPIO_PIN_RESET);

	  /*adcwaitdone=false;
	  __HAL_TIM_SET_COUNTER(&htim6, 0);
	  HAL_TIM_Base_Start_IT(&htim6);*/
	  /*TCRT:280usec varakozas kezdete---------------------------------------------------------------------*/

	  //if(spidata==0b00000001)
	  //{
		  // medianfilterW3(adcvalregister1, adcvalregister2, adcvalregister3); //kicsit thresholdal jo lehet
		  //getlinetype(&linetype, adcvalregister1, adcvalregister2, adcvalregister3, &thresholdforlinetype);//-----------------------------------

		  //sendlineregisters_to_uart(&huart2, &line_register1, &line_register2, &line_register3, 1000);
		  //sendadcvals_to_uart(&huart2, adcvalregister1, adcvalregister2, adcvalregister3, 1000);

		  // medianfilterW3(adcfilterregister1, adcfilterregister2, adcfilterregister3); //Medianhoz kisebb threshold
		  //getposition(&position, adcfilterregister1, adcfilterregister2, adcfilterregister3);
		 /* HAL_UART_Transmit(&huart4,&endline, sizeof(uint8_t), 100000);
		  HAL_UART_Transmit(&huart4,&CR, sizeof(uint8_t), 100000);*/
		  //send8bitdecimal_to_uart(&huart2, &linetype, 1000);
		//	  HAL_UART_Transmit(&huart2,&endline, sizeof(uint8_t), 100000);
		//	  HAL_UART_Transmit(&huart2,&CR, sizeof(uint8_t), 100000);
		 // sendadcvals_to_uart(&huart2, adcfilterregister1, adcfilterregister2, adcfilterregister3, 10000);

		  //if(!position) //Szaturacional==vonalelhagyasnal uccso erteket jegyezze meg.
			  //position=prevfilteredposition;
		  //else
			  //prevfilteredposition=position;

		  //handleWMAfilter(&filteredposition, &position);				//pozicio idobeli szurese

		  if(dovelocitymeasurement) //v=[mm/s], delta t = 10ms
		  {//valami szamitas hibas, wma filter furi.
			  state = HAL_TIM_Encoder_GetState(&htim2);
			  if(state==HAL_TIM_STATE_READY)
			  {
				  //dir = __HAL_TIM_DIRECTION_STATUS(&htim2);
				  encoder1=actualencoderval;
				  encoderdiff=encoder1-encoderprev;
				  //velocity=((encoderdiff*10000)/743); //v[mm/s]
				  //filteredvelocitydiff=-filteredvelocity;
				  //WMAfilter(&filteredvelocity, &velocity, velocityarray, VELOCITY_FILTER_DEPTH);
				  /*if(filteredvelocity>=0)
					  velocityabs = filteredvelocity;
				  else
					  velocityabs = -filteredvelocity;*/
				  encoderprev=encoder1;
				  //filteredvelocitydiff+=filteredvelocity;
				/*  for(int q=0; q<20; q++)
					  string[q]=0;
				  sprintf(string, "%"PRId32 "\t\n\r", filteredvelocity);
				  HAL_UART_Transmit(&huart2, string, sizeof(string), 10000);
				  HAL_UART_Transmit(&huart4, string, sizeof(string), 10000);*/
				  //HAL_UART_Transmit(&huart2,&endline, sizeof(uint8_t), 100000);
				  //HAL_UART_Transmit(&huart2,&CR, sizeof(uint8_t), 100000);
			  }
			  else
			  {
				  //error handling...
			  }
			  dovelocitymeasurement=false;
			  //timestamp++;
			  if(timestamp == 1000)
			  {
				motorpulsePWM = 6932;
				if (!stop)
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 2000)
			  {
				motorpulsePWM = 7032;
				if (!stop)
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 3000)
			  {
				motorpulsePWM = 6932;
				if (!stop)
					__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 4000)
			  {
			  motorpulsePWM = 7132;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 5000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 6000)
			  {
			  motorpulsePWM = 7232;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 7000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 8000)
			  {
			  motorpulsePWM = 7332;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 9000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 10000)
			  {
			  motorpulsePWM = 7432;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 11000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 12000)
			  {
			  motorpulsePWM = 7532;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 13000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 14000)
			  {
			  motorpulsePWM = 7632;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 15000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 16000)
			  {
			  motorpulsePWM = 7732;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 17000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 18000)
			  {
			  motorpulsePWM = 7832;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 19000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 20000)
			  {
			  motorpulsePWM = 7932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 21000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 22000)
			  {
			  motorpulsePWM = 8032;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 23000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 24000)
			  {
			  motorpulsePWM = 8132;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 25000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 26000)
			  {
			  motorpulsePWM = 8232;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 27000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 28000)
			  {
			  motorpulsePWM = 8332;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 29000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 30000)
			  {
			  motorpulsePWM = 8432;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 31000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 32000)
			  {
			  motorpulsePWM = 8532;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 33000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 34000)
			  {
			  motorpulsePWM = 8632;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 35000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 36000)
			  {
			  motorpulsePWM = 8732;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 37000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 38000)
			  {
			  motorpulsePWM = 8832;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 39000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 40000)
			  {
			  motorpulsePWM = 8932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 41000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 42000)
			  {
			  motorpulsePWM = 9032;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 43000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 44000)
			  {
			  motorpulsePWM = 9132;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 45000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 46000)
			  {
			  motorpulsePWM = 9232;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 47000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 48000)
			  {
			  motorpulsePWM = 9332;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 49000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 50000)
			  {
			  motorpulsePWM = 9432;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 51000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 52000)
			  {
			  motorpulsePWM = 9532;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 53000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 54000)
			  {
			  motorpulsePWM = 9632;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 55000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 56000)
			  {
			  motorpulsePWM = 9732;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 57000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 58000)
			  {
			  motorpulsePWM = 9832;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 59000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 60000)
			  {
			  motorpulsePWM = 9932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 61000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 62000)
			  {
			  motorpulsePWM = 10032;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  if(timestamp == 63000)
			  {
			  motorpulsePWM = 6932;
			  if (!stop)
			  	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  }
			  sprintf(string,"\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0\0");
			  sprintf(string,"%d %d\n\r",motorpulsePWM,encoderdiff);
			  HAL_UART_Transmit(&huart4, &string, sizeof(string)*sizeof(uint8_t), 10000);
			  /*if((timestamp%100)==0)
			  {
				  //kesobb ha lesz ido atirni, hogy pointereket kapjon a fg(), hatha ugy gyorsabb, mert az uart sok idot elvesz
				  dili_telemetria(&huart4, (uint32_t) linetype, (uint32_t) velocity_state, filteredposition, timestamp, (int32_t) actualencoderval, (int32_t) filteredvelocity, (int32_t) filteredvelocitydiff*100,  1000);
			  }*/
		  }
		  if (stop != stopm1)
		  {
			  if (!stop)
				  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM));
			  else if (encoderdiff > 10)
				  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (4000));
			  else
				  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (6932));
			  stopm1 = stop;
		  }

		  /*if (endrxuart) 	//uart command lekezelese
		  {
			  handle_QT_command(pData);
			  endrxuart=false;
		  }*/
		 /* sebessegto(1000);*///-------------------------------------------------------------

		//  Pszabalyozopozicio=(((filteredposition-100)*140)/2300)-70;	//2400 100 tartomany (filteredposition ertektartomanya) illesztese a -70 70 tartomanyhoz (Pszab bemeneti ertektartomany)
		//  szervoPszabalyozo((int16_t)Pszabalyozopozicio);				//szabalyozo PWM kezelojenek meghivasa
		 // szervoPszabalyozo((int16_t)filteredposition);				//szabalyozo PWM kezelojenek meghivasa

		  /*if(filteredvelocity!=0)
			  szervoPDszabalyozo(filteredposition, filteredvelocity);*/
		  //szervoPDszabalyozo(filteredposition, 1000);

		 /*send32bitdecimal_to_uart(&huart2,&filteredposition,10000);
		  HAL_UART_Transmit(&huart2,&tab, sizeof(uint8_t), 100000);
		  send8bitdecimal_to_uart(&huart2, &line_count, 1000);

		  HAL_UART_Transmit(&huart2,&endline, sizeof(uint8_t), 100000);
		  HAL_UART_Transmit(&huart2,&CR, sizeof(uint8_t), 100000);*/



		  /*line_register1=0;
		  line_register2=0;
		  line_register3=0;
		  line_count=0;
		  i=0;*/
	  //}


	  //while(!adcwaitdone);
	  /*TCRT:280usec varakozas vege---------------------------------------------------------------------*/
	  //HAL_TIM_Base_Stop_IT(&htim6);

	  //ADC inditasa, majd varakozas a konverzio vegere
	  /*adcvalid=false;
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcmeasuredvalues, 3);
	  while (!adcvalid);*/

	  //mert ADC ertekek eltarolasa
	  /*adcvalregister1[i]=adcmeasuredvalues[0];
	  adcvalregister2[i]=adcmeasuredvalues[1];
	  adcvalregister3[i]=adcmeasuredvalues[2];
	  if(i==4)
		  adcvalregister3[4]=adcvalregister3[4]-300; //20. TCRT kicsit erosebb, a zajt jobban athozza----------------------
*/

	  //Az ADC ertekek thresholdozasa: a minimalis szint alatti ertekeket nem vesszuk figyelembe i 0tol 7ig fut
	  //line register: 3 sima 8 bites tarolo, melyben taroljuk, hogy hol latunk vonalat.
/*	  if (adcvalregister1[i]>threshold)
	  {
		  line_register1+=spidata;
		  line_count++;
	  	  adcfilterregister1[i]=adcvalregister1[i];
	  }
	  else
	  {
		  adcfilterregister1[i]=0;
	  }
	  if (adcvalregister2[i]>threshold)
	  {
		  line_register2+=spidata;
		  line_count++;
	  	  adcfilterregister2[i]=adcvalregister2[i];
	  }
	  else
	  {
		  adcfilterregister2[i]=0;
	  }
	  if (adcvalregister3[i]>threshold)
	  {
		  line_register3+=spidata;
		  line_count++;
	  	  adcfilterregister3[i]=adcvalregister3[i];
	  }
	  else
	  {
		  adcfilterregister3[i]=0;
	  }
	  i++;
*/
	  /*State machine (8 state) for SPI data (LED drivers output) and MUX signals (MUX for analog input)*/
/*	  if (spidata==0b10000000)
	  {
		  spidata=0b00000001;
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

	 // HAL_Delay(10);*/
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

uint8_t send8bitdecimal_to_uart(UART_HandleTypeDef* huart, uint8_t* data,uint32_t Timeout)
{
	uint8_t data2 = *data;
	uint8_t datacopy;
	uint32_t decimals;
	uint8_t txbyte;
	uint8_t issent=0;
	uint8_t space=32;
	for (int i=3; i>0; i--)
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
uint32_t getposition(uint32_t* positionreg, uint16_t* adcfiltervals1, uint16_t* adcfiltervals2, uint16_t* adcfiltervals3)
{
	uint32_t weight=0;
	uint16_t* pdata;
	*positionreg=0;
	for (int i=0; i<3; i++)
	{
		if(i==0)
			pdata=adcfiltervals1;
		else if (i==1)
			pdata=adcfiltervals2;
		else if(i==2)
			pdata=adcfiltervals3;
		for(int j=1; j<9; j++)
		{
			*positionreg=*positionreg+(pdata[j-1]*100*((i*8)+j)); //*100: for a bigger resoltuion (at dividing data would be lost)
			weight+=pdata[j-1];
		}

	}
	*positionreg= *positionreg/weight;
	return 0;
}
void initWMAfilterarray()
{
	for(int i=0; i<FILTER_DEPTH; i++)
			WMAfilterarray[i]=1200;

}

void medianfilterW3(uint16_t* data1,uint16_t* data2, uint16_t* data3)
{
	uint16_t data[24];
	uint16_t* pdata;
	uint16_t bigger;
	uint16_t smaller;
	uint16_t middle;
	uint16_t window[3] = {0,0,0};
	int i;
	//Make one big array from 3 small
	for (i=0; i<3; i++)
		{
			if(i==0)
				pdata=data1;
			else if (i==1)
				pdata=data2;
			else if(i==2)
				pdata=data3;
			for(int j=0; j<8; j++)
			{
				data[i*8+j]=pdata[j];
			}

		}
	//Go from index 1 to 22
	for(i=1; i<23; i++)
	{
		window[0]=data[i-1];
		window[1]=data[i];
		window[2]=data[i+1];
		//Sort
		if(window[0]>window[1])
		{
			bigger=window[0];
			smaller=window[1];
		}
		else
		{
			bigger=window[1];
			smaller=window[0];
		}
		if(window[2]>bigger)
			middle=bigger;
		else if(window[2]>smaller)
			middle=window[2];
		else
			middle=smaller;
		//write back
		if((i/8)==0)
			data1[i%8]=middle;
		else if ((i/8)==1)
			data2[i%8]=middle;
		else
			data3[i%8]=middle;
	}
}

uint8_t handleWMAfilter(uint32_t* filteredposition, uint32_t* newelement) //put a new element into the FIFO, and calculate the new average (WeightedMovingAverage)
{
	*filteredposition=0;
	uint32_t coeffsum=0;
	for(int i=0; i<FILTER_DEPTH-1; i++)
	{
		WMAfilterarray[i]=WMAfilterarray[i+1];
		*filteredposition+=WMAfilterarray[i]*(i+1);
		coeffsum+=(i+1);
	}
	WMAfilterarray[FILTER_DEPTH-1]=*newelement;
	*filteredposition+=WMAfilterarray[FILTER_DEPTH-1]*(FILTER_DEPTH);
	coeffsum+=(FILTER_DEPTH);
	*filteredposition=*filteredposition/coeffsum;

	return 0;
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

uint8_t getlinetype(uint8_t* linetype, uint16_t* adcvals1, uint16_t* adcvals2, uint16_t* adcvals3, uint16_t* threshold)
{
	bool isline=false;
	bool islineprev=false;
	uint8_t edgenumber=0;
	uint16_t* pdata;
	static uint32_t encodervalue0;		//encoder ertek a vonal-objektum elejen
 	uint32_t encodervalue1=0;				//koztes encoder ertek
	static uint8_t state=NORMAL;
	static bool object_observe=false;
	static uint8_t end_fast_counter=0;  //Figyeljuk hohy hany lassito szakasz van. ha 5, megallunk
	uint8_t tab=9;

	if (end_fast_counter==2)
		sebessegto(0);

	for (int i=0; i<3; i++)
	{
		if(i==0)
			pdata=adcvals1;
		else if (i==1)
			pdata=adcvals2;
		else if(i==2)
			pdata=adcvals3;
		for(int j=1; j<9; j++)
		{
			if(pdata[j-1]>(*threshold))
			{
				isline=true;
			}
			else
			{
				isline=false;
			}
			if(isline^islineprev)
			{
				edgenumber++;
			}
			islineprev=isline;
		}
	}
	if(edgenumber==0)
		*linetype=NOLINE;
	else if(edgenumber==1 || edgenumber==2)
		*linetype=ONELINE;
	else if(edgenumber==3 || edgenumber==4)
		*linetype=TWOLINE;
	else if(edgenumber==5 || edgenumber==6)
		*linetype=THREELINE;
	else
		*linetype=LINERROR;






	/*send8bitdecimal_to_uart(&huart4, linetype, 10000);
	HAL_UART_Transmit(&huart4, &tab, sizeof(uint8_t), 1000);*/
	//Gyorsito, lassitoszakasz erzekeles
	//Tipp: object observe valtozo nem kell: true if state!=NORMAL, else false
	//Mok:

	//Mok vege
	/*if(*linetype==TWOLINE && velocity_state==FAST)
	{
		encodervalue0=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		sebessegto(SEB_LASSU);
		velocity_state=SLOW;
		state=END_FAST;
		setPD(PD_SLOW);

	}*/

	if (*linetype==THREELINE)
	{
		if(object_observe==false) //Valamelyik kezdete, nem tudni mi
		{
			object_observe=true;
			state=UNKNOWN;
			encodervalue0=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
		}
		if(object_observe==true)
		{
			encodervalue1=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);


			if(state==ONESTRIPE && encodervalue1-encodervalue0 > 200 && encodervalue1-encodervalue0 < 817) //Ha 1 kis stripeot lattunk, akkor gyanus hogy gyorsito, itt megnezzuk, hogy tenyleg az e (van e kb ugyanakkor kihagyas a kovi stripeig)
			{
				state=START_FAST;
				sebessegto((int32_t) SEB_GYORS);
				velocity_state=FAST;
				setPD(PD_FAST);//gyors parameterek
				encodervalue0=encodervalue1; //encodervalue0-ba kerül az aktualis pozicio, a kovetkezo stripe elejenel ezzel szamolunk

			}
			else if((state== UNKNOWN && encodervalue1-encodervalue0 > 1500))//Ha 800=? cm (2000=~27cm) -ota van 3 vonal -> lassito
			{
				state = END_FAST;
				end_fast_counter++;
				sebessegto(((int32_t)SEB_LASSU)-10000);
				velocity_state=SLOW;
				//setPD(PD_SLOW); //lassu parameterek
			}
			else if(state==START_FAST)
			{
				encodervalue1=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
				encodervalue0=encodervalue1; //encodervalue0-ba kerül az aktualis pozicio, a kovetkezo stripe elejenel ezzel szamolunk
			}
			else if(state==END_FAST && (filteredvelocity < SEB_LASSU-400))
			{
				sebessegto((int32_t)SEB_LASSU);
			}
		}

	}
	if(*linetype==ONELINE && object_observe==true)
	{
		if(state == END_FAST) //Ha lassito volt, akkor vege van az objektumnak 1 vonalnal
			{
				object_observe=false;
				state = NORMAL;
				setPD(PD_SLOW); //lassu parameterek
				sebessegto((int32_t)SEB_LASSU+450);
				//sebessegto(0);//mOOOOK
			}
		else if(state == UNKNOWN) //Ha megfigyeljuk, de semmit nem tudunk (eddig 3 vonal volt), most pedig egy vonal -> valszeg gyorsito
		{
			encodervalue1=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			if (encodervalue1-encodervalue0 > 200 &&  encodervalue1-encodervalue0 < 817) //Ha 1 stripe-nyi (50-110mm) a 3 vonalas korabbi resz -> valszeg gyorsito
			{
				encodervalue0=encodervalue1; //encodervalue0-ba kerül az aktualis pozicio, a kovetkezo stripe elejenel ezzel szamolunk
				state=ONESTRIPE;
			}
			else //Ha tul rovid a csik, vagy tul nagy (bar utobbi kb lehetetlen)
			{
				state=NORMAL;
				object_observe=false;
			}
		}
		else if(state == START_FAST) //gyorsito szakasz objektum vege
		{ //Ha eleg sokaig csak egy vonal van, akkor vege van.
		//Ha diff > 16cm -> akkor vege
			encodervalue1=HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
			if(encodervalue1-encodervalue0 > 1184)
			{
				///ha tul hamar adjuk ki a gyorsitast, akkor itt is lehet adni, ha kesobb szerenenk (csikozott vonal vege itt)
				state=NORMAL;
				object_observe=false;
			}
		}
	}

/*	send32bitdecimal_to_uart(&huart4, &encodervalue0, 10000);
	HAL_UART_Transmit(&huart4, &tab, sizeof(uint8_t), 10000);
	send32bitdecimal_to_uart(&huart4, &encodervalue1, 10000);
	HAL_UART_Transmit(&huart4, &tab, sizeof(uint8_t), 10000);
	send8bitdecimal_to_uart(&huart4, &state, 10000);*/

	return *linetype;
}

void szervoPszabalyozo(int16_t vonalpozicio)
{
	  TIM_OC_InitTypeDef sConfigOC;
	  int16_t servo_pulse;
	  servo_pulse = -((784*vonalpozicio*KC)/575)+(211631/23);
	  //servo_pulse = (-30.1*vonalpozicio)*KC+6707;
	  if (servo_pulse>9000)
		  servo_pulse=9000;
	  if (servo_pulse<6000)
		  servo_pulse=6000;
//__HAL_TIM_SET_COMPARE()
	  	sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = servo_pulse;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		{
		  Error_Handler();
		}
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	 // MX_TIM1_Init(servo_pulse);
	//  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
}
void szervoPDszabalyozo(uint32_t vonalpozicio, int32_t sebesseg)
{
	//szabalyzobol jovo u:beavatkozojel mertekegysege [rad], ertektartomanya: [-0,34;+0,34] (-20;+20 fok)
	//u szamitasahoz hasznaljuk a sebesseget[mm/s], es a poziciot [mm]
	//A vonalpoziciot is at kell szamolni mm-be [-70,70], jelenleg [100,2400] az ertektartomanya
	//A szervoba [SERVO_BAL,SERVO_JOBB] ertektartomanyu PWM-et kell beadni.


	uint8_t endline = 10;
	uint8_t CR = 13;
	static float elozopozicioMM = 0.0;
	float pozicioMM;
	float szabalyozokimenetRAD;
	float pulsePWM;
	uint32_t uintpulsePWM;
	//TIM_OC_InitTypeDef sConfigOC;

	pozicioMM = ((float)vonalpozicio-100.0)*140.0/2300.0-70.0;
	szabalyozokimenetRAD = ( (1.0 + (( TD_COEFF*L_FROM_ROTATION_AXIS*TSRECIP) / ((float)sebesseg*10.0)) )*pozicioMM - ((TD_COEFF*L_FROM_ROTATION_AXIS*TSRECIP) / ((float)sebesseg*10.0))*elozopozicioMM ) *KD/L_FROM_ROTATION_AXIS/10.0;
	//szabalyozokimenetRAD = ( (3.0 + (( TD_COEFF*L_FROM_ROTATION_AXIS*TSRECIP) / ((float)sebesseg*10.0)) )*pozicioMM - ((TD_COEFF*L_FROM_ROTATION_AXIS*TSRECIP) / ((float)sebesseg*10.0))*elozopozicioMM ) *KD/L_FROM_ROTATION_AXIS/10.0;

	//Mok, hogy szelesebb legyen a tartomany, amin mozgat, mert most statikusan nem mozgat balra rendesen
	if (szabalyozokimenetRAD < 0)
		szabalyozokimenetRAD=szabalyozokimenetRAD*1.35;


	pulsePWM = (szabalyozokimenetRAD + 0.34)*((SERVO_JOBB-SERVO_BAL)/0.68)+SERVO_BAL;



	if (pulsePWM > (float)SERVO_JOBB-30)
		pulsePWM = (float)SERVO_JOBB-30;
	if (pulsePWM < (float)SERVO_BAL+30)
		pulsePWM = (float)SERVO_BAL+30;
  	uintpulsePWM = (uint32_t) pulsePWM;

	/*send32bitdecimal_to_uart(&huart2, &uintpulsePWM, 10000 );
	HAL_UART_Transmit(&huart2,&endline, sizeof(uint8_t), 100000);
	HAL_UART_Transmit(&huart2,&CR, sizeof(uint8_t), 100000);*/

	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2 , (uint32_t) pulsePWM);
	elozopozicioMM=pozicioMM;
}
void setPD(uint32_t PD_type)
{
	if(PD_type==PD_SLOW)
	{
		KD=KD_SLOW;
		TD_COEFF=TD_COEFF_SLOW;
	}
	else if(PD_type==PD_FAST)
	{
		KD=KD_FAST;
		TD_COEFF=TD_COEFF_FAST;
	}

}

//TODO:negativ ertekek is legyenek jok

void sebessegto(int32_t mmpersec)
{
	//Atkonvertalja a mm/s erteket PWM Compare ertekre
	// meres: 1700 mm/s-re volt 7470 pulse a PWM, ez alapjan allitjuk be a "kovertalasi" egyenest
	int32_t motorpulsePWM;
	motorpulsePWM = (mmpersec*(MOTOR_1P7MPERS-MOTOR_0MPERS))/1700 + MOTOR_0MPERS;
	//vedelem: ne legyen tul nagy sebesseg negativ iranyba
	if (motorpulsePWM < 3692)
		motorpulsePWM=3692;
	//vedelem: max sebesseg
	if(motorpulsePWM > 10150) //volt: 7500: Korulbelul 2m/s , korabban 7326 volt
		motorpulsePWM = 10150;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, (motorpulsePWM)); //
	//send32bitdecimal_to_uart(&huart4, &motorpulsePWM, 10000);
	//send32bitdecimal_to_uart(&huart2, &motorpulsePWM, 10000);
}
void dili_telemetria(UART_HandleTypeDef *huart, uint32_t linestatus, uint32_t velocitystatus,uint32_t position, uint32_t timestamp, int32_t x, int32_t v, int32_t a,  uint32_t Timeout)
{
		uint32_t state[8];
		state[0] = reverse_byte_order_32(sizeof(state));
		state[1] = reverse_byte_order_32(linestatus);
		state[2] = reverse_byte_order_32(velocitystatus);
		state[3] = reverse_byte_order_32(position);
		state[4] = reverse_byte_order_32(timestamp);
		state[5] = reverse_byte_order_32(x);
		state[6] = reverse_byte_order_32(v);
		state[7] = reverse_byte_order_32(a);

		HAL_UART_Transmit(huart, (uint8_t*) state, sizeof(state), Timeout);
}

uint32_t reverse_byte_order_32(uint32_t value)
{
	uint8_t lolo = (value >> 0) & 0xFF;
	uint8_t lohi = (value >> 8) & 0xFF;
	uint8_t hilo = (value >> 16) & 0xFF;
	uint8_t hihi = (value >> 24) & 0xFF;
	return (hihi << 0) | (hilo << 8) | (lohi << 16) | (lolo << 24);
}
void handle_QT_command(uint32_t* pData)
{
	switch (pData[0]){
		case 1 :  	//STOP
			sebessegto(0);
			break;
		case 2 :	//Gyorsitas
			SEB_LASSU+=100;
			SEB_GYORS+=100;
			break;
		case 3 : 	//Lassitas
			SEB_LASSU-=100;
			SEB_GYORS-=100;
			break;
		case 4 :	//Set Td
			TD_COEFF=pData[1];
			break;
		case 5 :	//Set Kd
			KD=pData[1];
			break;
		default :;
	}
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
