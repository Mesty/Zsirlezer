float Simulator_dt;
uint32_t RobotState_status;
uint32_t RobotState_timestamp;
float RobotState_x;
float RobotState_v;
float RobotState_a;
uint32_t RobotState_light;
uint32_t inputStream[7]={0,0,0,0,0,0,0};
uint32_t receivedState_status;
uint32_t receivedState_timestamp;
float receivedState_x;
float receivedState_v;
float receivedState_a;
uint32_t receivedState_light;

void Simulator_start(float intervalsec);
void Simulator_tick();
void Simulator_dataReady();
void Uart_sendstate(UART_HandleTypeDef *huart, uint32_t status, uint32_t timestamp, float x, float v, float a, uint32_t light, uint32_t Timeout);
void Simulator_ReadFrom();

Simulator_start(90000000/1373/65501);

if(dataReady == 1)
{
  Simulator_dataReady();
  dataReady = 0;
}

void Simulator_start(float intervalsec)
{   // Kukori, Kotkoda, tojďż˝sbďż˝l lesz a csoda
	Simulator_dt = intervalsec;
	RobotState_status = ROBOTSTATE_STATUS_DEFAULT;
	RobotState_timestamp = 0;
	RobotState_x = 0.0F;
	RobotState_v = 0.0F;
	RobotState_a = 0.0F;
	RobotState_light = 0;
}
void Simulator_tick()
{   // A szoftvertervezďż˝s az egy biztos pont. Biztos nem jďż˝.
	RobotState_timestamp = (uint32_t) RobotState_timestamp + Simulator_dt;
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

	Uart_sendstate(&huart2, RobotState_status, RobotState_timestamp, RobotState_x, RobotState_v, RobotState_a, RobotState_light, 100000);
}
void Simulator_dataReady()
{   // Gyďż˝ ďż˝ppen ďż˝r egy e-mailt az AUTďż˝kďż˝soknak, mert ďż˝k kiszedtďż˝k a gyďż˝ri szervďż˝t a kocsibďż˝l
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
	receivedState_status = inputStream[1];
	receivedState_timestamp = inputStream[2];
	receivedState_x = (float) inputStream[3];
	receivedState_v = (float) inputStream[4];
	receivedState_a = (float) inputStream[5];
	receivedState_light = inputStream[6];
}
void Uart_sendstate(UART_HandleTypeDef *huart, uint32_t status, uint32_t timestamp, float x, float v, float a, uint32_t light, uint32_t Timeout)
{
	//uint32_t i = 20;
	uint32_t state[7];
	state[0] = reverse_byte_order_32(sizeof(state));
/*	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&status, sizeof(status), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)timestamp, sizeof(uint16_t), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&x, sizeof(x), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&v, sizeof(v), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&a, sizeof(a), Timeout);
	HAL_UART_Transmit(huart, &light, sizeof(light), Timeout);*/
/*	i = 1;
	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&status, sizeof(status), Timeout);
	i = 2;
	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&timestamp, sizeof(timestamp), Timeout);
	i = 3;
	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&x, sizeof(x), Timeout);
	i = 4;
	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&v, sizeof(v), Timeout);
	i = 5;
	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, (uint8_t *)&a, sizeof(a), Timeout);
	i = 6;
	HAL_UART_Transmit(huart, &i, sizeof(i), Timeout);
	HAL_UART_Transmit(huart, &light, sizeof(light), Timeout);*/
/*	for(i = 0; i < 4; ++i)
	{
		state[i+1] = *(((uint8_t *)&status)+3-i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+5] = *(((uint8_t *)&timestamp)+7-i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+13] = *(((uint8_t *)&x)+3-i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+17] = *(((uint8_t *)&v)+3-i);
	}
	for(i = 0; i < 4; ++i)
	{
		state[i+21] = *(((uint8_t *)&a)+3-i);
	}
	state[25] = light;*/
	/*state[1] = reverse_byte_order_32(status);
	state[2] = reverse_byte_order_32(timestamp);
	state[3] = reverse_byte_order_32((uint32_t *)&x);
	state[4] = reverse_byte_order_32((uint32_t *)&v);
	state[5] = reverse_byte_order_32((uint32_t *)&a);
	state[6] = reverse_byte_order_32(light);*/
	//HAL_UART_Transmit(huart, (uint8_t*) state, sizeof(state), Timeout);
}