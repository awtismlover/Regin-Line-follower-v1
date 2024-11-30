/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "Line_Follower.h"
#include "RingBuffer.h"
#include "SimpleParser.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	/*Defines*/
	#define true 1;
	#define false 0;
	#define ENDLINE '\n'

	LineFollower_t REGIN;

	RingBuffer_t RB, ReceiveBuffer;
	uint8_t ReceivedData[32];
	uint8_t ReceivedLines;

	/*PIDR regulator*/
	int Sensors_read = 0x00000000;
	int Position;
	float Ki = 0;
	float Kr = 0;
	int P, I, D, R;
	int Last_error = 0;
	int Errors[10] = {0,0,0,0,0,0,0,0,0,0};
	int Error_sum = 0;
	int Last_end = 0;	// 0 -> Left, 1 -> Right
	int Last_idle = 0;
	float Speed_level = 1;

	int ARR = 4;
	int actives = 0;

	/*Sharp turn speed*/

	/*Communication*/
	char buffer[28];
	uint8_t RxData;




	/*Time mode*/
	_Bool Is_time_mode = false;
	_Bool IS_DATA_OK = true;
	_Bool Is_Mode_changed;
	char char_value[2];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
	__HAL_TIM_SET_COUNTER(&htim2,0);

}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}


void motor_control (double pos_right, double pos_left)
{
	if (pos_left < 0 )
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, (int)(ARR*pos_left*-1));
		HAL_GPIO_WritePin(IN1_L_GPIO_Port, IN1_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_L_GPIO_Port, IN2_L_Pin, GPIO_PIN_RESET);
	}
	else
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, (int)(ARR*pos_left));
		HAL_GPIO_WritePin(IN1_L_GPIO_Port, IN1_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_L_GPIO_Port, IN2_L_Pin, GPIO_PIN_SET);
	}
	if (pos_right < 0 )
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, (int)(ARR*pos_right*-1));
		HAL_GPIO_WritePin(IN1_R_GPIO_Port, IN1_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(IN2_R_GPIO_Port, IN2_R_Pin, GPIO_PIN_RESET);
	}
	else
	{
		__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, (int)(ARR*pos_right));
		HAL_GPIO_WritePin(IN1_R_GPIO_Port, IN1_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(IN2_R_GPIO_Port, IN2_R_Pin, GPIO_PIN_SET);
	}
}


void sharp_turn () {

	if (Last_idle < 25)
	{
		if (Last_end == 1)
			motor_control(REGIN.Sharp_bend_speed_right, REGIN.Sharp_bend_speed_left);
		else
			motor_control(REGIN.Sharp_bend_speed_left, REGIN.Sharp_bend_speed_right);
	}
	else
	{
		if (Last_end == 1)
			motor_control(REGIN.Bend_speed_right, REGIN.Bend_speed_left);
		else
			motor_control(REGIN.Bend_speed_left, REGIN.Bend_speed_right);
	}
}
int QTR8_read ()
{
	HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, 1);

	Set_Pin_Output(SENSOR1_GPIO_Port, SENSOR1_Pin);
	Set_Pin_Output(SENSOR2_GPIO_Port, SENSOR2_Pin);
	Set_Pin_Output(SENSOR3_GPIO_Port, SENSOR3_Pin);
	Set_Pin_Output(SENSOR4_GPIO_Port, SENSOR4_Pin);
	Set_Pin_Output(SENSOR5_GPIO_Port, SENSOR5_Pin);
	Set_Pin_Output(SENSOR6_GPIO_Port, SENSOR6_Pin);
	Set_Pin_Output(SENSOR7_GPIO_Port, SENSOR7_Pin);
	Set_Pin_Output(SENSOR8_GPIO_Port, SENSOR8_Pin);

	HAL_GPIO_WritePin (SENSOR1_GPIO_Port, SENSOR1_Pin, 1);
	HAL_GPIO_WritePin (SENSOR2_GPIO_Port, SENSOR2_Pin, 1);
	HAL_GPIO_WritePin (SENSOR3_GPIO_Port, SENSOR3_Pin, 1);
	HAL_GPIO_WritePin (SENSOR4_GPIO_Port, SENSOR4_Pin, 1);
	HAL_GPIO_WritePin (SENSOR5_GPIO_Port, SENSOR5_Pin, 1);
	HAL_GPIO_WritePin (SENSOR6_GPIO_Port, SENSOR6_Pin, 1);
	HAL_GPIO_WritePin (SENSOR7_GPIO_Port, SENSOR7_Pin, 1);
	HAL_GPIO_WritePin (SENSOR8_GPIO_Port, SENSOR8_Pin, 1);

	delay_us(10);

	Set_Pin_Input(SENSOR1_GPIO_Port, SENSOR1_Pin);
	Set_Pin_Input(SENSOR2_GPIO_Port, SENSOR2_Pin);
	Set_Pin_Input(SENSOR3_GPIO_Port, SENSOR3_Pin);
	Set_Pin_Input(SENSOR4_GPIO_Port, SENSOR4_Pin);
	Set_Pin_Input(SENSOR5_GPIO_Port, SENSOR5_Pin);
	Set_Pin_Input(SENSOR6_GPIO_Port, SENSOR6_Pin);
	Set_Pin_Input(SENSOR7_GPIO_Port, SENSOR7_Pin);
	Set_Pin_Input(SENSOR8_GPIO_Port, SENSOR8_Pin);

	// Threshold
	 delay_us(4500);


	Sensors_read = 0x00000000;
	int pos = 0;
  int active = 0;

	if (HAL_GPIO_ReadPin(SENSOR1_GPIO_Port, SENSOR1_Pin)) {
		Sensors_read |= 0x00000001;
		pos += 1000;//1000
    active++;
		Last_end = 1;
	}
	if (HAL_GPIO_ReadPin(SENSOR2_GPIO_Port, SENSOR2_Pin)) {
		Sensors_read |= 0x00000010;
		pos += 2000;//2000
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR3_GPIO_Port, SENSOR3_Pin)) {
		Sensors_read |= 0x00000100;
		pos += 3000;//3000
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR4_GPIO_Port, SENSOR4_Pin)) {
		Sensors_read |= 0x00001000;
		pos += 4000;//4000
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR5_GPIO_Port, SENSOR5_Pin)) {
		Sensors_read |= 0x00010000;
		pos += 5000;//5000
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR6_GPIO_Port, SENSOR6_Pin)) {
		Sensors_read |= 0x00100000;
		pos += 6000;//6000
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR7_GPIO_Port, SENSOR7_Pin)) {
		Sensors_read |= 0x01000000;
		pos += 7000;//7000
    active++;
  }
	if (HAL_GPIO_ReadPin(SENSOR8_GPIO_Port, SENSOR8_Pin)) {
		Sensors_read |= 0x10000000;
		pos += 8000;//8000
    active++;
		Last_end = 0;
  }

  HAL_GPIO_WritePin(LEDON_GPIO_Port, LEDON_Pin, 0);

  actives = active;
	Position = pos/active;

	if (actives == 0)
		Last_idle++;
	else
		Last_idle = 0;

	return pos/active;
}


void forward_brake(int pos_right, int pos_left)
{
	if (actives == 0)
		sharp_turn();
	else
	  motor_control(pos_right, pos_left);
}

void past_errors (int error)
{
  for (int i = 9; i > 0; i--)
      Errors[i] = Errors[i-1];
  Errors[0] = error;
}

int errors_sum (int index, int abs)
{
  int sum = 0;
  for (int i = 0; i < index; i++)
  {
    if (abs == 1 && Errors[i] < 0)
      sum += -Errors[i];
    else
      sum += Errors[i];
  }
  return sum;
}

void PID_control() {
  uint16_t position = QTR8_read();
  int error = 4500 - position;
  past_errors(error);

  P = error;
  I = errors_sum(5, 0);
  D = error - Last_error;
  R = errors_sum(5, 1);
  Last_error = error;

  int motorspeed = P*REGIN.Kp + I*Ki + D*REGIN.Kd;

  int motorspeedl = REGIN.Base_speed_L + motorspeed - R*Kr;
  int motorspeedr = REGIN.Base_speed_R - motorspeed - R*Kr;

  if (motorspeedl > REGIN.Max_speed_L)
    motorspeedl = REGIN.Max_speed_L;
  if (motorspeedr > REGIN.Max_speed_R)
    motorspeedr = REGIN.Max_speed_R;

	forward_brake(motorspeedr, motorspeedl);
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM16_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  REGIN.Kp = 0.02;
  REGIN.Kd = 350 ;
  REGIN.Base_speed_R = 92;
  REGIN.Base_speed_L = 92;
  REGIN.Max_speed_R = 140;
  REGIN.Max_speed_L = 140;
  REGIN.Sharp_bend_speed_right=-90;
  REGIN.Sharp_bend_speed_left=185;
  REGIN.Bend_speed_right=-50;
  REGIN.Bend_speed_left=100;
  /*Start receiving data from Bluetooth*/
     HAL_UART_Receive_IT(&huart1,&RxData,1);

     /*Start and compare timers*/

    //HAL_TIM_Base_Start(&htim3);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
     //__HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, 100);
     __HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_2, 100);
     __HAL_TIM_SET_COMPARE (&htim4, TIM_CHANNEL_1, 100);
     HAL_TIM_Base_Start(&htim2);

     HAL_GPIO_TogglePin(STNDBY_GPIO_Port, STNDBY_Pin);
     HAL_Delay(2000);
     HAL_GPIO_TogglePin(STNDBY_GPIO_Port, STNDBY_Pin);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
     while (1)
      {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    	  PID_control();
    	  if(ReceivedLines > 0)
    	 	  {
    	 		  Parser_TakeLine(&ReceiveBuffer, &ReceivedData);
    	 		  Parser_Parse(ReceivedData,&REGIN);

    	 		  ReceivedLines--;
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
				if(RB_Write(&ReceiveBuffer, RxData) == RB_OK)
				{
					if(RxData == ENDLINE)
					{
						ReceivedLines++;
					}
				}
		    	HAL_UART_Receive_IT(&huart1,&RxData, 1);
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
