/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STATE_GREEN 0
#define STATE_YELLOW 1
#define STATE_RED 2

#define COUNTER_GREEN 2
#define COUNTER_YELLOW 3
#define COUNTER_RED 5

#define SET_A 0
#define SET_B 1

#define CODE_SEG_A ((uint8_t)0x01)
#define CODE_SEG_B ((uint8_t)0x02)
#define CODE_SEG_C ((uint8_t)0x04)
#define CODE_SEG_D ((uint8_t)0x08)
#define CODE_SEG_E ((uint8_t)0x10)
#define CODE_SEG_F ((uint8_t)0x20)
#define CODE_SEG_G ((uint8_t)0x40)

#define CODE_NUM_0 ((uint8_t)0x3F)
#define CODE_NUM_1 ((uint8_t)0x06)
#define CODE_NUM_2 ((uint8_t)0x5B)
#define CODE_NUM_3 ((uint8_t)0x4F)
#define CODE_NUM_4 ((uint8_t)0x66)
#define CODE_NUM_5 ((uint8_t)0x6D)
#define CODE_NUM_6 ((uint8_t)0x7D)
#define CODE_NUM_7 ((uint8_t)0x07)
#define CODE_NUM_8 ((uint8_t)0x7F)
#define CODE_NUM_9 ((uint8_t)0x6F)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

int GetNextState(int led_state);
int GetStateCounter(int led_state);

void DisplayLED(int led_set, int led_state);
void DisplayLEDError();
void ResetLED(int led_set);
uint16_t GetLEDSetPin(int led_set);

void Display7SEG(int segment_set, int num);
void Display7SEGError();
void Reset7SEG(int segment_set);
uint16_t Get7SEGSetPin(int segment_set);
uint8_t GetNumberCode(int number);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int GetNextState(int led_state){
	switch(led_state){
	case STATE_GREEN:
		return STATE_YELLOW;

	case STATE_YELLOW:
		return STATE_RED;

	case STATE_RED:
		return STATE_GREEN;

	default:
		return -1;
	}
}

int GetStateCounter(int led_state){
	switch(led_state){
	case STATE_GREEN:
		return COUNTER_GREEN;

	case STATE_YELLOW:
		return COUNTER_YELLOW;

	case STATE_RED:
		return COUNTER_RED;

	default:
		return -1;
	}
}

void ResetLED(int led_set){
	uint16_t start_pin = GetLEDSetPin(led_set);

	if(start_pin == -1){
		DisplayLEDError();
		return;
	}

	HAL_GPIO_WritePin(GPIOA, start_pin << 0, SET);
	HAL_GPIO_WritePin(GPIOA, start_pin << 1, SET);
	HAL_GPIO_WritePin(GPIOA, start_pin << 2, SET);
}

void DisplayLED(int led_set, int led_state){
	ResetLED(led_set);
	uint16_t start_pin = GetLEDSetPin(led_set);

	if(start_pin == -1){
		DisplayLEDError();
		return;
	}

	switch(led_state){
	case STATE_GREEN:
		HAL_GPIO_WritePin(GPIOA, start_pin << 2, RESET);
		return;

	case STATE_YELLOW:
		HAL_GPIO_WritePin(GPIOA, start_pin << 1, RESET);
		return;

	case STATE_RED:
		HAL_GPIO_WritePin(GPIOA, start_pin << 0, RESET);
		return;

	default:
		DisplayLEDError();
		return;
	}
}

uint16_t GetLEDSetPin(int led_set){
	switch(led_set){
	case SET_A:
		return LED_RED_0_Pin;

	case SET_B:
		return LED_RED_1_Pin;

	//Catch error when led_set isn't SET_A or SET_B
	default:
		return -1;
	}
}

void DisplayLEDError(){
	//All LED turn on
	HAL_GPIO_WritePin(GPIOA, LED_RED_0_Pin, RESET);
	HAL_GPIO_WritePin(GPIOA, LED_YELLOW_0_Pin, RESET);
	HAL_GPIO_WritePin(GPIOA, LED_GREEN_0_Pin, RESET);

	HAL_GPIO_WritePin(GPIOA, LED_RED_1_Pin, RESET);
	HAL_GPIO_WritePin(GPIOA, LED_YELLOW_1_Pin, RESET);
	HAL_GPIO_WritePin(GPIOA, LED_GREEN_1_Pin, RESET);
}

void Display7SEG(int segment_set, int num){
	Reset7SEG(segment_set);

	uint16_t start_pin = Get7SEGSetPin(segment_set);
	if(start_pin == -1){
		Display7SEGError();
		return;
	}

	uint8_t number_code = GetNumberCode(num);
	if(start_pin == -1){
		Display7SEGError();
		return;
	}

	HAL_GPIO_WritePin(GPIOB, start_pin << 0, !(CODE_SEG_A & number_code));
	HAL_GPIO_WritePin(GPIOB, start_pin << 1, !(CODE_SEG_B & number_code));
	HAL_GPIO_WritePin(GPIOB, start_pin << 2, !(CODE_SEG_C & number_code));
	HAL_GPIO_WritePin(GPIOB, start_pin << 3, !(CODE_SEG_D & number_code));
	HAL_GPIO_WritePin(GPIOB, start_pin << 4, !(CODE_SEG_E & number_code));
	HAL_GPIO_WritePin(GPIOB, start_pin << 5, !(CODE_SEG_F & number_code));
	HAL_GPIO_WritePin(GPIOB, start_pin << 6, !(CODE_SEG_G & number_code));

}

void Reset7SEG(int segment_set){
	uint16_t start_pin = Get7SEGSetPin(segment_set);
	if(start_pin == -1){
		Display7SEGError();
		return;
	}

	HAL_GPIO_WritePin(GPIOB, start_pin << 0, SET);
	HAL_GPIO_WritePin(GPIOB, start_pin << 1, SET);
	HAL_GPIO_WritePin(GPIOB, start_pin << 2, SET);
	HAL_GPIO_WritePin(GPIOB, start_pin << 3, SET);
	HAL_GPIO_WritePin(GPIOB, start_pin << 4, SET);
	HAL_GPIO_WritePin(GPIOB, start_pin << 5, SET);
	HAL_GPIO_WritePin(GPIOB, start_pin << 6, SET);
}

uint16_t Get7SEGSetPin(int segment_set){
	switch(segment_set){
	case SET_A:
		return SEG_A0_Pin;

	case SET_B:
		return SEG_A1_Pin;

	//Catch error
	default:
		return -1;
	}
}

void Display7SEGError(){
	//If error happen, only g-segment is on
	HAL_GPIO_WritePin(GPIOB, SEG_A0_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_B0_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_C0_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_D0_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_E0_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_F0_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_G0_Pin, RESET);

	HAL_GPIO_WritePin(GPIOB, SEG_A1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_B1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_C1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_D1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_E1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_F1_Pin, SET);
	HAL_GPIO_WritePin(GPIOB, SEG_G1_Pin, RESET);
}

uint8_t GetNumberCode(int number){
	switch(number){
	case 0:
		return CODE_NUM_0;

	case 1:
		return CODE_NUM_1;

	case 2:
		return CODE_NUM_2;

	case 3:
		return CODE_NUM_3;

	case 4:
		return CODE_NUM_4;

	case 5:
		return CODE_NUM_5;

	case 6:
		return CODE_NUM_6;

	case 7:
		return CODE_NUM_7;

	case 8:
		return CODE_NUM_8;

	case 9:
		return CODE_NUM_9;

	default:
		return -1;
	}
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
  /* USER CODE BEGIN 2 */

  int set_a_counter = COUNTER_RED;
  int set_a_state = STATE_RED;

  int set_b_counter = COUNTER_GREEN;
  int set_b_state = STATE_GREEN;

  DisplayLED(SET_A, set_a_state);
  DisplayLED(SET_B, set_b_state);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	if(set_a_counter <= 0){
		set_a_state = GetNextState(set_a_state);
		set_a_counter = GetStateCounter(set_a_state);
		DisplayLED(SET_A, set_a_state);
	}

	if(set_b_counter <= 0){
		set_b_state = GetNextState(set_b_state);
		set_b_counter = GetStateCounter(set_b_state);
		DisplayLED(SET_B, set_b_state);
	}

	Display7SEG(SET_A, set_a_counter--);
	Display7SEG(SET_B, set_b_counter--);

	HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_RED_0_Pin|LED_YELLOW_0_Pin|LED_GREEN_0_Pin|LED_RED_1_Pin
                          |LED_YELLOW_1_Pin|LED_GREEN_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SEG_A0_Pin|SEG_B0_Pin|SEG_C0_Pin|SEG_D1_Pin
                          |SEG_E1_Pin|SEG_F1_Pin|SEG_G1_Pin|SEG_D0_Pin
                          |SEG_E0_Pin|SEG_F0_Pin|SEG_G0_Pin|SEG_A1_Pin
                          |SEG_B1_Pin|SEG_C1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_RED_0_Pin LED_YELLOW_0_Pin LED_GREEN_0_Pin LED_RED_1_Pin
                           LED_YELLOW_1_Pin LED_GREEN_1_Pin */
  GPIO_InitStruct.Pin = LED_RED_0_Pin|LED_YELLOW_0_Pin|LED_GREEN_0_Pin|LED_RED_1_Pin
                          |LED_YELLOW_1_Pin|LED_GREEN_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_A0_Pin SEG_B0_Pin SEG_C0_Pin SEG_D1_Pin
                           SEG_E1_Pin SEG_F1_Pin SEG_G1_Pin SEG_D0_Pin
                           SEG_E0_Pin SEG_F0_Pin SEG_G0_Pin SEG_A1_Pin
                           SEG_B1_Pin SEG_C1_Pin */
  GPIO_InitStruct.Pin = SEG_A0_Pin|SEG_B0_Pin|SEG_C0_Pin|SEG_D1_Pin
                          |SEG_E1_Pin|SEG_F1_Pin|SEG_G1_Pin|SEG_D0_Pin
                          |SEG_E0_Pin|SEG_F0_Pin|SEG_G0_Pin|SEG_A1_Pin
                          |SEG_B1_Pin|SEG_C1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
