/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

// Define common angle constants for clarity and easy modification
#define FULL_ROTATION 360     // Full rotation in degrees
#define HALF_ROTATION 180     // Half rotation in degrees
#define QUARTER_ROTATION 90   // Quarter rotation in degrees

typedef enum {
    LETTER_A = 0,
    LETTER_B,
    LETTER_C,
    LETTER_D,
    LETTER_E,
    LETTER_F,
    LETTER_G,
    LETTER_H,
    LETTER_I,
    LETTER_J,
    LETTER_K,
    LETTER_L,
    LETTER_M,
    LETTER_N,
    LETTER_O,
    LETTER_P,
    LETTER_Q,
    LETTER_R,
    LETTER_S,
    LETTER_T,
    LETTER_U,
    LETTER_V,
    LETTER_W,
    LETTER_X,
    LETTER_Y,
    LETTER_Z,
    LETTER_SPACE,  // For pauses between letters
    LETTER_RESET   // Return to neutral position
} SignLetter;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

int currentPos1 = 0;
int currentPos2 = 0;
int currentPos3 = 0;
int currentPos4 = 0;
int currentPos5 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void setServoPosition(TIM_HandleTypeDef* htim, uint32_t channel, int degrees);
void moveAllServos(int degrees);
void setPresetPosition(int preset);
void checkServoLimits(int* position);

void signLetterA(void);
void signLetterB(void);
void signLetterC(void);
void signLetterD(void);
void signLetterE(void);
void signLetterF(void);
void signLetterG(void);
void signLetterH(void);
void signLetterI(void);
void signLetterJ(void);
void signLetterK(void);
void signLetterL(void);
void signLetterM(void);
void signLetterN(void);
void signLetterO(void);
void signLetterP(void);
void signLetterQ(void);
void signLetterR(void);
void signLetterS(void);
void signLetterT(void);
void signLetterU(void);
void signLetterV(void);
void signLetterW(void);
void signLetterX(void);
void signLetterY(void);
void signLetterZ(void);

void displayWord(const char* word);

// Structure to store servo positions for a letter
typedef struct {
    int servo1Pos;  // Thumb
    int servo2Pos;  // Index finger
    int servo3Pos;  // Middle finger
    int servo4Pos;  // Ring finger
    int servo5Pos;  // Pinky finger
    int delayMs;    // Delay after setting this position (for gesture timing)
} LetterPosition;

// Array of letter positions - to be filled in during testing
// Values here are placeholders and should be adjusted based on testing
LetterPosition letterPositions[28] = {
    // LETTER_A
    {180, 360, 360, 360, 360, 1000},
    // LETTER_B
    {360, 180, 180, 180, 180, 1000},
    // LETTER_C
    {270, 270, 270, 270, 270, 1000},
    // LETTER_D
    {360, 180, 360, 360, 360, 1000},
    // LETTER_E
    {180, 180, 180, 180, 180, 1000},
    // LETTER_F
    {270, 180, 180, 360, 360, 1000},
    // LETTER_G
    {180, 360, 180, 180, 180, 1000},
    // LETTER_H
    {180, 180, 180, 360, 360, 1000},
    // LETTER_I
    {180, 360, 360, 360, 180, 1000},
    // LETTER_J
    {180, 360, 360, 360, 180, 1500}, // J may need movement
    // LETTER_K
    {180, 180, 180, 360, 360, 1000},
    // LETTER_L
    {180, 180, 360, 360, 360, 1000},
    // LETTER_M
    {360, 270, 270, 270, 360, 1000},
    // LETTER_N
    {360, 270, 270, 360, 360, 1000},
    // LETTER_O
    {270, 270, 270, 270, 270, 1000},
    // LETTER_P
    {270, 180, 360, 360, 180, 1000},
    // LETTER_Q
    {270, 360, 180, 180, 180, 1000},
    // LETTER_R
    {180, 180, 180, 360, 360, 1000},
    // LETTER_S
    {180, 360, 360, 360, 360, 1000},
    // LETTER_T
    {180, 360, 180, 180, 180, 1000},
    // LETTER_U
    {360, 180, 180, 360, 360, 1000},
    // LETTER_V
    {360, 180, 180, 360, 360, 1000},
    // LETTER_W
    {360, 180, 180, 180, 360, 1000},
    // LETTER_X
    {360, 270, 360, 360, 360, 1000},
    // LETTER_Y
    {180, 360, 360, 360, 180, 1000},
    // LETTER_Z
    {180, 180, 360, 360, 360, 1500}, // Z may need movement
    // LETTER_SPACE
    {180, 180, 180, 180, 180, 1500},
    // LETTER_RESET
    {180, 180, 180, 180, 180, 1000}
};

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void setServo1Speed(int8_t speed)
//{
//    uint32_t pulse;
//    if(speed > 100) speed = 100;
//    if(speed < -100) speed = -100;
//
//    pulse = 1500 + (speed * 5);
//    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
//}
//
//void setServo2Speed(int8_t speed)
//{
//    uint32_t pulse;
//    if(speed > 100) speed = 100;
//    if(speed < -100) speed = -100;
//
//    pulse = 1500 + (speed * 5);
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
//}
//
//void setServo3Speed(int8_t speed)
//{
//    uint32_t pulse;
//    if(speed > 100) speed = 100;
//    if(speed < -100) speed = -100;
//
//    pulse = 1500 + (speed * 5);
//    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);  // Assuming Channel 2 for third servo
//}

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      /* Infinite loop */
      /* USER CODE BEGIN WHILE */

	  setPresetPosition(1);

	  HAL_Delay(10000);

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 79;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 79;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 79;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void setup() {
  moveAllServos(FULL_ROTATION);
}

// Convert degrees to PWM pulse value (1000-2000 μs)
uint32_t degreesToPulse(int degrees) {
    // Ensure degrees are within bounds
    if (degrees < 0) degrees = 0;
    if (degrees > 360) degrees = 360;

    // Convert degrees to pulse width (1000-2000 μs)
    // For 360 continuous rotation servo:
    // 1500 μs is stop
    // 1000 μs is full speed one direction
    // 2000 μs is full speed other direction
    return 1500 + ((degrees - 180) * 5);
}

void setServoPosition(TIM_HandleTypeDef* htim, uint32_t channel, int degrees) {
    uint32_t pulse = degreesToPulse(degrees);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void moveAllServos(int degrees) {
    checkServoLimits(&degrees);

    // Update all servos
    setServoPosition(&htim1, TIM_CHANNEL_1, degrees);
    setServoPosition(&htim2, TIM_CHANNEL_1, degrees);
    setServoPosition(&htim3, TIM_CHANNEL_1, degrees);
    setServoPosition(&htim4, TIM_CHANNEL_1, degrees);
    setServoPosition(&htim8, TIM_CHANNEL_1, degrees);

    // Update current positions
    currentPos1 = degrees;
    currentPos2 = degrees;
    currentPos3 = degrees;
    currentPos4 = degrees;
    currentPos5 = degrees;

    // Add delay for servo movement
    HAL_Delay(1000);
}

void checkServoLimits(int* position) {
    if (*position < 0) {
        *position = 0;
    }
    if (*position > FULL_ROTATION) {
        *position = FULL_ROTATION;
    }
}

void setPresetPosition(int preset) {
    switch(preset) {
        case 1: // All servos at full rotation
            moveAllServos(FULL_ROTATION);
            break;

        case 2: // All servos at half rotation
            moveAllServos(HALF_ROTATION);
            break;

        case 3: // All servos at quarter rotation
            moveAllServos(QUARTER_ROTATION);
            break;

        case 4: // Custom positions
            setServoPosition(&htim1, TIM_CHANNEL_1, 270);
            setServoPosition(&htim2, TIM_CHANNEL_1, 180);
            setServoPosition(&htim3, TIM_CHANNEL_1, 90);
            setServoPosition(&htim4, TIM_CHANNEL_1, 45);
            setServoPosition(&htim8, TIM_CHANNEL_1, 315);
            currentPos1 = 270;
            currentPos2 = 180;
            currentPos3 = 90;
            currentPos4 = 45;
            currentPos5 = 315;
            HAL_Delay(1000);
            break;

        case 5: // Another custom position set
            setServoPosition(&htim1, TIM_CHANNEL_1, 45);
            setServoPosition(&htim2, TIM_CHANNEL_1, 90);
            setServoPosition(&htim3, TIM_CHANNEL_1, 180);
            setServoPosition(&htim4, TIM_CHANNEL_1, 270);
            setServoPosition(&htim8, TIM_CHANNEL_1, 360);
            currentPos1 = 45;
            currentPos2 = 90;
            currentPos3 = 180;
            currentPos4 = 270;
            currentPos5 = 360;
            HAL_Delay(1000);
            break;
    }
}

/**
 * Set hand position for a specific letter
 * @param letter The letter to display
 */
void setLetterPosition(SignLetter letter) {
    // Get the positions for this letter
    LetterPosition pos = letterPositions[letter];

    // Set each servo position individually
    setServoPosition(&htim1, TIM_CHANNEL_1, pos.servo1Pos);
    setServoPosition(&htim2, TIM_CHANNEL_1, pos.servo2Pos);
    setServoPosition(&htim3, TIM_CHANNEL_1, pos.servo3Pos);
    setServoPosition(&htim4, TIM_CHANNEL_1, pos.servo4Pos);
    setServoPosition(&htim8, TIM_CHANNEL_1, pos.servo5Pos);

    // Update current positions
    currentPos1 = pos.servo1Pos;
    currentPos2 = pos.servo2Pos;
    currentPos3 = pos.servo3Pos;
    currentPos4 = pos.servo4Pos;
    currentPos5 = pos.servo5Pos;

    // Wait for the specified delay
    HAL_Delay(pos.delayMs);
}

/**
 * Display a word letter by letter
 * @param word The word to display
 */
void displayWord(const char* word) {
    int i = 0;

    // Display each letter
    while(word[i] != '\0') {
        char c = word[i];

        // Convert to uppercase for consistent handling
        if(c >= 'a' && c <= 'z') {
            c = c - 'a' + 'A';
        }

        // Display the letter
        if(c >= 'A' && c <= 'Z') {
            setLetterPosition((SignLetter)(c - 'A'));
        }
        else if(c == ' ') {
            setLetterPosition(LETTER_SPACE);
        }

        // Pause between letters
        HAL_Delay(500);

        // Move to next letter
        i++;
    }

    // Reset to neutral position after displaying the word
    setLetterPosition(LETTER_RESET);
}

/**
 * Individual letter functions for direct calling
 * These provide a cleaner interface for displaying individual letters
 */
void signLetterA(void) { setLetterPosition(LETTER_A); }
void signLetterB(void) { setLetterPosition(LETTER_B); }
void signLetterC(void) { setLetterPosition(LETTER_C); }
void signLetterD(void) { setLetterPosition(LETTER_D); }
void signLetterE(void) { setLetterPosition(LETTER_E); }
void signLetterF(void) { setLetterPosition(LETTER_F); }
void signLetterG(void) { setLetterPosition(LETTER_G); }
void signLetterH(void) { setLetterPosition(LETTER_H); }
void signLetterI(void) { setLetterPosition(LETTER_I); }
void signLetterJ(void) { setLetterPosition(LETTER_J); }
void signLetterK(void) { setLetterPosition(LETTER_K); }
void signLetterL(void) { setLetterPosition(LETTER_L); }
void signLetterM(void) { setLetterPosition(LETTER_M); }
void signLetterN(void) { setLetterPosition(LETTER_N); }
void signLetterO(void) { setLetterPosition(LETTER_O); }
void signLetterP(void) { setLetterPosition(LETTER_P); }
void signLetterQ(void) { setLetterPosition(LETTER_Q); }
void signLetterR(void) { setLetterPosition(LETTER_R); }
void signLetterS(void) { setLetterPosition(LETTER_S); }
void signLetterT(void) { setLetterPosition(LETTER_T); }
void signLetterU(void) { setLetterPosition(LETTER_U); }
void signLetterV(void) { setLetterPosition(LETTER_V); }
void signLetterW(void) { setLetterPosition(LETTER_W); }
void signLetterX(void) { setLetterPosition(LETTER_X); }
void signLetterY(void) { setLetterPosition(LETTER_Y); }
void signLetterZ(void) { setLetterPosition(LETTER_Z); }
void signSpace(void) { setLetterPosition(LETTER_SPACE); }
void resetHandPosition(void) { setLetterPosition(LETTER_RESET); }

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
