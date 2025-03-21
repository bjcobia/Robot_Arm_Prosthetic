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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Servo control definitions */
#define SERVO_STOP      1500    // Pulse width for stop (µs)
#define SERVO_MAX_CW    1000    // Pulse width for max clockwise rotation (µs)
#define SERVO_MAX_CCW   2000    // Pulse width for max counter-clockwise rotation (µs)

/* Servo pins mapping - adjust based on your hardware connections */
#define SERVO_THUMB     TIM1, TIM_CHANNEL_1
#define SERVO_INDEX     TIM2, TIM_CHANNEL_1
#define SERVO_MIDDLE    TIM3, TIM_CHANNEL_1
#define SERVO_RING      TIM4, TIM_CHANNEL_1
#define SERVO_PINKY     TIM8, TIM_CHANNEL_1

#define THUMB_CLOSED 10000
#define INDEX_CLOSED 10000
#define MIDDLE_CLOSED 10000
#define RING_CLOSED 10000
#define PINKY_CLOSED 10000

typedef enum {
    THUMB = 0,
    INDEX,
    MIDDLE,
    RING,
    PINKY
} Finger;

typedef enum {
    STOP = 0,
    CLOCKWISE,
    COUNTERCLOCKWISE
} Direction;

/* servo states */
typedef struct {
    int speed;          // Speed percentage (0-100)
    Direction dir;      // Current direction
    uint32_t pulse;     // Current pulse width
} ServoState;

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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Index_Finger */
osTimerId_t Index_FingerHandle;
const osTimerAttr_t Index_Finger_attributes = {
  .name = "Index_Finger"
};
/* Definitions for Thumb_Finger */
osTimerId_t Thumb_FingerHandle;
const osTimerAttr_t Thumb_Finger_attributes = {
  .name = "Thumb_Finger"
};
/* Definitions for Middle_Finger */
osTimerId_t Middle_FingerHandle;
const osTimerAttr_t Middle_Finger_attributes = {
  .name = "Middle_Finger"
};
/* Definitions for Ring_Finger */
osTimerId_t Ring_FingerHandle;
const osTimerAttr_t Ring_Finger_attributes = {
  .name = "Ring_Finger"
};
/* Definitions for Pinky_Finger */
osTimerId_t Pinky_FingerHandle;
const osTimerAttr_t Pinky_Finger_attributes = {
  .name = "Pinky_Finger"
};
/* USER CODE BEGIN PV */
char rxBuffer[1]; /* Buffer for receiving a single character */
char message[256]; /* Buffer to store the complete message */
uint16_t messageIndex = 0;
uint8_t messageReady = 0; /* Flag to indicate a message is in buffer waiting to be processed*/

/* Global servo states */
ServoState servoStates[5] = {
    {0, STOP, SERVO_STOP},  // THUMB
    {0, STOP, SERVO_STOP},  // INDEX
    {0, STOP, SERVO_STOP},  // MIDDLE
    {0, STOP, SERVO_STOP},  // RING
    {0, STOP, SERVO_STOP}   // PINKY
};

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
void StartDefaultTask(void *argument);
void Index(void *argument);
void Thumb(void *argument);
void Middle(void *argument);
void Ring(void *argument);
void Pinky(void *argument);

/* USER CODE BEGIN PFP */

void Servo_Init(void);
void Servo_SetMotion(Finger finger, Direction direction, int speed);
void Servo_StopAll(void);
void SignLetter(char letter);
int Direction_Decider(int* Desired_Position);

static void MX_USART2_UART_Init(void);
void ProcessReceivedMessage(char* msg);
uint8_t IsButtonPressed(void);

int16_t thumb_current=0;
int16_t index_current=0;
int16_t middle_current=0;
int16_t ring_current=0;
int16_t pinky_current=0;

int thumb_desired_position;
int index_desired_position;
int middle_desired_position;
int ring_desired_position;
int pinky_desired_position;
//
//int *ptr_thumb = &thumb_desired_position;
//int *ptr_index = &index_desired_position;
//int *ptr_middle;
//int *ptr_ring;
//int *ptr_pinky;


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
//  SignLetter('B', 2000);
//
//  HAL_Delay(2000);
//
//  memset(message, 0, sizeof(message));
//  HAL_UART_Receive_IT(&huart2, (uint8_t*)rxBuffer, 1);

  SignLetter('A');
//
//  HAL_Delay(2000);
//
//  Servo_StopAll();
//
//  HAL_Delay(3000);
//
//  SignLetter('B', 1000);
//
//  SignLetter('A', 2000);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of Index_Finger */
  Index_FingerHandle = osTimerNew(Index, osTimerOnce, NULL, &Index_Finger_attributes);

  /* creation of Thumb_Finger */
  Thumb_FingerHandle = osTimerNew(Thumb, osTimerPeriodic, NULL, &Thumb_Finger_attributes);

  /* creation of Middle_Finger */
  Middle_FingerHandle = osTimerNew(Middle, osTimerPeriodic, NULL, &Middle_Finger_attributes);

  /* creation of Ring_Finger */
  Ring_FingerHandle = osTimerNew(Ring, osTimerPeriodic, NULL, &Ring_Finger_attributes);

  /* creation of Pinky_Finger */
  Pinky_FingerHandle = osTimerNew(Pinky, osTimerPeriodic, NULL, &Pinky_Finger_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	if (IsButtonPressed() && messageReady)
//	  {
//		/* Process the message when button is pressed and message is available */
//		ProcessReceivedMessage(message);
//
//		/* Reset message buffer */
//		messageIndex = 0;
//		messageReady = 0;
//		memset(message, 0, sizeof(message));
//
//		/* Debounce */
//		HAL_Delay(200);
//	  }
//    /* USER CODE END WHILE */

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


/* Button press detection function - modify for your specific board */
uint8_t IsButtonPressed(void)
{
  /* Assuming B2 is connected to PC13 (common on many Nucleo boards) */
  /* Note: B2 is typically active LOW (returns 0 when pressed) */

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
  {
    return 1; /* Button pressed */
  }
  return 0; /* Button not pressed */
}

/* This function is called when a character is received via UART */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) /* Change to match your UART instance */
  {
    /* Silently add character to message buffer (no echo) */
    if (messageIndex < sizeof(message) - 1) /* Leave space for null terminator */
    {
      message[messageIndex++] = rxBuffer[0];
      message[messageIndex] = '\0'; /* Always keep null-terminated */

      /* Set flag indicating we have data ready to process */
      messageReady = 1;
    }
    else
    {
      /* Buffer overflow, reset */
      messageIndex = 0;
      memset(message, 0, sizeof(message));

      /* Consider sending an overflow message */
      char overflowMsg[] = "Buffer overflow! Message cleared.\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)overflowMsg, strlen(overflowMsg), 1000);
    }

    /* Start the next reception */
    HAL_UART_Receive_IT(huart, (uint8_t*)rxBuffer, 1);
  }
}

/* Process the complete received message, currently it echos back to the UART port once B1 has been pressed. Later this needs to be changes to sign the letters of the words */
void ProcessReceivedMessage(char* msg)
{
  /* First, send a notification that button was pressed */
  char buttonMsg[] = "Button B2 pressed - Echoing received message:\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)buttonMsg, strlen(buttonMsg), 1000);

  /* Echo the exact message that was received */
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 1000);

  /* Add a newline for better readability */
  char newline[] = "\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)newline, strlen(newline), 1000);

  /* Optional: Notify completion */
  char completeMsg[] = "Message echo complete\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t*)completeMsg, strlen(completeMsg), 1000);
}


/**
 * @brief Sets the speed and direction of a servo
 * @param finger: Which finger (THUMB, INDEX, MIDDLE, RING, PINKY)
 * @param direction: STOP, CLOCKWISE, or COUNTERCLOCKWISE
 * @param speed: Speed percentage (0-100)
 * @retval None
 */
void Servo_SetMotion(Finger finger, Direction direction, int speed) {
    uint32_t pulse;

    // Clamp speed to 0-100%
    if (speed < 0) speed = 0;
    if (speed > 100) speed = 100;

    // Calculate pulse width based on direction and speed
    if (direction == STOP) {
        pulse = SERVO_STOP;
    } else if (direction == CLOCKWISE) {
        // Map 0-100% to SERVO_STOP-SERVO_MAX_CW
        pulse = SERVO_STOP - ((SERVO_STOP - SERVO_MAX_CW) * speed / 100);
    } else { // COUNTERCLOCKWISE
        // Map 0-100% to SERVO_STOP-SERVO_MAX_CCW
        pulse = SERVO_STOP + ((SERVO_MAX_CCW - SERVO_STOP) * speed / 100);
    }

    // Update servo state
    servoStates[finger].speed = speed;
    servoStates[finger].dir = direction;
    servoStates[finger].pulse = pulse;

    // Apply pulse width to the appropriate timer
    switch (finger) {
        case THUMB:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
            break;
        case INDEX:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse);
            break;
        case MIDDLE:
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
            break;
        case RING:
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse);
            break;
        case PINKY:
            __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pulse);
            break;
    }
	}

	/**
	 * @brief Stops all servos
	 * @param None
	 * @retval None
	 */
	void Servo_StopAll(void) {
		Servo_SetMotion(THUMB, STOP, 0);
		Servo_SetMotion(INDEX, STOP, 0);
		Servo_SetMotion(MIDDLE, STOP, 0);
		Servo_SetMotion(RING, STOP, 0);
		Servo_SetMotion(PINKY, STOP, 0);

	    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	}

	/**
	 * @brief Initialize all servo timers and start PWM
	 * @param None
	 * @retval None
	 */
	void Servo_Init(void) {
	    // Start all PWM channels
	    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);

//	    // Initialize all servos to stop position
//	    Servo_StopAll();
	}

	int Direction_Decider(int* Desired_Position){
		if(*Desired_Position < 0){
			*Desired_Position = *Desired_Position * -1;
			return CLOCKWISE;
		}
		else{
			return COUNTERCLOCKWISE;
		}
	}

	/**
	 * @brief Example function to demonstrate a sign language letter
	 * @param letter: ASCII character (A-Z)
	 * @param duration: How long to hold the position (in ms)
	 * @retval None
	 */
	void SignLetter(char letter) {
	    // Reset to neutral position
	    Servo_Init();


	    // Set finger positions based on the letter
	    switch(letter) {
	        case 'A':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;
	            break;

	        case 'B':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;
	            break;

	        case 'C':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'D':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'E':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'F':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'G':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'H':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'I':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'J':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'K':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'L':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'M':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'N':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'O':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'P':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'Q':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'R':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'S':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'T':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'U':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'V':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'W':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'X':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'Y':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        case 'Z':
	        	thumb_desired_position = thumb_current - 1 * THUMB_CLOSED;
	        	index_desired_position = index_current - 1 * INDEX_CLOSED;
				middle_desired_position = middle_current - 1 * MIDDLE_CLOSED;
				ring_desired_position = ring_current - 1 * RING_CLOSED;
			    pinky_desired_position = pinky_current - 1 * PINKY_CLOSED;

	        default:
	            // Default position (rest)
	            Servo_StopAll();
	            break;
	    }

	    Servo_SetMotion(THUMB, Direction_Decider(&thumb_desired_position), 100);
	    Servo_SetMotion(INDEX, Direction_Decider(&index_desired_position), 100);
	    Servo_SetMotion(MIDDLE, Direction_Decider(&middle_desired_position), 100);
	    Servo_SetMotion(RING, Direction_Decider(&ring_desired_position), 100);
	    Servo_SetMotion(PINKY, Direction_Decider(&pinky_desired_position), 100);

	    // Return to neutral position
	    Servo_StopAll();
	}

	void ResetHand(void) {
	    // Make sure PWM is active
	    Servo_Init();

	    Servo_SetMotion(THUMB, COUNTERCLOCKWISE, 60);
	    Servo_SetMotion(INDEX, CLOCKWISE, 80);
	    Servo_SetMotion(MIDDLE, CLOCKWISE, 80);
	    Servo_SetMotion(RING, CLOCKWISE, 80);
	    Servo_SetMotion(PINKY, CLOCKWISE, 80);

	    // Give servos time to reach position
	    HAL_Delay(2000);
	}


	/**
	 * @brief Initialize all servo timers and start PWM
	 * @param None
	 * @retval None
	 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */


	osTimerStart(Index_FingerHandle, index_current);
	osTimerStart(Thumb_FingerHandle, thumb_current);
	osTimerStart(Middle_FingerHandle, thumb_current);
	osTimerStart(Ring_FingerHandle, thumb_current);
	osTimerStart(Pinky_FingerHandle, thumb_current);

	SignLetter('A');
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* Index function */
void Index(void *argument)
{
  /* USER CODE BEGIN Index */

  /* USER CODE END Index */
}

/* Thumb function */
void Thumb(void *argument)
{
  /* USER CODE BEGIN Thumb */

  /* USER CODE END Thumb */
}

/* Middle function */
void Middle(void *argument)
{
  /* USER CODE BEGIN Middle */

  /* USER CODE END Middle */
}

/* Ring function */
void Ring(void *argument)
{
  /* USER CODE BEGIN Ring */

  /* USER CODE END Ring */
}

/* Pinky function */
void Pinky(void *argument)
{
  /* USER CODE BEGIN Pinky */

  /* USER CODE END Pinky */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
