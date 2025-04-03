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
#include <stdlib.h>
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
#define SERVO_WRIST_BEND	TIM15, TIM_CHANNEL_1
#define SERVO_WRIST_ROTATE	TIM17, TIM_CHANNEL_1

#define THUMB_CLOSED 500		// Verified
#define INDEX_CLOSED 1600		// Verified
#define MIDDLE_CLOSED 1500		// Verified
#define RING_CLOSED 1500		// Verified
#define PINKY_CLOSED 900		//
#define WRIST_CLOSED 2000		//TBD idk how long it will take to get to 180.

typedef enum {
    THUMB = 0,
    INDEX,
    MIDDLE,
    RING,
    PINKY,
	WRIST_BEND,
	WRIST_ROTATE
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
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

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
/* Definitions for Wrist_Bend */
osTimerId_t Wrist_BendHandle;
const osTimerAttr_t Wrist_Bend_attributes = {
  .name = "Wrist_Bend"
};
/* Definitions for Wrist_Rotate */
osTimerId_t Wrist_RotateHandle;
const osTimerAttr_t Wrist_Rotate_attributes = {
  .name = "Wrist_Rotate"
};
/* USER CODE BEGIN PV */
char rxBuffer[1]; /* Buffer for receiving a single character */
char message[256]; /* Buffer to store the complete message */
uint16_t messageIndex = 0;
uint8_t messageReady = 0; /* Flag to indicate a message is in buffer waiting to be processed*/

/* Global servo states */
ServoState servoStates[7] = {
    {0, STOP, SERVO_STOP},  // THUMB
    {0, STOP, SERVO_STOP},  // INDEX
    {0, STOP, SERVO_STOP},  // MIDDLE
    {0, STOP, SERVO_STOP},  // RING
    {0, STOP, SERVO_STOP},  // PINKY
	{0, STOP, SERVO_STOP},  // WRIST_BEND
	{0, STOP, SERVO_STOP}	// WRIST_ROTATE
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
static void MX_TIM15_Init(void);
static void MX_TIM17_Init(void);
void StartDefaultTask(void *argument);
void Index(void *argument);
void Thumb(void *argument);
void Middle(void *argument);
void Ring(void *argument);
void Pinky(void *argument);
void wrist_bend(void *argument);
void wrist_rotate(void *argument);

/* USER CODE BEGIN PFP */

void Servo_Init(void);
void Servo_SetMotion(Finger finger, Direction direction, int speed);
void Servo_StopAll(void);
void SignLetter(char letter);
int Direction_Decider(int Desired_Position);
int TimeVariation(Finger finger, int desired_position);

static void MX_USART2_UART_Init(void);
void ProcessReceivedMessage(char* msg);
uint8_t IsButtonPressed(void);

volatile BaseType_t indexDone = pdFALSE;
volatile BaseType_t thumbDone = pdFALSE;
volatile BaseType_t middleDone = pdFALSE;
volatile BaseType_t ringDone = pdFALSE;
volatile BaseType_t pinkyDone = pdFALSE;
volatile BaseType_t wrist_BendDone = pdFALSE;
volatile BaseType_t wrist_RotateDone = pdFALSE;

volatile BaseType_t default_init = pdTRUE;

int thumb_current = 0;
int index_current = 0;
int middle_current = 0;
int ring_current = 0;
int pinky_current = 0;
int wrist_bend_current = 0;
int wrist_rotate_current = 0;

//debugging counter for subsequent letter signing
int16_t counter = 0;
char word[] = "HELLO";


int thumb_TravelTime = 0;
int index_TravelTime = 0;
int middle_TravelTime = 0;
int ring_TravelTime = 0;
int pinky_TravelTime = 0;
int wrist_bend_TravelTime = 0;
int wrist_rotate_TravelTime = 0;

int thumb_desired_position = 0;
int index_desired_position = 0;
int middle_desired_position = 0;
int ring_desired_position = 0;
int pinky_desired_position = 0;
int wrist_bend_desired_position = 0;
int wrist_rotate_desired_position = 0;


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
  MX_TIM15_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */

  // For testing:
//	thumb_desired_position = 0;
//	index_desired_position = 0;
//	middle_desired_position = 0;
//	ring_desired_position = 0;
//	pinky_desired_position = 900;

//  SignLetter('A');

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
  Thumb_FingerHandle = osTimerNew(Thumb, osTimerOnce, NULL, &Thumb_Finger_attributes);

  /* creation of Middle_Finger */
  Middle_FingerHandle = osTimerNew(Middle, osTimerOnce, NULL, &Middle_Finger_attributes);

  /* creation of Ring_Finger */
  Ring_FingerHandle = osTimerNew(Ring, osTimerOnce, NULL, &Ring_Finger_attributes);

  /* creation of Pinky_Finger */
  Pinky_FingerHandle = osTimerNew(Pinky, osTimerOnce, NULL, &Pinky_Finger_attributes);

  /* creation of Wrist_Bend */
  Wrist_BendHandle = osTimerNew(wrist_bend, osTimerOnce, NULL, &Wrist_Bend_attributes);

  /* creation of Wrist_Rotate */
  Wrist_RotateHandle = osTimerNew(wrist_rotate, osTimerOnce, NULL, &Wrist_Rotate_attributes);

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
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 79;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 65535;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
        case WRIST_BEND:
        	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pulse);
        	break;
        case WRIST_ROTATE:
        	__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, pulse);
        	break;
    	}
	}


// Helps decided whether servo should move clockwise or counter-clockwise
int Direction_Decider(int Desired_Position){
	if(Desired_Position < 0){
		return CLOCKWISE;
	}
	else if(Desired_Position > 0){
		return COUNTERCLOCKWISE;
	}
	else{
		return STOP;
	}
}

	/*Parameters: Which finger is being moved, and the desired position
	 * Returns: integer of calculated time to move finger that far
	 * This program calculated how long the finger must move to reach the desired position based off of observations. */
	int TimeVariation(Finger finger, int desired_position){
		// Checks if the finger is moving down, if it is no variation required
		if(desired_position < 0){
			desired_position *= -1;
			return desired_position;
		}
		else if(desired_position == 0)
			return 0;
		// Actual time variation calculations based on testing
		else {
		return desired_position *= 0.5;
		}
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

	// Controls whether hand signs "BYUI" (1), or "HELLO WORLD" (0)
	int byui = 1;

  for(;;)
  {
	  if(default_init){

			thumb_TravelTime = TimeVariation(THUMB, thumb_desired_position);
			index_TravelTime = TimeVariation(INDEX, index_desired_position);
			middle_TravelTime = TimeVariation(MIDDLE, middle_desired_position);
			ring_TravelTime = TimeVariation(RING, ring_desired_position);
			pinky_TravelTime = TimeVariation(PINKY, pinky_desired_position);
			wrist_bend_TravelTime = TimeVariation(WRIST_BEND, wrist_bend_desired_position);
			wrist_rotate_TravelTime = TimeVariation(WRIST_ROTATE, wrist_rotate_desired_position);

			if(thumb_TravelTime != 0){
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
				Servo_SetMotion(THUMB, Direction_Decider(thumb_desired_position), 100);
				osTimerStart(Thumb_FingerHandle, thumb_TravelTime);
			}
			else if(thumb_TravelTime == 0){
				thumbDone = pdTRUE;
			}

			if(index_TravelTime != 0){
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
				Servo_SetMotion(INDEX, Direction_Decider(index_desired_position), 100);
				osTimerStart(Index_FingerHandle, index_TravelTime);
			}
			else if(index_TravelTime == 0){
				indexDone = pdTRUE;
			}

			if(middle_TravelTime != 0){
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
				Servo_SetMotion(MIDDLE, Direction_Decider(middle_desired_position), 100);
				osTimerStart(Middle_FingerHandle, middle_TravelTime);
			}
			else if(middle_TravelTime == 0){
				middleDone = pdTRUE;
			}

			if(ring_TravelTime != 0){
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
				Servo_SetMotion(RING, Direction_Decider(ring_desired_position), 100);
				osTimerStart(Ring_FingerHandle, ring_TravelTime);
			}
			else if(ring_TravelTime == 0){
				ringDone = pdTRUE;
			}

			if(pinky_TravelTime != 0){
				HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
				Servo_SetMotion(PINKY, Direction_Decider(pinky_desired_position), 100);
				osTimerStart(Pinky_FingerHandle, pinky_TravelTime);
			}
			else if(pinky_TravelTime == 0){
				pinkyDone = pdTRUE;
			}

			default_init = pdFALSE;
	  }
	  // Checks if all timers have completed
	  if(indexDone && thumbDone && middleDone && ringDone && pinkyDone){
		  osDelay(1000);
		  indexDone = pdFALSE;
		  thumbDone = pdFALSE;
		  middleDone = pdFALSE;
		  ringDone = pdFALSE;
		  pinkyDone = pdFALSE;
		  wrist_BendDone = pdFALSE;
		  wrist_RotateDone = pdFALSE;

		  index_current = abs(index_desired_position);
		  thumb_current = abs(thumb_desired_position);
		  middle_current = abs(middle_desired_position);
		  ring_current = abs(ring_desired_position);
		  pinky_current = abs(pinky_desired_position);


		  if(byui == 0){

			  switch(counter){
			  case 0:
				  // Hard Code signing 'H'
				  thumb_desired_position = -1*THUMB_CLOSED;
				  index_desired_position = 0;
				  middle_desired_position = 0;
				  ring_desired_position = -1*RING_CLOSED;
				  pinky_desired_position = -1*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 1:
				  // Hard Code signing 'E'
				  thumb_desired_position = 0.5*THUMB_CLOSED;
				  index_desired_position = -0.25*INDEX_CLOSED;
				  middle_desired_position = -0.25*MIDDLE_CLOSED;
				  ring_desired_position = 0.75*RING_CLOSED;
				  pinky_desired_position = 0.75*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 2:
				  // Hard Code signing 'L'
				  thumb_desired_position = 0.5*THUMB_CLOSED;
				  index_desired_position = 0.5*INDEX_CLOSED;
				  middle_desired_position = -0.75*MIDDLE_CLOSED;
				  ring_desired_position = -0.75*RING_CLOSED;
				  pinky_desired_position = -0.75*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 3:
				  // Hard Code signing slight movement from 'L'
				  thumb_desired_position = -0.25*THUMB_CLOSED;
				  index_desired_position = -0.25*INDEX_CLOSED;
				  middle_desired_position = 0;
				  ring_desired_position = 0;
				  pinky_desired_position = 0;
				  default_init = pdTRUE;
				  break;
			  case 4:
				  // Hard Code signing back to 'L' for double L
				  thumb_desired_position = 0.5*THUMB_CLOSED;
				  index_desired_position = 0.5*INDEX_CLOSED;
				  middle_desired_position = 0;
				  ring_desired_position = 0;
				  pinky_desired_position = 0;
				  default_init = pdTRUE;
				  break;
			  case 5:
				  // Hard Code signing 'O'
				  thumb_desired_position = 0*THUMB_CLOSED;
				  index_desired_position = -0.3*INDEX_CLOSED;
				  middle_desired_position = 0.5*MIDDLE_CLOSED;
				  ring_desired_position = 0.75*RING_CLOSED;
				  pinky_desired_position = 0.75*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 6:
				  // Hard Code signing space/pause
				  thumb_desired_position = 0;
				  index_desired_position = 0;
				  middle_desired_position = 0;
				  ring_desired_position = 0;
				  pinky_desired_position = 0;
				  default_init = pdTRUE;
				  break;
			  case 7:
				  // Hard Code signing 'W'
				  thumb_desired_position = -1*THUMB_CLOSED;
				  index_desired_position = 0.75*INDEX_CLOSED;
				  middle_desired_position = 0.75*MIDDLE_CLOSED;
				  ring_desired_position = 0.5*RING_CLOSED;
				  pinky_desired_position = -0.5*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 8:
				  // Hard Code signing 'O'
				  thumb_desired_position = 0.5*THUMB_CLOSED;
				  index_desired_position = -0.3*INDEX_CLOSED;
				  middle_desired_position = -0.35*MIDDLE_CLOSED;
				  ring_desired_position = -0.3*RING_CLOSED;
				  pinky_desired_position = 0.75*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 9:
				  // Hard Code signing 'R'
				  thumb_desired_position = 0;
				  index_desired_position = 0.35*INDEX_CLOSED;
				  middle_desired_position = 0.5*MIDDLE_CLOSED;
				  ring_desired_position = -0.5*RING_CLOSED;
				  pinky_desired_position = -0.65*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 10:
				  // Hard Code signing 'L'
				  thumb_desired_position = 1*THUMB_CLOSED;
				  index_desired_position = 0.3*INDEX_CLOSED;
				  middle_desired_position = -1*MIDDLE_CLOSED;
				  ring_desired_position = 0;
				  pinky_desired_position = 0;
				  default_init = pdTRUE;
				  break;
			  case 11:
				  // Hard Code signing 'D'
				  thumb_desired_position = -1*THUMB_CLOSED;
				  index_desired_position = 0;
				  middle_desired_position = 0.25*MIDDLE_CLOSED;
				  ring_desired_position = 0.25*RING_CLOSED;
				  pinky_desired_position = 0.25*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  case 12:
				  // Hard code reset
				  thumb_desired_position = 0.5*THUMB_CLOSED;
				  index_desired_position = -0.2*INDEX_CLOSED;
				  middle_desired_position = 0.5*MIDDLE_CLOSED;
				  ring_desired_position = 0.65*RING_CLOSED;
				  pinky_desired_position = 0.65*PINKY_CLOSED;
				  default_init = pdTRUE;
				  break;
			  default:
				  break;
			  }
		  }
		  else if(byui == 1){
			  switch(counter){
			  case 0:
				  default_init = pdTRUE;
				  // Hard Code signing
				  thumb_desired_position = -1*THUMB_CLOSED;
				  index_desired_position = 0;
				  middle_desired_position = 0;
				  ring_desired_position = 0;
				  pinky_desired_position = 0;
				  break;
			  case 1:
				  default_init = pdTRUE;
				  // Hard Code signing
				  thumb_desired_position = THUMB_CLOSED;
				  index_desired_position = -1*INDEX_CLOSED;
				  middle_desired_position = -1*MIDDLE_CLOSED;
				  ring_desired_position = -1*RING_CLOSED;
				  pinky_desired_position = 0;
				  break;
			  case 2:
				  default_init = pdTRUE;
				  // Hard Code signing
				  thumb_desired_position = -1*THUMB_CLOSED;
				  index_desired_position = 1.1*INDEX_CLOSED;
				  middle_desired_position = MIDDLE_CLOSED;
				  ring_desired_position = 0;
				  pinky_desired_position = -1*PINKY_CLOSED;
				  break;
			  case 3:
				  default_init = pdTRUE;
					thumb_desired_position = 0;
					index_desired_position = -1*INDEX_CLOSED;
					middle_desired_position = -1* MIDDLE_CLOSED;
					ring_desired_position = 0;
					pinky_desired_position = PINKY_CLOSED;
				  break;
			  case 4:
				  default_init = pdTRUE;
				  // Hard Code signing
				  thumb_desired_position = -0.75*THUMB_CLOSED;
				  index_desired_position = 0.75*INDEX_CLOSED;
				  middle_desired_position = 0.75*MIDDLE_CLOSED;
				  ring_desired_position = RING_CLOSED;
				  pinky_desired_position = 0;
				  break;
			  default:
				  break;
			  }
		  }
			  counter += 1;
	  }

	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* Index function */
void Index(void *argument)
{
  /* USER CODE BEGIN Index */
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	indexDone = pdTRUE;
  /* USER CODE END Index */
}

/* Thumb function */
void Thumb(void *argument)
{
  /* USER CODE BEGIN Thumb */
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	thumbDone = pdTRUE;
  /* USER CODE END Thumb */
}

/* Middle function */
void Middle(void *argument)
{
  /* USER CODE BEGIN Middle */
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
	middleDone = pdTRUE;
  /* USER CODE END Middle */
}

/* Ring function */
void Ring(void *argument)
{
  /* USER CODE BEGIN Ring */
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	ringDone = pdTRUE;
  /* USER CODE END Ring */
}

/* Pinky function */
void Pinky(void *argument)
{
  /* USER CODE BEGIN Pinky */
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	pinkyDone = pdTRUE;
  /* USER CODE END Pinky */
}

/* wrist_bend function */
void wrist_bend(void *argument)
{
  /* USER CODE BEGIN wrist_bend */
		HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
		wrist_BendDone = pdTRUE;
		wrist_bend_current = wrist_bend_desired_position;
  /* USER CODE END wrist_bend */
}

/* wrist_rotate function */
void wrist_rotate(void *argument)
{
  /* USER CODE BEGIN wrist_rotate */
	HAL_TIM_PWM_Stop(&htim17, TIM_CHANNEL_1);
	wrist_RotateDone = pdTRUE;
	wrist_rotate_current = wrist_rotate_desired_position;
  /* USER CODE END wrist_rotate */
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
