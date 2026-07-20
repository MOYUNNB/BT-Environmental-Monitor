/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
#include "sensor_data.h"
#include "bluetooth.h"
#include "aht20.h"
#include "ina226.h"
#include "sd3078.h"
#include "icm42688.h"
#include "lcd.h"
#include "lcd_page.h"
#include "key.h"
#include "ws2812.h"
#include "tf_card.h"
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
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Sensor_Rea */
osThreadId_t Task_Sensor_ReaHandle;
const osThreadAttr_t Task_Sensor_Rea_attributes = {
  .name = "Task_Sensor_Rea",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for Task_LCD_Update */
osThreadId_t Task_LCD_UpdateHandle;
const osThreadAttr_t Task_LCD_Update_attributes = {
  .name = "Task_LCD_Update",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Task_TF_Log */
osThreadId_t Task_TF_LogHandle;
const osThreadAttr_t Task_TF_Log_attributes = {
  .name = "Task_TF_Log",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for Task_Bluetooth */
osThreadId_t Task_BluetoothHandle;
const osThreadAttr_t Task_Bluetooth_attributes = {
  .name = "Task_Bluetooth",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for Task_Key_Scan */
osThreadId_t Task_Key_ScanHandle;
const osThreadAttr_t Task_Key_Scan_attributes = {
  .name = "Task_Key_Scan",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for Task_WS2812 */
osThreadId_t Task_WS2812Handle;
const osThreadAttr_t Task_WS2812_attributes = {
  .name = "Task_WS2812",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for xQueue_SensorData */
osMessageQueueId_t xQueue_SensorDataHandle;
const osMessageQueueAttr_t xQueue_SensorData_attributes = {
  .name = "xQueue_SensorData"
};
/* Definitions for xQueue_BT_Command */
osMessageQueueId_t xQueue_BT_CommandHandle;
const osMessageQueueAttr_t xQueue_BT_Command_attributes = {
  .name = "xQueue_BT_Command"
};
/* Definitions for xSemaphore_I2C */
osMutexId_t xSemaphore_I2CHandle;
const osMutexAttr_t xSemaphore_I2C_attributes = {
  .name = "xSemaphore_I2C"
};
/* Definitions for xSemaphore_SensorData */
osMutexId_t xSemaphore_SensorDataHandle;
const osMutexAttr_t xSemaphore_SensorData_attributes = {
  .name = "xSemaphore_SensorData"
};
/* Definitions for xSemaphore_SPI2 */
osMutexId_t xSemaphore_SPI2Handle;
const osMutexAttr_t xSemaphore_SPI2_attributes = {
  .name = "xSemaphore_SPI2"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);
void StartSensorRead(void *argument);
void StartLCDUpdate(void *argument);
void StartTFLog(void *argument);
void StartBluetooth(void *argument);
void StartKeyScan(void *argument);
void StartWS2812(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of xSemaphore_I2C */
  xSemaphore_I2CHandle = osMutexNew(&xSemaphore_I2C_attributes);

  /* creation of xSemaphore_SensorData */
  xSemaphore_SensorDataHandle = osMutexNew(&xSemaphore_SensorData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* creation of xSemaphore_SPI2 */
  xSemaphore_SPI2Handle = osMutexNew(&xSemaphore_SPI2_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of xQueue_SensorData */
  xQueue_SensorDataHandle = osMessageQueueNew (10, sizeof(SensorData_t), &xQueue_SensorData_attributes);

  /* creation of xQueue_BT_Command */
  xQueue_BT_CommandHandle = osMessageQueueNew (5, sizeof(BT_CmdPacket_t), &xQueue_BT_Command_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Sensor_Rea */
  Task_Sensor_ReaHandle = osThreadNew(StartSensorRead, NULL, &Task_Sensor_Rea_attributes);

  /* creation of Task_LCD_Update */
  Task_LCD_UpdateHandle = osThreadNew(StartLCDUpdate, NULL, &Task_LCD_Update_attributes);

  /* creation of Task_TF_Log */
  Task_TF_LogHandle = osThreadNew(StartTFLog, NULL, &Task_TF_Log_attributes);

  /* creation of Task_Bluetooth */
  Task_BluetoothHandle = osThreadNew(StartBluetooth, NULL, &Task_Bluetooth_attributes);

  /* creation of Task_Key_Scan */
  Task_Key_ScanHandle = osThreadNew(StartKeyScan, NULL, &Task_Key_Scan_attributes);

  /* creation of Task_WS2812 */
  Task_WS2812Handle = osThreadNew(StartWS2812, NULL, &Task_WS2812_attributes);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 14;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 104;   /* 84MHz / (104+1) = 800KHz for WS2812 */
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, FLASH_CS_Pin|IMU_CS_Pin|LCD_CS_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_R_Pin|LED_G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLASH_CS_Pin IMU_CS_Pin LCD_CS_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = FLASH_CS_Pin|IMU_CS_Pin|LCD_CS_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY3_Pin */
  GPIO_InitStruct.Pin = KEY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY1_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY2_Pin */
  GPIO_InitStruct.Pin = KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EC11_A_Pin EC11_B_Pin SD_DETECT_Pin */
  GPIO_InitStruct.Pin = EC11_A_Pin|EC11_B_Pin|SD_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
  /* 系统监控任务: 打印调试信息 */
  for(;;)
  {
    osDelay(5000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorRead */
/**
* @brief Function implementing the Task_Sensor_Rea thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorRead */
void StartSensorRead(void *argument)
{
  /* USER CODE BEGIN StartSensorRead */
  uint32_t seq = 0;

  for(;;)
  {
    SensorData_t data = {0};
    data.seq_num = seq++;

    /* 1. 读取 SD3078 RTC 时间 */
    SD3078_Time_t rtc;
    if (SD3078_GetTime(&rtc) == SD3078_OK) {
      data.timestamp.year    = rtc.year;
      data.timestamp.month   = rtc.month;
      data.timestamp.day     = rtc.day;
      data.timestamp.hour    = rtc.hour;
      data.timestamp.minute  = rtc.minute;
      data.timestamp.second  = rtc.second;
      data.sensors_ok |= SENSOR_OK_SD3078;
    }

    /* 2. 读取 AHT20 温湿度 */
    if (AHT20_ReadData(&data.env.temperature, &data.env.humidity) == AHT20_OK) {
      data.sensors_ok |= SENSOR_OK_AHT20;
    }

    /* 3. 读取 INA226 电源数据 */
    if (INA226_ReadAll(&data.power.bus_voltage, &data.power.current, &data.power.power) == INA226_OK) {
      data.sensors_ok |= SENSOR_OK_INA226;
    }

    /* 4. 读取 ICM42688 IMU */
    if (ICM42688_ReadAccel(&data.imu.accel_x, &data.imu.accel_y, &data.imu.accel_z) == ICM42688_OK) {
      ICM42688_ReadGyro(&data.imu.gyro_x, &data.imu.gyro_y, &data.imu.gyro_z);
      ICM42688_ReadTemp(&data.imu.temp_c);
      data.sensors_ok |= SENSOR_OK_ICM42688;
    }

    /* 5. 发布数据 (更新全局 + 推入队列) */
    SensorData_Update(&data);

    osDelay(100);
  }
  /* USER CODE END StartSensorRead */
}

/* USER CODE BEGIN Header_StartLCDUpdate */
/**
* @brief Function implementing the Task_LCD_Update thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDUpdate */
void StartLCDUpdate(void *argument)
{
  /* USER CODE BEGIN StartLCDUpdate */
  SensorData_t data;

  for(;;)
  {
    /* 从队列取最新传感器数据 (阻塞 200ms) */
    if (osMessageQueueGet(xQueue_SensorDataHandle, &data, NULL, 200) == osOK) {
      LCD_Page_Refresh(&data);
    }

    osDelay(200);
  }
  /* USER CODE END StartLCDUpdate */
}

/* USER CODE BEGIN Header_StartTFLog */
/**
* @brief Function implementing the Task_TF_Log thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTFLog */
void StartTFLog(void *argument)
{
  /* USER CODE BEGIN StartTFLog */
  SensorData_t data;
  char timestamp[24];

  for(;;)
  {
    /* 读取全局传感器数据 */
    SensorData_Read(&data);

    /* 构造时间戳 */
    snprintf(timestamp, sizeof(timestamp), "%04u-%02u-%02u %02u:%02u:%02u",
             (unsigned)data.timestamp.year, (unsigned)data.timestamp.month,
             (unsigned)data.timestamp.day,
             (unsigned)data.timestamp.hour, (unsigned)data.timestamp.minute,
             (unsigned)data.timestamp.second);

    /* 写入 CSV */
    float accel[3] = {data.imu.accel_x, data.imu.accel_y, data.imu.accel_z};
    float gyro[3]  = {data.imu.gyro_x, data.imu.gyro_y, data.imu.gyro_z};
    TF_LogSensor(timestamp,
                 data.env.temperature, data.env.humidity,
                 data.power.bus_voltage, data.power.current, data.power.power,
                 accel, gyro);

    osDelay(1000);
  }
  /* USER CODE END StartTFLog */
}

/* USER CODE BEGIN Header_StartBluetooth */
/**
* @brief Function implementing the Task_Bluetooth thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBluetooth */
void StartBluetooth(void *argument)
{
  /* USER CODE BEGIN StartBluetooth */
  SensorData_t data;
  BT_CmdPacket_t cmd;

  for(;;)
  {
    /* 1. 读取传感器数据并发送 */
    SensorData_Read(&data);
    BLUETOOTH_SendSensorData(data.env.temperature, data.env.humidity,
                             data.power.bus_voltage, data.power.current, data.power.power);

    /* 2. 检查蓝牙命令 */
    if (BLUETOOTH_GetCmd(&cmd)) {
      if (cmd.cmd == BT_CMD_SET_LED) {
        WS2812_SetAll((uint8_t)cmd.r, (uint8_t)cmd.g, (uint8_t)cmd.b);
        WS2812_Update();
      }
    }

    osDelay(1000);
  }
  /* USER CODE END StartBluetooth */
}

/* USER CODE BEGIN Header_StartKeyScan */
/**
* @brief Function implementing the Task_Key_Scan thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKeyScan */
void StartKeyScan(void *argument)
{
  /* USER CODE BEGIN StartKeyScan */
  for(;;)
  {
    KEY_Scan();

    /* EC11 翻页 */
    int8_t delta = EC11_GetDelta();
    if (delta > 0) {
      PageID_t next = (PageID_t)((g_current_page + 1) % PAGE_COUNT);
      LCD_Page_Switch(next);
    } else if (delta < 0) {
      PageID_t prev = (PageID_t)((g_current_page + PAGE_COUNT - 1) % PAGE_COUNT);
      LCD_Page_Switch(prev);
    }

    /* 按键处理 */
    KeyID_t key = KEY_GetPressed();
    if (key == KEY_1) {
      LCD_Page_Switch(PAGE_DATA);  /* KEY1 返回数据页 */
    }

    osDelay(10);
  }
  /* USER CODE END StartKeyScan */
}

/* USER CODE BEGIN Header_StartWS2812 */
/**
* @brief Function implementing the Task_WS2812 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartWS2812 */
void StartWS2812(void *argument)
{
  /* USER CODE BEGIN StartWS2812 */
  SensorData_t data;

  for(;;)
  {
    SensorData_Read(&data);
    float t = data.env.temperature;

    /* 温度映射颜色: 蓝→青→绿→黄→红 */
    uint8_t r = 0, g = 0, b = 0;
    if (t < 10.0f) {
      r = 0;   g = 0;   b = 255;  /* 蓝色 (冷) */
    } else if (t < 20.0f) {
      r = 0;   g = 255; b = 255;  /* 青色 */
    } else if (t < 30.0f) {
      r = 0;   g = 255; b = 0;    /* 绿色 (舒适) */
    } else if (t < 40.0f) {
      r = 255; g = 255; b = 0;    /* 黄色 */
    } else {
      r = 255; g = 0;   b = 0;    /* 红色 (热) */
    }

    WS2812_SetAll(r, g, b);
    WS2812_Update();

    osDelay(1000);
  }
  /* USER CODE END StartWS2812 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
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
#ifdef USE_FULL_ASSERT
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
