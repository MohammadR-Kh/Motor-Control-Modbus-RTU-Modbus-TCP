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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "motor.h"
#include "uart_rs_485.h"
#include "register_map.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "ethernet_config.h"
#include "ethernet.h"
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
IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for MODBUS_TCP_Task */
osThreadId_t MODBUS_TCP_TaskHandle;
const osThreadAttr_t MODBUS_TCP_Task_attributes = {
  .name = "MODBUS_TCP_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for MODBUS_RTU_Task */
osThreadId_t MODBUS_RTU_TaskHandle;
const osThreadAttr_t MODBUS_RTU_Task_attributes = {
  .name = "MODBUS_RTU_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Monitor_Task */
osThreadId_t Monitor_TaskHandle;
const osThreadAttr_t Monitor_Task_attributes = {
  .name = "Monitor_Task",
  .stack_size = 540 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Watchdog_Task */
osThreadId_t Watchdog_TaskHandle;
const osThreadAttr_t Watchdog_Task_attributes = {
  .name = "Watchdog_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_Task */
osThreadId_t Motor_TaskHandle;
const osThreadAttr_t Motor_Task_attributes = {
  .name = "Motor_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM9_Init(void);
static void MX_SPI1_Init(void);
static void MX_IWDG_Init(void);
void Start_MODBUS_TCP_Task(void *argument);
void Start_MODBUS_RTU_Task(void *argument);
void Start_Monitor_Task(void *argument);
void Start_Watchdog_Task(void *argument);
void Start_Motor_Task(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern volatile int32_t encoder_position;
extern volatile uint32_t encoder_last_cnt;
extern volatile float motor_rpm;
extern volatile float pwm;
extern volatile float target_rpm_cmd;
extern volatile float target_rpm_ramped;
extern volatile int32_t target_position;
extern volatile float position_speed_cmd;
extern volatile uint8_t position_reached;
extern PID_t motor_pid;

extern MotorMode_t motor_mode;

extern uint8_t rx_buf[RX_BUF_SIZE];
extern volatile uint8_t uart_idle_flag;
static volatile uint32_t rx_last_pos = 0;
volatile uint32_t uart_rx_total_bytes = 0;
extern volatile uint32_t rx_overflow_count;
extern volatile uint8_t rx_overflow_flag;
uint32_t last_bmp_tick = 0;
uint8_t line_buf[LINE_MAX_LEN];
volatile float motor_rpm_raw = 0;
uint8_t speed_locked;
float locked_pwm;

volatile uint32_t wd_tcp_counter = 0;
volatile uint32_t wd_rtu_counter = 0;
volatile uint32_t wd_control_counter = 0;

volatile uint32_t ulHighFrequencyTimerTicks = 0U;

void configureTimerForRunTimeStats(void){
	ulHighFrequencyTimerTicks = 0U;
	HAL_TIM_Base_Start_IT(&htim10);
}
unsigned long getRunTimeCounterValue(void){
	return (unsigned long)ulHighFrequencyTimerTicks;
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM10_Init();
  MX_USART1_UART_Init();
  MX_TIM9_Init();
  MX_SPI1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  Encoder_Start();
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  RS485_RX_ENABLE();
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
  uart_dma_rx_start();
  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
  reg_wizchip_spi_cbfunc(wizchip_read, wizchip_write);
  W5500_Reset();
  uint8_t txsize[8] = {2,2,2,2,2,2,2,2};
  uint8_t rxsize[8] = {2,2,2,2,2,2,2,2};
  wizchip_init(txsize, rxsize);
  Ethernet_Init();
  HAL_TIM_Base_Start_IT(&htim9);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MODBUS_TCP_Task */
  MODBUS_TCP_TaskHandle = osThreadNew(Start_MODBUS_TCP_Task, NULL, &MODBUS_TCP_Task_attributes);

  /* creation of MODBUS_RTU_Task */
  MODBUS_RTU_TaskHandle = osThreadNew(Start_MODBUS_RTU_Task, NULL, &MODBUS_RTU_Task_attributes);

  /* creation of Monitor_Task */
  Monitor_TaskHandle = osThreadNew(Start_Monitor_Task, NULL, &Monitor_Task_attributes);

  /* creation of Watchdog_Task */
  Watchdog_TaskHandle = osThreadNew(Start_Watchdog_Task, NULL, &Watchdog_Task_attributes);

  /* creation of Motor_Task */
  Motor_TaskHandle = osThreadNew(Start_Motor_Task, NULL, &Motor_Task_attributes);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 3000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 6;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 6;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 799;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 99;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_Start_MODBUS_TCP_Task */
/**
  * @brief  Function implementing the MODBUS_TCP_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_MODBUS_TCP_Task */
void Start_MODBUS_TCP_Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  TCP_Server();
	  wd_tcp_counter++;
	  osDelay(2);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_MODBUS_RTU_Task */
/**
* @brief Function implementing the MODBUS_RTU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MODBUS_RTU_Task */
void Start_MODBUS_RTU_Task(void *argument)
{
  /* USER CODE BEGIN Start_MODBUS_RTU_Task */
  /* Infinite loop */
  for(;;)
  {
	  if (uart_idle_flag) {
		  uart_idle_flag = 0;

		  uint32_t write_pos = RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
		  uint32_t read_pos = rx_last_pos;


		  uint32_t avail;
		  if (write_pos >= read_pos) avail = write_pos - read_pos;
		  else avail = (RX_BUF_SIZE - read_pos) + write_pos;

		  if (avail == 0) {
			  continue;
		  }
		  uart_rx_total_bytes += avail;

		  if (avail >= LINE_MAX_LEN) {
			  rx_overflow_count++;
			  rx_overflow_flag = 1;


			  uint32_t keep = LINE_MAX_LEN - 1;
			  uint32_t skip = avail - keep;


			  read_pos = (read_pos + skip) % RX_BUF_SIZE;
			  avail = keep;
		  }


		  uint32_t first_chunk = RX_BUF_SIZE - read_pos;
		  if (first_chunk > avail) first_chunk = avail;

		  memcpy(line_buf, &rx_buf[read_pos], first_chunk);
		  if (avail > first_chunk) {
			  memcpy(line_buf + first_chunk, &rx_buf[0], avail - first_chunk);
		  }
		  line_buf[avail] = 0;


		  rx_last_pos = (read_pos + avail) % RX_BUF_SIZE;
		  modbus_handle_frame(line_buf, avail);
	  }
	  wd_rtu_counter++;
	  osDelay(1);
  }
  /* USER CODE END Start_MODBUS_RTU_Task */
}

/* USER CODE BEGIN Header_Start_Monitor_Task */
/**
* @brief Function implementing the Monitor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Monitor_Task */
void Start_Monitor_Task(void *argument)
{
  /* USER CODE BEGIN Start_Monitor_Task */
	char statsBuffer[1024];
	  /* Infinite loop */
	  for(;;)
	  {
	    vTaskGetRunTimeStats(statsBuffer);
	    HAL_UART_Transmit(&huart1, (uint8_t*)statsBuffer, strlen(statsBuffer), HAL_MAX_DELAY);
	    const char *nl = "\r\n";
	    HAL_UART_Transmit(&huart1, (uint8_t*)nl, 2, HAL_MAX_DELAY);
	    TaskStatus_t pxTaskStatusArray[10];
	    UBaseType_t uxArraySize;
	    uint32_t totalRunTime;
	    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, 10, &totalRunTime);

	    HAL_UART_Transmit(&huart1, (uint8_t*)"\nStack Usage:\n", 14, 100);

	    for (int i = 0; i < uxArraySize; i++)
	    {
	        char line[64];
	        sprintf(line,
	            "%s -> Free stack: %i bytes\r\n",
	            pxTaskStatusArray[i].pcTaskName,
	            pxTaskStatusArray[i].usStackHighWaterMark * 4
	        );
	        HAL_UART_Transmit(&huart1, (uint8_t*)line, strlen(line), 100);
	    }

	    osDelay(5000);
	  }
  /* USER CODE END Start_Monitor_Task */
}

/* USER CODE BEGIN Header_Start_Watchdog_Task */
/**
* @brief Function implementing the Watchdog_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Watchdog_Task */
void Start_Watchdog_Task(void *argument)
{
  /* USER CODE BEGIN Start_Watchdog_Task */
  /* Infinite loop */
	uint32_t last_tcp = 0;
	uint32_t last_rtu = 0;
	uint32_t last_control = 0;

	for(;;)
	{
		uint8_t tcp_ok = 0;
		uint8_t rtu_ok = 0;
		uint8_t control_ok = 0;

		if(wd_tcp_counter != last_tcp)
		{
			tcp_ok = 1;
			last_tcp = wd_tcp_counter;
		}

		if(wd_rtu_counter != last_rtu)
		{
			rtu_ok = 1;
			last_rtu = wd_rtu_counter;
		}

		if(wd_control_counter != last_control)
		{
			control_ok = 1;
			last_control = wd_control_counter;
		}

		if(tcp_ok && rtu_ok && control_ok)
		{
			HAL_IWDG_Refresh(&hiwdg);
		}
		osDelay(100);
	}
  /* USER CODE END Start_Watchdog_Task */
}

/* USER CODE BEGIN Header_Start_Motor_Task */
/**
* @brief Function implementing the Motor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Motor_Task */
void Start_Motor_Task(void *argument)
{
  /* USER CODE BEGIN Start_Motor_Task */
  /* Infinite loop */
  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  motor_mode = (MotorMode_t)holding_regs[REG_MODE];
	  int16_t rpm_cmd = (int16_t)holding_regs[REG_TARGET_RPM];
	  target_rpm_cmd = (float)rpm_cmd;
	  int16_t pos_cmd = (int16_t)holding_regs[REG_TARGET_POSITION];
	  target_position = (pos_cmd * ENCODER_CPR * 36)/360;
	  switch (motor_mode)
	  {
		  case MOTOR_MODE_IDLE:
		  {
			  Motor_SetPWM(0);
			  speed_locked = 0;
			  locked_pwm = 0;
			  target_rpm_cmd = 0;
			  target_rpm_ramped = 0;
			  encoder_last_cnt = TIM2->CNT;
			  encoder_position = 0;
			  pwm = 0;
			  motor_rpm = 0;
			  PID_Reset(&motor_pid);
			  break;
		  }
		  case MOTOR_MODE_SPEED:
		  {
			  Ramp_Update();
			  float error = target_rpm_ramped - motor_rpm;
			  if(speed_locked)
			  	  {
				  if(fabs(error) > 10) speed_locked = 0;
				 }

				 if(!speed_locked)
				 {
					 pwm = PID_Update(&motor_pid, target_rpm_ramped, motor_rpm);
					 if(fabs(error) < 5)
					 {
						 speed_locked = 1;
						 locked_pwm = pwm;
					 }
				 }
				 if(speed_locked)
					 Motor_SetPWM(locked_pwm);
				 else
					 Motor_SetPWM(pwm);
				 break;
		  }

		  case MOTOR_MODE_POSITION:
		  {
			  int32_t pos_error = target_position - encoder_position;

			  if (abs(pos_error) <= POSITION_TOLERANCE_COUNTS)
			  {
				  target_rpm_cmd = 0;
					Motor_SetPWM(0);
					PID_Reset(&motor_pid);
					motor_mode = MOTOR_MODE_IDLE;
					holding_regs[REG_MODE] = MOTOR_MODE_IDLE;
				}
			  else{
				  Position_Controller(target_position, encoder_position);
			  }
			  Ramp_Update();
			  pwm = PID_Update(&motor_pid, target_rpm_ramped, motor_rpm);
			  Motor_SetPWM(pwm);
			  break;
		  }
	  }
	  holding_regs[REG_ACTUAL_RPM] =
		  (int16_t)motor_rpm;

	  holding_regs[REG_ACTUAL_POSITION] =
		  (int16_t)encoder_position;

	  holding_regs[REG_PWM_OUTPUT] =
		  (uint16_t)pwm;
	  wd_control_counter++;
  }

  /* USER CODE END Start_Motor_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM9)
  {
	  uint32_t current_cnt = TIM2->CNT;
	  int32_t delta = (int32_t)(current_cnt - encoder_last_cnt);
	  encoder_last_cnt = current_cnt;
	  encoder_position += delta;
	  motor_rpm_raw = (delta * CONTROL_HZ * 60.0f) / ENCODER_CPR;
	  motor_rpm += 0.1f * (motor_rpm_raw - motor_rpm);
	  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	  vTaskNotifyGiveFromISR(Motor_TaskHandle, &xHigherPriorityTaskWoken);
	  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
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
