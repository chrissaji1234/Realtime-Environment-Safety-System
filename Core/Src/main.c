/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @author			: Christopher Saji
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  *	@materials
  *
  *	STM32 NUCLEO-L4R5ZI
  *
  *	SparkFun Sound Detector
  *
  *	UVM-30A
  *
  *	Vibration Motor
  *
  *	2x LED
  *
  *	2x 220 OHM Resistors
  *
  *	1602 LCD Display
  *
  *	Breadboard
  *
  *	2.54mm Dupont Jumper Cables
  *
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <i2c-lcd.h>
#include <stdio.h>
#include <stdlib.h>
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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId readAudioSensorHandle;
osThreadId feedbackDevicesHandle;
osThreadId lcdOutputHandle;
osThreadId readUVSensorHandle;
osSemaphoreId Binary_SemaphoreHandle;
/* USER CODE BEGIN PV */

uint32_t audio_raw;
uint32_t uv_raw;

uint8_t audio_str[20];
uint8_t uv_str[20];

int db_current;
int uv_current;

float uv_voltage;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
void readAudioSensor_EF(void const * argument);
void feedbackDevices_EF(void const * argument);
void lcdOutput_EF(void const * argument);
void readUVSensor_EF(void const * argument);

/* USER CODE BEGIN PFP */

void ADC_Select_CH8(void);
void ADC_Select_CH14(void);

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
  MX_LPUART1_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  lcd_init();			// Initialize LCD and bring cursor to home
  lcd_clear();
  lcd_put_cur(0,0);

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);		// Calibrate ADC for better accuracy

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of Binary_Semaphore */
  osSemaphoreDef(Binary_Semaphore);
  Binary_SemaphoreHandle = osSemaphoreCreate(osSemaphore(Binary_Semaphore), 1);

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
  /* definition and creation of readAudioSensor */
  osThreadDef(readAudioSensor, readAudioSensor_EF, osPriorityRealtime, 0, 128);
  readAudioSensorHandle = osThreadCreate(osThread(readAudioSensor), NULL);

  /* definition and creation of feedbackDevices */
  osThreadDef(feedbackDevices, feedbackDevices_EF, osPriorityNormal, 0, 128);
  feedbackDevicesHandle = osThreadCreate(osThread(feedbackDevices), NULL);

  /* definition and creation of lcdOutput */
  osThreadDef(lcdOutput, lcdOutput_EF, osPriorityNormal, 0, 128);
  lcdOutputHandle = osThreadCreate(osThread(lcdOutput), NULL);

  /* definition and creation of readUVSensor */
  osThreadDef(readUVSensor, readUVSensor_EF, osPriorityRealtime, 0, 128);
  readUVSensorHandle = osThreadCreate(osThread(readUVSensor), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
//  sConfig.Channel = ADC_CHANNEL_14;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_8;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /*
   * Make sure to comment out the configure regular channel for both channels
   * and to change hadc1.Init.NbrOfConversion from 2 to 1. This is so the threads
   * can configure each ADC channel individually.
  */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.Timing = 0x107075B0;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : STLK_RX_Pin STLK_TX_Pin */
  GPIO_InitStruct.Pin = STLK_RX_Pin|STLK_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ADC_Select_CH8(void)	// Configures the ADC to use channel 8
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}


void ADC_Select_CH14(void)	// Configures the ADC to use channel 14
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	 {
	   Error_Handler();
	 }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_readAudioSensor_EF */
/**
  * @brief  Function implementing the readAudioSensor thread.
  * @param  argument: Not used
  * @retval None
  * @purpose This is the entry function for thread readAudioSensor. It begins
  * by acquiring if the semaphore is available. If it isn't, the thread waits until it is
  * available. If it is, it calls the custom ADC_Select_CH8 function to configure the ADC
  * to read channel 8. The thread uses polling to acquire the raw analog value. Using
  * a linear regression formula calculated before hand using an SPL meter, the thread
  * calculates the dB value. It finally stores the value into a string and releases
  * the semaphore.
  */
/* USER CODE END Header_readAudioSensor_EF */
void readAudioSensor_EF(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  char *strtosend1 = "Entering readAudioSensor\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend1, strlen(strtosend1), HAL_MAX_DELAY);

	  osSemaphoreWait(Binary_SemaphoreHandle, osWaitForever);

	  ADC_Select_CH8();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  audio_raw = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

	  db_current = (audio_raw * 0.0269) + 23.143;				// Calculated linear regression formula using SPL meter.
	  	  	  	  	  	  	  	  	  	  	  	  	  	  	// Took ADC readings at different dB levels.

	  snprintf(audio_str, sizeof(audio_str), "dB: %d", db_current);

//	  char *strtosend2 = "Exiting readAudioSensor\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend2, strlen(strtosend2), HAL_MAX_DELAY);

	  osSemaphoreRelease(Binary_SemaphoreHandle);


    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_feedbackDevices_EF */
/**
* @brief Function implementing the feedbackDevices thread.
* @param argument: Not used
* @retval None
* @purpose This is the entry function for the thread feedbackDevices. This thread controls the LCD
* and vibration motor that allow the user to know if the current dB and UV reading is dangerous
* respectively. If the current dB reading is higher than or equal to 85 dB, the red LED will turn
* on. Otherwise, the green LED will be illuminated. In addition to this, if the UV index is greater
* than or equal to 8, the vibration motor will begin to vibrate.
*/
/* USER CODE END Header_feedbackDevices_EF */
void feedbackDevices_EF(void const * argument)
{
  /* USER CODE BEGIN feedbackDevices_EF */
  /* Infinite loop */
  for(;;)
  {

//	  char *strtosend1 = "Entering feedbackDevices\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend1, strlen(strtosend1), HAL_MAX_DELAY);

	  if (db_current < 85){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	  }else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	  }

	  if (uv_current >= 8){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
	  }else{
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	  }



//	  char *strtosend2 = "Exiting feedbackDevices\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend2, strlen(strtosend2), HAL_MAX_DELAY);



    osDelay(200);
  }
  /* USER CODE END feedbackDevices_EF */
}

/* USER CODE BEGIN Header_lcdOutput_EF */
/**
* @brief Function implementing the lcdOutput thread.
* @param argument: Not used
* @retval None
* @purpose This is the entry function for the thread lcdOut. This thread controls the output of the
* LCD display. The thread begins by clearing the current information on the LCD and bringing
* the cursor back to home. It then outputs the current dB measurement to the screen. It then
* moves the cursor to the start of the next line and then displays the current UV index reading
* to the screen.
*/
/* USER CODE END Header_lcdOutput_EF */
void lcdOutput_EF(void const * argument)
{
  /* USER CODE BEGIN lcdOutput_EF */
  /* Infinite loop */
  for(;;)
  {

//	  char *strtosend1 = "Entering lcdOutputDevices\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend1, strlen(strtosend1), HAL_MAX_DELAY);

	  lcd_clear();
	  lcd_put_cur(0,0);

	  lcd_send_string(audio_str);
	  lcd_put_cur(1,0);
	  lcd_send_string(uv_str);

//	  char *strtosend2 = "Exciting lcdOutputDevices\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend2, strlen(strtosend2), HAL_MAX_DELAY);


    osDelay(200);
  }
  /* USER CODE END lcdOutput_EF */
}

/* USER CODE BEGIN Header_readUVSensor_EF */
/**
* @brief Function implementing the readUVSensor thread.
* @param argument: Not used
* @retval None
* @purpose This is the entry function for the thread readUVSensor. The thread begins by checking if the
* semaphore is available or not. If the semaphore is not available, the thread will wait for it. If the
* semaphore is available, the thread will call the custom ADC_Select_CH14 function to configure the ADC
* to read channel 14. The thread uses polling to acquire the raw analog value. The thread then converts
* the raw analog reading to the real miliVolt reading. Next, the thread then checks what the UV index
* based on the voltage calculated. Finally, the current UV index is converted to a string and the
* semaphore is released.
*/
/* USER CODE END Header_readUVSensor_EF */
void readUVSensor_EF(void const * argument)
{
  /* USER CODE BEGIN readUVSensor_EF */
  /* Infinite loop */
  for(;;)
  {
//	  char *strtosend1 = "Entering readUVSensor\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend1, strlen(strtosend1), HAL_MAX_DELAY);

	  osSemaphoreWait(Binary_SemaphoreHandle, osWaitForever);

	  ADC_Select_CH14();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uv_raw = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

	  uv_voltage = (uv_raw * (3.3/4095)) * 1000;

	  if (uv_voltage < 50){
		  uv_current = 0;
	  }
	  else if (uv_voltage >= 50 && uv_voltage <= 227){
		  uv_current = 1;
	  }
	  else if (uv_voltage >= 228 && uv_voltage <= 318){
		  uv_current = 2;
	  }
	  else if (uv_voltage >= 319 && uv_voltage <= 408){
		  uv_current = 3;
	  }
	  else if (uv_voltage >= 409 && uv_voltage <= 503){
		  uv_current = 4;
	  }
	  else if (uv_voltage >= 504 && uv_voltage <= 606){
		  uv_current = 5;
	  }
	  else if (uv_voltage >= 607 && uv_voltage <= 696){
		  uv_current = 6;
	  }
	  else if (uv_voltage >= 697 && uv_voltage <= 795){
		  uv_current = 7;
	  }
	  else if (uv_voltage >= 796 && uv_voltage <= 881){
		  uv_current = 8;
	  }
	  else if (uv_voltage >= 882 && uv_voltage <= 976){
		  uv_current = 9;
	  }
	  else if (uv_voltage >= 977 && uv_voltage <= 1079){
		  uv_current = 10;
	  }
	  else if (uv_voltage >= 1080){
		  uv_current = 11;
	  }

	  snprintf(uv_str, sizeof(uv_str), "UV: %d", uv_current);


//	  char *strtosend2 = "Exiting readUVSensor\n";
//	  HAL_UART_Transmit(&hlpuart1, (uint8_t *) strtosend2, strlen(strtosend2), HAL_MAX_DELAY);

	  osSemaphoreRelease(Binary_SemaphoreHandle);


    osDelay(100);
  }
  /* USER CODE END readUVSensor_EF */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
