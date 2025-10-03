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
#include "Din_LED.h"
#include "stdbool.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHARLIE_START_PIN 2
#define CHARLIE_PIN_COUNT 6
#define PWM_CYCLE_LENGTH 255  // Max brightness

// Map index 0–5 to PA2–PA7
#define GET_PIN(index) (1 << (CHARLIE_START_PIN + (index)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
#define MAX_LED 12
LEDs leds [MAX_LED + 3];
//rgb_color led_pattern[MAX_LED];
rgb_color time_pattern[MAX_LED];
rgb_color charging_pattern[MAX_LED];
//rgb_color all_pattern[MAX_LED];
//rgb_color null_pattern[MAX_LED];
rgb_color none = {0, 0, 0, 0};
//rgb_color all = {255, 255, 255, 0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */
void Charlieplex_Reset_All(void);
void Charlieplex_Light_LED(uint8_t highIndex, uint8_t lowIndex);
void Digital_show(uint8_t hours, uint8_t minutes, bool date);
void Init_BH1750FVI(void);
uint16_t BH1750_ReadLightLevel(void);
void delay_us (uint16_t us);
void beep(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t minutes = 15;
uint8_t hours = 6;
uint8_t am = 0;
uint8_t shifted_hours = 6;
uint16_t batteryVal = 0;
uint8_t shifted_minutes = 3;
bool showingLeds = false;
bool showingDigital = false;
bool changeTime = false;
bool changeColor = false;
bool charging = false;
bool shouldExitIf = false;
uint8_t toggler = false;
uint8_t RX_Buffer [1];
uint8_t x = 0;
uint8_t colorTheme = 0;
uint16_t lightLevel = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	rgb_color hour_color = {80, 255, 70, 255};
	rgb_color minut_color = {20, 90, 255, 255};
	//rgb_color red = {250, 0, 0, 4};
	//rgb_color blue = {0, 0, 250, 4};
	rgb_color green = {0, 250, 0, 4};
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim21); // KDYZTAK ODSTRAN - ZATIM NIC NEDELA
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  Charlieplex_Reset_All();
  Init_BH1750FVI();
  time_pattern[shifted_hours] = hour_color;
  time_pattern[shifted_minutes] = minut_color;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_ADC_Start(&hadc);
//
//	  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
//
//	  batteryVal = HAL_ADC_GetValue(&hadc);
//
//	  HAL_ADC_Stop(&hadc);

	 /* HAL_SuspendTick();
	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	  HAL_ResumeTick();
  */

	  // RGB LEDS --------------------------------------------------------
	  if (showingLeds && !charging) {
		  //TIM2 ->CCR1 = 60;
		  beep();
		  x = 0;
		  shouldExitIf = false;
		  //TIM2->CCR2=55;
		  while (HAL_GPIO_ReadPin(Button_R_GPIO_Port, Button_R_Pin)) {
			  x++;
			  if (x >= 200) {
				  changeTime = true;
				  shouldExitIf = true;
				  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
				  __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(&hrtc, RTC_FLAG_WUTF);
				  break;
			  }
			  HAL_Delay(10);
		  }
		  if (!shouldExitIf) {
			  if (shifted_hours==shifted_minutes) {
				  hour_color.a = BH1750_ReadLightLevel();
				  minut_color.a = BH1750_ReadLightLevel();
				  for (int i = 0; i < 5; i++) {
					  if (toggler) {
						  time_pattern[shifted_hours] = hour_color;
					  } else {
						  time_pattern[shifted_minutes] = minut_color;
					  }
					  HAL_Delay(100);
					  toggler = !toggler;

					  turn_spec_LEDs(leds, time_pattern);
					  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
					  HAL_Delay(500);
					  clear_LEDs(leds);
					  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
					  HAL_Delay(500);
					  clear_LEDs(leds);
					  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
					  HAL_Delay(50);
				  }
			  } else {
				  hour_color.a = BH1750_ReadLightLevel();
				  minut_color.a = BH1750_ReadLightLevel();
				  time_pattern[shifted_hours] = hour_color;
				  time_pattern[shifted_minutes] = minut_color;

				  turn_spec_LEDs(leds, time_pattern);
				  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
				  HAL_Delay(5000);

				  clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
				  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
				  HAL_Delay(50);
			  }

			  showingLeds = false;
		  }
	  }
	  // DIGITAL LEDS ------------------------------------------------
	  if (showingDigital && !charging) {
		  x=0;
		  shouldExitIf = false;
		  while (HAL_GPIO_ReadPin(Button_LT_GPIO_Port, Button_LT_Pin)) {
			  x++;
			  if (x >= 200) {
				  charging = true;
				  shouldExitIf = true;
				  x=0;
				  break;
			  }
			  HAL_Delay(10);
		  }
		  while (HAL_GPIO_ReadPin(Button_LB_GPIO_Port, Button_LB_Pin)) {
			  x++;
			  if (x >= 200) {
				  changeColor = true;
				  shouldExitIf = true;
				  x=0;
				  break;
			  }
			  HAL_Delay(10);
		  }
		  beep();
		  if (!shouldExitIf) {
			  for(int i = 0; i < 3000; i++){
				  Digital_show(hours, minutes,0);
			  }
			  Charlieplex_Reset_All();
		  }
		  showingDigital = false;
	  }
	  // CHARGING WATCH ----------------------------------------------
	  if (charging) {
		  if (!showingDigital || shouldExitIf) {
			  //TIM2 ->CCR1 = 60;
			  x++;
			  uint8_t chargedLEDnum = 0;
			  HAL_ADC_Start(&hadc);
			  HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
			  batteryVal = HAL_ADC_GetValue(&hadc);
			  HAL_ADC_Stop(&hadc);
			  HAL_Delay(10);
			  batteryVal = HAL_ADC_GetValue(&hadc);
			  HAL_ADC_Stop(&hadc);
			  HAL_Delay(10);
			  batteryVal = HAL_ADC_GetValue(&hadc);
			  HAL_ADC_Stop(&hadc);
			  HAL_Delay(10);
			  if (batteryVal > 2550) batteryVal = 2550;
			  if (batteryVal < 2050) batteryVal = 2050;
			  chargedLEDnum = (int)((batteryVal - 2050) * 12 / (2550 - 2050));
			  green.a = BH1750_ReadLightLevel();
			  for (int i = 0; i < chargedLEDnum; i++) {
				  charging_pattern[i] = green;
			  }
			  turn_spec_LEDs(leds, charging_pattern);
			  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
			  if (x>=100 && shouldExitIf) {
				  shouldExitIf = false;
				  charging = false;
				  showingDigital = false;
				  showingLeds = false;
				  clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
				  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
				  HAL_Delay(10);
				  clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
				  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
			  }
			  HAL_Delay(10);
		  } else {
			  clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
			  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
			  HAL_Delay(10);
			  clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
			  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
		  }
	  }
	  // CHANGING TIME ----------------------------------------------
	  while(changeTime) {
		  //TIM2 ->CCR1 = 60;
		  //beep();
		  hour_color.a = BH1750_ReadLightLevel();
		  minut_color.a = BH1750_ReadLightLevel();
		  time_pattern[shifted_hours] = hour_color;
		  time_pattern[shifted_minutes] = minut_color;
		  if (shifted_hours==shifted_minutes) {
			  if (toggler) {
				  time_pattern[shifted_hours] = hour_color;
			  } else {
				  time_pattern[shifted_minutes] = minut_color;
			  }
			  toggler = !toggler;
		  }
		  turn_spec_LEDs(leds, time_pattern);
		  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
		  for(int i = 0; i < 300; i++){
			  Digital_show(hours, minutes,0);
		  }
		  clear_LEDs(leds);
		  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
		  for(int i = 0; i < 300; i++){
			  Digital_show(hours, minutes,0);
		  }

	  }
	  // CHANGING COLOR ----------------------------------------------
	  while(changeColor) {
	  		  //TIM2 ->CCR1 = 60;
	  		  //beep();
	  		  shouldExitIf = false;
	  		  if (colorTheme == 0) {
				  hour_color = (rgb_color){80, 255, 70, 255};
				  minut_color = (rgb_color){20, 90, 255, 255};
	  		  } else if (colorTheme == 1) {
				  hour_color = (rgb_color){255, 10, 0, 255};
				  minut_color = (rgb_color){255, 100, 0, 255};
	  		  } else if (colorTheme == 2) {
				  hour_color = (rgb_color){144, 10, 255, 255};
				  minut_color = (rgb_color){255, 15, 50, 255};
	  		  } else {
				  hour_color = (rgb_color){255, 10, 0, 255};
				  minut_color = (rgb_color){20, 90, 255, 255};
	  		  }
	  		  hour_color.a = BH1750_ReadLightLevel();
	  		  minut_color.a = BH1750_ReadLightLevel();
	  		  time_pattern[shifted_hours] = hour_color;
	  		  time_pattern[shifted_minutes] = minut_color;
	  		  if (shifted_hours==shifted_minutes) {
	  			  if (toggler) {
	  				  time_pattern[shifted_hours] = hour_color;
	  				  turn_spec_LEDs(leds, time_pattern);
					  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
					  HAL_Delay(250);
	  			  } else {
	  				  time_pattern[shifted_minutes] = minut_color;
	  				  turn_spec_LEDs(leds, time_pattern);
					  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
					  HAL_Delay(250);
	  			  }
	  			  toggler = !toggler;
	  		  }
	  		  turn_spec_LEDs(leds, time_pattern);
	  		  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
	  		  HAL_Delay(10);

	  	  }
	  if (charging==false) {
		  // Enter Stop Mode
		  __HAL_RCC_DMA1_CLK_DISABLE();
	  	  TIM2 ->CCR1 = 0;
	  	  HAL_SuspendTick();

	  	  //HAL_PWR_EnableSleepOnExit();

	  	  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_RTC;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00B07CB4;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  /*if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 259, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }*/
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Row1_Pin|Row2_Pin|Row3_Pin|Row4_Pin
                          |Row5_Pin|Row6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_LT_Pin Button_LB_Pin Button_R_Pin */
  GPIO_InitStruct.Pin = Button_LT_Pin|Button_LB_Pin|Button_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Row1_Pin Row2_Pin Row3_Pin Row4_Pin
                           Row5_Pin Row6_Pin */
  GPIO_InitStruct.Pin = Row1_Pin|Row2_Pin|Row3_Pin|Row4_Pin
                          |Row5_Pin|Row6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CHRG_Pin */
  GPIO_InitStruct.Pin = CHRG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHRG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DONE_Pin */
  GPIO_InitStruct.Pin = DONE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DONE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//SystemClock_Config();
	//HAL_ResumeTick();
	SystemClock_Config();
	HAL_ResumeTick();
	//HAL_PWR_DisableSleepOnExit();
	__HAL_RCC_DMA1_CLK_ENABLE();
//	HAL_ADC_Start(&hadc);
//	HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
//	batteryVal = HAL_ADC_GetValue(&hadc);
//	HAL_ADC_Stop(&hadc);
//	HAL_Delay(10);
//	batteryVal = HAL_ADC_GetValue(&hadc);
//	HAL_ADC_Stop(&hadc);
//	HAL_Delay(10);
//	batteryVal = HAL_ADC_GetValue(&hadc);
//	HAL_ADC_Stop(&hadc);
//	HAL_Delay(10);
	if (GPIO_Pin == Button_LB_Pin) {
		if (charging) {
			showingDigital = !showingDigital;
		} else {//if (batteryVal>=2250) {
			showingDigital = true;
		}
		if (changeTime) {
			hours++;
			if (hours >= 24) {
				hours = 0;
			}
			shifted_hours = hours%12;//(hours<=6) ? hours+6 : hours-6;
			if (shifted_hours == 0) {
				time_pattern[11] = none;
			} else {
				time_pattern[shifted_hours-1] = none;
			}
		} else if (changeColor) {
			beep();
			colorTheme-=1;
			colorTheme=colorTheme%4;
		}
    }
    if (GPIO_Pin == Button_LT_Pin) {
		if (charging) {
			showingDigital = !showingDigital;
		} else {//if (batteryVal>=2250) {
			showingDigital = true;
		}
		if (changeTime) {
			minutes+=1;
			if (minutes >= 60) {
				minutes = 0;
			}
			shifted_minutes = minutes/5;
			if (shifted_minutes == 0) {
				time_pattern[11] = none;
			} else {
				time_pattern[shifted_minutes-1] = none;
			}
		} else if (changeColor) {
			beep();
			colorTheme+=1;
			colorTheme=colorTheme%4;
		}
	}
    if (GPIO_Pin == Button_R_Pin) {
    	//if (batteryVal>=2250) {
    		showingLeds = true;
    	//}
    	if (charging == true) {
    		showingDigital = !showingDigital;
    	}
    	if (changeTime) {
    		changeTime = false;
    		showingLeds = false;
    		showingDigital = false;
    		clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
    		HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
    		/** Enable the WakeUp
    		 */
    		if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 59, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
    		{
    			Error_Handler();
    		}
    	} else if (changeColor) {
    		changeColor = false;
    		showingLeds = false;
    		showingDigital = false;
    		clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
    		HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
    	}
	}
    if (GPIO_Pin == CHRG_Pin) {
    	if (!shouldExitIf) {
			charging = !charging;
			showingLeds = false;
			showingDigital = false;
			beep();
    	}
		if (charging==false) {
			clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
			beep();
			clear_LEDs(leds);//turn_spec_LEDs(leds, null_pattern);
			HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
		}


	}

    //HAL_SuspendTick();
    //HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc) {
	/*
	SystemClock_Config();
	HAL_ResumeTick();
	HAL_PWR_DisableSleepOnExit();
	clear_LEDs(leds, MAX_LED);
	HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)leds, (MAX_LED * 24) + 72);
	HAL_Delay(50);
	*/
    minutes+=1;
	if (minutes >= 60) {
		minutes = 0;
		time_pattern[11] = none;
		shifted_minutes = 0;
		hours++;
		if (hours >= 24) {
			hours = 0;
		}
		shifted_hours = hours%12;
		if (shifted_hours == 0) {
			time_pattern[11] = none;
		} else {
			time_pattern[shifted_hours-1] = none;
		}
	} else {
		shifted_minutes = minutes/5;
		time_pattern[shifted_minutes-1] = none;
	}
}

void Charlieplex_Reset_All(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    for (uint8_t i = 0; i < CHARLIE_PIN_COUNT; i++) {
        GPIO_InitStruct.Pin = GET_PIN(i);
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

void Charlieplex_Light_LED(uint8_t highIndex, uint8_t lowIndex) {
    if (highIndex == lowIndex || highIndex >= CHARLIE_PIN_COUNT || lowIndex >= CHARLIE_PIN_COUNT)
        return;  // Invalid input

    Charlieplex_Reset_All();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Set high pin: output, high
    GPIO_InitStruct.Pin = GET_PIN(highIndex);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GET_PIN(highIndex), GPIO_PIN_SET);

    // Set low pin: output, low
    GPIO_InitStruct.Pin = GET_PIN(lowIndex);
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GET_PIN(lowIndex), GPIO_PIN_RESET);
}

void Digital_show(uint8_t hoursD, uint8_t minutesD, bool date) {
	typedef struct {
		uint8_t x;
		uint8_t y;
	} Pair;

	static const Pair hour2[10][7] = {
		{ {1,0}, {2,0}, {3,0}, {4,0}, {5,0}, {0,1} },		// number 0
	    { {2,0}, {3,0} },									// number 1
	    { {1,0}, {2,0}, {2,1}, {5,0}, {4,0} },				// number 2
	    { {1,0}, {2,0}, {2,1}, {3,0}, {4,0} },				// number 3
	    { {0,1}, {2,0}, {2,1}, {3,0} },						// number 4
	    { {1,0}, {0,1}, {2,1}, {3,0}, {4,0} },				// number 5
	    { {1,0}, {0,1}, {2,1}, {5,0}, {4,0}, {3,0} },		// number 6
	    { {1,0}, {2,0}, {3,0} },							// number 7
	    { {1,0}, {2,0}, {3,0}, {4,0}, {5,0}, {0,1}, {2,1} },// number 8
	    { {1,0}, {2,0}, {3,0}, {4,0}, {0,1}, {2,1} }		// number 9
	};
	static const Pair hour1[10][7] = {
		{ {3,1}, {4,1}, {5,1}, {0,2}, {1,2}, {3,2} },		// number 0
		{ {4,1}, {5,1} },									// number 1
		{ {3,1}, {4,1}, {4,2}, {1,2}, {0,2} },				// number 2
		{ {3,1}, {4,1}, {4,2}, {5,1}, {0,2} },				// number 3
		{ {3,2}, {4,1}, {4,2}, {5,1} },						// number 4
		{ {3,1}, {3,2}, {4,2}, {5,1}, {0,2} },				// number 5
		{ {3,1}, {5,1}, {0,2}, {1,2}, {3,2}, {4,2} },		// number 6
		{ {3,1}, {4,1}, {5,1} },							// number 7
		{ {3,1}, {4,1}, {5,1}, {0,2}, {1,2}, {3,2}, {4,2} },// number 8
		{ {3,1}, {4,1}, {5,1}, {0,2}, {3,2}, {4,2} }		// number 9
	};
	static const Pair minute2[10][7] = {
		{ {5,2}, {0,3}, {1,3}, {2,3}, {4,3}, {5,3} },		// number 0
		{ {0,3}, {1,3} },									// number 1
		{ {5,2}, {0,3}, {0,4}, {4,3}, {2,3} },				// number 2
		{ {5,2}, {0,3}, {0,4}, {1,3}, {2,3} },				// number 3
		{ {5,3}, {0,3}, {0,4}, {1,3} },						// number 4
		{ {5,2}, {5,3}, {0,4}, {1,3}, {2,3} },				// number 5
		{ {5,2}, {5,3}, {0,4}, {4,3}, {1,3}, {2,3} },		// number 6
		{ {5,2}, {0,3}, {1,3} },							// number 7
		{ {5,2}, {0,3}, {1,3}, {2,3}, {4,3}, {5,3}, {0,4} },// number 8
		{ {5,2}, {0,3}, {1,3}, {2,3}, {5,3}, {0,4} }		// number 9
	};
	static const Pair minute1[10][7] = {
		{ {1,4}, {2,4}, {3,4}, {5,4}, {0,5}, {1,5} },		// number 0
		{ {2,4}, {3,4} },									// number 1
		{ {1,4}, {2,4}, {2,5}, {0,5}, {5,4} },				// number 2
		{ {1,4}, {2,4}, {2,5}, {3,4}, {5,4} },				// number 3
		{ {1,5}, {2,4}, {2,5}, {3,4} },						// number 4
		{ {1,4}, {1,5}, {2,5}, {3,4}, {5,4} },				// number 5
		{ {1,4}, {1,5}, {2,5}, {0,5}, {3,4}, {5,4} },		// number 6
		{ {1,4}, {2,4}, {3,4} },							// number 7
		{ {1,4}, {2,4}, {3,4}, {5,4}, {0,5}, {1,5}, {2,5} },// number 8
		{ {1,4}, {2,4}, {3,4}, {5,4}, {1,5}, {2,5} }		// number 9
	};
	static const uint8_t LED_len[10] = {6, 2, 5, 5, 4, 5, 6, 3, 7, 6};
	uint8_t hour_tens = hoursD / 10;
	uint8_t hour_ones = hoursD % 10;
	uint8_t minute_tens = minutesD / 10;
	uint8_t minute_ones = minutesD % 10;

	for (int i = 0; i < LED_len[hour_tens]; i++) {
		if (hour_tens != 0) {
			Charlieplex_Light_LED(hour2[hour_tens][i].x, hour2[hour_tens][i].y);
		}
	}
	for (int i = 0; i < LED_len[hour_ones]; i++) {
		Charlieplex_Light_LED(hour1[hour_ones][i].x, hour1[hour_ones][i].y);
	}
	for (int i = 0; i < LED_len[minute_tens]; i++) {
		if (minute_tens != 0) {
			Charlieplex_Light_LED(minute2[minute_tens][i].x, minute2[minute_tens][i].y);
		}
	}
	for (int i = 0; i < LED_len[minute_ones]; i++) {
		Charlieplex_Light_LED(minute1[minute_ones][i].x, minute1[minute_ones][i].y);
	}
	Charlieplex_Reset_All();
}

#define BH1750_ADDRESS         (0x23 << 1) // I2C 7-bit address shifted for HAL
#define BH1750_POWER_ON        0x01
#define BH1750_RESET           0x07
#define BH1750_CONT_H_RES_MODE 0x10 // Continuous high-res mode (1 lx resolution)
void Init_BH1750FVI(void) {
    uint8_t cmd;

    cmd = BH1750_POWER_ON;
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    cmd = BH1750_RESET;
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(10);

    cmd = BH1750_CONT_H_RES_MODE;
    HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDRESS, &cmd, 1, HAL_MAX_DELAY);
    HAL_Delay(200);  // Allow first reading to stabilize
}
uint16_t BH1750_ReadLightLevel(void) {
    uint8_t data[2];
    float lux_min = 50.0f;
	float lux_max = 60000.0f;
    HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDRESS, data, 2, HAL_MAX_DELAY);


    float lux = (data[0] << 8) | data[1];
    lux = lux / 1.2; // Convert to lux according to datasheet: value / 1.2
    if (lux < lux_min) lux = lux_min;
    if (lux > lux_max) lux = lux_max;


    float b = (logf(lux) - logf(lux_min)) / (logf(lux_max) - logf(lux_min));
	return 2 + (uint8_t)(b * 253.0f);
}
void delay_us (uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim21,0);
	while (__HAL_TIM_GET_COUNTER(&htim21) < us);
}
void beep(void) {
	for(unsigned int i = 0; i < 10000; i++) {
	  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
	  delay_us(185);
	  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
	  delay_us(185);
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
