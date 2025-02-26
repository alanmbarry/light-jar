/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;

accel_vect_t accel_vect_current;
accel_vect_t accel_vect_prev;

uint32_t led_enabled;

static volatile uint32_t RTCSecondTdelayCounter = 0;
static volatile uint32_t AlarmITTriggered = 0;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
uint32_t ProcessAccelVal(int16_t vectelem);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static __IO uint32_t TimingDelay;
void Delay(uint32_t nTime)
{
    TimingDelay = nTime;
	while(TimingDelay != 0);
}


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* Concept of code
   * Configuration
   * Configuration of accelerometer
   *
   * Slow check mode
   * Measure accelerometer vector
   * Sleep for 0.5s ?
   * Measure accelerometer vector
   * Measure delta
   * If > x then go to fast update mode, else stay in slow check mode
   *
   * Pin usage:
   * 	B11 - SDA2 - to accelerometer
   * 	B10 - SCL2 - to accelerometer
   * 	A0  - PWM for LED channel
   * 	A1  - PWM for LED channel
   * 	A2  - PWM for LED channel
   * 	A3  - power gate control for 12V step-up regulator

   *
   */
  led_enabled = 0;


  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();


  /* Configure the system clock */
  SystemClock_Config();


  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_TIM2_Init();

  uint32_t mainwhileloopcount = 0;
  GPIO_PinState PinStateLED;
  PinStateLED = GPIO_PIN_SET;
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, PinStateLED);
  PinStateLED = GPIO_PIN_RESET;
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, PinStateLED);

  // Turn off +12V regulator on powerup
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);





  // Configure the MPU6050
  // Set the full-scale range of the accelerometer to +/- 4g
  mpu6050_writereg_simple(&hi2c2, false, 0x1C, 0x08);
  // Enable low-power cycle mode and disable the temperature sensor
  mpu6050_writereg_simple(&hi2c2, false, 0x6b, 0x28);

  // Register 0x6C is the power management register
  // Disable the gyros, set LP_WAKE_CTRL[1:0] to 1, which takes accelerometer readings
  // at 5Hz
  // Set to 0 for 1.25Hz  10uA current
  // Set to 1 for 5Hz  -  20uA current
  // Set to 2 for 20Hz
  // Set to 3 for 40Hz
  // LP_WAKE_CTRL occupies bits [7:6] of the register

  // 1.25 Hz:
  mpu6050_writereg_simple(&hi2c2, false, 0x6c, 0x07);

  const uint32_t GO_TO_SLEEP_TIMEOUT = 63;
  //int32_t temp_int32;
  //uint32_t pulseval_current;
  PinStateLED = GPIO_PIN_SET;
  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, PinStateLED);

  uint32_t state_timeout;
  enum programstate{
	  SLEEP,
	  AWAKEN,
	  AWAKE,
	  GO_TO_SLEEP} state;

  state = SLEEP;
  state_timeout = 0;
  accel_vect_prev = (accel_vect_t){.x = 0, .y = 0, .z = 0};

  uint32_t debugx = 0;
  uint32_t debugy = 0;
  uint32_t debugz = 0;


  while (1)
  {
	  switch(state){
	  case SLEEP:
		  PinStateLED = GPIO_PIN_SET;
		  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, PinStateLED);
		  mpu6050_get_accel_vect(&hi2c2, false, &accel_vect_current);
		  if(AccelVectDeltAbs(accel_vect_current, accel_vect_prev) > 3000)
		  {
			  state = AWAKEN;
			  state_timeout = 0;
			  accel_vect_prev = accel_vect_current;
			  // 40 Hz accelerometer refresh:
			  mpu6050_writereg_simple(&hi2c2, false, 0x6c, 0xC7);
		  }
		  else
		  {
		      state = SLEEP;
		      TIM2->CCR1 = 0;
		      TIM2->CCR2 = 0;
		      TIM2->CCR3 = 0;

			  //RTCAlarmDelayNoSleep(1000);
			  RTCAlarmDelayWithStop(1000);

			  accel_vect_prev = accel_vect_current;

		  }
		  break;
	  case AWAKEN:
		  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		  mpu6050_get_accel_vect(&hi2c2, false, &accel_vect_current);
		  TIM2->CCR1 = (ProcessAccelVal(accel_vect_current.x) * state_timeout) >> 6;
		  TIM2->CCR2 = (ProcessAccelVal(accel_vect_current.y) * state_timeout) >> 6;
		  TIM2->CCR3 = (ProcessAccelVal(accel_vect_current.z) * state_timeout) >> 6;
	  	  if(state_timeout >= 63)
	  	  {
	  		  state = AWAKE;
	  		  accel_vect_prev = accel_vect_current;
	  		  state_timeout = 0;
	  	  }
	  	  else
	  	  {
	  		  state = AWAKEN;
	  		  accel_vect_prev = accel_vect_current;
	  		  state_timeout = state_timeout + 1;
	  	  }

		  RTCAlarmDelayNoSleep(20);

	      break;
	  case AWAKE:
		  if(PinStateLED == GPIO_PIN_SET)
			  PinStateLED = GPIO_PIN_RESET;
		  else
			  PinStateLED = GPIO_PIN_SET;
		  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, PinStateLED);
		  mpu6050_get_accel_vect(&hi2c2, false, &accel_vect_current);
		  debugx = ProcessAccelVal(accel_vect_current.x);
		  debugy = ProcessAccelVal(accel_vect_current.y);
		  debugz = ProcessAccelVal(accel_vect_current.z);
		  TIM2->CCR1 = debugx;
		  TIM2->CCR2 = debugy;
		  TIM2->CCR3 = debugz;

		  if(AccelVectDeltAbs(accel_vect_current, accel_vect_prev) > 1000)
		  {
			  state_timeout = 0;
		  }
		  else
		  {
			  state_timeout = state_timeout + 1;
		  }
  		  accel_vect_prev = accel_vect_current;


		  if(state_timeout >= 500)
		  {
			  state = GO_TO_SLEEP;
			  state_timeout = 0;
		  }

		  RTCAlarmDelayNoSleep(20);

		  break;
	  case GO_TO_SLEEP:
		  PinStateLED = GPIO_PIN_SET;
		  HAL_GPIO_WritePin (GPIOC, GPIO_PIN_13, PinStateLED);
		  mpu6050_get_accel_vect(&hi2c2, false, &accel_vect_current);
		  TIM2->CCR1 = (ProcessAccelVal(accel_vect_current.x) * (GO_TO_SLEEP_TIMEOUT - state_timeout)) >> 6;
		  TIM2->CCR2 = (ProcessAccelVal(accel_vect_current.y) * (GO_TO_SLEEP_TIMEOUT - state_timeout)) >> 6;
		  TIM2->CCR3 = (ProcessAccelVal(accel_vect_current.z) * (GO_TO_SLEEP_TIMEOUT - state_timeout)) >> 6;
	  	  if(state_timeout >= 63)
	  	  {
	  		  state = SLEEP;
	  		  accel_vect_prev = accel_vect_current;
	  		  state_timeout = 0;
			  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
			  // 1.25 Hz accelerometer refresth:
			  mpu6050_writereg_simple(&hi2c2, false, 0x6c, 0x07);

	  	  }
	  	  else
	  	  {
	  		  state = GO_TO_SLEEP;
	  		  state_timeout = state_timeout + 1;
	  	  }

		  RTCAlarmDelayNoSleep(20);

		  break;
	  }

  }

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

	  /** Initializes the CPU, AHB and APB busses clocks
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
	  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  /** Initializes the CPU, AHB and APB busses clocks
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
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  //hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  // The second signal will be at 1024Hz with AsynchPrediv == 32
  hrtc.Init.AsynchPrediv = 32;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;

  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  // Enable the 'second' interrupt
  HAL_RTCEx_SetSecond_IT(&hrtc);

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.Pin = GPIO_PIN_13;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed =  GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.Pin = GPIO_PIN_3;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed =  GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStructure);


  /*
  GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  // GPIO_MODE_AF_PP - AF means 'Alternate Function', i.e. on-chip peripheral,
  // PWM in this case
  GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed =  GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init (GPIOA, &GPIO_InitStructure);
  */

}

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
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4096;
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
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  //sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);

  TIM2->CCR2 = 0;
  TIM2->CCR1 = 0;

  //TIM_CCxChannelCmd ( TIMx,  Channel,ChannelState)

}

/* USER CODE BEGIN 4 */
uint32_t ProcessAccelVal(int16_t vectelem)
{
	  uint32_t returnval;
	  // Accelerometer value goes from ~ +/- 16384
	  // Offset and saturate (to avoid overflow later)
	  //returnval = abs(vectelem);
	  if(vectelem < 0)
	  {
		  returnval = 0;
	  }
	  else if(vectelem > 16383)
	  {
		  returnval = 16383;
	  }
	  else
	  {
		  returnval = vectelem;
	  }
	  return returnval >> 2;
}



void RTCSecondEventCallback(RTC_HandleTypeDef *hrtc)
{

  if (RTCSecondTdelayCounter != 0x00)
  {
	  RTCSecondTdelayCounter --;
  }
}

// Delay nTime Second interrupt events
void RTCSecDelay(uint32_t nTime)
{
  RTCSecondTdelayCounter = nTime;
  while(RTCSecondTdelayCounter != 0);
}


void RTCAlarmDelayNoSleep(uint32_t nTime)
{
  // Disable Alarm interrupt whilst messing around here
  __HAL_RTC_ALARM_DISABLE_IT(&hrtc, RTC_IT_ALRA);
  //HAL_RTCEx_DeactivateSecond(&hrtc);

  /* Wait until OK to write to RTC regs  */
  while ((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
  }
  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

  /* Set RTC COUNTER MSB word */
  WRITE_REG(hrtc.Instance->ALRH, (nTime >> 16U));
  /* Set RTC COUNTER LSB word */
  WRITE_REG(hrtc.Instance->ALRL, (nTime & RTC_ALRL_RTC_ALR));

  /* Set RTC COUNTER MSB word */
  WRITE_REG(hrtc.Instance->CNTH, (0 >> 16U));
  /* Set RTC COUNTER LSB word */
  WRITE_REG(hrtc.Instance->CNTL, (0 & RTC_CNTL_RTC_CNT));


  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);


  /* Wait till RTC has finished handling the write transfer  */
  while ((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
  }

  // Enable Alarm interrupt
  AlarmITTriggered = 0;
  __HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
  //HAL_RTCEx_SetSecond_IT(&hrtc);

  while(AlarmITTriggered == 0)
  {
  }
}

void RTCAlarmDelayWithStop(uint32_t nTime)
{
  // Disable Alarm interrupt whilst messing around here
  __HAL_RTC_ALARM_DISABLE_IT(&hrtc, RTC_IT_ALRA);
  //HAL_RTCEx_DeactivateSecond(&hrtc);

  /* Wait until OK to write to RTC regs  */
  while ((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
  }
  /* Disable the write protection for RTC registers */
  __HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);

  /* Set RTC COUNTER MSB word */
  WRITE_REG(hrtc.Instance->ALRH, (nTime >> 16U));
  /* Set RTC COUNTER LSB word */
  WRITE_REG(hrtc.Instance->ALRL, (nTime & RTC_ALRL_RTC_ALR));

  /* Set RTC COUNTER MSB word */
  WRITE_REG(hrtc.Instance->CNTH, (0 >> 16U));
  /* Set RTC COUNTER LSB word */
  WRITE_REG(hrtc.Instance->CNTL, (0 & RTC_CNTL_RTC_CNT));

  __HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

  /* Wait till RTC has finished handling the write transfer  */
  while ((hrtc.Instance->CRL & RTC_CRL_RTOFF) == (uint32_t)RESET)
  {
  }

  // Enable Alarm interrupt

  __HAL_RTC_ALARM_ENABLE_IT(&hrtc, RTC_IT_ALRA);
 // Note: To enter Stop mode, all EXTI Line pending bits (in Pending register
 // (EXTI_PR)) and RTC Alarm flag must be reset. Otherwise, the Stop mode entry
 // procedure is ignored and program execution continues.
 // To wake up from the Stop mode with an RTC alarm event, it is necessary to:
 // Configure the EXTI Line 17 to be sensitive to rising edges (Interrupt or Event
 // modes) using the EXTI_Init() function.
 // Enable the RTC Alarm Interrupt using the RTC_ITConfig() function
 // Configure the RTC to generate the RTC alarm using the RTC_SetAlarm()
 // and RTC_AlarmCmd() functions.

  EXTI->EMR = EXTI_EMR_MR17;              // event unmask line 17 (RTC)
  EXTI->RTSR = EXTI_RTSR_TR17;            // rising edge detection


  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
}

void RTCOverflowEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hrtc);

}

void RTCAlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
  AlarmITTriggered = 1;
}

// Enable the overflow interrupt in the RTC
HAL_StatusTypeDef RTC_Set_Overflow_IT(RTC_HandleTypeDef *hrtc)
{
  /* Check input parameters */
  if (hrtc == NULL)
  {
    return HAL_ERROR;
  }

  /* Process Locked */
  __HAL_LOCK(hrtc);

  hrtc->State = HAL_RTC_STATE_BUSY;

  /* Enable overflow interrupt */
  __HAL_RTC_OVERFLOW_ENABLE_IT(hrtc, RTC_IT_SEC);

  hrtc->State = HAL_RTC_STATE_READY;

  /* Process Unlocked */
  __HAL_UNLOCK(hrtc);

  return HAL_OK;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
