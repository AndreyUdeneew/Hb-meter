/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
uint32_t leds[3];
uint32_t ledActive;
uint16_t dat_660_discr_1,dat_660_discr_2,dat_880_discr_1,dat_880_discr_2,dat_940_discr_1,dat_940_discr_2;
float data_660nm_1,data_660nm_2,data_880nm_1,data_880nm_2,data_940nm_1,data_940nm_2,delta1,delta2,delta3;
float OD_660nm,OD_880nm,OD_940nm,Ua_660nm,Ua_880nm,Ua_940nm,CHb,CHbO2,CH2O;
float det;

// 	Extinction matrix
float E_hb_660=0.8; float E_HbO2_660=0.08; float E_H2O_660=0.001;
float E_hb_880=0.2; float E_HbO2_880=0.3;  float E_H2O_880=0.05;
float E_hb_940=0.2; float E_HbO2_940=0.3;  float E_H2O_940=0.08;

//inverted extinction matrix
float E_Hb_660_=119.048; float E_HbO2_660_=29.762; float E_H2O_660_=29.762;
float E_Hb_880_=11.905; float E_HbO2_880_=44.643;  float E_H2O_880_=44.643;
float E_Hb_940_=0.149; float E_HbO2_940_=7.441;  float E_H2O_940_=11.905;

uint8_t i=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void timerUpCallBack(void);
void timerEventHandler(uint8_t i);
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
	leds[0]=0x2000;
	leds[1]=0x4000;
	leds[2]=0x8000;
	i=0;
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1);
  ADC1->CR2 |= ADC_CR2_CAL; // запуск калибровки
  while ((ADC1->CR2 & ADC_CR2_CAL) != 0) ; // ожидание окончания калибровки
//  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  	GPIOC->ODR|=1<<13;
//	  	HAL_Delay(500);
//	  	GPIOC->ODR&=~(1<<13);
//	  	HAL_Delay(500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(GPIOC, OUT_1_Pin|OUT_2_Pin|OUT_3_Pin);

  /**/
  GPIO_InitStruct.Pin = OUT_1_Pin|OUT_2_Pin|OUT_3_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void timerUpCallBack(void){
i+=1;
if(i>2){
	i=0;
}
timerEventHandler(i);

}
void timerEventHandler(uint8_t i){
	GPIOC->ODR=leds[i];
	ledActive=leds[i];
//	if(i==0){				//	660 nm LED
//	 HAL_ADC_Start(&hadc1);
//     HAL_ADC_PollForConversion(&hadc1,100);
//     dat_660_discr_1=(uint16_t)HAL_ADC_GetValue(&hadc1);
//     data_660nm_1 = ((float)dat_660_discr_1)*3/4096;
//	 HAL_ADC_Start(&hadc1);
//     HAL_ADC_PollForConversion(&hadc1,100);
//
//     dat_660_discr_2=(uint16_t)HAL_ADC_GetValue(&hadc1);
//     data_660nm_2 = ((float)dat_660_discr_2)*3/4096;

//	 ADC1->CR2 &= ~ADC_CR2_ADON; // запретить АЦП
//	 ADC1->SQR1 =0; // 1 регулярный канал
//	 ADC1->SQR3 =0; // 1 преобразование - канал 0
//	 ADC1->CR2 &= ~ADC_CR2_CONT; // запрет непрерывного режима
//	 ADC1->CR1 &= ~ADC_CR1_SCAN; // запрет режима сканирования
//	 ADC1->CR2 |= ADC_CR2_ADON; // разрешить АЦП
//	 ADC1->CR2 |= ADC_CR2_SWSTART; // запуск АЦП
////	   while(!(ADC1->SR & ADC_SR_EOC)){} ; // ожидание завершения преобразования
//	 HAL_Delay(1);
//     dat_660_discr_1=ADC1->DR * 3 / 4096. ; // пересчет в напряжение

//     ADC1->CR2 |= ADC_CR2_JSWSTART; // запуск АЦП
//     while(!(ADC1->SR & ADC_SR_EOC)) ; // ожидание завершения преобразования
//     dat_660_discr_2=ADC1->JDR1 * 3 / 4096. ; // пересчет в напряжение
//
//
//     if(data_660nm_1>=data_660nm_2){
////     delta1=dat1-dat2;
//      OD_660nm=(float)logf(data_660nm_2/data_660nm_1);
//     }
//     else{
////    	 delta1=dat2-dat1;
//    	 OD_660nm=(float)logf(data_660nm_1/data_660nm_2);
//     }
//     Ua_660nm=OD_660nm/(-0.508);
//}
//	if(i==1){			//	880 nm LED
//	 HAL_ADC_Start(&hadc1);
//     HAL_ADC_PollForConversion(&hadc1,100);
//     dat_880_discr_1=(uint16_t)HAL_ADC_GetValue(&hadc1);
//     data_880nm_1 = ((float)dat_880_discr_1)*3/4096;
//
//	 HAL_ADC_Start(&hadc1);
//     HAL_ADC_PollForConversion(&hadc1,100);
//     dat_880_discr_2=(uint16_t)HAL_ADC_GetValue(&hadc1);
//     data_880nm_2 = ((float)dat_880_discr_2)*3/4096;
//
//     if(data_880nm_1>=data_880nm_2){
////          delta2=dat3-dat4;
//           OD_880nm=(float)logf(data_880nm_2/data_880nm_1);
//          }
//          else{
////         	 delta2=dat4-dat3;
//         	 OD_880nm=(float)logf(data_880nm_1/data_880nm_2);
//          }
//     Ua_880nm=OD_880nm/(-0.508);
//}
//	if(i==2){			//	940 nm LED
//	 HAL_ADC_Start(&hadc1);
//     HAL_ADC_PollForConversion(&hadc1,100);
//     dat_940_discr_1=(uint16_t)HAL_ADC_GetValue(&hadc1);
//     data_940nm_1 = ((float)dat_940_discr_1)*3/4096;
//
//	 HAL_ADC_Start(&hadc1);
//     HAL_ADC_PollForConversion(&hadc1,100);
//     dat_940_discr_2=(uint16_t)HAL_ADC_GetValue(&hadc1);
//     data_940nm_2 = ((float)dat_940_discr_2)*3/4096;
//
//     if(data_940nm_1>=data_940nm_2){
////          delta3=dat5-dat6;
//          OD_940nm=(float)logf(data_940nm_2/data_940nm_1);
//          }
//          else{
////         	 delta3=dat6-dat5;
//         	OD_940nm=(float)logf(data_940nm_1/data_940nm_2);
//          }
//     Ua_940nm=OD_940nm/(-0.508);
//
//     CHb=(E_Hb_660_*Ua_660nm+E_Hb_880_*Ua_880nm+E_Hb_940_*Ua_940nm)*67000;
//	 CHbO2=(E_HbO2_660_*Ua_660nm+E_HbO2_880_*Ua_880nm+E_HbO2_940_*Ua_940nm)*67000;
//	 CH2O=(E_H2O_660_*Ua_660nm+E_H2O_880_*Ua_880nm+E_H2O_940_*Ua_940nm)*18;
//}
	// here we will cound Hb concentrations

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
//	  HAL_GPIO_TogglePin(LED13_GPIO_Port,LED13_Pin);
	    HAL_Delay(250);
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
