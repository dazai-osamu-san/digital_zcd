/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stdint.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 2048
#define NUM_TAPS 311
#define FS 8000
uint16_t adcBuffer[BUFFER_SIZE];

static float32_t inputF32[BUFFER_SIZE];
static float32_t outputF32[BUFFER_SIZE];
static float32_t firStateF32[BUFFER_SIZE + NUM_TAPS - 1];
static float32_t fftOutput[BUFFER_SIZE];
static float32_t fftMag[BUFFER_SIZE / 2]; 

arm_fir_instance_f32 S;
arm_rfft_fast_instance_f32 S_RFFT;

static float32_t firCoeffs32[NUM_TAPS] = {
    -0.000051,
    -0.000041,
    -0.000031,
    -0.000021,
    -0.000011,
    0.000000,
    0.000011,
    0.000023,
    0.000035,
    0.000047,
    0.000061,
    0.000074,
    0.000088,
    0.000103,
    0.000119,
    0.000135,
    0.000152,
    0.000169,
    0.000186,
    0.000205,
    0.000223,
    0.000242,
    0.000262,
    0.000281,
    0.000301,
    0.000320,
    0.000340,
    0.000359,
    0.000378,
    0.000396,
    0.000413,
    0.000429,
    0.000444,
    0.000458,
    0.000470,
    0.000481,
    0.000489,
    0.000496,
    0.000499,
    0.000500,
    0.000499,
    0.000494,
    0.000486,
    0.000474,
    0.000458,
    0.000439,
    0.000415,
    0.000388,
    0.000355,
    0.000319,
    0.000277,
    0.000231,
    0.000181,
    0.000125,
    0.000065,
    -0.000000,
    -0.000070,
    -0.000144,
    -0.000223,
    -0.000306,
    -0.000393,
    -0.000484,
    -0.000579,
    -0.000677,
    -0.000778,
    -0.000881,
    -0.000987,
    -0.001095,
    -0.001203,
    -0.001313,
    -0.001422,
    -0.001532,
    -0.001640,
    -0.001746,
    -0.001851,
    -0.001952,
    -0.002049,
    -0.002142,
    -0.002230,
    -0.002312,
    -0.002387,
    -0.002454,
    -0.002513,
    -0.002563,
    -0.002603,
    -0.002632,
    -0.002650,
    -0.002655,
    -0.002648,
    -0.002627,
    -0.002592,
    -0.002541,
    -0.002475,
    -0.002393,
    -0.002294,
    -0.002178,
    -0.002045,
    -0.001893,
    -0.001722,
    -0.001534,
    -0.001326,
    -0.001099,
    -0.000853,
    -0.000588,
    -0.000303,
    0.000000,
    0.000322,
    0.000663,
    0.001022,
    0.001399,
    0.001793,
    0.002204,
    0.002631,
    0.003073,
    0.003530,
    0.004001,
    0.004485,
    0.004981,
    0.005488,
    0.006004,
    0.006530,
    0.007063,
    0.007602,
    0.008146,
    0.008693,
    0.009243,
    0.009794,
    0.010344,
    0.010892,
    0.011436,
    0.011976,
    0.012509,
    0.013034,
    0.013550,
    0.014054,
    0.014547,
    0.015026,
    0.015489,
    0.015937,
    0.016366,
    0.016776,
    0.017167,
    0.017535,
    0.017881,
    0.018204,
    0.018501,
    0.018774,
    0.019020,
    0.019238,
    0.019429,
    0.019592,
    0.019726,
    0.019830,
    0.019905,
    0.019950,
    0.019965,
    0.019950,
    0.019905,
    0.019830,
    0.019726,
    0.019592,
    0.019429,
    0.019238,
    0.019020,
    0.018774,
    0.018501,
    0.018204,
    0.017881,
    0.017535,
    0.017167,
    0.016776,
    0.016366,
    0.015937,
    0.015489,
    0.015026,
    0.014547,
    0.014054,
    0.013550,
    0.013034,
    0.012509,
    0.011976,
    0.011436,
    0.010892,
    0.010344,
    0.009794,
    0.009243,
    0.008693,
    0.008146,
    0.007602,
    0.007063,
    0.006530,
    0.006004,
    0.005488,
    0.004981,
    0.004485,
    0.004001,
    0.003530,
    0.003073,
    0.002631,
    0.002204,
    0.001793,
    0.001399,
    0.001022,
    0.000663,
    0.000322,
    0.000000,
    -0.000303,
    -0.000588,
    -0.000853,
    -0.001099,
    -0.001326,
    -0.001534,
    -0.001722,
    -0.001893,
    -0.002045,
    -0.002178,
    -0.002294,
    -0.002393,
    -0.002475,
    -0.002541,
    -0.002592,
    -0.002627,
    -0.002648,
    -0.002655,
    -0.002650,
    -0.002632,
    -0.002603,
    -0.002563,
    -0.002513,
    -0.002454,
    -0.002387,
    -0.002312,
    -0.002230,
    -0.002142,
    -0.002049,
    -0.001952,
    -0.001851,
    -0.001746,
    -0.001640,
    -0.001532,
    -0.001422,
    -0.001313,
    -0.001203,
    -0.001095,
    -0.000987,
    -0.000881,
    -0.000778,
    -0.000677,
    -0.000579,
    -0.000484,
    -0.000393,
    -0.000306,
    -0.000223,
    -0.000144,
    -0.000070,
    -0.000000,
    0.000065,
    0.000125,
    0.000181,
    0.000231,
    0.000277,
    0.000319,
    0.000355,
    0.000388,
    0.000415,
    0.000439,
    0.000458,
    0.000474,
    0.000486,
    0.000494,
    0.000499,
    0.000500,
    0.000499,
    0.000496,
    0.000489,
    0.000481,
    0.000470,
    0.000458,
    0.000444,
    0.000429,
    0.000413,
    0.000396,
    0.000378,
    0.000359,
    0.000340,
    0.000320,
    0.000301,
    0.000281,
    0.000262,
    0.000242,
    0.000223,
    0.000205,
    0.000186,
    0.000169,
    0.000152,
    0.000135,
    0.000119,
    0.000103,
    0.000088,
    0.000074,
    0.000061,
    0.000047,
    0.000035,
    0.000023,
    0.000011,
    0.000000,
    -0.000011,
    -0.000021,
    -0.000031,
    -0.000041,
    -0.000051
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void processData(void){
    // Convert ADC data to float32_t
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        inputF32[i] = (float32_t)adcBuffer[i];
    }

    // Apply FIR filter
    arm_fir_f32(&S, inputF32, outputF32, BUFFER_SIZE);

    // Remove DC bias
    float32_t meanValue;
    arm_mean_f32(outputF32, BUFFER_SIZE, &meanValue);
    for (uint32_t i = 0; i < BUFFER_SIZE; i++)
    {
        outputF32[i] -= meanValue;
    }

    // ----------------------------
    // Zero-Crossing Detection Code
    // ----------------------------

    float32_t zeroCrossingTimes[BUFFER_SIZE]; // Store interpolated zero-crossing times
    uint32_t zeroCrossingCount = 0;
    char msg[100];

    // Detect zero crossings with linear interpolation
    for (uint32_t i = 1; i < BUFFER_SIZE; i++)
    {
        float32_t previousSample = outputF32[i - 1];
        float32_t currentSample = outputF32[i];

        // Check for zero crossing (sign change)
        if ((previousSample >= 0.0f && currentSample < 0.0f) ||
            (previousSample < 0.0f && currentSample >= 0.0f))
        {
            // Linear interpolation to estimate zero-crossing time
            float32_t delta = 0.0f;
            float32_t slope = currentSample - previousSample;
            if (slope != 0.0f)
            {
                delta = -previousSample / slope;
            }
            // If slope is zero, delta remains 0

            float32_t zeroCrossingTime = (i - 1) + delta; // Fractional index
            zeroCrossingTimes[zeroCrossingCount++] = zeroCrossingTime;
        }
    }

    // Ensure we have detected enough zero crossings
    if (zeroCrossingCount < 3) // Need at least 3 zero crossings to calculate one full period
    {
        snprintf(msg, sizeof(msg), "Insufficient zero crossings detected.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Calculate periods between zero crossings (full periods)
    float32_t periods[BUFFER_SIZE / 2];
    uint32_t periodCount = 0;

    for (uint32_t i = 2; i < zeroCrossingCount; i += 2)
    {
        float32_t periodSamples = zeroCrossingTimes[i] - zeroCrossingTimes[i - 2];
        periods[periodCount++] = periodSamples;
    }

    // Ensure we have at least one period
    if (periodCount == 0)
    {
        snprintf(msg, sizeof(msg), "No complete periods detected.\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return;
    }

    // Compute average period
    float32_t totalPeriod = 0.0f;
    for (uint32_t i = 0; i < periodCount; i++)
    {
        totalPeriod += periods[i];
    }
    float32_t averagePeriodSamples = totalPeriod / (float32_t)periodCount;

    // Calculate frequency from zero crossings
    float32_t zeroCrossingFrequency = (float32_t)FS / averagePeriodSamples;

    // -------------------------------------------
    // Circular Buffer to Average Last 4 Frequency Samples
    // -------------------------------------------

    #define FREQ_BUFFER_SIZE 4
    static float32_t frequencyBuffer[FREQ_BUFFER_SIZE] = {0};
    static uint32_t frequencyIndex = 0;

    // Update frequency buffer (circular buffer)
    frequencyBuffer[frequencyIndex] = zeroCrossingFrequency;
    frequencyIndex = (frequencyIndex + 1) % FREQ_BUFFER_SIZE; // Wrap around after reaching buffer size

    // Compute mean frequency using CMSIS-DSP function
    float32_t meanFrequency = 0.0f;
    arm_mean_f32(frequencyBuffer, FREQ_BUFFER_SIZE, &meanFrequency);

    // Send mean frequency over UART
    snprintf(msg, sizeof(msg), "Mean Frequency: %.2f Hz\r\n", meanFrequency);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

    // Perform FFT
    arm_rfft_fast_f32(&S_RFFT, outputF32, fftOutput, 0);

    // Compute magnitude of FFT
    arm_cmplx_mag_f32(fftOutput, fftMag, BUFFER_SIZE / 2);

    // Find the peak frequency
    float32_t maxValue;
    uint32_t maxIndex;
    arm_max_f32(fftMag, BUFFER_SIZE / 2, &maxValue, &maxIndex);

    // Calculate frequency resolution
    float32_t frequencyResolution = (float32_t)FS / (float32_t)BUFFER_SIZE;

    // Calculate peak frequency
    float32_t peakFrequency = frequencyResolution * (float32_t)maxIndex;
    // Send peak frequency over UART
    
    snprintf(msg, sizeof(msg), "Peak Frequency: %.2f Hz\r\n", peakFrequency);
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
} 

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  processData();
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  arm_fir_init_f32(&S, NUM_TAPS, firCoeffs32, firStateF32, BUFFER_SIZE);
  arm_rfft_fast_init_f32(&S_RFFT, BUFFER_SIZE);
  HAL_TIM_Base_Start(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, BUFFER_SIZE);
  /* USER CODE END 2 */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 20-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 525-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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