/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t u32APB1TimerClock;
  uint32_t u32APB2TimerClock;
} sTimerClocks_t;
typedef sTimerClocks_t* psTimerClocks_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUF_LEN (4u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t au16Val1[3u][BUF_LEN];
uint16_t au16Val2[3u][BUF_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void vCalcTimerClocks(psTimerClocks_t const psTimerClocks);
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  sTimerClocks_t sTimerClocks = {0u};
  vCalcTimerClocks(&sTimerClocks);

  //ADC
  // Generate ADC Interrupt for Injected channels
  hadc1.Instance->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_EOCIE;
  hadc2.Instance->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_EOCIE;
  hadc3.Instance->CR1 |= ADC_CR1_JEOCIE | ADC_CR1_EOCIE;

  // Power on ADCs
  hadc1.Instance->CR2 |= ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS;
  hadc2.Instance->CR2 |= ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS;
  hadc3.Instance->CR2 |= ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS;

  hadc1.DMA_Handle->Instance->CR &= ~(DMA_SxCR_EN);
  while(hadc1.DMA_Handle->Instance->CR & DMA_SxCR_EN);

  DMA2->LIFCR = 0x0F7D0F7D;
  DMA2->HIFCR = 0x0F7D0F7D;

  hadc1.DMA_Handle->Instance->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  hadc1.DMA_Handle->Instance->CR |= (DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_DBM);
  hadc1.DMA_Handle->Instance->M0AR = (uint32_t)(&(au16Val1[0u][0u]));
  hadc1.DMA_Handle->Instance->M1AR = (uint32_t)(&(au16Val2[0u][0u]));
  hadc1.DMA_Handle->Instance->NDTR = BUF_LEN;
  hadc1.DMA_Handle->Instance->PAR = (uint32_t)(&hadc1.Instance->DR);

  hadc1.DMA_Handle->Instance->CR |= (DMA_SxCR_EN);

  hadc2.DMA_Handle->Instance->CR &= ~(DMA_SxCR_EN);
  while(hadc2.DMA_Handle->Instance->CR & DMA_SxCR_EN);

  DMA2->LIFCR = 0x0F7D0F7D;
  DMA2->HIFCR = 0x0F7D0F7D;

  hadc2.DMA_Handle->Instance->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  hadc2.DMA_Handle->Instance->CR |= (DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_DBM);
  hadc2.DMA_Handle->Instance->M0AR = (uint32_t)(&(au16Val1[1u][0u]));
  hadc2.DMA_Handle->Instance->M1AR = (uint32_t)(&(au16Val2[1u][0u]));
  hadc2.DMA_Handle->Instance->NDTR = BUF_LEN;
  hadc2.DMA_Handle->Instance->PAR = (uint32_t)(&hadc2.Instance->DR);

  hadc2.DMA_Handle->Instance->CR |= (DMA_SxCR_EN);

  hadc3.DMA_Handle->Instance->CR &= ~(DMA_SxCR_EN);
  while(hadc3.DMA_Handle->Instance->CR & DMA_SxCR_EN);

  DMA2->LIFCR = 0x0F7D0F7D;
  DMA2->HIFCR = 0x0F7D0F7D;

  hadc3.DMA_Handle->Instance->CR &= ~(DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE);
  hadc3.DMA_Handle->Instance->CR |= (DMA_SxCR_TEIE | DMA_SxCR_DMEIE | DMA_SxCR_DBM);
  hadc3.DMA_Handle->Instance->M0AR = (uint32_t)(&(au16Val1[2u][0u]));
  hadc3.DMA_Handle->Instance->M1AR = (uint32_t)(&(au16Val2[2u][0u]));
  hadc3.DMA_Handle->Instance->NDTR = BUF_LEN;
  hadc3.DMA_Handle->Instance->PAR = (uint32_t)(&hadc3.Instance->DR);

  hadc3.DMA_Handle->Instance->CR |= (DMA_SxCR_EN);

  htim6.Instance->DIER |= TIM_DIER_UIE;
  htim6.Instance->PSC = 10000u-1u;
  htim6.Instance->ARR = (uint32_t)(1u * sTimerClocks.u32APB1TimerClock / (htim6.Instance->PSC + 1u) - 1u);
  htim6.Instance->CR1 |= TIM_CR1_CEN;



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void vCalcTimerClocks(psTimerClocks_t const psTimerClocks)
{
    uint32_t u32AHBPrescaler = 0u;
    uint32_t u32APB1Prescaler = 0u;
    uint32_t u32APB2Prescaler = 0u;

    switch(RCC->CFGR & RCC_CFGR_HPRE)
    {
      case RCC_CFGR_HPRE_DIV2: u32AHBPrescaler = 2u; break;
      case RCC_CFGR_HPRE_DIV4: u32AHBPrescaler = 4u; break;
      case RCC_CFGR_HPRE_DIV16: u32AHBPrescaler = 16u; break;
      case RCC_CFGR_HPRE_DIV64: u32AHBPrescaler = 64u; break;
      case RCC_CFGR_HPRE_DIV128: u32AHBPrescaler = 128u; break;
      case RCC_CFGR_HPRE_DIV256: u32AHBPrescaler = 256u; break;
      case RCC_CFGR_HPRE_DIV512: u32AHBPrescaler = 512u; break;
      default: u32AHBPrescaler = 1u; break;
    }

    switch(RCC->CFGR & RCC_CFGR_PPRE1)
    {
      case RCC_CFGR_PPRE1_DIV2: u32APB1Prescaler = 2u; break;
      case RCC_CFGR_PPRE1_DIV4: u32APB1Prescaler = 4u; break;
      case RCC_CFGR_PPRE1_DIV8: u32APB1Prescaler = 8u; break;
      case RCC_CFGR_PPRE1_DIV16: u32APB1Prescaler = 16u; break;
      default: u32APB1Prescaler = 1u; break;
    }

    switch(RCC->CFGR & RCC_CFGR_PPRE1)
    {
      case RCC_CFGR_PPRE2_DIV2: u32APB2Prescaler = 2u; break;
      case RCC_CFGR_PPRE2_DIV4: u32APB2Prescaler = 4u; break;
      case RCC_CFGR_PPRE2_DIV8: u32APB2Prescaler = 8u; break;
      case RCC_CFGR_PPRE2_DIV16: u32APB2Prescaler = 16u; break;
      default: u32APB2Prescaler = 1u; break;
    }

    if (RCC->DCKCFGR1 & RCC_DCKCFGR1_TIMPRE)
    {
      if ((u32APB1Prescaler == 1u) || (u32APB1Prescaler == 2u) || (u32APB1Prescaler == 4u))
      {
	psTimerClocks->u32APB1TimerClock = SystemCoreClock / u32AHBPrescaler;
      }
      else
      {
	psTimerClocks->u32APB1TimerClock = SystemCoreClock * 4u / u32AHBPrescaler / u32APB1Prescaler;
      }

      if ((u32APB2Prescaler == 1u) || (u32APB2Prescaler == 2u) || (u32APB2Prescaler == 4u))
      {
	psTimerClocks->u32APB2TimerClock = SystemCoreClock / u32AHBPrescaler;
      }
      else
      {
	psTimerClocks->u32APB2TimerClock = SystemCoreClock * 4u / u32AHBPrescaler / u32APB2Prescaler;
      }
    }
    else
    {
      if (u32APB1Prescaler == 1u)
      {
	psTimerClocks->u32APB1TimerClock = SystemCoreClock / u32AHBPrescaler / u32APB1Prescaler;
      }
      else
      {
	psTimerClocks->u32APB1TimerClock = SystemCoreClock * 2u / u32AHBPrescaler / u32APB1Prescaler;
      }

      if (u32APB2Prescaler == 1u)
      {
	psTimerClocks->u32APB2TimerClock = SystemCoreClock / u32AHBPrescaler / u32APB2Prescaler;
      }
      else
      {
	psTimerClocks->u32APB2TimerClock = SystemCoreClock * 2u / u32AHBPrescaler / u32APB2Prescaler;
      }
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
  GPIOB->BSRR = GPIO_PIN_14;
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

