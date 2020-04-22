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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
  /** 
   * @brief    main of SentryCloud
   * @details  
   * @author   ThunderDoge
   * @date      2020-2-18
   * @version  0.1
   * @par Copyright (c):  OnePointFive, the UESTC RoboMaster Team. 2019~2020
   */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task_SentiCloud.hpp"  // the only depend file.
#include "SEGGER_SYSVIEW.h"		// SEGGER SystemView Support
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define __MAIN_DEBUG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __MAIN_DEBUG

#include "app_vision.hpp"

int ans=0;

uint8_t data_normal[] = {0xff ,0x13 ,0x9a, 0x99, 0x99 ,0x3f, 0x9a, 0x99 ,0x59, 0x40 ,0x01, 0x02 ,0x00, 0x00, 0x00, 0x00 ,0xf9, 0x0d};
uint8_t data_trush[] = 
{0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40 ,0x1 ,0xfa ,0x0 ,0x0 ,0x0 ,0x0 ,0xf1 ,0xd,			// ÏÂÃæÊÂÀ¬»ø
0x13, 0x9a ,0x99,0x99, 0x3f,0x13, 0x9a ,0x99,0x99, 0x3f,
// 
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfb ,0x0 ,0x0 ,0x0 ,0x0 ,0xf2 ,0xd};
uint8_t data_trush_forward[] = {0x13, 0x9a ,0x99,0x99, 0x3f,
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40
,0x1 ,0xfa ,0x0};
uint8_t data_half[] = {0x54 ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40
,0x1 ,0xfa ,0x20 ,0xa0 ,0x0 ,0x0 ,0x41 ,0x6d,0x54 ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x20 ,0xa0 ,0x0 ,0x0 ,0x41 ,0x6d,
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x0};
uint8_t data_after_forw[] = {0x0 ,0x0 ,0x0 ,0xf1 ,0xd,
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfb ,0x0 ,0x0 ,0x0 ,0x0 ,0xf2 ,0xd };

uint8_t data5_trush_head_tail[] = {0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,
0x1 ,0xfa ,0x0 , 0xff ,0x13 ,0x0 ,0xf1 ,0xd,
0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0xa,0xb,0x5f,0xd};

#define ADD_ARRAY_TO_BUF(BufX)    \
{ 	\
	memcpy(Vision_Rxbuffer + not_analysed_index , BufX,sizeof(BufX) ); \
	not_analysed_index+=sizeof(BufX); \
}



void vision_test()
{
	uint8_t data_normal[] = {0xff ,0x13 ,0x9a, 0x99, 0x99 ,0x3f, 0x9a, 0x99 ,0x59, 0x40 ,0x01, 0x02 ,0x00, 0x00, 0x00, 0x00 ,0xf9, 0x0d};
uint8_t data_trush[] = 
{0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40 ,0x1 ,0xfa ,0x0 ,0x0 ,0x0 ,0x0 ,0xf1 ,0xd,			// ÏÂÃæÊÂÀ¬»ø
0x13, 0x9a ,0x99,0x99, 0x3f,0x13, 0x9a ,0x99,0x99, 0x3f,
// 
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfb ,0x0 ,0x0 ,0x0 ,0x0 ,0xf2 ,0xd};
uint8_t data_trush_forward[] = {0x13, 0x9a ,0x99,0x99, 0x3f,
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40
,0x1 ,0xfa ,0x0};
uint8_t data_half[] = {0x54 ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40
,0x1 ,0xfa ,0x20 ,0xa0 ,0x0 ,0x0 ,0x41 ,0x6d,0x54 ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x20 ,0xa0 ,0x0 ,0x0 ,0x41 ,0x6d,
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x0};
uint8_t data_after_forw[] = {0x0 ,0x0 ,0x0 ,0xf1 ,0xd,
0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfb ,0x0 ,0x0 ,0x0 ,0x0 ,0xf2 ,0xd };

uint8_t data5_trush_head_tail[] = {0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,
0x1 ,0xfa ,0x0 , 0xff ,0x13 ,0x0 ,0xf1 ,0xd,
0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0xa,0xb,0x5f,0xd};

	ADD_ARRAY_TO_BUF(data5_trush_head_tail);
	app_vision_analysis_intgrated();
	
	HAL_Delay(200);
	
//	ADD_ARRAY_TO_BUF(data_after_forw);
//	app_vision_analysis_intgrated();
	
}

#endif // __MAIN_DEBUG



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
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
    //TaskStarter() called in MX_FREERTOS_Init();
	Cloud_Init();	// åœ¨æ“ä½œç³»ç»Ÿåˆå§‹åŒ–ä¹‹å‰ ç¡¬ä»¶åˆå§‹åŒ–
  SEGGER_SYSVIEW_Conf();            /* Configure and initialize SystemView  */
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  #ifdef __MAIN_DEBUG
  vision_test();
  #endif // __MAIN_DEBUG
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
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
