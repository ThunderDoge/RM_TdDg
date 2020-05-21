/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "stdio.h"	// stdio.h may cause CAN RX error
#include "stdio.h"
#include "task_SentiCloud.hpp"
#include <string.h>
#include "app_AmmoFeed.hpp"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __MAIN_DEBUG
uint8_t buf[100];
uint8_t s[] = "uart-test";

#ifdef __VISION_TEST

int v_case;

uint8_t str_normal[] = {0xff,0x13,0x9a,0x99,0x99,0x3f,0x9a,0x99,0x59,0x40,0x1,0x98,0x0,0x0,0x0,0x0,0x8f,0xd};
uint8_t s1[] = {0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x0 ,0x0 ,0x0 ,0x0 ,0xf1 ,0xd};
uint8_t trush[] = {0x13, 0x9a ,0x99,0x99, 0x3f};
uint8_t s2[] = {0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfb ,0x0 ,0x0 ,0x0 ,0x0 ,0xf2 ,0xd };
	
uint8_t s3_a[] = {0x13, 0x9a ,0x99,0x99, 0x3f,0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x0};
uint8_t s3_b_s[] = {0x0 ,0x0 ,0x0 ,0xf1 ,0xd,0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfb ,0x0 ,0x0 ,0x0 ,0x0 ,0xf2 ,0xd };

uint8_t trush4[]={0x54 ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x20 ,0xa0 ,0x0 ,0x0 ,0x41 ,0x6d};
uint8_t s4_a[]={0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59 ,0x40,0x1 ,0xfa ,0x0};
uint8_t s4_b[]={0x0 ,0x0 ,0x0 ,0xf1 ,0xd};	

uint8_t s5[] = {0xff ,0x13 ,0x9a ,0x99 ,0x99 ,0x3f ,0x9a ,0x99 ,0x59,0x40,0x1,0xfa,0x0,0xff,
0x13,0x0,0xf1,0xd,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0xa,0xb,0x5f,0xd};

void v_buf_insert(uint8_t* pdata, int size)
{
    memcpy(Vision_Rxbuffer+not_analysed_index,  pdata, size);
    not_analysed_index += size;
    if(not_analysed_index>120)
        not_analysed_index = 120;
}
void v_buf_clr()
{
    not_analysed_index = 0;
    memset(Vision_Rxbuffer,0,sizeof(Vision_Rxbuffer));
}

#endif // __VISION_TEST

#endif // __MAIN_DEBUG



//#ifdef __stdio_h

//#ifdef __GNUC__
//  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif

//PUTCHAR_PROTOTYPE
//{
//	if(HAL_UART_Transmit(&huart5,(uint8_t*)&ch,1,0xff) == HAL_OK)
//		return ch;
//	else
//		return EOF;
//}

//#undef PUTCHAR_PROTOTYPE

//#endif
extern Motor_t DJI_2006;
pid FeedSpeed(20, 0, 1, 1000, 7000);
pid FeedPositon(0.5, 0.01, 0, 1000, 20000, 0, 200);
AmmoFeed ttt(1,0x201,&DJI_2006,7, -1, &FeedSpeed, &FeedPositon);

int k;

int32_t ttt_free_spd=-3000;
int16_t ttt_d_time=100;
int16_t ttt_trig;

extern float p,y;

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
  MX_SPI1_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */
  
  Cloud_Init();	// Sentinal Cloud Hardware Init 硬件初始�??
  
	#ifdef __MAIN_DEBUG
	HAL_UART_Receive_IT(&huart5,buf,50);
	#endif
	#ifndef __MAIN_DEBUG
	
	//TaskStarter();	// FreeRTOS 任务启动
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init(); 

  /* Start scheduler */
//  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  #endif
  
  ttt.Enable_Block(5000,200,5);
  
  while (1)
  {
	  CloudEntity.ShooterSwitchCmd(0);
	  CloudEntity.SetAngleTo(p,y);
	  CloudEntity.PitchSecondMotor.cooperative = 1;
	  	  switch(k)
	  {
		  case 0:
			  ttt.Safe_Set();
				CloudEntity.ShooterSwitchCmd(0);
		  break;
		  case 1:
				CloudEntity.ShooterSwitchCmd(0);
		  break;
		  case 2:
			  ttt.Free_Once_Set(ttt_d_time,ttt_trig);
		  break;
		  case 3:
			  ttt.Burst_Set(3,ttt_d_time,ttt_trig);
		  break;
		  case 4:
			  CloudEntity.ShooterSwitchCmd(1);
				
		  break;
		  case 5:
			  CloudEntity.ShooterSwitchCmd(1);
			  ttt.Free_Once_Set(ttt_d_time,ttt_1trig);
				
		  break;
	  }
	  manager::CANSend();
	  HAL_Delay(2);

//		HAL_UART_Transmit_IT(&huart3,s,sizeof(s));
//	  HAL_UART_Transmit_IT(&huart3,(uint8_t*)"test\r\n",sizeof("test\r\n"));
//	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_12);
//	  HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_11);
#ifdef __VISION_TEST
	  switch(v_case)
	  {
		  case 0:
            v_buf_insert(str_normal,sizeof(str_normal));
            app_vision_analysis_intgrated();

            v_buf_clr();
            break;
		  case 1:
            v_buf_insert(s1, sizeof(s1));
            v_buf_insert(trush, sizeof(trush));
            app_vision_analysis_intgrated();

            v_buf_insert(s2, sizeof(s2));
            app_vision_analysis_intgrated();
            v_buf_clr();
            break;
		  case 2:
            v_buf_insert(s3_a, sizeof(s3_a));
            app_vision_analysis_intgrated();
            v_buf_insert(s3_b_s, sizeof(s3_b_s));
            app_vision_analysis_intgrated();
            v_buf_clr();
			  break;
		  case 3:
            v_buf_insert(trush4, sizeof(trush4));
            v_buf_insert(trush4, sizeof(trush4));
            v_buf_insert(s4_a, sizeof(s4_a));
            app_vision_analysis_intgrated();

            v_buf_insert(s4_b, sizeof(s4_b));
            app_vision_analysis_intgrated();

            v_buf_clr();
			  break;
		  case 4:
          v_buf_insert(s5, sizeof(s5));
		  app_vision_analysis_intgrated();
          v_buf_clr();
			  break;
	  }
      

#endif // __VISION_TEST
	  
	  //HAL_Delay(100);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
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
	while(1U)
	{
		HAL_Delay(1);	// Error waiting debug.
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	printf("Wrong parameters value: file %s on line %d\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
