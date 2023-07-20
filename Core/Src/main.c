/* USER CODE BEGIN Header */
/**
  ******************************************************************************
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
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
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
uint8_t RecieveBuffer[1];//暂存接收到的字符
uint8_t seep=0;
/* USER CODE END PV */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 if(RecieveBuffer[0]=='+')
		{
     seep=1;
		}
	 if(RecieveBuffer[0]=='-')
		{
     seep=2;
		}
		if(RecieveBuffer[0]=='q')
		{
     seep=3;
		}
		if(RecieveBuffer[0]=='t')
		{
     seep=4;
		}
		if(RecieveBuffer[0]=='z')
		{
     seep=5;
		}
		if(RecieveBuffer[0]=='y')
		{
     seep=6;
		}
		if(RecieveBuffer[0]=='w')
		{
     seep=7;
		}
	HAL_UART_Receive_IT(&huart3,RecieveBuffer,1);
}
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  uint8_t temperature = 1;                     //温度值
	uint8_t humidity = 1; 
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
  MX_TIM3_Init();
  MX_USART3_UART_Init();
	MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  /* USER CODE END 2 */
  OLED_Init();			//初始化OLED  
	OLED_Clear(0); 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int Speed;
	Speed=0;
	HAL_UART_Receive_IT(&huart3,RecieveBuffer,1);
	DHT11_Rst();
	uint32_t ADC_Value[120];
	uint8_t tsd;
	uint32_t ad1,ad2,ad3;
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC_Value,120);
  while (1)
  {
		uint8_t ph,lz,hd;
		ph=0;
		lz=0;
		hd=0;
		HAL_Delay(20);
		for(tsd=0,ad1=0,ad2=0,ad3=0;tsd<120;)
		{
			ad1+=ADC_Value[tsd++];
			ad2+=ADC_Value[tsd++];
			ad3+=ADC_Value[tsd++];
		}
		ad1/=40;
		ad2/=40;
		ad3/=40;
		ph=-5.7541*ad1+16.654;
		lz=-6.946*ad2+65.2;
		hd=-0.0192*ad3+4.1086;
		if(seep==1)
		{
			if(Speed<1000)
			Speed=Speed+100;
			else
				Speed=1000;
			seep=0;
		}
		if(seep==2)
		{
			if(Speed>0)
			Speed=Speed-100;
			else
				Speed=0;
			seep=0;
		}
			if(seep==3)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3, GPIO_PIN_SET);
		}
			if(seep==4)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
		}
			if(seep==5)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2|GPIO_PIN_7, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOI,GPIO_PIN_3, GPIO_PIN_SET);
		}
			if(seep==6)
		{
			HAL_GPIO_WritePin(GPIOI,GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7, GPIO_PIN_SET);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,Speed);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,Speed);
    /* USER CODE END WHILE */
     DHT11_Read_Data(&humidity,&temperature);     //检测出温湿度的值
		OLED_ShowString(0,2,"WENDU:",16);
		OLED_ShowString(0,4,"SHINDU:",16);
		OLED_ShowNum(55,2,humidity,4,16);
		OLED_ShowNum(55,4,temperature,4,16);
		OLED_ShowString(0,0,"Speed:",16);
		OLED_ShowNum(50,0,Speed,4,16);		
		OLED_ShowString(75,6,"PH:",16);
		OLED_ShowNum(90,6,ph,3,16);
		OLED_ShowString(0,6,"LZ:1.3",16);
		OLED_ShowNum(50,0,lz,3,16);
		OLED_ShowString(85,0,"HD:7%",16);
		OLED_ShowNum(90,0,hd,3,16);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 18;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
