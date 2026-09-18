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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_Servo.h"
#include "Printf_DMA.H"
#include "BSP_433.H"
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

///* 左侧十字按键。 */
//#define CMD_Cross_Up       (0x5751U)
//#define CMD_Cross_Down     (0x585AU)
//#define CMD_Cross_LEFT     (0x5341U)
//#define CMD_Cross_RIGHT    (0x584DU)

///* 右侧纵向排列的两个按键。 */
//#define CMD_Forward        (0x4F50U)
//#define CMD_Back           (0x4B4CU)

///* 下方横向排列的两个按键。 */
//#define CMD_One            (0x4342U)
//#define CMD_Two            (0x4944U)

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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	BSP_Servo_Init();
	Set_uart_433_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//		Red_uart_433();
		
//		Witch_uart_433();
			
//		Set_uart_433();
		
//		BSP_Chassis_Drive(0,-9000);
		
//		BSP_Servo_SetFrontRightWheelSpeed(0);
//		BSP_Servo_SetFrontLeftWheelSpeed(0);
//		BSP_Servo_SetRearLeftWheelSpeed(0);
//		BSP_Servo_SetRearRightWheelSpeed(0);
		
//		if(BSP_433_GetKeyState(CMD_Cross_LEFT)){
//			BSP_Chassis_Drive(0,-5000);
//		}else if(BSP_433_GetKeyState(CMD_Cross_RIGHT)){
//			BSP_Chassis_Drive(0,5000);
//		}else if(CMD_Forward){
//			BSP_Chassis_Drive(8000,0);
//		}else if(CMD_Back){
//			BSP_Chassis_Drive(-8000,0);
//		}else{
//			BSP_Chassis_Drive(0,0);
//		}

		if(BSP_433_GetKeyState(CMD_Cross_LEFT)){
			BSP_Chassis_Drive(3000,0);
		}else if(BSP_433_GetKeyState(CMD_Cross_RIGHT)){
			BSP_Chassis_Drive(-3000,0);
		}else	if(BSP_433_GetKeyState(CMD_Forward)){
			BSP_Chassis_Drive(0,-7500);
		}else if(BSP_433_GetKeyState(CMD_Back)){
			BSP_Chassis_Drive(0,7500);
		}else {
			BSP_Chassis_Drive(0,0);
		}
		
//		HAL_Delay(500);
//		HAL_Delay(500);
		
//		//				//???????
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
//		HAL_Delay(500);
		
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
//		
//		qianjin();
//		tem_num+=100;
//		konzhi(tem_num);
//		if(tem_num>1800){
//			tem_num=1200;
//		}
		
		
//							/* ???????????? */
//			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
//			{
//					/* ?? PB12 ??? */
//					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//					konzhi(1300);
//					/* ??????(?????),????????? */
//					HAL_Delay(20);
//					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET)
//					{
//							// ?????????????
//					}
//			}
//			
//			
//			/* ???????????? */
//			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
//			{
//					/* ?? PB12 ??? */
//					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//					konzhi(1500);
//					/* ??????(?????),????????? */
//					HAL_Delay(20);
//					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET)
//					{
//							// ?????????????
//					}
//			}
//			
//							/* ???????????? */
//			if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
//			{
//					/* ?? PB12 ??? */
//					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//					konzhi(1700);
//					/* ??????(?????),????????? */
//					HAL_Delay(20);
//					while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET)
//					{
//							// ?????????????
//					}
//			}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
