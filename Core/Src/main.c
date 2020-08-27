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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "x_nucleo_plc01a1.h"
#include "rnk.h"
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
static uint8_t INPUT_READ_FLAG = 1;//Flag for reading input channel status
static uint8_t ChannelsOn = 0x00;//Number of channels in on state
static uint16_t WD_Reset = 0;//Watchdog count (in ms)
static uint8_t Tick_10 = 0;//10ms count
static uint8_t* Input_Data;//Input channels state
static uint8_t oData = 0;
static uint8_t* Relay_Status;//Output relay status
static uint8_t* Current_Limiter_Status;//Input current limiter status
static uint32_t FreezeTime = 1000;//Time in ms for which outpus to be freezed

uint8_t tx_buffer[50];
uint8_t rx_buffer[50];

#ifdef OUTPUT_CYCLING
/* The frequency range of Output_cycling is expressed in Hz */ 
static uint16_t OC_Frequency = 50;
/* 10% duty cycle to 90% duty cycle (%) */
static uint8_t OC_Duty = 50;
#endif /* OUTPUT_CYCLING */

/* Private function prototypes -----------------------------------------------*/
static void initializePlc(void);
void PLC_Handler(void);
uint8_t* CURRENT_LIMITER_Handler(void);
void RELAY_Handler(uint8_t*);


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
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM10_Init();
  MX_SPI5_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
//	SEND_DATA(DEBUG, (uint8_t*)"STARTING STATION- PIONER 2\r\n", 28 );
//	SEND_DATA(DEBUG, (uint8_t*)"DRIVERS LOADING... \r\n", 21 );
	HAL_Delay(20);	

	/* Initialization PLC Driver*/
	initializePlc();
	
	SEND_DATA(DEBUG, (uint8_t*)"DRIVERS PLC LOADING..... \r\n", 20 );
  
  /* Reset relay at startup to avoid FAULT */
  BSP_RELAY_Reset();
//	SEND_DATA(DEBUG, (uint8_t*)"RELAY RESESTING....... \r\n", 24 );

// 
  /* wait for 100 ms at startup */
  HAL_Delay(200);
	
  /* Enable Relay Outputs */
  BSP_RELAY_EN_Out();  
//	SEND_DATA(DEBUG, (uint8_t*)"RESET OUTPUTS...........\r\n", 25 );
	HAL_Delay(20);

	/* PIONEER-2 INITIALIZATION.....*/
	RNK_Initialization();
//	SEND_DATA(DEBUG, (uint8_t*)"PIONEER-2 INITIALIZATION..............\r\n", 40 );


//	GO_MOTION(RIG, 500); 


//	HAL_GPIO_WritePin(H1_RED_ALARM_GPIO_Port, H1_RED_ALARM_Pin, GPIO_PIN_SET);
//	
//	HAL_GPIO_WritePin(H2_GREEN_READY_GPIO_Port, H2_GREEN_READY_Pin, GPIO_PIN_RESET);	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		RNK_Basic();

    //PLC_Handler();
		

//		}
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void initializePlc(void)
{
  /* Initialize Relay and Current Limiter */
  BSP_Relay_Init();
  BSP_CurrentLimiter_Init();  
}

void PLC_Handler(void)
{
  if (INPUT_READ_FLAG)          
  {
    INPUT_READ_FLAG = 0;
		
#ifdef OUTPUT_CYCLING      
    /* Reset & set CS1 to refresh VNI watchdog */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);    
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);               
#else
    /* Handler for input current limiter */
    Input_Data = CURRENT_LIMITER_Handler();

    /* Handler for output relay */    
    RELAY_Handler(f);
#endif /* OUTPUT_CYCLING */  
  }      
}

uint8_t* CURRENT_LIMITER_Handler(void)
{
  static uint8_t* clData = NULL;
  
  clData = BSP_CURRENT_LIMITER_Read();
  
  Current_Limiter_Status = clData;
  
  if (BSP_GetCurrentLimiterStatus(Current_Limiter_Status) != CURRENT_LIMITER_OK)
  {
    /* Set input error code here */
  }
  
  return (clData+1);
}


void RELAY_Handler(uint8_t* iData)
{
  /* Uncomment the relevant function as required */ 

  /* Set Output same as input */
  oData = BSP_Signal_Mirror(*iData);
  /* Freeze selected outputs for a given time */
//  oData = BSP_Output_Freeze(*iData,0xFF,&FreezeTime);
  /* Regroup outputs */
//  oData = BSP_Output_Regroup(0xFF);          
  /* Get the sum of input channels that are high */
//  ChannelsOn = BSP_Inputs_Sum(*iData); oData = 0x00;
  /* Set Outputs same as the required states */
//  oData = BSP_Output_ON(0xFF);  
  /* Set Outputs same as required states */
//  oData = BSP_Output_OFF(0xFF); 
  /* Set Outputs state according to the inputs state AND with required logic  */
//  oData = BSP_Inputs_AND(*iData,0x0F); 
  /* Set Outputs state according to the inputs state OR with required logic */
//  oData = BSP_Inputs_OR(*iData,0x0F); 
  /* Set Outputs state according to the inputs state NOT */
//  oData = BSP_Inputs_NOT(*iData); 
  /* Set Outputs state according to the inputs state XOR */
//  oData = BSP_Inputs_XOR(*iData,0x00); 
  
  Relay_Status = BSP_RELAY_SetOutputs(&oData);
  
  if (BSP_GetRelayStatus(Relay_Status) != RELAY_OK)
  {
    /* Set output error code here */
  }
}


void HAL_SYSTICK_Callback(void)
{
  if (WD_Reset <= 100)
  {
		WD_Reset++;

	}
  else
  {
    Tick_10++;
    if (Tick_10 == 10)
    {
      Tick_10 = 0;
      INPUT_READ_FLAG = 1;
    }
  }
  
  if (FreezeTime != 0)
    FreezeTime--;
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
