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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BuckConverter_UART.h"
#include "BuckConverter_PID.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PID_TypeDef pid;
DXLLineTypeDef Master;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t  adc_buffer[3];
uint8_t IN_Voltage, OUT_Voltage;
float Input_Voltage, Output_Voltage, Output_Current;
double Pid_Value = 0;
uint32_t ACSoffset = 3102;
uint32_t ZeroOffset = 0;
uint8_t MainBoard_Response[10];
uint8_t LookUp_Table[10];
uint8_t RX_Buffer[8];
uint8_t Packet_Length;
int counter = 0;
float duty;
float VoltSense;
uint8_t set_power;
uint8_t Volt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Power(uint8_t SetPower);
void Set_OutputVoltage(uint8_t Voltage);
void LookUpTable_Update(void);
void Parameters_init(void);
void MainBoardRead_Data(uint8_t *Packet);

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
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, adc_buffer, 3);
  HAL_TIM_Base_Start_IT(&htim3);
  PID_Init(&pid, 1, 0, 0);

	DXLStruct_Initializer(&Master);
	
	MainBoard_Response[0] = 0xFF;
	MainBoard_Response[1] = 0xFF;
	MainBoard_Response[2] = MY_ID;
	MainBoard_Response[4] = DXL_NoError;
	Master.Buffer[Master.BufferLastIndex + 7] = set_power;
	Power(set_power);
	HAL_UART_Receive_IT(&huart3, &Master.InputData, 1);
	duty = 12 * 100 / Input_Voltage;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)duty);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

	HAL_UART_Receive_IT(&huart3, &Master.InputData, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		LookUpTable_Update();
		if(counter >= 100 )
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
			counter = 0;
		}
		Set_OutputVoltage(Volt);

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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		// HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		Packet_Length = Master_CheckPacket(&huart3, &Master);
		if(Packet_Length > 0)
		{
			if(Master.Buffer[Master.BufferLastIndex + 2] == MY_ID)
			{
				if(DXL_CheckSumCalc(&Master.Buffer[Master.BufferLastIndex]) == Master.Buffer[Master.BufferLastIndex + Packet_Length - 1])
				{
					switch(Master.Buffer[Master.BufferLastIndex + 4])
					{
						case DXL_READ_DATA:
							MainBoardRead_Data(&Master.Buffer[Master.BufferLastIndex]);					
						break;
						case DXL_WRITE_DATA:
							if(Master.Buffer[Master.BufferLastIndex + 5]==0)
								Volt = Master.Buffer[Master.BufferLastIndex + 6];
							else 
								Power(Master.Buffer[Master.BufferLastIndex + 6]);
						break;
					}
				}
			}
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(TX_EN_GPIO_Port, TX_EN_Pin, GPIO_PIN_RESET);
}

void Set_OutputVoltage(uint8_t Voltage)
{ 
	if( Voltage < 15 )
	{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		//ShutDown_Gpio
	Pid_Value = PID_Compute(&pid, Output_Voltage, Voltage);
	duty = Voltage * 100 / Input_Voltage;
	duty += Pid_Value;
	/* if(Pid_Value<1 && Pid_Value > 0)
	{
		Pid_Value = 1;
	}
	else if(Pid_Value > -1 && Pid_Value < 0)
	{
		Pid_Value = -1;
	}*/
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (uint16_t)duty);
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{ 
	Input_Voltage = adc_buffer[2] * (3.3/4095) * (1/0.1076);  
	Output_Voltage = adc_buffer[1] * (3.3/4095) * (1/0.1076);
	VoltSense = adc_buffer[0] + ZeroOffset;
	Output_Current =(((ACSoffset) - (VoltSense))*0.806)/185;
	if (Output_Current > 4)
	{
		counter++;
	}
	
}

void LookUpTable_Update(void)
{
	IN_Voltage  = Input_Voltage  * 10;
	OUT_Voltage = Output_Voltage * 10;
	LookUp_Table[0] = IN_Voltage;
	LookUp_Table[1] = OUT_Voltage;
}

void MainBoardRead_Data(uint8_t* Packet)
{
	MainBoard_Response[3] = Packet[6] + 2;
	for(uint8_t i = 0; i < Packet[6]; i++)
	{
		MainBoard_Response[i + 5] = LookUp_Table[Packet[5] + i];
	}
	MainBoard_Response[Packet[6] + 5] = DXL_CheckSumCalc(MainBoard_Response);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart3, MainBoard_Response, MainBoard_Response[3] + 4);
}
void Power(uint8_t SetPower)
{
	if(SetPower == 0x01)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
if(SetPower == 0x00)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
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
