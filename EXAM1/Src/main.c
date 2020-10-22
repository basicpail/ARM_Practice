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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SHT2x_ADDR              0x40<<1
#define SHT2x_HOLD_MASTER_T     0xE3
#define SHT2x_HOLD_MASTER_RH    0xE5
#define SHT2x_NOHOLD_MASTER_T   0xF3
#define SHT2x_NOHOLD_MASTER_RH  0xF5
#define SHT2x_WRITE_USER_REG    0xE6
#define SHT2x_READ_USER_REG     0xE7
#define SHT2x_SOFT_RESET        0xFE

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rxData;
uint8_t CCR1_val=0;
uint32_t adcData=0;
uint32_t val[1];
char str[10];
char val2[4];
unsigned int piano[8] = {523,587,659,698,783,880,987,1046};
unsigned int piano2[10] = {1318,1174,1046,987,880,783,698,659,587,523};
int i=0;
int key =1;
int oldkey =1;

uint8_t i2cData[2];
uint16_t VALUE = 0;
uint8_t mode;
float TEMP, HUMI;
char str_i2c[16];

unsigned char menu[] = "+++ MENU +++\r\n L : LED\r\n C : CDS(DMA)\r\n 0~7 : PIANO\r\n T : TEMP,HUMID\r\n Push Button : stop\r\n";

/* USER CODE END PV */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  
  lcdInit();
  lcdGotoXY(0,0);
  
  HAL_UART_Receive_IT(&huart1, &rxData, 1);
  
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
  
  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_DMA(&hadc1,val,1);
  
  HAL_UART_Transmit(&huart1,menu,91,10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(rxData == 'T')
    {
      mode = SHT2x_NOHOLD_MASTER_T;
      HAL_I2C_Master_Transmit(&hi2c1, SHT2x_ADDR, &mode,1,10);
      HAL_Delay(300);
      HAL_I2C_Master_Receive(&hi2c1,SHT2x_ADDR,i2cData,2,10);
      VALUE = i2cData[0] << 8 | i2cData[1];
      TEMP = -46.85 + 175.72 *((float)VALUE/65536);
      lcdGotoXY(0,0);
      sprintf(str_i2c,"TEMP : %5.3lf", TEMP);
      lcdPrint(str_i2c);
      
      mode = SHT2x_NOHOLD_MASTER_RH;
      HAL_I2C_Master_Transmit(&hi2c1, SHT2x_ADDR, &mode,1,10);
      HAL_Delay(300);
      HAL_I2C_Master_Receive(&hi2c1,SHT2x_ADDR,i2cData,2,10);
      VALUE = i2cData[0] << 8 | i2cData[1];
      HUMI = -6 + 125*((float)VALUE/65536);
      lcdGotoXY(0,1);
      sprintf(str_i2c,"HUMID : %5.3lf", HUMI);
      lcdPrint(str_i2c);
    }
    
    if(rxData == 'L')
    {
      TIM10->CCR1 = CCR1_val;
      CCR1_val++;
      if(CCR1_val > 80) CCR1_val = 0;
      HAL_Delay(10);
    }
  
    if(rxData == 'C')
    {
      
      lcdGotoXY(0,0);
      lcdPrintData("ADC DMA_TEST!",13);
      lcdGotoXY(0,1);
      sprintf(str,"CdsValue: ");
      lcdPrintData(str,10);
      
      val2[0] = val[0]/1000 + 48;
      val2[1] = (val[0]/100)%10 + 48;
      val2[2] = (val[0]/10)%10 + 48;
      val2[3] = val[0]%10 + 48;
      lcdPrintData(val2,4);
      HAL_Delay(300);
    }
    
    if((rxData-48 >=0 && rxData-48 <= 9))
    {
      TIM9->ARR = piano2[rxData-48]-1;
      TIM9->CCR1 = piano2[rxData-48]/5;
      HAL_Delay(100);
      TIM9->ARR = 0;
      TIM9->CCR1 = 0;
      rxData = 'z';L
    }
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART1)
  {
    HAL_UART_Receive_IT(&huart1, &rxData, 1);
    HAL_UART_Transmit(&huart1, &rxData, 1,10);
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
  {
    rxData = 'z';
    HAL_UART_Transmit(&huart1,menu,91,10);
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
