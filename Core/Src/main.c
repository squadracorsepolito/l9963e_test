/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "crc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "L9963E.h"
#include <stdio.h>
#include "string.h"
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

HAL_StatusTypeDef sample_callback(void) {
  HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);
  return HAL_OK;
}

HAL_StatusTypeDef dioca(char *buffer, uint32_t size) {
  static char buf[256];
  while(huart2.gState == HAL_UART_STATE_BUSY_TX);
  strncpy(buf, buffer, 256);
  return HAL_UART_Transmit_IT(&huart2, (uint8_t*)buf, size);
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
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
/*
  uint8_t data1[5] = {0xF9, 0xFF, 0xBF, 0x70, 0xC2};
  uint8_t data2[5] = {0xFD, 0xCE, 0x3E, 0x1C, 0xC2};
  uint8_t data3[5] = {0x01, 0x00, 0x08, 0x70, 0xC2};
  uint8_t data4[5] = {0x64, 0x10, 0x08, 0x30, 0xE3};
  uint8_t data5[5] = {0x47, 0x10, 0x8C, 0xF0, 0xC0};
  uint8_t data6[5] = {0x0E, 0x80, 0x80, 0x78, 0x82};
  uint8_t data7[5] = {0xBE, 0x54, 0x19, 0x64, 0xC2};
  //uint8_t data[5] = {0xC2, 0x70, 0xBF, 0xFF, 0xF9};
  volatile uint8_t crc = L9963E_crc_calc(data1);
  crc = L9963E_crc_calc(data2);
  crc = L9963E_crc_calc(data3);
  crc = L9963E_crc_calc(data4);
  crc = L9963E_crc_calc(data5);
  crc = L9963E_crc_calc(data6);
  crc = L9963E_crc_calc(data7);
*/

  L9963E_HandleTypeDef hl9;
  L9963E_init(&hl9, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO10_GPIO_Port, GPIO10_Pin, BNE_GPIO_Port, BNE_Pin);

  /*ERROR_UTILS_error_set(&error_h, 2, 0);
  ERROR_UTILS_error_set(&error_h, 3, 0);*/
  //HAL_UART_Transmit(&huart2, (uint8_t*)"error set\r\n", 11, 10);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    L9963E_RegisterUnionTypeDef reg1 = {
      .DEV_GEN_CFG = {
        .chip_ID = 1,
        .iso_freq_sel = 0
      }
    };
    L9963E_RegisterUnionTypeDef reg2 = {
      .DEV_GEN_CFG = {
        .chip_ID = 1,
        .iso_freq_sel = 0
      }
    };
    if(L9963E_reg_read(&hl9, 0x1, DEV_GEN_CFG, &reg1) != HAL_OK) {
      L9963E_reg_write(&hl9, 0, DEV_GEN_CFG, &reg2);
    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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