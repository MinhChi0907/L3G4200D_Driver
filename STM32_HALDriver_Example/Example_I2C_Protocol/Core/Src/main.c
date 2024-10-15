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
#include "L3G4200D_HAL_I2C.h"
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
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
DevMod_Typedef User_Mod = {
   .reg1 = {
       .CTRL_REG1_Bits.data_rate = OUTPUT_DATA_RATE_100,
       .CTRL_REG1_Bits.bandwidth = BANDWIDTH_1,
       .CTRL_REG1_Bits.pwr_mode = PWR_NORMAL_MODE,
       .CTRL_REG1_Bits.x_axis_status = ENABLE_X_AXIS,
       .CTRL_REG1_Bits.y_axis_status = ENABLE_Y_AXIS,
       .CTRL_REG1_Bits.z_axis_status = ENABLE_Z_AXIS
   },
   .reg2 = {
       .CTRL_REG2_Bits.cutoff_frequency = HIGH_PASS_FILTER_CUT_OFF_FREQUENCY_5,
       .CTRL_REG2_Bits.high_pass_filter_mode = NORMAL_MODE
   },
   .reg3 = {
       .CTRL_REG3_Bits.boot_available_int1_pin_status = DISABLE_BOOT_STATUS_ON_INT1,
       .CTRL_REG3_Bits.data_rdy_int2_pin_status = DISABLE_DATA_RDY_ON_INT2,
       .CTRL_REG3_Bits.itr_active_int1_pin_level = ITR_ACTIVE_ON_INT1_HIGH,
       .CTRL_REG3_Bits.itr_fifo_empty_int2_pin_status = DISABLE_ITR_FIFO_EMPTY_ON_INT2,
       .CTRL_REG3_Bits.itr_fifo_overun_int2_pin_status = DISABLE_ITR_FIFO_ORUN_ON_INT2,
       .CTRL_REG3_Bits.itr_fifo_wtm_int2_pin_status = DISABLE_ITR_FIFO_WTM_ON_INT2,
       .CTRL_REG3_Bits.itr_int1_pin_status = DISABLE_ITR_ON_INT1,
       .CTRL_REG3_Bits.pushpull_opendrain_mode = PUSH_PULL_ENABLE
   },
   .reg4 = {
       .CTRL_REG4_Bits.data_update_mode = DBU_CONTINUOUS_UPDATE,
       .CTRL_REG4_Bits.full_scale_value = FULL_SCALE_MODE_1,
       .CTRL_REG4_Bits.self_test_mode = SELF_TEST_NORMAL_MODE,
       .CTRL_REG4_Bits.sort_data_mode = BLE_DATA_LSB_LOW_ADD,
       .CTRL_REG4_Bits.spi_interface_mode = SPI_INTERFACE_4WIRE_MODE
   },
   .reg5 = {
       .CTRL_REG5_Bits.fifo_status = DISABLE_FIFO,
       .CTRL_REG5_Bits.high_pass_filter_status = DISABLE_HIGH_PASS_FILTER,
       .CTRL_REG5_Bits.int1_pin_mode = INT1_SELECT_MODE_1,
       .CTRL_REG5_Bits.out_mode = OUT_SELECT_MODE_1,
       .CTRL_REG5_Bits.reboot_mode = BOOT_NORMAL_MODE
   },
   .fifo_ctr_conf = {
       .FIFO_CTRL_REG_Bits.fifo_mode = FIFO_BYPASS_MODE,
       .FIFO_CTRL_REG_Bits.wtm_level = DEFAULT_VALUE          /* Set by user. Value: [0:31] */
   },
   .int1_conf = {
       .INT1_CFG_Bits.AND_OR_mode_select = ENABLE_OR_COMBINATION_INTERRUPT_EVENT,
       .INT1_CFG_Bits.itr_X_high_event_status = DISABLE_INTERRUPT_X_HIGH_EVENT,
       .INT1_CFG_Bits.itr_X_low_event_status = DISABLE_INTERRUPT_X_LOW_EVENT,
       .INT1_CFG_Bits.itr_Y_high_event_status = DISABLE_INTERRUPT_Y_HIGH_EVENT,
       .INT1_CFG_Bits.itr_Y_low_event_status = DISABLE_INTERRUPT_Y_LOW_EVENT,
       .INT1_CFG_Bits.itr_Z_high_event_status = DISABLE_INTERRUPT_Z_HIGH_EVENT,
       .INT1_CFG_Bits.itr_Z_low_event_status = DISABLE_INTERRUPT_Z_LOW_EVENT,
       .INT1_CFG_Bits.latch_itr_status = DISABLE_INTERRUPT_REQUEST_LATCH
   },
   .int1_dura = {
       .INT1_DURATION_Bits.wait_status = DISABLE_WAIT_MODE,
       .INT1_DURATION_Bits.wait_time = DEFAULT_VALUE         /* Set by user. Value: [0:127] */
   },
   .ref_conf = {
       .ref_value = DEFAULT_VALUE         /* Set by user. Value: [0:255] */
   },
   .ths_xh = {
       .value = DEFAULT_VALUE             /* Set by user. Value: [0:127] */
   },
   .ths_xl = {
       .value = DEFAULT_VALUE             /* Set by user. Value: [0:255] */
   },
   .ths_yh = {
       .value = DEFAULT_VALUE             /* Set by user. Value: [0:127] */
   },
   .ths_yl = {
       .value = DEFAULT_VALUE             /* Set by user. Value: [0:255] */
   },
   .ths_zh = {
       .value = DEFAULT_VALUE             /* Set by user. Value: [0:127] */
   },
   .ths_zl = {
       .value = DEFAULT_VALUE             /* Set by user. Value: [0:255] */
   }
};

L3G4200D_I2C_Typedef Sensor;
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
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  while(L3G4200D_I2C_Init(&Sensor, &hi2c2, &User_Mod, L3G4200D_ADDRESS));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    L3G4200D_I2C_GetTemperature(&Sensor);
    L3G4200D_I2C_Get_X_Axis_Accelerations(&Sensor);
    L3G4200D_I2C_Get_Y_Axis_Accelerations(&Sensor);
    L3G4200D_I2C_Get_Z_Axis_Accelerations(&Sensor);
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

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
