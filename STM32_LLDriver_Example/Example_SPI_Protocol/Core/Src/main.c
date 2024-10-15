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
#include "L3G4200D_LL_SPI.h"
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
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
L3G4200D_SPI4W_Typedef Sensor;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* System interrupt init*/
  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),15, 0));

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  LL_SPI_Enable(SPI1);
  /* USER CODE BEGIN 2 */
  while(L3G4200D_SPI_4WIRE_Init(&Sensor, SPI1, &User_Mod, GPIOA, LL_GPIO_PIN_4));
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    L3G4200D_SPI_4WIRE_GetTemperature(&Sensor);
    L3G4200D_SPI_4WIRE_Get_X_Axis_Accelerations(&Sensor);
    L3G4200D_SPI_4WIRE_Get_Y_Axis_Accelerations(&Sensor);
    L3G4200D_SPI_4WIRE_Get_Z_Axis_Accelerations(&Sensor);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_Init1msTick(72000000);
  LL_SetSystemCoreClock(72000000);
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  LL_SPI_InitTypeDef SPI_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  SPI_InitStruct.CRCPoly = 10;
  LL_SPI_Init(SPI1, &SPI_InitStruct);
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
