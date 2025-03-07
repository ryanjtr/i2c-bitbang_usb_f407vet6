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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include <stdbool.h>

#include <string.h>
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

extern uint8_t ReceivedData[100]; // Data buffer
extern i2c_t i2c_handler;
//extern uint32_t count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void get_slave_address(void);
static void sync_app(void);
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
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  I2C_BITBANG_Init();

//  uart_printf("slave-f407vet\r\n");



//
//  uint8_t led2on[5]={0xAA,0x01,0x02,0x01,0xFF};
//  uint8_t led2off[5]={0xAA,0x01,0x02,0x00,0xFF};
//  uint8_t led1on[5]={0xAA,0x01,0x01,0x01,0xFF};
//  uint8_t led1off[5]={0xAA,0x01,0x01,0x00,0xFF};
//  uint8_t led3on[5]={0xAA,0x01,0x03,0x01,0xFF};
//  uint8_t led3off[5]={0xAA,0x01,0x03,0x00,0xFF};
//  uint8_t led1toggle[5]={0xAA,0x01,1,0x02,0xFF};
//  uint8_t data_to_matalab[10]={0xAA,0x02,0x02,0x0D,0x04,0x45,0x46,0x47,0x48,0xFF};
//  uint8_t data1_to_matalab[10]={0xAA,0x02,0x02,0x1D,0x04,0x6D,0x6E,0x6F,0x70,0xFF};
//  uint8_t data2_to_matalab[10]={0xAA,0x02,0x02,0x2D,0x04,0x7B,0x7C,0x7D,0x7E,0xFF};
//  i2c_handler.num_of_address=3;
//  i2c_handler.list_addr_slave[0] = 0x67;
//  i2c_handler.list_addr_slave[1] = 0x68;
//  i2c_handler.list_addr_slave[2] = 0x69;


//  sync_app();
  get_slave_address();

  LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_3);//Trigger board F1


//  LL_GPIO_TogglePin(GPIOC, LL_GPIO_PIN_9); //OFF
//  i2c_handler.slave_txdata[0x10]=0x03;
//  for (int i=0x10+1;i<256;i++)
//  {
//	  i2c_handler.slave_txdata[i]=i2c_handler.slave_txdata[i-1]+1;
//  }
  //sent signal to get slave addresses
//    xTaskCreate(read_from_eeprom, "read data from eeprom", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//    vTaskStartScheduler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//

//	  CDC_Transmit_FS(data_to_matalab, sizeof(data_to_matalab));
//	  LL_mDelay(2000);
//	  CDC_Transmit_FS(data1_to_matalab, sizeof(data1_to_matalab));
//	  LL_mDelay(2000);
//	  CDC_Transmit_FS(data2_to_matalab, sizeof(data2_to_matalab));
//	  LL_mDelay(2000);
//	  CDC_Transmit_FS(led1toggle, sizeof(led1toggle));
//	  LL_mDelay(3000);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 120, LL_RCC_PLLP_DIV_2);
  LL_RCC_PLL_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSE, LL_RCC_PLLM_DIV_4, 120, LL_RCC_PLLQ_DIV_5);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  while (LL_PWR_IsActiveFlag_VOS() == 0)
  {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_2);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(60000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9|LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = 115200;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOH);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);

  /**/
  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_3|LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void get_slave_address(void)
{
//	uint8_t signal_to_get_slave_addr[4]={0xAA,0x02,0x01,0xFF};
//	LL_mDelay(2000);
//	CDC_Transmit_FS(signal_to_get_slave_addr, sizeof(signal_to_get_slave_addr));
//	LL_mDelay(3000);
//	i2c_handler.num_of_address=ReceivedData[3];
//	for(int i=4;i<i2c_handler.num_of_address+5;i++)
//		{i2c_handler.list_addr_slave[i-4]=ReceivedData[i];}

	  i2c_handler.num_of_address=3;
	  i2c_handler.list_addr_slave[0] = 0x67;
	  i2c_handler.list_addr_slave[1] = 0x68;
	  i2c_handler.list_addr_slave[2] = 0x69;

//	LL_mDelay(3000);


}

static void sync_app(void)
{
	uint8_t dumb[2]={0xAA,0xBB};
	bool is_sync=false;
	uint8_t timeout=10;
	uint8_t count=0;
	while(!is_sync)
	{
		LL_mDelay(1000);
		if(ReceivedData[0]==0xAA && ReceivedData[1]==0xBB)
		{
			is_sync=true;
			CDC_Transmit_FS(dumb, sizeof(dumb));
			break;
		}
		if(++count > timeout)
			NVIC_SystemReset();  // Reset MCU
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
