
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#include "../display/ssd1306.h"

I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;

int _write(int file, char *data, int len){
		 if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)){
						errno = EBADF;
						return -1;
		 }
		 HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

		 return (status == HAL_OK ? len : 0);
}

void trace_toggle(int tag){
	if(tag == 1){
		HAL_GPIO_TogglePin(trace_1_GPIO_Port, trace_1_Pin);
	}else if(tag == 2){
		HAL_GPIO_TogglePin(trace_2_GPIO_Port, trace_2_Pin);
	}
}

void trace_on(int tag){
	if(tag == 1){
		HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_SET);
	}else if(tag == 2){
		HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_SET);
	}
}

void trace_off(int tag){
	if(tag == 1){
		HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_RESET);
	}else if(tag == 2){
		HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_RESET);
	}
}

void led_task_1(void *pvParameters);
void led_task_2(void *pvParameters);


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);


int main(void)
{

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();

	printf("\r\n\r\nDisplay Test\r\n");
// SSD1306_Init (); // initialize the display 

// SSD1306_GotoXY (10,10); // goto 10, 10 
// SSD1306_Puts ("HELLO", &Font_11x18, 1); // print Hello 
// SSD1306_GotoXY (10, 30); 
// SSD1306_Puts ("WORLD !!", &Font_11x18, 1); 
// SSD1306_UpdateScreen(); // update screen
	xTaskCreate(led_task_1, "led_task_1", 128, NULL, tskIDLE_PRIORITY+1, NULL);
	xTaskCreate(led_task_2, "led_task_2", 128, NULL, tskIDLE_PRIORITY+2, NULL);

	vTaskStartScheduler();

	while (1){
	}
}


void led_task_1(void *pvParameters){
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	TickType_t xLastWakeTime = xTaskGetTickCount();
	static int connected = 0;
	HAL_StatusTypeDef res;
	int i = 0 ;

	while(1){
		if(connected == 0){
			res = SSD1306_Init(0x78); 
			if( res != HAL_OK){
				printf("Display connection err: %d, with addr 0x%02X  \r\n", res, i);
			}else{
				HAL_GPIO_WritePin(main_led_GPIO_Port, main_led_Pin, SET);
				connected = 1;
				printf("Display connected with addr 0x%02X \\r\n", i);

				SSD1306_GotoXY (10,10); // goto 10, 10 
				SSD1306_Puts ("HELLO", &Font_11x18, 1); // print Hello 
				SSD1306_GotoXY (10, 30); 
				SSD1306_Puts ("WORLD !!", &Font_11x18, 1); 
				SSD1306_UpdateScreen(); // update screen

			}
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
		}else{
			vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));
		}
	}
	printf("Destroying task 1 \r\n");
	vTaskDelete(NULL);
}

void led_task_2(void *pvParameters){
	vTaskSetApplicationTaskTag( NULL, ( void * ) 2 );
	while(1){
		HAL_GPIO_TogglePin(sec_led_GPIO_Port, sec_led_Pin);
		printf("1\r\n");
		vTaskDelay(550 / portTICK_PERIOD_MS);
	}
}

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
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK){
	Error_Handler();
  }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, main_led_Pin|sec_led_Pin|trace_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, trace_3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, trace_1_Pin|trace_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : main_led_Pin sec_led_Pin trace_3_Pin */
  GPIO_InitStruct.Pin = main_led_Pin|sec_led_Pin|trace_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : trace_1_Pin trace_2_Pin */
  GPIO_InitStruct.Pin = trace_1_Pin|trace_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
