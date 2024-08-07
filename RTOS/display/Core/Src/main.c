
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
#include <stdbool.h>
#include <stdint.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

#include "../display/ssd1306.h"

typedef enum {
	LEFT = 1,
	RIGHT = 2,
	UP = 4,
	DOWN = 5,
	UNPRESS = 0
} button_t;

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

void display_task(void *pvParameters);
void button_task(void *pvParameters);
xTaskHandle xButton_task_handle = NULL;
xTaskHandle xDisplay_task_handle = NULL;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);

QueueHandle_t button_queue;

bool block = true;

int main(void)
{

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();

	printf("\r\n\r\nDisplay Test\r\n");

	button_queue = xQueueCreate(20, sizeof(button_t));

	xTaskCreate(display_task, "display_task", 128, NULL, tskIDLE_PRIORITY+2, &xDisplay_task_handle);
	xTaskCreate(button_task, "button_task", 128, NULL, tskIDLE_PRIORITY+1, &xButton_task_handle);

	vTaskSetApplicationTaskTag( xDisplay_task_handle, ( void * ) 1 );
	vTaskSetApplicationTaskTag( xButton_task_handle, ( void * ) 2 );

	vTaskStartScheduler();

	while (1){
	}
}


void display_task(void *pvParameters){

    UBaseType_t uxHighWaterMark;

	uxHighWaterMark = uxTaskGetStackHighWaterMark( NULL );
	printf("Display WaterMark at the beggining: %d words\r\n", uxHighWaterMark);
	TickType_t xLastWakeTime = xTaskGetTickCount();

	static int connected = 0;

	button_t button_res = 0;	
	uint8_t position_x = 10;
	uint8_t position_y = 10;

	while(1){
		if(connected == 0){
			HAL_StatusTypeDef res = SSD1306_Init(0x78); 
			if( res != HAL_OK){
				printf("Display connection err: %d\r\n", res);
			}else{
				HAL_GPIO_WritePin(main_led_GPIO_Port, main_led_Pin, SET);
				connected = 1;
				printf("Display connected.\r\n" );

				SSD1306_Clear();
				vTaskPrioritySet(xButton_task_handle, tskIDLE_PRIORITY+3);
				// xTaskNotifyGive(xButton_task_handle);
				xTaskNotify(xButton_task_handle, 0xffff, eSetValueWithOverwrite); 
			}
		}else{
			xQueueReceive(button_queue, &button_res, 0);
			while(button_res != 0){
				SSD1306_GotoXY (position_x,position_y); 
				switch(button_res){
					case(LEFT):
						printf("Button left pressed \r\n");
						SSD1306_Puts ("L", &Font_11x18, 1); 
						break;
					case(RIGHT):
						printf("Button right pressed \r\n");
						SSD1306_Puts ("R", &Font_11x18, 1);
						break;
					case(UP):
						printf("Button up pressed \r\n");
						SSD1306_Puts ("U", &Font_11x18, 1); 
						break;
					case(DOWN):
						printf("Button down pressed \r\n");
						SSD1306_Puts ("D", &Font_11x18, 1); 
						break;
					default: break;
				}
				SSD1306_UpdateScreen(); 
				if(position_x < 100){
					position_x += 10;
				}else{
					position_x = 10;
					position_y += 15;
					if(position_y > 40){
						position_y = 10;
						SSD1306_Clear();
					}
				}
				button_res = 0;
				xQueueReceive(button_queue, &button_res, 0);
			}
		}
		
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(150));
	}
	printf("Destroying Display task 1 \r\n");
	vTaskDelete(xDisplay_task_handle);
}

void button_task(void *pvParameters){
	uint8_t counter = 0;

	printf("Button WaterMark at the beggining: %d words\r\n", uxTaskGetStackHighWaterMark(NULL));
	printf("Button task started, waiting for display ready\r\n");
	// ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	uint32_t notification_message = 42;
	xTaskNotifyWait(0, 0, &notification_message, portMAX_DELAY);

	printf("Display ready, starting button task, message: 0x%04X\r\n", notification_message);

	uint8_t button_left_var = 0, button_right_var = 0, button_up_var = 0, button_down_var = 0;
	button_t current_button = UNPRESS;

	while(1){
		// HAL_GPIO_TogglePin(sec_led_GPIO_Port, sec_led_Pin);

		if(button_left_var == 1){
			if(HAL_GPIO_ReadPin(button_l_GPIO_Port, button_l_Pin) == GPIO_PIN_SET){
				button_left_var = 0;
				current_button = LEFT;
				xQueueSend(button_queue, &current_button, 0);
			}
		}else{
			if(HAL_GPIO_ReadPin(button_l_GPIO_Port, button_l_Pin) == GPIO_PIN_RESET){
				button_left_var = 1;
			}
		}

		if(button_right_var == 1){
			if(HAL_GPIO_ReadPin(button_r_GPIO_Port, button_r_Pin) == GPIO_PIN_SET){
				button_right_var = 0;
				current_button = RIGHT;
				xQueueSend(button_queue, &current_button, 0);
			}
		}else{
			if(HAL_GPIO_ReadPin(button_r_GPIO_Port, button_r_Pin) == GPIO_PIN_RESET){
				button_right_var = 1;
			}
		}

		if(button_up_var == 1){
			if(HAL_GPIO_ReadPin(button_u_GPIO_Port, button_u_Pin) == GPIO_PIN_SET){
				button_up_var = 0;
				current_button = UP;
				xQueueSend(button_queue, &current_button, 0);
			}
		}else{
			if(HAL_GPIO_ReadPin(button_u_GPIO_Port, button_u_Pin) == GPIO_PIN_RESET){
				button_up_var = 1;
			}
		}

		if(button_down_var == 1){
			if(HAL_GPIO_ReadPin(button_d_GPIO_Port, button_d_Pin) == GPIO_PIN_SET){
				button_down_var = 0;
				current_button = DOWN;
				xQueueSend(button_queue, &current_button, 0);
			}
		}else{
			if(HAL_GPIO_ReadPin(button_d_GPIO_Port, button_d_Pin) == GPIO_PIN_RESET){
				button_down_var = 1;
			}
		}

		// if(counter++ > 10){
		// 	counter = 0;
		// 	printf("Button WaterMark at running: %d words\r\n", uxTaskGetStackHighWaterMark(NULL));
		// }	
		vTaskDelay(100/ portTICK_PERIOD_MS);
	}
	printf("Destroying Button task \r\n");
	vTaskDelete(xButton_task_handle);
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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

static void MX_I2C2_Init(void)
{
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
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
  GPIO_InitStruct.Pin = main_led_Pin|sec_led_Pin|trace_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, display_reset_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = display_reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//   GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, trace_1_Pin|trace_2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = trace_1_Pin|trace_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = button_l_Pin|button_r_Pin|button_u_Pin|button_d_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = button_r_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
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
