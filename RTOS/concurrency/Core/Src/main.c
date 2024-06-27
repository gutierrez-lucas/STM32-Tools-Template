
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
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

int _write(int file, char *data, int len){
		 if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)){
						errno = EBADF;
						return -1;
		 }
		 HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);

		 return (status == HAL_OK ? len : 0);
}

static void change_adc_channel(uint32_t channel){
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK){
			Error_Handler();
	}
}

void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName){
	// printf("Stack Overflow inminent\r\n");
}

void warn_stakoverflow(void){
	printf("Stack Overflow inminent\r\n");
}

void trace_toggle(int tag){
	if(tag == 1){
		HAL_GPIO_TogglePin(trace_1_GPIO_Port, trace_1_Pin);
	}else if(tag == 2){
		HAL_GPIO_TogglePin(trace_2_GPIO_Port, trace_2_Pin);
	}else if(tag == 3){
		HAL_GPIO_TogglePin(trace_3_GPIO_Port, trace_3_Pin);
	}else{
		return;
	}
}

void trace_on(int tag){
	if(tag == 1){
		HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_SET);
	}else if(tag == 2){
		HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_SET);
	}else if(tag == 3){
		HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_SET);
	}else{
		return;
	}
}

void trace_off(int tag){
	if(tag == 1){
		HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_RESET);
	}else if(tag == 2){
		HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_RESET);
	}else if(tag == 3){
		HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_RESET);
	}else{
		return;
	}
}

void display_task(void *pvParameters);
void button_task(void *pvParameters);
void conversion_task(void *pvParameters);
xTaskHandle xButton_task_handle = NULL;
xTaskHandle xDisplay_task_handle = NULL;
xTaskHandle xConversion_task_handle = NULL;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

QueueHandle_t button_queue;
QueueHandle_t conversion_queue;

typedef struct {
	uint32_t x;
	uint32_t y;
} adc_t;

bool block = true;

int main(void)
{

	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C2_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();

	printf("\r\n\r\nDisplay Test\r\n");

	button_queue = xQueueCreate(20, sizeof(char));
	conversion_queue = xQueueCreate(20, sizeof(adc_t));

	xTaskCreate(display_task, "display_task", 230, NULL, tskIDLE_PRIORITY+2, &xDisplay_task_handle);
	xTaskCreate(button_task, "button_task", 128, NULL, tskIDLE_PRIORITY+1, &xButton_task_handle);
	xTaskCreate(conversion_task, "conversion_task", 128, NULL, tskIDLE_PRIORITY+2, &xConversion_task_handle);

	vTaskSetApplicationTaskTag( xDisplay_task_handle, ( void * ) 1 );
	vTaskSetApplicationTaskTag( xButton_task_handle, ( void * ) 2 );
	vTaskSetApplicationTaskTag( xConversion_task_handle, ( void * ) 4 );

	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	vTaskStartScheduler();

	while (1){
	}
}

UBaseType_t task_watermark;
uint32_t counter = 0;

void funct(uint32_t n){
	uint32_t i[n];
	for(uint32_t j = 0; j < n; j++){
		i[j] = j;
	}
}

void display_task(void *pvParameters){

	// printf("Display WaterMark at the beggining: %d words\r\n", (UBaseType_t)uxTaskGetStackHighWaterMark(xDisplay_task_handle));

	TickType_t xLastWakeTime = xTaskGetTickCount();

	static int connected = 0;

	// uint32_t *inter = (uint32_t)malloc(sizeof(uint32_t));
	// *(inter) = 0;

	char button_res;	
	adc_t adc_res;
	char aux[10];

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
				xTaskNotifyGive(xButton_task_handle);
				xTaskNotifyGive(xConversion_task_handle);
			}
		}else{
			HAL_TIM_Base_Stop_IT(&htim2);
			while(uxQueueMessagesWaiting(button_queue) != 0){
				xQueueReceive(button_queue, &button_res, 0);
				SSD1306_GotoXY (position_x,position_y); 
				SSD1306_Putc (button_res, &Font_11x18, 1); 
				SSD1306_UpdateScreen(); 
			}
		
			while(uxQueueMessagesWaiting(conversion_queue) != 0){
				xQueueReceive(conversion_queue, &adc_res, 0);

				SSD1306_GotoXY (position_x,position_y+15); 
				sprintf(aux, "%04d", adc_res.x);
				SSD1306_Puts (aux, &Font_11x18, 1); 

				SSD1306_GotoXY (position_x,position_y+30); 
				sprintf(aux, "%04d", adc_res.y);
				SSD1306_Puts (aux, &Font_11x18, 1); 


				SSD1306_UpdateScreen(); 
			}
			HAL_TIM_Base_Start_IT(&htim2);
		}
		// ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		task_watermark = uxTaskGetStackHighWaterMark(xDisplay_task_handle);
		// counter++;
		// funct(counter);
		// realloc(inter, sizeof(counter*sizeof(uint32_t)));
		// *(inter+counter) = task_watermark;
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200));
	}
	printf("Destroying Display task 1 \r\n");
	vTaskDelete(xDisplay_task_handle);
}

void button_task(void *pvParameters){

	printf("Button WaterMark at the beggining: %d words\r\n", uxTaskGetStackHighWaterMark(NULL));
	printf("Button task started, waiting for display ready\r\n");

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	uint8_t button_left_var = 0, button_right_var = 0, button_up_var = 0, button_down_var = 0;
	char current_button;

	while(1){

		if(button_left_var == 1){
			if(HAL_GPIO_ReadPin(button_l_GPIO_Port, button_l_Pin) == GPIO_PIN_SET){
				button_left_var = 0;
				current_button = 'L';
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
				current_button = 'R';
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
				current_button = 'U';
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
				current_button = 'D';
				xQueueSend(button_queue, &current_button, 0);
			}
		}else{
			if(HAL_GPIO_ReadPin(button_d_GPIO_Port, button_d_Pin) == GPIO_PIN_RESET){
				button_down_var = 1;
			}
		}

		vTaskDelay(100/ portTICK_PERIOD_MS);
	}
	printf("Destroying Button task \r\n");
	vTaskDelete(xButton_task_handle);
}

void conversion_task(void *pvParameters){

	uint16_t counter = 0;
	adc_t adc_out;
	uint32_t adc1 = 0;
	uint32_t adc2 = 0;

	printf("Conversion task,, waiting for display ready\r\n");

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	printf("Ready, activating timer\r\n");

	HAL_TIM_Base_Start_IT(&htim2);

	while(1){
		ulTaskNotifyTake(pdFALSE, portMAX_DELAY);
		
		HAL_ADC_Start_IT(&hadc1);
		xTaskNotifyWait(0, 0, &adc1, portMAX_DELAY);

		change_adc_channel(ADC_CHANNEL_7);
		HAL_ADC_Start_IT(&hadc1);
		xTaskNotifyWait(0, 0, &adc2, portMAX_DELAY);

		change_adc_channel(ADC_CHANNEL_6);

		if( counter++ == 3000){
			adc_out.x = adc1;
			adc_out.y = adc2;
			counter = 0;
			xQueueSend(conversion_queue, &adc_out, 0);
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}else if (htim->Instance == TIM2) {
		trace_toggle(3);
		vTaskNotifyGiveFromISR(xConversion_task_handle, &xHigherPriorityTaskWoken);
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xTaskNotifyFromISR(xConversion_task_handle,(uint32_t)HAL_ADC_GetValue(&hadc1), eSetValueWithOverwrite, &xHigherPriorityTaskWoken); 
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1; 
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	// sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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


static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
	Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOB, main_led_Pin|sec_led_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = main_led_Pin|sec_led_Pin;

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//   HAL_GPIO_WritePin(GPIOB, display_reset_Pin, GPIO_PIN_SET);
//   GPIO_InitStruct.Pin = display_reset_Pin;
//   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
// //   GPIO_InitStruct.Pull = GPIO_PULLUP;
//   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, trace_3_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = trace_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOA, trace_1_Pin|trace_2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = trace_1_Pin|trace_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
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
