
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO


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
	}else if(tag == 0){ // IDLE
		HAL_GPIO_TogglePin(trace_3_GPIO_Port, trace_3_Pin);
	}else if(tag == 4){
		HAL_GPIO_TogglePin(trace_4_GPIO_Port, trace_4_Pin);
	}else if(tag == 5){
		HAL_GPIO_TogglePin(trace_5_GPIO_Port, trace_5_Pin);
	}else{
		return;
	}
}

void trace_on(int tag){
	if(tag == 1){
		HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_SET);
	}else if(tag == 2){
		HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_SET);
	}else if(tag == 0){
		HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_SET);
	}else if(tag == 4){
		HAL_GPIO_WritePin(trace_4_GPIO_Port, trace_4_Pin, GPIO_PIN_SET);
	}else if(tag == 5){
		HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_SET);
	}else{
		return;
	}
}

void trace_off(int tag){
	if(tag == 1){
		HAL_GPIO_WritePin(trace_1_GPIO_Port, trace_1_Pin, GPIO_PIN_RESET);
	}else if(tag == 2){
		HAL_GPIO_WritePin(trace_2_GPIO_Port, trace_2_Pin, GPIO_PIN_RESET);
	}else if(tag == 0){
		HAL_GPIO_WritePin(trace_3_GPIO_Port, trace_3_Pin, GPIO_PIN_RESET);
	}else if(tag == 4){
		HAL_GPIO_WritePin(trace_4_GPIO_Port, trace_4_Pin, GPIO_PIN_RESET);
	}else if(tag == 5){
		HAL_GPIO_WritePin(trace_5_GPIO_Port, trace_5_Pin, GPIO_PIN_RESET);
	}else{
		return;
	}
}

void adc_task(void *pvParameters);
void print_task(void *pvParameters);
xTaskHandle adc_task_handle = NULL;
xTaskHandle print_task_handle = NULL;

void SystemClock_Config(void);

static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
// static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

QueueHandle_t adc_queue;

#define BUFFER_SIZE 200

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_USART1_UART_Init();
	// MX_TIM2_Init();
	MX_ADC1_Init();

	printf("\r\n\r\nACD system\r\n");

	adc_queue = xQueueCreate(BUFFER_SIZE+100, sizeof(uint16_t));

	xTaskCreate(adc_task, "adc_task", 128, NULL, tskIDLE_PRIORITY+2, &adc_task_handle);
	xTaskCreate(print_task, "print_task", 230, NULL, tskIDLE_PRIORITY+1, &print_task_handle);

	vTaskSetApplicationTaskTag( print_task_handle, ( void * ) TRACE_PRINT );
	vTaskSetApplicationTaskTag( adc_task_handle, ( void * ) TRACE_ADC );

	vTaskStartScheduler();

	while (1){
	}
}

uint8_t max_queue_stored = 0;
UBaseType_t task_watermark;

void print_task(void *pvParameters){
	printf("PRINT task\r\n");

	char str_buffer[10];
	uint32_t acumulator = 0;
	uint16_t adc_buffer[BUFFER_SIZE];
	uint8_t counter = 0, i = 0;

	while(1){
		while( uxQueueMessagesWaiting(adc_queue) != 0 && counter < BUFFER_SIZE ){
			if( xQueueReceive(adc_queue, &adc_buffer+counter, 0) == pdTRUE ){
				counter++;
			}
		}
		if( counter == BUFFER_SIZE ){
			for(i = 0; i < counter; i++){
				acumulator+=adc_buffer[i];
				sprintf(str_buffer, "%05d\r\n", adc_buffer[i]);
				printf("%s", str_buffer);
			}
			acumulator/=BUFFER_SIZE;
			sprintf(str_buffer, "%05d\r\n", acumulator);
			printf("%s", str_buffer);
			acumulator = 0;
			counter = 0;
			max_queue_stored = 0;
		}
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
	}
	printf("Destroying print task \r\n");
	vTaskDelete(print_task_handle);
}


void adc_task(void *pvParameters){
	printf("ACD task\r\n");
         
	TickType_t xLastWakeTime;
	uint16_t value;

	while(1){
		xLastWakeTime = xTaskGetTickCount();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		value = HAL_ADC_GetValue(&hadc1);
		xQueueSend(adc_queue, (uint16_t*)&value, 0);
		xTaskNotifyGive(print_task_handle);

		max_queue_stored++;

		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
	}
	printf("Destroying ADC task \r\n");
	vTaskDelete(adc_task_handle);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
// 	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	if (htim->Instance == TIM1) {
		HAL_IncTick();
// 	}else if (htim->Instance == TIM2) {
// 		trace_toggle(3);
// 		vTaskNotifyGiveFromISR(xConversion_task_handle, &xHigherPriorityTaskWoken);
	}
}

// void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
// 	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
// 	xTaskNotifyFromISR(xConversion_task_handle,(uint32_t)HAL_ADC_GetValue(&hadc1), eSetValueWithOverwrite, &xHigherPriorityTaskWoken); 
// }

void SystemClock_Config(void){
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

static void MX_ADC1_Init(void){

	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
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

// static void MX_I2C2_Init(void){
// 	hi2c2.Instance = I2C2;
// 	hi2c2.Init.ClockSpeed = 400000;
// 	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
// 	hi2c2.Init.OwnAddress1 = 0;
// 	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
// 	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
// 	hi2c2.Init.OwnAddress2 = 0;
// 	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
// 	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
// 	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
// 	{
// 		Error_Handler();
// 	}
// }


// static void MX_TIM2_Init(void)
// {

//   /* USER CODE BEGIN TIM2_Init 0 */

//   /* USER CODE END TIM2_Init 0 */

//   TIM_SlaveConfigTypeDef sSlaveConfig = {0};
//   TIM_MasterConfigTypeDef sMasterConfig = {0};

//   /* USER CODE BEGIN TIM2_Init 1 */

//   /* USER CODE END TIM2_Init 1 */
//   htim2.Instance = TIM2;
//   htim2.Init.Prescaler = 71;
//   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//   htim2.Init.Period = 60000;
//   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
//   if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//   {
// 	Error_Handler();
//   }
//   sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
//   sSlaveConfig.InputTrigger = TIM_TS_ITR1;
//   if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
//   {
// 	Error_Handler();
//   }
//   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//   {
// 	Error_Handler();
//   }
//   /* USER CODE BEGIN TIM2_Init 2 */

//   /* USER CODE END TIM2_Init 2 */

// }

static void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 19200;
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

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();


  HAL_GPIO_WritePin(GPIOA, trace_1_Pin|trace_2_Pin|trace_4_Pin|trace_5_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = trace_1_Pin|trace_2_Pin|trace_4_Pin|trace_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, trace_3_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = trace_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
