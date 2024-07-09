#include "main.h"
#include <stdbool.h>

#define NUM_OF_LEDS 64 
#define BYTES_PER_LED 3 
#define MATRIX_SIZE 8

#define HIGH_BIT 58 //if our pwm period is 90, 64%(90)=57.6 close to 58
#define LOW_BIT 29  //if our pwm period is 90, 32%(90)=28.8 close to 29

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

uint8_t rgbw_arr[NUM_OF_LEDS * BYTES_PER_LED * 8 + 1];//every pixel colour info is 24 bytes long

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);

void rgb_matrix_clear_buffer(uint8_t *buffer, uint16_t bytenumber) {
	for (uint32_t i = 0; i < bytenumber-1; ++i) {
		buffer[i] = LOW_BIT;
	}
	buffer[bytenumber] = 0;//needs to be 0 to silent PWM at the end of transaction
}

typedef enum rotation_t {
	ROTATION_0 = 0,
	ROTATION_90 = 1,
	ROTATION_180 = 2,
	ROTATION_270 = 3
} rotation_t;

typedef enum rgb_mode_t {
	OFF = 0,
	LIMITS = 1
} rgb_mode_t;

typedef enum show_mode_t{
	POINTS = 0,
	LINES = 1
}show_mode_t;

void rgb_process_colors(rgb_mode_t mode, uint8_t* colors, uint8_t amplitude){
	switch(mode){
		case(OFF):
			colors[0] = 0;colors[1] = 0;colors[2] = 0; break;
		case(LIMITS):
			switch(amplitude){
				case 0: colors[0] = 0; colors[1] = 200; colors[2] = 0; break;
				case 1: colors[0] = 10; colors[1] = 200; colors[2] = 100; break;
				case 2: colors[0] = 10; colors[1] = 75; colors[2] = 80; break;
				case 3: colors[0] = 10; colors[1] = 0; colors[2] = 80; break;
				case 4: colors[0] = 300; colors[1] = 0; colors[2] = 80; break;
				case 5: colors[0] = 80; colors[1] = 0; colors[2] = 80; break;
				case 6: colors[0] = 80; colors[1] = 60; colors[2] = 10; break;
				case 7: colors[0] = 0; colors[1] = 0; colors[2] = 10; break;
				default: colors[0] = 80; colors[1] = 10; colors[2] = 0; break;
			}
			break;
		default: colors[0] = 0;colors[1] = 0;colors[2] = 0; break;
	}
}

void rgb_process_rotation(rotation_t rotation, uint8_t* coordinates, uint8_t x, uint8_t y){
	switch (rotation) {
		case ROTATION_0:
			coordinates[1] = y;
			coordinates[0] = x;
			break;
		case ROTATION_90:
			coordinates[1] = x;
			coordinates[0] = MATRIX_SIZE - y + 1;
			break;
		case ROTATION_180:
			coordinates[1] = MATRIX_SIZE - y + 1;
			coordinates[0] = MATRIX_SIZE - x + 1;
			break;
		case ROTATION_270:
			coordinates[1] = MATRIX_SIZE - x + 1;
			coordinates[0] = y;
			break;
		default: coordinates[1] = y; coordinates[0] = x; break;
	}
}

uint32_t rgb_matrix_set_pixel(uint8_t *buffer, rgb_mode_t mode, uint8_t x, uint8_t y, rotation_t rotation) {

	if(x*y>NUM_OF_LEDS){return -1;}//in case we mess up

	uint8_t y_rot, x_rot;
	uint8_t colors[3] = {100,0,0};
	uint8_t coordinates[2] = {1,1};

	rgb_process_colors(mode, colors, y);
	rgb_process_rotation(rotation, coordinates, x, y);

	uint8_t led_index = (coordinates[1]-1)*MATRIX_SIZE + coordinates[0] -1;

	for (uint32_t i = 0; i < BYTES_PER_LED * 8; ++i) { //we need to store every bit

		if (i < 8) { //this means first byte R
			if (colors[1] & (0x80 >> i)) { //this is a mask for reading every bit inside the byte R
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

		if ((i >= 8) & (i < 16)) { //this means second byte G
			if (colors[0] & (0x80 >> (i - 8))) {
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

		if ((i >= 16) & (i < 24)) { //this means third byte B
			if (colors[2] & (0x80 >> (i - 16))) {
				buffer[i + led_index * BYTES_PER_LED * 8] = HIGH_BIT;
			} else {
				buffer[i + led_index * BYTES_PER_LED * 8] = LOW_BIT;
			}
		}

	}
	return 1;
}

bool done = false;

void matrix_test_secuential(rotation_t rotation){
	static uint8_t x_counter = 1, y_counter = 1;

	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	rgb_matrix_set_pixel(&rgbw_arr, LIMITS, x_counter, y_counter, rotation);
	if(x_counter == 8){
		x_counter = 1;
		if(y_counter == 8){
			y_counter = 1;
		}else{
			y_counter++;
		}
	}else{
		x_counter++;
	}
}

void matrix_draw_vertical_line(uint8_t x, uint8_t y1, uint8_t y2, rotation_t rotation){
	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	for(uint8_t i=y1; i<=y2; i++){
		rgb_matrix_set_pixel(&rgbw_arr, LIMITS, x, i, rotation);
	}
}

void matrix_test_pyramid(rotation_t rotation){
	static uint8_t iteration_x = 1, iteration_y = 1;
	static uint8_t rotation_diff = 0;

	matrix_draw_vertical_line(iteration_x, 1, iteration_y, rotation+rotation_diff);
	if(iteration_x >= 5){
		iteration_y--;
	}else{
		iteration_y++;
	}
	iteration_x++;
	if(iteration_x == 9){
		iteration_x = 1;
		iteration_y = 1;
		rotation_diff ++;
		if(rotation_diff == 4){
			rotation_diff = 0;
		}
	}
}


char test_matrix[9][8] = {
	{0, 1, 1, 0, 0, 1, 0, 1},
	{0, 1, 2, 3, 3, 2, 1, 2},
	{1, 2, 3, 4, 4, 2, 2, 2},
	{2, 4, 6, 5, 7, 6, 6, 3},
	{1, 3, 4, 4, 6, 5, 5, 3},
	{0, 1, 2, 3, 4, 4, 2, 1},
	{0, 0, 1, 1, 1, 2, 1, 0},
	{0, 0, 0, 1, 1, 1, 0, 0},
	{0, 0, 0, 1, 0, 0, 0, 0}
};

void matrix_test_vertical_levels(rotation_t rotation, show_mode_t mode){
	static uint8_t table = 0;
	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));
	for(uint8_t i=1; i<=MATRIX_SIZE; i++){
		if(mode == POINTS){
			rgb_matrix_set_pixel(&rgbw_arr, LIMITS, i, test_matrix[table][i-1]+1, rotation);
		}else if(mode == LINES){
			for(uint8_t j=1; j<=test_matrix[table][i-1]; j++){
				rgb_matrix_set_pixel(&rgbw_arr, LIMITS, i, j, rotation);
			}
		}
	}
	table++;
	if(table == 9){
		table = 0;
	}
}

int main(void) {
	HAL_Init();

	SystemClock_Config();

	MX_GPIO_Init();
	MX_DMA_Init();
	MX_TIM1_Init();

	rgb_matrix_clear_buffer(&rgbw_arr, sizeof(rgbw_arr));

	bool mode = true;

	while (1) {

		matrix_test_vertical_levels(ROTATION_90, POINTS);
		HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, &rgbw_arr,sizeof(rgbw_arr));

		HAL_Delay(100);
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	htim1.Instance->CCR1 = 0;
	done = true;
}

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 90 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
