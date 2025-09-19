/* Practical 3F0 code
 * Modified by: Jesse Adams and Nstika Ntshebe
 * Date: 19 September 2025 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdint.h>
#define MAX_ITER 100
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE BEGIN PV */
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)
	volatile uint32_t fixed_checksums[5];
	volatile uint32_t fixed_exec[5];
	volatile uint32_t execution_time = 0;
	volatile uint64_t checksum = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define variables you think you might need
// - Performance timing variables (e.g execution time, throughput, pixels per second, clock cycles)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);

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

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();


  /* USER CODE BEGIN 2 */


  //int sizes[] = {128, 160, 192, 244, 256};
  //int max_iters[] = {100, 250, 500, 750, 1000};
  int heights[] = {256, 512, 800, 1024, 1920};
  int widths[] = {256, 512, 800, 1024, 1080};
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

  /* USER CODE END 2 */
for (int i = 0; i < 5; i++)
 {

	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	  uint32_t start_ms = HAL_GetTick(); // record start time


	  checksum = calculate_mandelbrot_fixed_point_arithmetic(heights[i], widths[i], MAX_ITER); // call mandelbrot and store in checksum
	  fixed_checksums[i] = checksum;
	  uint32_t end_ms = HAL_GetTick(); // record end time

	  fixed_exec[i] = end_ms - start_ms; // calculate execution time

	  HAL_Delay(200);
	}



	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
	__BKPT(0);
  }

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3
                           PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Function signatures you defined previously , implement them here

/* Fixed point mandelbrot sum */
 uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;

   // Fixed variables
	#define FIXED_SHIFT 12
	#define FIXED_ONE (1 << FIXED_SHIFT)
	int x0, y0, xi, yi, x_temp, iteration;

	 for (int y = 0; y < height; y++) {
		 y0 = ((y * (2 * FIXED_ONE)) / height) - (FIXED_ONE * 1);
		 for (int x = 0; x < width; x++) {
			 x0 = ((x * (3.5 * FIXED_ONE)) / width) - (FIXED_ONE * 2.5);
			 xi = 0;
			 yi = 0;
			 iteration = 0;
			 while (iteration < max_iterations && (((xi * xi) >> FIXED_SHIFT) + ((yi * yi) >> FIXED_SHIFT) < (4 << FIXED_SHIFT))) {
				 x_temp = ((xi * xi) >> FIXED_SHIFT) - ((yi * yi) >> FIXED_SHIFT) + x0;
				 yi = (((2 * xi * yi) >> FIXED_SHIFT) + y0);
				 xi = x_temp;
				 iteration++;
			 }
			 mandelbrot_sum += iteration;
		 }

	 }
    return mandelbrot_sum;

}


 /* Double precision mandelbrot sum */
  uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
     uint64_t mandelbrot_sum = 0;
     //TODO: Complete the function implementation
     for (int y = 0; y < height; y++) {
         	for (int x = 0; x < width; x++) {
         		double x0 = (x / (double)width * 3.5) - 2.5;
         		double y0 = (y / (double)height * 2.0) - 1.0;
         		double xi = 0;
         		double yi = 0;
         		int iteration = 0;
         		while (iteration < max_iterations && ((xi*xi + yi*yi) < 4.0)) {
         			double x_temp = xi*xi - yi*yi;
         			yi = 2.0 * xi * yi + y0;
         			xi = x_temp + x0;
         			iteration++;
         		}
         		mandelbrot_sum += iteration;
         	}
         }
     return mandelbrot_sum;
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
#ifdef USE_FULL_ASSERT
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
