/* USER CODE BEGIN Header */
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "freertos.h"
#include "queue.h"
#include "pca9685_driver.h"
#include "lcd_1602_driver.h"
#include "mpu6050_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 64

#define BASE_MAX_ANGLE 180
#define BASE_MIN_ANGLE 0

#define HINGE1_MAX_ANGLE 110
#define HINGE1_MIN_ANGLE 80

#define HINGE2_MAX_ANGLE 110
#define HINGE2_MIN_ANGLE 80

#define HINGE3_MAX_ANGLE 110
#define HINGE3_MIN_ANGLE 80

#define TWIST_MAX_ANGLE 130
#define TWIST_MIN_ANGLE 30

#define HAND_MAX_ANGLE 120
#define HAND_MIN_ANGLE 40

#define STATIC_ADJUSTMENT 3
#define MIN_ANALOG_VALUE 20

#define LEVEL_TOLERANCE 5

#define BASE_PCA_PIN 0
#define HINGE1_PCA_PIN 3
#define HINGE2_PCA_PIN 6
#define HINGE3_PCA_PIN 9
#define TWIST_PCA_PIN 12
#define HAND_PCA_PIN 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* Definitions for controlMotors */
osThreadId_t controlMotorsHandle;
const osThreadAttr_t controlMotors_attributes = {
  .name = "controlMotors",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for displayAngle */
osThreadId_t displayAngleHandle;
const osThreadAttr_t displayAngle_attributes = {
  .name = "displayAngle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
QueueHandle_t joystick_data_queue;

uint8_t base_angle = 90; // Z-axis
uint8_t hinge1_angle = 90; // X-axis
uint8_t hinge2_angle = 90; // Y-aixs
uint8_t hinge3_angle = 90; // Y-hat
uint8_t twist_angle = 0; // X-hat
uint8_t hand_value = 0; // Slider

char mpu_angle_string[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
void StartControlMotors(void *argument);
void StartDisplayAngle(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t clamp(int8_t value, int8_t min_val, int8_t max_val)
{
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

int8_t map_value(int8_t value, int8_t min_in, int8_t max_in, int8_t min_out, int8_t max_out)
{
    // Scale x from [in_min, in_max] to [out_min, out_max]
    return (value - min_in) * (max_out - min_out) / (max_in - min_in) + min_out;
}

void update_motor_pos(char *data) // Data = "x:y:z:slider:xhat:yhat"
{
	// X-axis | Hinge 1
	char *token = strtok(data, ":");
	if (token != NULL)
	{
		int8_t x_adjust = atoi(token);
		if (abs(x_adjust) >= MIN_ANALOG_VALUE) hinge1_angle = clamp(hinge1_angle += x_adjust, HINGE1_MIN_ANGLE, HINGE1_MAX_ANGLE);
	}

	// Y-axis | Hinge 2
	token = strtok(NULL, ":");
	if (token != NULL)
	{
		int8_t y_adjust = atoi(token);
		if (abs(y_adjust) >= MIN_ANALOG_VALUE) hinge2_angle = clamp(hinge2_angle += y_adjust, HINGE2_MIN_ANGLE, HINGE2_MAX_ANGLE);
	}

	// Z-axis | Base
	token = strtok(NULL, ":");
	if (token != NULL)
	{
		int8_t z_adjust = atoi(token);
		if (abs(z_adjust) >= MIN_ANALOG_VALUE) base_angle = clamp(base_angle += z_adjust, BASE_MIN_ANGLE, BASE_MAX_ANGLE);
	}

	// Slider | Hand
	token = strtok(NULL, ":");
	if (token != NULL)
	{
		hand_value = map_value(atoi(token), 0, 100, HAND_MIN_ANGLE, HAND_MAX_ANGLE);
	}

	// X-hat | Twist
	token = strtok(NULL, ":");
	if (token != NULL)
	{
		int8_t xhat_adjust = atoi(token) * STATIC_ADJUSTMENT;
		twist_angle = clamp(twist_angle += xhat_adjust, TWIST_MIN_ANGLE, TWIST_MAX_ANGLE);
	}

	// Y-hat | Hinge 3
	token = strtok(NULL, ":");
	if (token != NULL)
	{
		int8_t yhat_adjust = atoi(token) * STATIC_ADJUSTMENT;
		hinge3_angle = clamp(hinge3_angle += yhat_adjust, HINGE3_MIN_ANGLE, HINGE3_MAX_ANGLE);
	}

	pca9685_setservo_angle(BASE_PCA_PIN, base_angle);
	pca9685_setservo_angle(HINGE1_PCA_PIN, hinge1_angle);
	pca9685_setservo_angle(HINGE2_PCA_PIN, hinge2_angle);
	pca9685_setservo_angle(HINGE3_PCA_PIN, hinge3_angle);
	pca9685_setservo_angle(TWIST_PCA_PIN, twist_angle);
	pca9685_setservo_angle(HAND_PCA_PIN, hand_value);
}

void display_angle_lcd(int8_t angle)
{
	if (abs(angle) <= LEVEL_TOLERANCE)
	{
		sprintf(mpu_angle_string, "Level!");
		clear_lcd();
		set_lcd_cursor(0, 0);
		send_lcd_string(mpu_angle_string);
	}
	else
	{
		sprintf(mpu_angle_string, "%d°", angle);
		clear_lcd();
		set_lcd_cursor(0, 0);
		send_lcd_string(mpu_angle_string);
	}
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
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  mpu6050_init(&hi2c1);

  joystick_data_queue = xQueueCreate(5, RX_BUFFER_SIZE);
  if (joystick_data_queue == NULL) {
      // Handle queue creation failure
      while (1);
  }


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of controlMotors */
  controlMotorsHandle = osThreadNew(StartControlMotors, NULL, &controlMotors_attributes);

  /* creation of displayAngle */
  displayAngleHandle = osThreadNew(StartDisplayAngle, NULL, &displayAngle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	static char rx_buffer[RX_BUFFER_SIZE];
	static uint8_t buffer_index = 0;

	char received_char;

	HAL_UART_Receive(&huart2, (uint8_t*)&received_char, 1, HAL_MAX_DELAY);

	if (received_char == '\n')
	{
		rx_buffer[buffer_index] = '\0';
		buffer_index = 0;

		xQueueSend(joystick_data_queue, &rx_buffer, portMAX_DELAY);
	}
	else if (buffer_index < RX_BUFFER_SIZE - 1)
	{
		rx_buffer[buffer_index++] = received_char;
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartControlMotors */
/**
  * @brief  Function implementing the controlMotors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartControlMotors */
void StartControlMotors(void *argument)
{
	/* USER CODE BEGIN 5 */
	char joystick_data[RX_BUFFER_SIZE];

	/* Infinite loop */
	for(;;)
	{
		if (xQueueReceive(joystick_data_queue, &joystick_data, portMAX_DELAY) == pdPASS) update_motor_pos(joystick_data);
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartDisplayAngle */
/**
* @brief Function implementing the displayAngle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayAngle */
void StartDisplayAngle(void *argument)
{
	/* USER CODE BEGIN StartDisplayAngle */
	float accelerometer_data[3];
	int8_t angle;

	/* Infinite loop */
	for(;;)
	{
		mpu6050_get_accel(accelerometer_data);

		float x = accelerometer_data[0];
		float y = accelerometer_data[1];
		float z = accelerometer_data[2];

		angle = (int8_t)(atan2(z, sqrt(x * x + y * y)) * 180.0 / M_PI );

		display_angle_lcd(angle);

		osDelay(100);
	}
	/* USER CODE END StartDisplayAngle */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
