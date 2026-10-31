/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "tim.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "mpu6050.h"
#include"stm32f1xx_it.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"

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

MPU6050_t mpu6050_date;
uint8_t rx_byte;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Servo_SetAngle(uint8_t angle, uint32_t channel)
{
  // 1. 限制角度范围，防止输入错误导致舵机卡死
  if (angle > 180) angle = 180;

  // 2. 套用公式：算出比较值
  // (2000.0 / 180.0) 其实就是 11.11
  uint16_t ccr_value = (uint16_t)(50 + (uint32_t)angle * 200 / 180);
  // 3. 把算出的值写入定时器寄存器，舵机就会动
  __HAL_TIM_SET_COMPARE(&htim1, channel, ccr_value);
}
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();//开启OLED
  MPU6050_Init(&hi2c1);//开启MPU6050传感

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  Servo_SetAngle(0,   TIM_CHANNEL_2);  HAL_Delay(1000);
  Servo_SetAngle(90,  TIM_CHANNEL_2);  HAL_Delay(1000);
  Servo_SetAngle(180, TIM_CHANNEL_2);  HAL_Delay(1000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);


  while (1)
  {
    MPU6050_Read_All(&hi2c1, &mpu6050_date);//读取MPU6050数据
    OLED_ShowString(0, 0, "ROLL: ",OLED_6X8);
    OLED_ShowFloatNum(40,0, mpu6050_date.KalmanAngleX,2,2,OLED_6X8);
    OLED_ShowString(0, 20, "PITCH: ",OLED_6X8);
    OLED_ShowFloatNum(40,20, mpu6050_date.KalmanAngleY,2,2,OLED_6X8);
    OLED_Update();
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

/* USER CODE BEGIN 4 */
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   // 在这里写你的业务逻辑，这里绝对安全，不会被 CubeMX 清空！
//   if (huart->Instance == USART2) {
//     if (SG90[0] == 'w') {
//       Servo_SetAngle(SG90[1] - '0', TIM_CHANNEL_4);
//     }
//     else if (SG90[0] == 's') {
//       Servo_SetAngle(SG90[1] - '0', TIM_CHANNEL_4);
//     }
//     else if (SG90[0] == 'a') {
//   Servo_SetAngle(SG90[1] - '0', TIM_CHANNEL_2);
//     }
//     else if (SG90[0] == 'd') {
//   Servo_SetAngle(SG90[1] - '0', TIM_CHANNEL_2);
//     }
//     HAL_UART_Receive_IT(&huart2, SG90, 2);
//   }
// }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t state = 0;      // 0=等字母 1=等十位 2=等个位
  static uint8_t ch_cmd = 0;     // 保存 TIM_CHANNEL_2 / TIM_CHANNEL_4
  static uint8_t value  = 0;

  if (huart->Instance == USART2)
  {
    uint8_t c = rx_byte;

    if (state == 0)
    {
      if (c == 'w' || c == 's')      { ch_cmd = TIM_CHANNEL_4; state = 1; value = 0; }
      else if (c == 'a' || c == 'd') { ch_cmd = TIM_CHANNEL_2; state = 1; value = 0; }
      /* \r \n 空格 等一律忽略 */
    }
    else if (state == 1)
    {
      if (c >= '0' && c <= '9') { value = c - '0'; state = 2; }
      else { state = 0; }
    }
    else /* state == 2 */
    {
      if (c >= '0' && c <= '9')
      {
        value = value * 10 + (c - '0');
        if (value > 180) value = 180;
        Servo_SetAngle(value, ch_cmd);
      }
      state = 0;   /* 两位数字结束，后面的 \r\n 自然被丢掉 */
    }

    HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
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
