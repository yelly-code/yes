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
#include "stm32f1xx_it.h"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// ---------------- 可调参数 ----------------
#define PID_PERIOD_MS       20      // PID 采样周期 20ms
#define SERVO_STEP_DEFAULT  5       // 默认步进
#define TARGET_LIMIT        45.0f   // PID 模式目标角度限幅
#define PID_OUT_LIMIT       45.0f   // PID 输出限幅，舵机行程 = 中位 ±45°
#define SERVO_YAW_CENTER    90      // Yaw 舵机中位
#define SERVO_PITCH_CENTER  90      // Pitch 舵机中位
#define SERVO_YAW_SIGN      (+1)    // Yaw 方向
#define SERVO_PITCH_SIGN    (+1)    // Pitch 方向1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

MPU6050_t mpu6050_date;                 // 融合后的姿态数据
volatile uint8_t  rx_byte;              // 串口单字节缓冲（
volatile uint32_t rx_cnt = 0;           //

volatile uint8_t  pid_mode = 0;         // 0: 角度模式 1: PID 模式
PID_t    pid_yaw, pid_pitch;            // 两个轴的 PID
volatile float yaw_target   = 0.0f;     // PID 目标角度
volatile float pitch_target = 0.0f;
uint32_t pid_tick = 0;                  // 上次 PID 计算时刻
static   int16_t servo_angle[2] = {SERVO_YAW_CENTER, SERVO_PITCH_CENTER};   // 两轴当前舵机角度 [0]=PA9 [1]=PA11

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Servo_SetPulse(uint16_t pulse_us, uint32_t channel);
void Servo_SetAngle(uint8_t angle, uint32_t channel);
void Servo_Jog(uint8_t axis, int16_t delta);

// 按微秒脉宽写 CCR：TIM1 计数频率 100kHz，1 个计数 = 10us，周期 2000 = 20ms(50Hz)
void Servo_SetPulse(uint16_t pulse_us, uint32_t channel)
{
  // 超出 0.5~2.5ms 会把舵机顶在机械限位上堵转发热
  if (pulse_us < 500)  pulse_us = 500;
  if (pulse_us > 2500) pulse_us = 2500;
  __HAL_TIM_SET_COMPARE(&htim1, channel, (uint16_t)(pulse_us / 10));
}

// 按角度写舵机：0°->0.5ms  90°->1.5ms  180°->2.5ms
void Servo_SetAngle(uint8_t angle, uint32_t channel)
{
  if (angle > 180) angle = 180;
  Servo_SetPulse((uint16_t)(500 + (uint32_t)angle * 2000 / 180), channel);
}

// 增量转动：手动模式下由串口命令直接调用，角度自动限幅 0~180
void Servo_Jog(uint8_t axis, int16_t delta)
{
  // 方向取反统一在这里做，两个模式共用 SERVO_xxx_SIGN
  if (axis == 0) delta = (int16_t)(delta * SERVO_YAW_SIGN);
  else           delta = (int16_t)(delta * SERVO_PITCH_SIGN);

  int16_t a = servo_angle[axis] + delta;
  if (a < 0)   a = 0;
  if (a > 180) a = 180;

  servo_angle[axis] = a;
  Servo_SetAngle((uint8_t)a, (axis == 0) ? TIM_CHANNEL_2 : TIM_CHANNEL_4);
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
  OLED_Init();              // 0.96 寸 OLED
  MPU6050_Init(&hi2c1);     // 六轴陀螺仪

  // 避免上电第一个周期没有脉冲
  Servo_SetPulse(1500, TIM_CHANNEL_2);
  Servo_SetPulse(1500, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);


  PID_Init(&pid_yaw,   0.8f, 0.05f, 0.05f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
  PID_Init(&pid_pitch, 0.8f, 0.05f, 0.05f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
  yaw_target   = 0.0f;
  pitch_target = 0.0f;
  pid_tick     = HAL_GetTick();

  servo_angle[0] = SERVO_YAW_CENTER;
  servo_angle[1] = SERVO_PITCH_CENTER;
  Servo_SetAngle(SERVO_YAW_CENTER,   TIM_CHANNEL_2);
  Servo_SetAngle(SERVO_PITCH_CENTER, TIM_CHANNEL_4);


  __HAL_UART_CLEAR_OREFLAG(&huart2);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    MPU6050_Read_All(&hi2c1, &mpu6050_date);   // 更新 KalmanAngleX/Y


    uint32_t now = HAL_GetTick();
    if (pid_mode && (now - pid_tick >= PID_PERIOD_MS))
    {
      float dt = (now - pid_tick) / 1000.0f;
      pid_tick = now;
      if (dt > 0.2f) dt = 0.2f;                // 主循环卡顿后防止微分项爆炸

      // 抗积分饱和
      if (pid_yaw.out   >=  PID_OUT_LIMIT - 0.1f || pid_yaw.out   <= -PID_OUT_LIMIT + 0.1f) pid_yaw.integral   = 0.0f;
      if (pid_pitch.out >=  PID_OUT_LIMIT - 0.1f || pid_pitch.out <= -PID_OUT_LIMIT + 0.1f) pid_pitch.integral = 0.0f;

      float out_y = PID_Update(&pid_yaw,   yaw_target,   mpu6050_date.KalmanAngleX, dt);
      float out_p = PID_Update(&pid_pitch, pitch_target, mpu6050_date.KalmanAngleY, dt);

      // 舵机中位 ± PID 修正量
      servo_angle[0] = (int16_t)(SERVO_YAW_CENTER   + SERVO_YAW_SIGN   * out_y);
      servo_angle[1] = (int16_t)(SERVO_PITCH_CENTER + SERVO_PITCH_SIGN * out_p);

      Servo_SetAngle((uint8_t)servo_angle[0], TIM_CHANNEL_2);
      Servo_SetAngle((uint8_t)servo_angle[1], TIM_CHANNEL_4);
    }

    // ---------------- OLED 显示 ----------------
    OLED_ShowString(0, 0, "YAW:  ", OLED_6X8);
    OLED_ShowFloatNum(40, 0, mpu6050_date.KalmanAngleX, 2, 2, OLED_6X8);
    OLED_ShowString(0, 12, "PITCH:", OLED_6X8);
    OLED_ShowFloatNum(40, 12, mpu6050_date.KalmanAngleY, 2, 2, OLED_6X8);
    OLED_ShowString(0, 24, pid_mode ? "MODE:PID" : "MODE:MAN", OLED_6X8);
    OLED_ShowString(0, 36, "A:", OLED_6X8);
    OLED_ShowNum(16, 36, servo_angle[0], 3, OLED_6X8);
    OLED_ShowString(56, 36, "B:", OLED_6X8);
    OLED_ShowNum(72, 36, servo_angle[1], 3, OLED_6X8);
    OLED_ShowString(0, 48, "T:", OLED_6X8);
    OLED_ShowSignedNum(16, 48, (int32_t)yaw_target, 2, OLED_6X8);
    OLED_ShowSignedNum(48, 48, (int32_t)pitch_target, 2, OLED_6X8);
    OLED_ShowString(0, 56, "RX:", OLED_6X8);
    OLED_ShowNum(24, 56, rx_cnt, 5, OLED_6X8);
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


// 单独发一个小写 p 切换 PID 模式
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t state  = 0;   // 0=等字母 1=等十位 2=等个位
  static uint8_t letter = 0;
  static uint8_t value  = 0;

  if (huart->Instance != USART2) return;

  uint8_t c = rx_byte;
  rx_cnt++;

  if (state == 0)
  {
    if (c == 'p' || c == 'P')
    {
      pid_mode = !pid_mode;
      PID_Reset(&pid_yaw);      // 切模式时清掉积分
      PID_Reset(&pid_pitch);
      yaw_target   = 0.0f;
      pitch_target = 0.0f;
      pid_tick     = HAL_GetTick();
    }
    else if (c == 'a' || c == 'd' || c == 'w' || c == 's')
    {
      letter = c;
      value  = 0;
      state  = 1;
    }
  }
  else if (state == 1)
  {
    if (c >= '0' && c <= '9') { value = c - '0'; state = 2; }
    else state = 0;
  }
  else
  {
    if (c >= '0' && c <= '9')
    {
      uint8_t step = (uint8_t)(value * 10 + (c - '0'));
      if (step == 0) step = SERVO_STEP_DEFAULT;
      if (step > 90) step = 90;

      if (pid_mode)
      {
        // 闭环模式：命令改的是“目标角度”，舵机由 PID 驱动
        if      (letter == 'a') yaw_target   += step;
        else if (letter == 'd') yaw_target   -= step;
        else if (letter == 'w') pitch_target += step;
        else                    pitch_target -= step;

        if (yaw_target   >  TARGET_LIMIT) yaw_target   =  TARGET_LIMIT;
        if (yaw_target   < -TARGET_LIMIT) yaw_target   = -TARGET_LIMIT;
        if (pitch_target >  TARGET_LIMIT) pitch_target =  TARGET_LIMIT;
        if (pitch_target < -TARGET_LIMIT) pitch_target = -TARGET_LIMIT;
      }
      else
      {
        // 手动模式：命令直接增量驱动舵机，范围 0~180°
        uint8_t axis = (letter == 'a' || letter == 'd') ? 0 : 1;
        int16_t dir  = (letter == 'a' || letter == 'w') ? +1 : -1;
        Servo_Jog(axis, (int16_t)(dir * step));
      }
    }
    state = 0;
  }

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
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
  * @brief  Reports the source file name and the source line number
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
