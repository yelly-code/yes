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
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_PERIOD_MS      20      /* 20ms 跑一次 = 50Hz */
#define SERVO_STEP_DEFAULT 5       /* 串口按键默认步进 */
#define TARGET_LIMIT       90.0f   /* 目标角度限制 ±55°（和下面的输出限幅保持一致） */
#define PID_OUT_LIMIT      90.0f   /* PID 输出限幅：决定舵机行程 90±55 = 35~145° */
#define SERVO_YAW_SIGN     (+1)    /* 方向反了改 -1 */
#define SERVO_PITCH_SIGN   (+1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

MPU6050_t mpu6050_date;
volatile uint8_t  rx_byte;          /* 串口接收字节，中断里写，必须 volatile */
volatile uint32_t rx_cnt = 0;       /* 已收到的字节数，显示在 OLED 上便于排查 */
PID_t    pid_yaw, pid_pitch;        /* 左右轴、上下轴 */
volatile float yaw_target   = 0.0f; /* 目标：0° = 保持水平（中断里改，要 volatile） */
volatile float pitch_target = 0.0f;
uint32_t pid_tick     = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* a/d/w/s 后面跟 00 时使用的默认步进（度） */
#define SERVO_STEP_DEFAULT  5

/* 某个舵机方向反了，就把对应的改成 1 */
#define SERVO_CH2_INVERT    0      /* CH2 = PA9  （a / d 控制） */
#define SERVO_CH4_INVERT    0      /* CH4 = PA11 （w / s 控制） */

/* 直接按“微秒脉宽”写 CCR。本定时器 100kHz，1 个计数 = 10us，周期 2000 = 20ms(50Hz)。
   低于 500us 或高于 2500us 都会把舵机顶在机械限位上，表现就是嗡嗡抖/堵转发热。 */
void Servo_SetPulse(uint16_t pulse_us, uint32_t channel)
{
  if (pulse_us < 500)  pulse_us = 500;
  if (pulse_us > 2500) pulse_us = 2500;
  __HAL_TIM_SET_COMPARE(&htim1, channel, (uint16_t)(pulse_us / 10));
}

void Servo_SetAngle(uint8_t angle, uint32_t channel)
{
  // 限制角度范围，防止输入错误导致舵机卡死
  if (angle > 180) angle = 180;

  // 0° -> 0.5ms(500us)，90° -> 1.5ms(1500us)，180° -> 2.5ms(2500us)
  Servo_SetPulse((uint16_t)(500 + (uint32_t)angle * 2000 / 180), channel);
}

/* 两个轴当前角度：[0] = CH2/PA9，[1] = CH4/PA11 */
static int16_t servo_angle[2] = {90, 90};

/* 相对转动：delta 为正 = 角度增大，为负 = 角度减小，自动限幅 0~180 */
void Servo_Jog(uint8_t axis, int16_t delta)
{
#if SERVO_CH2_INVERT
  if (axis == 0) delta = -delta;
#endif
#if SERVO_CH4_INVERT
  if (axis == 1) delta = -delta;
#endif

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
  OLED_Init();//开启OLED
  MPU6050_Init(&hi2c1);//开启MPU6050传感

  /* 先写 1.5ms 的比较值，再开 PWM：避免上电第一个周期输出 0us（没脉冲）造成抖动 */
  Servo_SetPulse(1500, TIM_CHANNEL_2);
  Servo_SetPulse(1500, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  //pid初始化：先小增益止荡。Kp 1.2->0.6，Ki 0.50->0.10，Kd 0.02->0.05
  //等不再来回摆之后，再按 Kp -> Kd -> Ki 的顺序慢慢加回去
  PID_Init(&pid_yaw,   0.9f, 0.05f, 0.00f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
  PID_Init(&pid_pitch, 0.9f, 0.05f, 0.00f, -PID_OUT_LIMIT, PID_OUT_LIMIT);
  yaw_target   = 0.0f;
  pitch_target = 0.0f;
  pid_tick     = HAL_GetTick();

  /* 上电回中，不要停在 180°，否则舵机会一直顶在机械限位上堵转发热 */
  servo_angle[0] = 90;
  servo_angle[1] = 90;
  Servo_SetAngle(90, TIM_CHANNEL_2);
  Servo_SetAngle(90, TIM_CHANNEL_4);

  /* 先把上电期间可能积压的 ORE 清掉，再立刻开启串口接收（不要拖到最后） */
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    MPU6050_Read_All(&hi2c1, &mpu6050_date);//读取MPU6050数据

    uint32_t now = HAL_GetTick();                     /* ② 到点才算 PID */
    if (now - pid_tick >= PID_PERIOD_MS)
    {
      float dt = (now - pid_tick) / 1000.0f;
      pid_tick = now;
      if (dt > 0.2f) dt = 0.2f;                     /* 卡顿后防止微分爆掉 */

      /* 抗饱和：上一拍输出已经顶在限幅上，就先把积分清掉，避免积分锁死 */
      if (pid_yaw.out   >=  PID_OUT_LIMIT - 0.1f || pid_yaw.out   <= -PID_OUT_LIMIT + 0.1f) pid_yaw.integral   = 0.0f;
      if (pid_pitch.out >=  PID_OUT_LIMIT - 0.1f || pid_pitch.out <= -PID_OUT_LIMIT + 0.1f) pid_pitch.integral = 0.0f;

      float out_y = PID_Update(&pid_yaw,   yaw_target,   mpu6050_date.KalmanAngleX, dt);
      float out_p = PID_Update(&pid_pitch, pitch_target, mpu6050_date.KalmanAngleY, dt);

      /* ③ 90° 是舵机中位，PID 输出是"修正量" */
      Servo_SetAngle((uint8_t)(90 + SERVO_YAW_SIGN   * out_y), TIM_CHANNEL_2);
      Servo_SetAngle((uint8_t)(90 + SERVO_PITCH_SIGN * out_p), TIM_CHANNEL_4);

      /* 同步给 OLED 显示，A:/B: 显示的就是真正下发的角度 */
      servo_angle[0] = (int16_t)(90 + SERVO_YAW_SIGN   * out_y);
      servo_angle[1] = (int16_t)(90 + SERVO_PITCH_SIGN * out_p);
    }

    OLED_ShowString(0, 0, "YAW:  ",OLED_6X8);        /* 左右轴 */
    OLED_ShowFloatNum(40,0, mpu6050_date.KalmanAngleX,2,2,OLED_6X8);
    OLED_ShowString(0, 20, "PITCH: ",OLED_6X8);       /* 上下轴 */
    OLED_ShowFloatNum(40,20, mpu6050_date.KalmanAngleY,2,2,OLED_6X8);
    OLED_ShowString(0, 40, "RX:",OLED_6X8);          /* 调试用：收不到数就一直显示 0 */
    OLED_ShowNum(24, 40, rx_cnt, 6, OLED_6X8);
    OLED_ShowString(0, 48, "A:",OLED_6X8);           /* PA9  当前角度 */
    OLED_ShowNum(16, 48, servo_angle[0], 3, OLED_6X8);
    OLED_ShowString(56, 48, "B:",OLED_6X8);          /* PA11 当前角度 */
    OLED_ShowNum(72, 48, servo_angle[1], 3, OLED_6X8);
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
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  static uint8_t state  = 0;     // 0=等字母 1=等十位 2=等个位
  static uint8_t letter = 0;     // 保存 'a'/'d'/'w'/'s'
  static uint8_t value  = 0;

  if (huart->Instance == USART2)
  {
    uint8_t c = rx_byte;
    rx_cnt++;

    if (state == 0)
    {
      if (c == 'a' || c == 'd' || c == 'w' || c == 's')
      {
        letter = c;
        state = 1;
        value = 0;
      }
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
        if (value > 90) value = 90;               /* 一次最多走 90°，防止手滑 */

        /* 方案A：串口只改"目标角度"，舵机交给 PID 闭环，不再直接写舵机 */
        float step = (value == 0) ? (float)SERVO_STEP_DEFAULT : (float)value;

        if      (letter == 'a') yaw_target   += step;
        else if (letter == 'd') yaw_target   -= step;
        else if (letter == 'w') pitch_target += step;
        else                    pitch_target -= step;   /* 's' */

        if (yaw_target   >  TARGET_LIMIT) yaw_target   =  TARGET_LIMIT;
        if (yaw_target   < -TARGET_LIMIT) yaw_target   = -TARGET_LIMIT;
        if (pitch_target >  TARGET_LIMIT) pitch_target =  TARGET_LIMIT;
        if (pitch_target < -TARGET_LIMIT) pitch_target = -TARGET_LIMIT;
      }
      state = 0;   /* 两位数字结束，后面的 \r\n 自然被丢掉 */
    }

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);                       // 清 ORE/FE/NE
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);  // 立刻重新开收
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
