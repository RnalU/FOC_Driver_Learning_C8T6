/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "math.h"
#include "as5600.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CLOCK_FRE 72000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int fputc(int ch,FILE *f)
{
	uint8_t temp[1]={ch};
	HAL_UART_Transmit(&huart1,temp,1,2);
	return ch;
}

float fre = 30000;
uint32_t us_counter_period_tim2 = 0;
	
float angle_rad = 0;
float angle_grad = 0;
float angle_rad_with_track = 0;
float ratation_count = 0;
float angle_rad_velocity = 0;

float target_angle = 60;
float target_velocity = 5;
float Kp = 0.2;
float Uq = 0;
float Dir = -1;

char t_data[50] = "";
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
float voltage_limit=1.5;
float voltage_power_supply=12;
float shaft_angle=0,open_loop_timestamp=0;
float zero_electric_angle=0,Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0,dc_a=0,dc_b=0,dc_c=0;
int period=100;
int pole_pairs=7;
int index1=0;
uint16_t raw1,raw2,raw3;
uint16_t adc_values[3];
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == (SysTick_CTRL_COUNTFLAG_Msk));
}
uint32_t getCurrentMicros(void)
{
  /* Ensure COUNTFLAG is reset by reading SysTick control and status register */
  LL_SYSTICK_IsActiveCounterFlag();
  uint32_t m = HAL_GetTick();
  const uint32_t tms = SysTick->LOAD + 1;
  __IO uint32_t u = tms - SysTick->VAL;
  if (LL_SYSTICK_IsActiveCounterFlag()) {
    m = HAL_GetTick();
    u = tms - SysTick->VAL;
  }
  return (m * 1000 + (u * 1000) / tms);
}

float _normalizeAngle(float angle) {
    // 将角度调整到 -2*M_PI 到 2*M_PI 的范围内
	  float M_PI=3.1415926;
    angle = angle < -2*M_PI ? angle + 2*M_PI * ((int)(-angle/(2*M_PI)) + 1) 
                            : angle;
    angle = angle > 2*M_PI ? angle - 2*M_PI * ((int)(angle/(2*M_PI))) 
                            : angle;

    // 如果角度小于 0，则加上 2π 使其归一化到 0 到 2π 之间
    return angle >= 0 ? angle : angle + 2*M_PI;
}

float _electricalAngle() {
  return _normalizeAngle(angle_rad * pole_pairs - zero_electric_angle);
}

void setPwm(float Ua, float Ub, float Uc) {

//	// 限制上限
	Ua = _constrain(Ua, 0.0f, voltage_limit);
	Ub = _constrain(Ub, 0.0f, voltage_limit);
	Uc = _constrain(Uc, 0.0f, voltage_limit);
	// 计算占空??
	// 限制占空比从0??1
	dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 0.8f );
	dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 0.8f );
	dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 0.8f );

	//写入PWM到PWM 0 1 2 通道
	TIM1->CCR1 = (uint32_t) roundf(dc_a*period);
	TIM1->CCR2 = (uint32_t) roundf(dc_b*period);
	TIM1->CCR3 = (uint32_t) roundf(dc_c*period);

}

void setPhaseVoltage(float Uq,float Ud, float angle_el) {
  // 帕克逆变??
  Ualpha =  -Uq*sin(angle_el);
  Ubeta =   Uq*cos(angle_el);

  // 克拉克???变??
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}


//??环???度函数
float velocityOpenloop(float target_velocity){
//	uint32_t now_us = getCurrentMicros();
 	uint32_t now_us = HAL_GetTick();
//  Provides a tick value in microseconds.

  //计算当前每个Loop的运行时间间??
//    float Ts = (now_us - open_loop_timestamp) * 1e-3f;
	float Ts=5E-3f;

  // 通过乘以时间间隔和目标???度来计算需要转动的机械角度，存储在 shaft_angle 变量中???
  //在此之前，还??要对轴角度进行归??化，以确保其值在 0 ?? 2π 之间??
	
//  shaft_angle = _normalizeAngle(shaft_angle + target_velocity*Ts);
	shaft_angle = _normalizeAngle(angle_rad);
	
	//以目标???度?? 10 rad/s 为例，如果时间间隔是 1 秒，则在每个循环中需要增?? 10 * 1 = 10 弧度的角度变化量，才能使电机转动到目标???度??
  //如果时间间隔?? 0.1 秒，那么在每个循环中??要增加的角度变化量就?? 10 * 0.1 = 1 弧度，才能实现相同的目标速度??
  //因此，电机轴的转动角度取决于目标速度和时间间隔的乘积??

  // Uq is not related to voltage limit
  float Uq = 5;

  // setPhaseVoltage(Uq,  0, _electricalAngle(shaft_angle, pole_pairs));

//  open_loop_timestamp = now_us;  //用于计算下一个时间间??

  return Uq;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)  // 1us * 65535
	{
		us_counter_period_tim2 += 65536;
		
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	// us级计数器
	HAL_TIM_Base_Start_IT(&htim2);
	motor_init();
	__HAL_TIM_SET_PRESCALER(&htim1, (int)((CLOCK_FRE / fre) / 100));
	u8 i=0; 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	angle_rad = i2c_AS5600_get_angle();
	angle_rad_with_track = i2c_AS5600_get_angle_with_track();
	angle_rad_velocity = as5600_get_velocity();
  angle_rad_velocity = lowpassfilter(angle_rad_velocity);
	angle_grad = rad_to_grad(angle_rad);
	
//	sprintf(t_data, "rad_angle=%.2f, grad_angle=%.2f", angle_rad, angle_grad);
//	printf("rad_angle=%.2f, grad_angle=%.2f\r\n", angle_rad, angle_grad);
	  printf("velocity=%.3f\r\n", angle_rad_velocity);
	// velocityOpenloop(1);
	  
	
	// 位置闭环控制	
	// Uq = _constrain(Kp * (target_angle- angle_grad), -6, 6);
	// setPhaseVoltage(Uq, 0, _electricalAngle());
	  
	// 速度闭环控制
	Uq = _constrain(Kp * (-(target_velocity - angle_rad_velocity) * 180 / PI), -6, 6);
	setPhaseVoltage(Uq, 0, _electricalAngle());
	                        
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
