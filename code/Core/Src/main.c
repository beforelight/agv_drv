/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "iwdg.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drv_beep.h"
#include "drv_car.h"
#include "drv_digi.h"
#include "drv_imu_instance.h"
#include "drv_shell.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_RNG_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  // MX_USART2_UART_Init();
  __disable_irq();
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
  beep_on(500);
  shell_init();
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
  car_init(5);
  __enable_irq();
  imu_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  char ch;
  while (1) {
    if (FIFO_UsedSize(&shell_fifo)) {
      FIFO_Get(&shell_fifo, &ch, 1);
      shellHandler(&lshell, ch);
    }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim8) {
    // 1ms period
    static uint32_t tick;
    tick++;
    digi_tick_callback();
    if (tick % 2 == 0) {
      // 2ms period
      HAL_ADCEx_InjectedStart_IT(&hadc1); // trigger
    }
    if (tick % 5 == 0) {
      // 5ms period
      HAL_IWDG_Refresh(&hiwdg);
      car.Recive_Timer += 5;
      if (car.Recive_Timer > 1000) {
        // clear speed_dst when diconnected from ros node
        car.Recive_Timer = 0;
        if (
            fabsf(car.speed_dst[0]) > 0.001f || fabsf(car.speed_dst[1]) > 0.001f
            || fabsf(car.speed_dst[2]) > 0.001f || fabsf(car.speed_dst[3]) > 0.001f) {
          car.speed_dst[0] = 0;
          car.speed_dst[1] = 0;
          car.speed_dst[2] = 0;
          car.speed_dst[3] = 0;
          beep_on(1000); // 1s beep on
        }
      }
      car_speed_get();
      car_speed_ctrl();
      led_increase(5);
      if (car.spd_ctrl_identify_mode) {
        car.upload_frame.fdata[0] = car.speed[0];
        car.upload_frame.fdata[1] = car.speed[1];
        car.upload_frame.fdata[2] = car.speed[2];
        car.upload_frame.fdata[3] = car.speed[3];
        car.upload_frame.fdata[0 + 4] = car.u_out[0];
        car.upload_frame.fdata[1 + 4] = car.u_out[1];
        car.upload_frame.fdata[2 + 4] = car.u_out[2];
        car.upload_frame.fdata[3 + 4] = car.u_out[3];
        HAL_UART_Transmit(&huart3, (uint8_t *) &car.upload_frame, sizeof(Frame_t), ~0);
      } else if (car.upload_flag == 1) {
        car.upload_frame.fdata[0] = car.speed[0];
        car.upload_frame.fdata[1] = car.speed[1];
        car.upload_frame.fdata[2] = car.speed[2];
        car.upload_frame.fdata[3] = car.speed[3];
        car.upload_frame.fdata[0 + 4] = car.speed_dst[0];
        car.upload_frame.fdata[1 + 4] = car.speed_dst[1];
        car.upload_frame.fdata[2 + 4] = car.speed_dst[2];
        car.upload_frame.fdata[3 + 4] = car.speed_dst[3];
        HAL_UART_Transmit(&huart3, (uint8_t *) &car.upload_frame, sizeof(Frame_t), ~0);
      }
    }
    if (tick % 50 == 0) {
      beep_increase(50);
      if (imu0 != NULL && IMU_IsOpen(imu0)) {
        IMU_ReadSensorNonBlocking(imu0);
        /*IMU_ReadSensorBlocking(imu0);
        imu_read_callback();*/
      }
      car_upload_to_ros_node();
    }
    if (tick % 2000 == 0) {
      // 2s period
      led_on(100);

      // vbus warning
      if (car.vbus_warning)
        beep_on(700);
    }
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  static uint32_t u_verf;
  static uint32_t u_vbus;
  static float vref = 1.21f;
  static float vbus = 12.6f;
#define ADC_LEFT_SHIFT (6)
  if (hadc == &hadc1) {
    u_verf = (u_verf * ((1 << ADC_LEFT_SHIFT) - 1)
              + (HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) << ADC_LEFT_SHIFT))
             >> ADC_LEFT_SHIFT;
    u_vbus = (u_vbus * ((1 << ADC_LEFT_SHIFT) - 1)
              + ((HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2)
                  + HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3)
                  + HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4) * 2)
                 << (ADC_LEFT_SHIFT - 2)))
             >> ADC_LEFT_SHIFT;

    static uint32_t tick;
    tick++;
    if (tick % (1 << ADC_LEFT_SHIFT) == 0) {
      vref = (float) u_verf * 3.3f / (float) (4096 << ADC_LEFT_SHIFT);
      vbus = (float) u_vbus * 1.21f * 11.0f / (float) u_verf; // 11.0f = divider ratio
      digi_set_num_no_refresh(vbus * 10.0f + 0.5f);           // 0.5f for rounding
      car.vbus = vbus;

      // vbus warning
      if (car.vbus > 5.4f) {
        if ((!car.vbus_warning) && car.vbus < 10.0)
          car.vbus_warning = true;
        if (car.vbus_warning && car.vbus > 10.2)
          car.vbus_warning = false;
      } else {
        car.vbus_warning = false;
      }
    }
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c == &hi2c1) {
    imu_read_callback();
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
  while (1) {
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
