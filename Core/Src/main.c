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
#include "lptim.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "soft_timer.h"
#include "macro.h"
#include "utils.h"
#include "coeffs.h"
#include "lora.h"
#include "ism330dhcx_reg.h"
#include "math.h"
#include "spi_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint16_t w_steps;
	uint16_t w_lie_down;
	uint16_t w_standing;
	uint16_t w_lie_down_cnt;
	bool o_possible_lie_down;
	bool o_exact_lie_down;
	bool o_possible_standing;
	bool o_exact_standing;
	bool o_step_detected;
} step_attr_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static lora_t radio;
static char ac_pedometer_data[k_PEDOMETER_DATA_SIZE] =
{
	[0] = k_SOH,
	[k_PEDOMETER_DATA_SIZE - 1] = k_EOT,
};

static step_attr_t s_step;
static rising_edge_detection_t s_step_red;
static ton_t s_step_ton;

static stmdev_ctx_t dev_ctx;
static ism330dhcx_pin_int2_route_t int2_route;
static ism330dhcx_pin_int1_route_t int1_route;

static float af_acc_mg[3];
static int16_t ai_acc_raw[3];

static uint8_t b_acc_wake_up_src;
static uint8_t b_status_reg;

static uint8_t whoamI, rst;
static bool o_tilted = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lora_init_sequence(void);
void acc_init_sequence(void);
static void set_stop_mode(void);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void delay_ms(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Tilt = acos( Gz / sqrt(Gx2+Gy2+Gz2) ) -> for small angles
// Tilt = atan2( sqrt(Gx2 + Gy2) , Gz ) no need to check if Gz=0 -> for -45 ... +45
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
	MX_SPI1_Init();
	MX_RTC_Init();
	MX_LPTIM1_Init();
	/* USER CODE BEGIN 2 */
	LL_GPIO_SetOutputPin(CS_IMU_GPIO_Port, CS_IMU_Pin);
	LL_GPIO_SetOutputPin(CS_LORA_GPIO_Port, CS_LORA_Pin);
	
	lora_init_sequence();
	acc_init_sequence();
	
	set_stop_mode();
	/* USER CODE END 2 */

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV4;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC | RCC_PERIPHCLK_LPTIM1;
	PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	PeriphClkInit.LptimClockSelection = RCC_LPTIM1CLKSOURCE_LSI;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

static void set_stop_mode(void)
{
	HAL_SuspendTick();
	HAL_PWR_EnableSleepOnExit();
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
//	HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
	spi_transmit(SPI1, &reg, 1);
	spi_transmit(SPI1, bufp, len);
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_RESET);
	spi_transmit(SPI1, &reg, 1);
	spi_transmit_receive(SPI1, bufp, bufp, len);
	HAL_GPIO_WritePin(CS_IMU_GPIO_Port, CS_IMU_Pin, GPIO_PIN_SET);
	return 0;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	static uint8_t b_rtc_wakeup = 0;

	SystemClock_Config();

	ism330dhcx_acceleration_raw_get(&dev_ctx, ai_acc_raw);
	af_acc_mg[0] = ism330dhcx_from_fs4g_to_mg(ai_acc_raw[0]);

	if (af_acc_mg[0] < k_X_ACCEL_PRESET1)
	{
		o_tilted = true;
	}
	else if (af_acc_mg[0] > k_X_ACCEL_PRESET1) // to prevent unstable range X_ACCEL_PRESET2
	{
		o_tilted = false;
	}
	if (o_tilted && !s_step.o_step_detected) // watch lie-down
	{
		++s_step.w_lie_down;
	}
	else if (!o_tilted && s_step.o_step_detected) // watch standing
	{
		++s_step.w_standing;
	}

	if (s_step.w_lie_down >= k_LIE_DOWN_POSSIBLE)
	{
		s_step.o_possible_lie_down = true;
	}
	if (s_step.w_lie_down >= k_LIE_DOWN_EXACT)
	{
		s_step.o_exact_lie_down = true;
	}

	if (s_step.w_standing >= k_STANDING_POSSIBLE)
	{
		s_step.o_possible_standing = true;
	}
	if (s_step.w_standing >= k_STANDING_EXACT)
	{
		s_step.o_exact_standing = true;
	}

	s_step.o_step_detected = false;

	// LORA send
	if (b_rtc_wakeup == 90)
	{
// 	send LORA information every 15 min acc.to this callback period
		b_rtc_wakeup = 0;
		lora_send_msg(&radio, (uint8_t*) ac_pedometer_data, k_PEDOMETER_DATA_SIZE);
		CLEAR_STRUCT(s_step);
	}
	++b_rtc_wakeup;
}

void callback_acc(void)
{
	SystemClock_Config();
	HAL_SuspendTick();

	LPTIM1->CNT = 0;
	while (1)
	{
		ism330dhcx_read_reg(&dev_ctx, ISM330DHCX_WAKE_UP_SRC, &b_acc_wake_up_src, 1);
		ism330dhcx_read_reg(&dev_ctx, ISM330DHCX_STATUS_REG, &b_status_reg, 1);

		if (READ_BIT_POS(b_acc_wake_up_src, 4))
		{
			break;
		}
		if (READ_BIT_POS(b_status_reg, 0))
		{
			ism330dhcx_acceleration_raw_get(&dev_ctx, ai_acc_raw);
			af_acc_mg[0] = ism330dhcx_from_fs4g_to_mg(ai_acc_raw[0]);

			if (af_acc_mg[0] < k_X_ACCEL_PRESET1)
			{
				o_tilted = true;
			}
			else if (af_acc_mg[0] > k_X_ACCEL_PRESET2) // to prevent unstable range X_ACCEL_PRESET2
			{
				o_tilted = false;
				(void) o_rising_edge_detection(&s_step_red, o_tilted);
			}

			//***************************************************************************************//
			// Step detection
			if (TON(&s_step_ton, o_tilted, LPTIM1->CNT, k_STEP_DEBOUNCE))
			{
				TON(&s_step_ton, 0, LPTIM1->CNT, k_STEP_DEBOUNCE);
				if (o_rising_edge_detection(&s_step_red, o_tilted))
				{
					++s_step.w_steps;
					s_step.o_step_detected = true;
				}
			}
		}
	}
}

void lora_init_sequence(void)
{
	radio.Modulation = LORA;
	radio.COB = RFM98;
	radio.Frequency = 434000;
	radio.OutputPower = 17; //dBm
	radio.PreambleLength = 12;
	radio.FixedPktLength = false; //explicit header mode for LoRa
	radio.PayloadLength = k_PEDOMETER_DATA_SIZE;
	radio.CrcDisable = true;
	radio.SFSel = SF9;
	radio.BWSel = BW125K;
	radio.CRSel = CR4_5;
	lora_init(&radio);
	lora_standby();
}

void acc_init_sequence(void)
{
	dev_ctx.write_reg = platform_write;
	dev_ctx.read_reg = platform_read;
	dev_ctx.handle = SPI1;

	delay_ms(k_BOOT_TIME);
	ism330dhcx_device_id_get(&dev_ctx, &whoamI);
	if (whoamI != ISM330DHCX_ID)
		while (1);

	/* Restore default configuration */
	ism330dhcx_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do
	{
		ism330dhcx_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Start device configuration. */
	ism330dhcx_i2c_interface_set(&dev_ctx, ISM330DHCX_I2C_DISABLE);
	ism330dhcx_spi_mode_set(&dev_ctx, ISM330DHCX_SPI_4_WIRE);

	ism330dhcx_device_conf_set(&dev_ctx, PROPERTY_ENABLE);
	ism330dhcx_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	ism330dhcx_xl_data_rate_set(&dev_ctx, ISM330DHCX_XL_ODR_26Hz);
	ism330dhcx_xl_full_scale_set(&dev_ctx, ISM330DHCX_4g);

	/*****		ACTIVITY INACTIVITY		*****/

	ism330dhcx_wkup_ths_weight_set(&dev_ctx, ISM330DHCX_LSb_FS_DIV_64); // FS_XL / 64
	ism330dhcx_wkup_dur_set(&dev_ctx, 2); // (= val(max 3) * 1 / ODR_XL) = = 77 ms
	ism330dhcx_act_sleep_dur_set(&dev_ctx, 0); // 16 / ODR_XL for 0  --- 512 / ODR for other val (615 ms)
	ism330dhcx_wkup_threshold_set(&dev_ctx, 3); // val * (FS_XL / 64) 187.5 mg
	ism330dhcx_act_mode_set(&dev_ctx, ISM330DHCX_XL_12Hz5_GY_NOT_AFFECTED);

	ism330dhcx_pin_mode_set(&dev_ctx, ISM330DHCX_OPEN_DRAIN);
	ism330dhcx_pin_polarity_set(&dev_ctx, ISM330DHCX_ACTIVE_HIGH);

	ism330dhcx_pin_int2_route_get(&dev_ctx, &int2_route);
	int2_route.md2_cfg.int2_sleep_change = PROPERTY_ENABLE;
	ism330dhcx_pin_int2_route_set(&dev_ctx, &int2_route);
}
static void delay_ms(uint32_t ms)
{
	__IO uint32_t tmp = SysTick->CTRL;
	((void) tmp);

	while (ms)
	{
		if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
		{
			--ms;
		}
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
