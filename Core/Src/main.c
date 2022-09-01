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
#include "edge_detection.h"
#include "coeffs.h"
#include "lora.h"
#include "math.h"
#include "spi_hal.h"
#include "AS3933.h"
#include "lis2hh12_reg.h"
#include "checksum.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint32_t dw_id;
	uint32_t dw_ref;
	uint16_t w_steps;
	uint16_t w_crc;
	uint8_t b_time_of_rest;
	uint8_t b_time_of_stand;
	uint8_t b_rest_stand_cnt;
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
static uint8_t ab_pedometer_data[k_PEDOMETER_DATA_SIZE] =
{
	[0] = k_SOH,
	[k_PEDOMETER_DATA_SIZE - 1] = k_EOT,
};

static step_attr_t s_step = 
{
	.dw_id = k_ID,
	.dw_ref = k_REF,
};

static stmdev_ctx_t dev_ctx;
static lis2hh12_pin_int1_route_t int1_route;

static float af_acc_mg[3];
static int16_t ai_acc_raw[3];

static uint8_t b_acc_wake_up_src;
static uint8_t b_status_reg;
static uint8_t b_rtc_wakeup_cnt;

static uint8_t whoamI, rst;
static bool o_tilted = false;

static uint8_t b_standing_watch_cnt = 0;
static uint8_t b_rest_watch_cnt = 0;
static bool	o_standing = true;
static bool o_resting = false;
static bool o_rest = false;
static bool o_stand = false;

uint32_t k_X_ACCEL_PRESET1=				850;
uint32_t k_X_ACCEL_PRESET2=				860;
uint32_t k_STEP_DEBOUNCE=					3700;

uint8_t b_ask_data = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void lora_init_sequence(void);
void acc_init_sequence(void);
void as3933_init_sequence(void);
void update_pedometer_data(void);
void reset_vars_after_15min(void);
static void set_stop_mode(void);
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void delay_ms(uint32_t ms);
static void delay_Xms(void);
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
	LL_GPIO_SetOutputPin(CS_ACC_GPIO_Port, CS_ACC_Pin);
	LL_GPIO_SetOutputPin(CS_LORA_GPIO_Port, CS_LORA_Pin);
	LL_GPIO_ResetOutputPin(CS_AS3933_GPIO_Port, CS_AS3933_Pin);
	
	acc_init_sequence();
	delay_ms(100);
	lora_init_sequence();
	delay_ms(100);
	as3933_init_sequence();
	delay_ms(100);
	
	ab_pedometer_data[1] = (uint8_t)(s_step.dw_id);
	ab_pedometer_data[2] = (uint8_t)(s_step.dw_id >> 8);
	ab_pedometer_data[3] = (uint8_t)(s_step.dw_id >> 16);
	ab_pedometer_data[4] = (uint8_t)(s_step.dw_id >> 24);
	ab_pedometer_data[5] = (uint8_t)(s_step.dw_ref);
	ab_pedometer_data[6] = (uint8_t)(s_step.dw_ref >> 8);
	ab_pedometer_data[7] = (uint8_t)(s_step.dw_ref >> 16);
	ab_pedometer_data[8] = (uint8_t)(s_step.dw_ref >> 24);
	
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_LPTIM1;
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
	LL_GPIO_ResetOutputPin(CS_ACC_GPIO_Port, CS_ACC_Pin);
	spi_transmit(SPI1, &reg, 1);
	spi_transmit(SPI1, bufp, len);
	LL_GPIO_SetOutputPin(CS_ACC_GPIO_Port, CS_ACC_Pin);
	return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
	reg |= 0x80;
	LL_GPIO_ResetOutputPin(CS_ACC_GPIO_Port, CS_ACC_Pin);
	spi_transmit(SPI1, &reg, 1);
	spi_transmit_receive(SPI1, bufp, bufp, len);
	LL_GPIO_SetOutputPin(CS_ACC_GPIO_Port, CS_ACC_Pin);
	return 0;
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	SystemClock_Config();
	
	b_status_reg = 0;
	while(!READ_BIT_POS(b_status_reg, 3))
	{
		lis2hh12_read_reg(&dev_ctx, LIS2HH12_STATUS, &b_status_reg, 1);
	}

	lis2hh12_acceleration_raw_get(&dev_ctx, ai_acc_raw);
	af_acc_mg[0] = lis2hh12_from_fs4g_to_mg(ai_acc_raw[0]);

	if (af_acc_mg[0] < k_X_ACCEL_PRESET1)
	{
		o_tilted = true;
	}
	else if (af_acc_mg[0] > k_X_ACCEL_PRESET1) // to prevent unstable range X_ACCEL_PRESET2
	{
		o_tilted = false;
	}
	if (o_tilted && !s_step.o_step_detected) // watch resting
	{
		o_resting = true;
		o_standing = false;
	}
	else if (!o_tilted && s_step.o_step_detected) // watch standing
	{
		o_standing = true;
		o_resting = false;
	}

	if (o_resting && !o_standing)	// RESTING
	{
		++b_rest_watch_cnt;
		if(b_rest_watch_cnt >= k_REST_EXACT)
		{
			++s_step.b_time_of_rest;
			if(o_stand) // detection sit-down
			{
				o_rest = true;
			}
		}
	}
	
	if(o_standing && !o_resting)  // STANDING
	{
		++b_standing_watch_cnt;
		if (b_standing_watch_cnt >= k_STAND_EXACT)
		{
			++s_step.b_time_of_stand;
			o_stand = true;
		}
	}
	
	if(o_stand && o_rest)
	{
		o_stand = false;
		o_rest = false;
		++s_step.b_rest_stand_cnt;
	}

	s_step.o_step_detected = false;
	
	++b_rtc_wakeup_cnt;
	// LORA send
	if (b_rtc_wakeup_cnt >= k_SEND_LORA) // 18 is 3 min : 90 is 15 min
	{
		// 	send LORA information every 15 min acc.to this callback period
		if(b_rest_watch_cnt >= 8){s_step.b_time_of_rest += 8;}
		update_pedometer_data();
		lora_send_msg(&radio, ab_pedometer_data, k_PEDOMETER_DATA_SIZE);
		reset_vars_after_15min();
	}
}

void callback_acc(void)
{
	static uint8_t	_10s = 0;
	SystemClock_Config();
	LPTIM1->CNT = 0;
	
	while (1)
	{
		lis2hh12_read_reg(&dev_ctx, LIS2HH12_STATUS, &b_status_reg, 1);
		
		if (LL_GPIO_IsInputPinSet(ACC_INT1_GPIO_Port, ACC_INT1_Pin))
		{
			break;
		}
		
		if (READ_BIT_POS(b_status_reg, 3))
		{
			lis2hh12_acceleration_raw_get(&dev_ctx, ai_acc_raw);
			af_acc_mg[0] = lis2hh12_from_fs4g_to_mg(ai_acc_raw[0]);

			if (af_acc_mg[0] < k_X_ACCEL_PRESET1)
			{
				o_tilted = true;
			}
			else if (af_acc_mg[0] > k_X_ACCEL_PRESET2) // to prevent unstable range X_ACCEL_PRESET2
			{
				o_tilted = false;
				(void) edge_detection(ED_STEP, o_tilted);
			}

			//***************************************************************************************//
			// Step detection
			if (TON(TON_STEP, o_tilted, LPTIM1->CNT, k_STEP_DEBOUNCE))
			{
				TON(TON_STEP, 0, 0, 0);
				if (edge_detection(ED_STEP, o_tilted))
				{
					++s_step.w_steps;
					s_step.o_step_detected = true;
				}
			}
		}
		
		if (TON_16U(TON_WATCH_RTC_WAKEUP, 1, LPTIM1->CNT, k_WATCH_RTC_WAKEUP))
		{
			TON_16U(TON_WATCH_RTC_WAKEUP, 0, 0, 0);
			++_10s;
			if(_10s >= 50)
			{
				_10s = 0;
				++b_rtc_wakeup_cnt;
				if(b_rtc_wakeup_cnt >= k_SEND_LORA)
				{
					update_pedometer_data();
					lora_send_msg(&radio, ab_pedometer_data, k_PEDOMETER_DATA_SIZE);
					reset_vars_after_15min();
				}
			}
		}
	}
}


void callback_as3933_wake(void)
{	
	uint8_t i = 0;
	bool o_dat_pin = false;
	bool o_cl_dat_pin = false;

	TON_16U(TON_AS3933_TIMEOUT, 0, 0, 0);
	LPTIM1->CNT = 0;
	b_ask_data = 0;
	
	while(i < 8)
	{
		o_dat_pin 	 = LL_GPIO_IsInputPinSet(DAT_GPIO_Port, DAT_Pin);
		o_cl_dat_pin = LL_GPIO_IsInputPinSet(CL_DAT_GPIO_Port, CL_DAT_Pin);
			
		if (edge_detection(ED_CL_DAT, o_cl_dat_pin))
		{
			o_dat_pin?SET_BIT_POS(b_ask_data, i++):RESET_BIT_POS(b_ask_data, i++);
		}
		
		if(TON_16U(TON_AS3933_TIMEOUT, 1, LPTIM1->CNT, k_AS3933_TIMEOUT))
		{
			TON_16U(TON_AS3933_TIMEOUT, 0, 0, 0);
			break;
		}
	}
			
	spi1_user_init_high_1edge();
	as3933_set_listening_mode();
	spi1_user_init_low_1edge();
}

void callback_btn(void)
{
	SystemClock_Config();
	delay_Xms();
	update_pedometer_data();
	lora_send_msg(&radio, ab_pedometer_data, k_PEDOMETER_DATA_SIZE);
}

void update_pedometer_data(void)
{		
		ab_pedometer_data[9] = (uint8_t)(s_step.w_steps);
		ab_pedometer_data[10] = (uint8_t)(s_step.w_steps >> 8);
		
		ab_pedometer_data[11] = (uint8_t)(s_step.b_time_of_rest);
		ab_pedometer_data[12] = (uint8_t)(s_step.b_rest_stand_cnt);
	
		s_step.w_crc = crc_16(ab_pedometer_data + 1, 12);
		
		ab_pedometer_data[13] = (uint8_t)(s_step.w_crc);
		ab_pedometer_data[14] = (uint8_t)(s_step.w_crc >> 8);
}

void reset_vars_after_15min(void)
{
		CLEAR_STRUCT(s_step);
		// reset local variables
		b_rtc_wakeup_cnt = 0;
		b_standing_watch_cnt = 0;
		b_rest_watch_cnt = 0;
		o_stand = false;
		o_rest = false;
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
	lis2hh12_dev_id_get(&dev_ctx, &whoamI);
	if (whoamI != LIS2HH12_ID)
		while (1);

	/* Restore default configuration */
	lis2hh12_dev_reset_set(&dev_ctx, PROPERTY_ENABLE);

	do
	{
		lis2hh12_dev_reset_get(&dev_ctx, &rst);
	} while (rst);

	/* Start device configuration. */
	lis2hh12_i2c_interface_set(&dev_ctx, LIS2HH12_I2C_DISABLE);
	lis2hh12_spi_mode_set(&dev_ctx, LIS2HH12_SPI_4_WIRE);
	
	lis2hh12_xl_axis_set(&dev_ctx, (lis2hh12_xl_axis_t){.xen = PROPERTY_ENABLE});

	lis2hh12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
	lis2hh12_xl_data_rate_set(&dev_ctx, LIS2HH12_XL_ODR_50Hz);
	lis2hh12_xl_full_scale_set(&dev_ctx, LIS2HH12_4g);
	
	lis2hh12_auto_increment_set(&dev_ctx, LIS2HH12_ENABLE);
	
	lis2hh12_pin_mode_set(&dev_ctx, LIS2HH12_PUSH_PULL);
	lis2hh12_pin_polarity_set(&dev_ctx, LIS2HH12_ACTIVE_HIGH);
	
	/*****		ACTIVITY INACTIVITY		*****/
	lis2hh12_act_threshold_set(&dev_ctx, 8); // Full scale / 128 [mg] = 31.25 mg
	lis2hh12_act_duration_set(&dev_ctx, 4); // (8/50) * 4 = 640 ms

	lis2hh12_pin_int1_route_get(&dev_ctx, &int1_route);
	int1_route.int1_inact = PROPERTY_ENABLE;
	lis2hh12_pin_int1_route_set(&dev_ctx, int1_route);
}


void as3933_init_sequence(void)
{
	spi1_user_init_high_1edge();
	as3933_write_Rx(AS3933_R0, AS3933_ON_OFF | AS3933_EN_A3 | AS3933_DAT_MASK);
	as3933_write_Rx(AS3933_R1, AS3933_AGC_UD | AS3933_EN_XTAL | AS3933_EN_MANCH | AS3933_EN_WPAT);
	as3933_write_Rx(AS3933_R3, AS3933_SET_FS_ENV(2) | AS3933_SET_FS_SCL(5));
	as3933_write_Rx(AS3933_R4, AS3933_SET_T_OFF(3) | AS3933_SET_D_RES(0) | AS3933_SET_GR(0));
	as3933_write_Rx(AS3933_R7, AS3933_SET_T_HBIT(10));
	as3933_write_Rx(AS3933_R19, AS3933_CAP_CH3_8pF);	
	as3933_reset_rssi();
	as3933_set_listening_mode();
	spi1_user_init_low_1edge();
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

static void delay_Xms(void)
{
	//3700 -> 100 ms
	LPTIM1->CNT = 0;
	while (LPTIM1->CNT < 3700){}
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
