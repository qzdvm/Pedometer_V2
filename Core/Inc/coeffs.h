#ifndef COEFFS_H
#define COEFFS_H

#define k_BOOT_TIME      				5000
// this acceleretions matchs with tilt-sensor tilt on z axis.
// for other axises, imu must be match with tilt-sensor
#define k_X_ACCEL_PRESET1				700
#define k_X_ACCEL_PRESET2				715
#define k_STEP_DEBOUNCE					7437//201 ms 1 LSB = (1/37Khz) and 7437 = 201 ms (LPTIM1 set for 37KHz)
#define k_LIE_DOWN_POSSIBLE			4
#define k_LIE_DOWN_EXACT				8
#define k_STANDING_POSSIBLE			4
#define	k_STANDING_EXACT				8
#define k_SYSTEM_CORE_CLOCK			16000000
#define k_TO_ms						16000

#define SENSOR_BUS 						hi2c1

#endif
