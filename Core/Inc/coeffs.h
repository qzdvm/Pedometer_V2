#ifndef COEFFS_H
#define COEFFS_H

#define k_BOOT_TIME      				5000
// this acceleretions matchs with tilt-sensor tilt on z axis.
// for other axises, imu must be match with tilt-sensor
#define k_X_ACCEL_PRESET1				700
#define k_X_ACCEL_PRESET2				715
#define k_STEP_DEBOUNCE					4440//201 ms 1 LSB = (1/37Khz) and 7437 = 201 ms (LPTIM1 set for 37KHz) , 5920 = 160 ms, 120 ms = 4440
#define k_REST_POSSIBLE			4
#define k_REST_EXACT				8
#define k_STAND_POSSIBLE		4
#define	k_STAND_EXACT				8

#define	k_SOH										0x01
#define k_EOT										0x04
#define k_PEDOMETER_DATA_SIZE		16

#define k_ID		12345678U
#define k_REF		123456U
#define k_REC 	0U

#define SENSOR_BUS 						hi2c1

#endif
