/*
 * mpu6050.h
 *
 *  Created on: Feb 17, 2026
 *      Author: B. Tharish
 */

#ifndef SENSORS_MPU6050_H_
#define SENSORS_MPU6050_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"

typedef enum
{
	MPU6050_ADDRESS	 = 0x68,
	SMPLRT_DIV_REG   = 0x19,
	DLPF_REG 		 = 0x1A,                        //Low Pass Filter
	GYRO_CONFIG_REG  = 0x1B,
	ACCEL_CONFIG_REG = 0x1C,
	ACCEL_XOUT_H_REG = 0x3B,
	TEMP_OUT_H_REG 	 = 0x41,
	GYRO_XOUT_H_REG  = 0x43,
	PWR_MGMT_1_REG   = 0x6B,
	WHO_AM_I_REG     = 0x75
}Mpu6050_reg_addrs;

typedef enum
{
  MPU6050_DataRate_8KHz = 0x00,     // Sample rate set to 8 kHz  //
  MPU6050_DataRate_4KHz = 0x01,     // Sample rate set to 4 kHz  //
  MPU6050_DataRate_2KHz = 0x03,     // Sample rate set to 2 kHz  //
  MPU6050_DataRate_1KHz = 0x07,     // Sample rate set to 1 kHz  //
  MPU6050_DataRate_500Hz = 0x0F,    // Sample rate set to 500 Hz //
  MPU6050_DataRate_250Hz = 0x1F,    // Sample rate set to 250 Hz //
  MPU6050_DataRate_125Hz = 0x3F,    // Sample rate set to 125 Hz //
  MPU6050_DataRate_100Hz = 0x4F,    // Sample rate set to 100 Hz //
} MPU6050_sample_rate_conf;

typedef enum
{
  MPU6050_Accelerometer_2G = 0x00,  // Range is +- 2G  //
  MPU6050_Accelerometer_4G = 0x01,  // Range is +- 4G  //
  MPU6050_Accelerometer_8G = 0x02,  // Range is +- 8G  //
  MPU6050_Accelerometer_16G = 0x03  // Range is +- 16G //
} MPU6050_Accelerometer_conf;

typedef enum
{
  MPU6050_Gyroscope_250_deg = 0x00,  // Range is +- 250 degrees/s  //
  MPU6050_Gyroscope_500_deg = 0x01,  // Range is +- 500 degrees/s  //
  MPU6050_Gyroscope_1000_deg = 0x02, // Range is +- 1000 degrees/s //
  MPU6050_Gyroscope_2000_deg = 0x03  // Range is +- 2000 degrees/s //
} MPU6050_Gyroscope_conf;

typedef struct{
	int16_t Ax_Acc_Rawvalue;
	int16_t Ay_Acc_Rawvalue;
	int16_t Az_Acc_Rawvalue;
	float Ax;
	float Ay;
	float Az;


	int16_t Gx_Acc_Rawvalue;
	int16_t Gy_Acc_Rawvalue;
	int16_t Gz_Acc_Rawvalue;
	float Gx;
	float Gy;
	float Gz;
}MPU6050_t;

typedef struct{

	float KalmanAngleRoll, KalmanAnglePitch;
	float Kalman1DOutput[];

}Kalman_t;

typedef struct MPU6050_configuration{
	float gyro_sensitivity;
	float accel_sensitivity;
  } MPU6050_config_t;



/* Defines -------------------------------------------------------------------*/

// I2C Related //
#define MPU6050_ADDR 		    0x68    // 7-Bit address //
#define MPU6050_WRITE_ADDR 		0xD0    // 8-Bit Write address //
#define MPU6050_READ_ADDR 		0xD1    // 8-Bit Read address //
#define MPU6050_DEVICE_ID 		0x68
#define HAL_I2C_TIMEOUT 	  	100

// MPU6050 Return codes //
#define MPU6050_SUCCESS   0
#define MPU6050_FAILURE   1

// Gyro sensitivities in g/s //
#define GYRO_SENS_250_DEG	  ((float) 131.0)
#define GYRO_SENS_500_DEG	  ((float) 65.5)
#define GYRO_SENS_1K_DEG	  ((float) 32.8)
#define GYRO_SENS_2K_DEG	  ((float) 16.4)

// Gyro register value for sensitivity //
#define GYRO_SCALE_250_DEG    0x00
#define GYRO_SCALE_500_DEG    0x08
#define GYRO_SCALE_1K_DEG     0x10
#define GYRO_SCALE_2K_DEG     0x18

// Accel sensitivities in g/s //
#define ACCEL_SENS_2G	  ((float) 16384.0)
#define ACCEL_SENS_4G	  ((float) 8192.0)
#define ACCEL_SENS_8G	  ((float) 4096.0)
#define ACCEL_SENS_16G	((float) 2048.0)

// Accel register value for sensitivity //
#define ACCEL_SCALE_2G    0x00
#define ACCEL_SCALE_4G    0x08
#define ACCEL_SCALE_8G    0x10
#define ACCEL_SCALE_16G   0x18

// Power Managlement Reg 1 Parameters //
#define PWR_WAKE_UP       0x00
#define ENABLE_SLEEP      0x40
#define ENABLE_CYCLE      0x20
#define TEMP_DIS          0x08    // Disable temperature sensor //


// Kalman calculation constants //
#define RAD_TO_DEG 57.295779513082320876798154814105



/* Functions -----------------------------------------------------------------*/

uint8_t MPU6050_init		(uint8_t accel_config, uint8_t gyro_config, uint8_t sample_rate);
void 	MPU6050_Read_Gyro		(MPU6050_t *mpu6050_struct);
void 	MPU6050_Read_Accel      (MPU6050_t *mpu6050_struct);

#endif /* SENSORS_MPU6050_H_ */
