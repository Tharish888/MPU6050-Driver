/*
 * mpu6050.c
 *
 *  Created on: Feb 17, 2026
 *      Author: B. Tharish
 */


#include "mpu6050.h"
#include <stdint.h>
#include "stm32f1xx_hal.h"

extern I2C_HandleTypeDef hi2c1;


MPU6050_config_t mpu6050_config;   //Accelerometer and gyroscope sensitivity calibration

static void i2c_MPU6050_write_reg(uint8_t reg, uint8_t *data, uint8_t num_of_bytes)
{
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_WRITE_ADDR , reg, 1, data, num_of_bytes, 1000);
}

static void i2c_MPU6050_read_reg(uint8_t reg, uint8_t *data, uint8_t num_of_bytes)
{
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_WRITE_ADDR , reg, 1, data, num_of_bytes, 1000);
}

static uint8_t accel_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Accelerometer_2G:
      reg_value = ACCEL_SCALE_2G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_2G;
      break;
    case MPU6050_Accelerometer_4G:
      reg_value = ACCEL_SCALE_4G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_4G;
      break;
    case MPU6050_Accelerometer_8G:
      reg_value = ACCEL_SCALE_8G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_8G;
      break;
    case MPU6050_Accelerometer_16G:
      reg_value = ACCEL_SCALE_16G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_16G;
      break;
    default:
      reg_value = ACCEL_SCALE_2G;
      mpu6050_config.accel_sensitivity = ACCEL_SENS_2G;
      break;
  }

  return reg_value;
}

static uint8_t gyro_sensitiviy_config(uint8_t config)
{
  uint8_t reg_value; // Value to be written to the register //

  // Based on the configuration chosen by the user //
  // Choose the sensitivity value and the register value for the MPU6050 //
  switch (config) {
    case MPU6050_Gyroscope_250_deg:
      reg_value = GYRO_SCALE_250_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_250_DEG;
      break;
    case MPU6050_Gyroscope_500_deg:
      reg_value = GYRO_SCALE_500_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_500_DEG;
      break;
    case MPU6050_Gyroscope_1000_deg:
      reg_value = GYRO_SCALE_1K_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_1K_DEG;
      break;
    case MPU6050_Gyroscope_2000_deg:
      reg_value = GYRO_SCALE_2K_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_2K_DEG;
      break;
    default:
      reg_value = GYRO_SCALE_250_DEG;
      mpu6050_config.gyro_sensitivity = GYRO_SENS_250_DEG;
      break;
  }

  return reg_value;
}

uint8_t MPU6050_init(uint8_t accel_config, uint8_t gyro_config, uint8_t sample_rate)
{
	uint8_t check;
	uint8_t writeValue;

	i2c_MPU6050_read_reg(WHO_AM_I_REG, &check, 1);       // Read the device ID of MPU6050 to check if it is in tally with the datasheet

	if (check == MPU6050_DEVICE_ID)
	{
		//Set the power mangement register to 0x00 to wake up the sensor and set the clock to 8 MHz internal oscillator
		writeValue = PWR_WAKE_UP;
		i2c_MPU6050_write_reg(PWR_MGMT_1_REG, &writeValue, 1);

		//Set the sample rate by writing the SMPLRT_DIV register
		writeValue = sample_rate;
		i2c_MPU6050_write_reg(SMPLRT_DIV_REG, &writeValue, 1);

		//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) ==> Sample rate = BK /(1+7) = 1KHz
		writeValue = accel_sensitiviy_config(accel_config);
//		writeValue = ACCEL_SENS_2G;
		i2c_MPU6050_write_reg(ACCEL_CONFIG_REG, &writeValue, 1);

		//Configure the Gyro
		writeValue =  gyro_sensitiviy_config(gyro_config);
//		writeValue =  GYRO_SCALE_250_DEG;
		i2c_MPU6050_write_reg(GYRO_CONFIG_REG, &writeValue, 1);

		//Configure the Digital Low Pass Filter: DLPF
		writeValue = 0x05;
		i2c_MPU6050_write_reg(DLPF_REG, &writeValue, 1);

		return MPU6050_SUCCESS;
	}
	return MPU6050_FAILURE;
}


void MPU6050_Read_Accel(MPU6050_t *mpu6050_struct)
{
	uint8_t readValues[6];

	i2c_MPU6050_read_reg(ACCEL_XOUT_H_REG, readValues, 6);


	mpu6050_struct->Ax_Acc_Rawvalue = (int16_t)((readValues[0] << 8) | readValues[1]);
	mpu6050_struct->Ay_Acc_Rawvalue = (int16_t)((readValues[2] << 8) | readValues[3]);
	mpu6050_struct->Az_Acc_Rawvalue = (int16_t)((readValues[4] << 8) | readValues[5]);

	mpu6050_struct->Ax = mpu6050_struct-> Ax_Acc_Rawvalue/(mpu6050_config.accel_sensitivity);
	mpu6050_struct->Ay = mpu6050_struct-> Ay_Acc_Rawvalue/(mpu6050_config.accel_sensitivity);
	mpu6050_struct->Az = mpu6050_struct-> Az_Acc_Rawvalue/(mpu6050_config.accel_sensitivity);

//	mpu6050_struct->Ax = mpu6050_struct-> Ax_Acc_Rawvalue/ACCEL_SENS_2G;
//	mpu6050_struct->Ay = mpu6050_struct-> Ay_Acc_Rawvalue/ACCEL_SENS_2G;
//	mpu6050_struct->Az = mpu6050_struct-> Az_Acc_Rawvalue/ACCEL_SENS_2G;
}

void MPU6050_Read_Gyro(MPU6050_t *mpu6050_struct)
{
  uint8_t readValues[6];

  // Read 6 BYTES of data starting from GYRO_XOUT_H register //
  i2c_MPU6050_read_reg(GYRO_XOUT_H_REG, readValues, 6);

  mpu6050_struct->Gx_Acc_Rawvalue = (int16_t) (readValues[0] << 8 | readValues[1]);
  mpu6050_struct->Gy_Acc_Rawvalue = (int16_t) (readValues[2] << 8 | readValues[3]);
  mpu6050_struct->Gz_Acc_Rawvalue = (int16_t) (readValues[4] << 8 | readValues[5]);

  // Convert the RAW values into into dps , so we have          //
  // to divide according to the Full scale value set in FS_SEL  //
  mpu6050_struct->Gx = mpu6050_struct->Gx_Acc_Rawvalue / (mpu6050_config.gyro_sensitivity);
  mpu6050_struct->Gy = mpu6050_struct->Gy_Acc_Rawvalue / (mpu6050_config.gyro_sensitivity);
  mpu6050_struct->Gz = (mpu6050_struct->Gz_Acc_Rawvalue / (mpu6050_config.gyro_sensitivity));

//  mpu6050_struct->Gx = mpu6050_struct->Gx_Acc_Rawvalue / GYRO_SENS_500_DEG;
//  mpu6050_struct->Gy = mpu6050_struct->Gy_Acc_Rawvalue / GYRO_SENS_500_DEG;
//  mpu6050_struct->Gz = mpu6050_struct->Gz_Acc_Rawvalue / GYRO_SENS_500_DEG;
}

