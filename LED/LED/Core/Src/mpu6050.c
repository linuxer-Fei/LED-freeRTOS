/*
 * mpu6050.c
 *
 *  Created on: May 22, 2022
 *      Author: SSS
 */
#include "mpu6050.h"
#include "i2c.h"
#include "stm32f1xx_hal_i2c.h"

uint8_t mpu6050_id = 0;

static uint8_t mpu6060_i2c_read_Byte(uint8_t I2C_Addr,uint8_t reg, uint8_t *buf)
{
	return HAL_I2C_Mem_Read(&hi2c1, I2C_Addr, reg,
		  I2C_MEMADD_SIZE_8BIT, buf, 1, 100);
}

static uint8_t mpu6060_i2c_read_block(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	return HAL_I2C_Mem_Read(&hi2c1, dev, reg,
			I2C_MEMADD_SIZE_8BIT, data, length, 200);
}

static uint8_t mpu6060_i2c_write_Byte(uint8_t I2C_Addr, uint8_t reg, uint8_t data)
{
	uint8_t  *pData;
	pData = &data;

	return HAL_I2C_Mem_Write(&hi2c1, I2C_Addr, reg,
			I2C_MEMADD_SIZE_8BIT, pData, 1, 100);
}

static uint8_t mpu6060_i2c_write_block(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
	  return HAL_I2C_Mem_Write(&hi2c1, dev, reg,
			  I2C_MEMADD_SIZE_8BIT, data, length, 200);
}

static void mpu6050_set_clock_source(uint8_t source)
{
	mpu6060_i2c_write_Byte(SLAVE_ADDR, MPU6050_RA_PWR_MGMT_1, source);
}

static void mpu6050_set_gyro_range(uint8_t range)
{
	mpu6060_i2c_write_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_CONFIG, range);
}

static void mpu6050_set_acc_range(uint8_t range)
{
	mpu6060_i2c_write_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_CONFIG, range);
}

static void mpu6050_set_sleep_mode(uint8_t mode)
{
	mpu6060_i2c_write_Byte(SLAVE_ADDR, MPU6050_RA_PWR_MGMT_1, mode);
}

static void mpu6050_get_who_am_i(void)
{
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_WHO_AM_I, &mpu6050_id);
}

float mpu6050_get_temperature(void)
{
	  float result;
	  uint8_t temp_h;
	  uint8_t temp_l;

	  mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_TEMP_OUT_H, &temp_h);
	  mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_TEMP_OUT_L, &temp_l);

	  result = (temp_h<<8)+ temp_l;
	  if (result > 32768)
		  result -= 65536;

	  result = 36.53 + result / 340;
	  return result;
}

void mpu6050_get_gyro_acc_data(struct gyro_acc_data *data)
{
	float x = 0;
	float y = 0;
	float z = 0;
	uint8_t temp_h = 0;
	uint8_t temp_l = 0;

	if (!data)
		return;

	//gyro
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_XOUT_H, &temp_h);
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_XOUT_L, &temp_l);
	x = (temp_h<<8)+ temp_l;
	if (x > 32768)
		x -= 65536;

	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_YOUT_H, &temp_h);
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_YOUT_L, &temp_l);
	y = (temp_h<<8)+ temp_l;
	if (y > 32768)
		y -= 65536;

	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_ZOUT_H, &temp_h);
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_GYRO_ZOUT_L, &temp_l);
	z = (temp_h<<8)+ temp_l;
	if (z > 32768)
		z -= 65536;

	data->gyro_x = x / (16.40 * 57.30);
	data->gyro_y = y / (16.40 * 57.30);
	data->gyro_z = z / (16.40 * 57.30);

	//acc
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_XOUT_H, &temp_h);
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_XOUT_L, &temp_l);
	x = (temp_h<<8)+ temp_l;
	if (x > 32768)
		x -= 65536;

	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_YOUT_H, &temp_h);
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_YOUT_L, &temp_l);
	y = (temp_h<<8)+ temp_l;
	if (y > 32768)
		y -= 65536;

	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_ZOUT_H, &temp_h);
	mpu6060_i2c_read_Byte(SLAVE_ADDR, MPU6050_RA_ACCEL_ZOUT_L, &temp_l);
	z = (temp_h<<8)+ temp_l;
	if (z > 32768)
		z -= 65536;

	data->acc_x = x / 16348 * 9.8;
	data->acc_y = y / 16348 * 9.8;
	data->acc_z = z / 16348 * 9.8;
}

uint8_t get_mpu6050_id(void)
{
	return mpu6050_id;
}

void mpu6050_init(void)
{
	mpu6050_set_clock_source(MPU6050_CLOCK_PLL_YGYRO);
	mpu6050_set_gyro_range(MPU6050_GYRO_FS_2000);
	mpu6050_set_acc_range(MPU6050_ACCEL_FS_2);
	mpu6050_set_sleep_mode(0);

	mpu6050_get_who_am_i();
}
