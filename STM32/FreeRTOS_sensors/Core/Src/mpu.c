/*
 * mpu.c
 *
 *  Created on: Jan 16, 2026
 *      Author: Kiran Duriseti
 */

#include "main.h"
#include <string.h>
#include <stdint.h>
#include "i2c.h"
#include "stm32L4xx_hal.h"

#define MPU_add 0x68 << 1 //logic low
#define ACK 0x75 //who am I
#define power 0x6B
#define power_2 0x6C
#define sample_rate 0x19
#define gyro_config 0x1B
#define acc_config 0x1C
#define multi_acc 0x3B
#define multi_gyro 0x43
#define acc_normalize 16384.0
#define gyro_normalize 131.0
#define config 0x1A

int32_t gyro_bias_x = 0;
int32_t gyro_bias_y = 0;
int32_t gyro_bias_z = 0;

uint8_t data;
uint8_t check; //check = 0x68

extern I2C_HandleTypeDef hi2c2;

HAL_StatusTypeDef ok;

uint8_t mpu_read(uint8_t reg){
	data = 0;
	HAL_I2C_Mem_Read(&hi2c2, MPU_add, reg, 1, &data, 1, 100);
	return data;
}

void mpu_write(uint8_t reg, uint8_t d){
	HAL_I2C_Mem_Write(&hi2c2, MPU_add, reg, 1, &d, 1, 100);
}

void mpu_sleep(void) {
	//data = 64;
	//HAL_I2C_Mem_Write(&hi2c2, MPU_add, power, 1, &Data, 1, HAL_MAX_DELAY);
	mpu_write(power, 64);
}

void mpu_wake(void) {
	//data = 0;
	//HAL_I2C_Mem_Write(&hi2c2, MPU_add, power, 1, &Data, 1, HAL_MAX_DELAY);
	mpu_write(power, 0);
}

HAL_StatusTypeDef mpu_read_gyro(int16_t *x, int16_t *y, int16_t *z) {
	uint8_t rx[6];

	HAL_StatusTypeDef ok = HAL_I2C_Mem_Read(&hi2c2, MPU_add, multi_gyro, 1, rx, 6, 100);

	if (ok != HAL_OK) return ok;
	*x = (int16_t)(rx[0] << 8 | rx[1]);
	*y = (int16_t)(rx[2] << 8 | rx[3]);
	*z = (int16_t)(rx[4] << 8 | rx[5]);
	return HAL_OK;
}

void mpu_calibrate_gyro(uint16_t samples)
{
    int64_t sx = 0, sy = 0, sz = 0;
    for (uint16_t i = 0; i < samples; i++) {
        int16_t rx, ry, rz;
        mpu_read_gyro(&rx, &ry, &rz);

        sx += rx;
        sy += ry;
        sz += rz;
        HAL_Delay(2);
    }

    gyro_bias_x = (int32_t)(sx / samples);
    gyro_bias_y = (int32_t)(sy / samples);
    gyro_bias_z = (int32_t)(sz / samples);
}

HAL_StatusTypeDef mpu_read_acc(int16_t *x, int16_t *y, int16_t *z) {
	uint8_t rx[6];

	HAL_StatusTypeDef ok = HAL_I2C_Mem_Read(&hi2c2, MPU_add, multi_acc, 1, rx, 6, 100);

	if (ok != HAL_OK) return ok;

	int16_t raw_x, raw_y, raw_z;
	raw_x = (int16_t)(rx[0] << 8 | rx[1]);
	raw_y = (int16_t)(rx[2] << 8 | rx[3]);
	raw_z = (int16_t)(rx[4] << 8 | rx[5]);
	*x = raw_x;
	*y = raw_y;
	*z = raw_z;

	return HAL_OK;
}

void mpu_init_gyro(void){
	//only enables gyro readings (disable acc and temp to save power)
	//HAL_I2C_Mem_Read(&hi2c2, MPU_add, ACK, 1, &check, 1, HAL_MAX_DELAY);
	check = mpu_read(ACK);
	//mpu_wake();
	mpu_write(power, 0x09); //disabled tempurature sensor
	HAL_Delay(50);

	mpu_write(power_2, 0x38); //diable acc

	//mpu_write(config, 0);
	mpu_write(config, 0x3);
	mpu_write(sample_rate, 0x0);

	//mpu_write(gyro_config, 0);
	mpu_write(gyro_config, 0x18);

	mpu_calibrate_gyro(20);
}

void mpu_init(void){
	//HAL_I2C_Mem_Read(&hi2c2, MPU_add, ACK, 1, &check, 1, HAL_MAX_DELAY);
	ok = HAL_I2C_IsDeviceReady(&hi2c2, (0x68<<1), 2, 100);
	check = mpu_read(ACK);
	mpu_wake();
	HAL_Delay(50);
	//data = 0; //8KHz
	//HAL_I2C_Mem_Write(&hi2c2, MPU_add, sample_rate, 1, &data, 1, HAL_MAX_DELAY);
	mpu_write(sample_rate, 0);
	//DLPF == 0, therfore 8KHz sample rate
	//data = 0; //+- 2g
	//HAL_I2C_Mem_Write(&hi2c2, MPU_add, acc_config, 1, &data, 1, HAL_MAX_DELAY);
	mpu_write(acc_config, 0);
	//data = 0; //+- 250 degrees/s
	//HAL_I2C_Mem_Write(&hi2c2, MPU_add, acc_config, 1, &data, 1, HAL_MAX_DELAY);
	mpu_write(gyro_config, 0);
}
