/*
 * mpu.h
 *
 *  Created on: Jan 16, 2026
 *      Author: Kiran Duriseti
 */

#ifndef INC_MPU_H_
#define INC_MPU_H_

#include "stm32L4xx_hal.h"

void mpu_init_gyro(void);
void mpu_init(void);
void mpu_sleep(void);
void mpu_wake(void);
HAL_StatusTypeDef mpu_read_gyro(int16_t *x, int16_t *y, int16_t *z);
HAL_StatusTypeDef mpu_read_acc(int16_t *x, int16_t *y, int16_t *z);

extern int32_t gyro_bias_x;
extern int32_t gyro_bias_y;
extern int32_t gyro_bias_z;

#endif /* INC_MPU_H_ */
