#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "main.h"

#include <stdint.h>

#define MPU6050_I2C_ADDR 0x68

// register of MPU6050
#define MPU6050_SMPLRT_DIV 0x19
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_FIFO_EN 0x23
#define MPU6050_INT_PIN_CFG 0x37
#define MPU6050_INT_ENABLE 0x38
#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_PWR_MGMT_2 0x6C
#define MPU6050_FIFO_COUNTH 0x70
#define MPU6050_WHO_AM_I 0x75

uint8_t MPU6050_Read_Whoami(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);

uint8_t mpu_dmp_init(void);

uint8_t MPU6050_dmp_get_data(float *pitch, float *roll, float *yaw);

#endif // __MPU6050_H__
