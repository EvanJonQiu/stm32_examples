#ifndef __MPU6050_UTILS_H__
#define __MPU6050_UTILS_H__

#include <stdint.h>

uint8_t mpu6050_write_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t mpu6050_read_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data);
void mpu6050_get_ms(unsigned long *time);
void mpu6050_delay(uint32_t Delay);


#endif // __MPU6050_UTILS_H__

