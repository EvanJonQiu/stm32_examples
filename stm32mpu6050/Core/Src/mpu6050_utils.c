#include "mpu6050_utils.h"

#include "main.h"

extern I2C_HandleTypeDef hi2c1;

uint8_t mpu6050_write_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);  // 写入数据
  if (ret != HAL_OK) {
    return 1;
  }
  return 0;
}

uint8_t mpu6050_read_reg(uint8_t addr, uint8_t reg, uint8_t length, uint8_t *data)
{
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);  // 读取数据
  if (ret != HAL_OK) {
      return 1;
  }
  return 0;
}

void mpu6050_get_ms(unsigned long *time)
{

}

void mpu6050_delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}
