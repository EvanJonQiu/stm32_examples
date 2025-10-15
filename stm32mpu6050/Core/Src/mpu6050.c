#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>

#define MPU6050_DMP_SAMPLE_RATE 100
//q30格式,long转float时的除数.
#define q30  1073741824.0f

//陀螺仪方向设置
static signed char gyro_orientation[9] = {
  1, 0, 0,
  0, 1, 0,
  0, 0, 1
};

uint8_t MPU6050_Set_SMPLRT_DIV(I2C_HandleTypeDef *hi2c, uint16_t rate);
uint8_t MPU6050_Set_CONFIG(I2C_HandleTypeDef *hi2c, uint16_t dlpf_cfg);

uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx);
uint16_t inv_row_2_scale(const signed char *row);
uint8_t run_self_test(void);

uint8_t MPU6050_Read_Whoami(I2C_HandleTypeDef *hi2c)
{
    uint8_t whoami_data = 0;
    HAL_StatusTypeDef read_ret = HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &whoami_data, 1, 100);
    if (read_ret != HAL_OK) {
        return 1;  // 返回0表示读取失败
    }
    return whoami_data;  // 返回实际读取的值
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
  // uint8_t data = 0x80;
  // HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x80到PWR_MGMT_1寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }
  // HAL_Delay(100);
  // data = 0x00;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x00到PWR_MGMT_1寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // data = 3 << 3;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x03到GYRO_CONFIG寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // data = 0 << 3;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x00到GYRO_CONFIG寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // if (MPU6050_Set_SMPLRT_DIV(hi2c, 50) != 0) {
  //   return 1;
  // }

  // data = 0;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x00到INT_ENABLE寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // data = 0;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x00到USER_CTRL寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // data = 0;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_FIFO_EN, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x00到FIFO_EN寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // data = 0x80;
  // ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x80到INT_PIN_CFG寄存器
  // if (ret != HAL_OK) {
  //   return 1;
  // }

  // data = MPU6050_Read_Whoami(hi2c);
  // if (data != MPU6050_I2C_ADDR) {
  //   return 1;
  // } else {
  //   data = 0x01;
  //   ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x01到PWR_MGMT_1寄存器
  //   if (ret != HAL_OK) {
  //     return 1;
  //   }

  //   data = 0x00;
  //   ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x00到PWR_MGMT_2寄存器
  //   if (ret != HAL_OK) {
  //     return 1;
  //   }

  //   if (MPU6050_Set_SMPLRT_DIV(hi2c, 50) != 0) {
  //     return 1;
  //   }
  // }

  // HAL_Delay(100);
  // return 0;
  if (mpu_init(NULL) != 0) {
    return 1;
  }
  return 0;
}

uint8_t MPU6050_Set_SMPLRT_DIV(I2C_HandleTypeDef *hi2c, uint16_t rate)
{
  if (rate > 1000) {
    rate = 1000;
  }
  if (rate < 4) {
    rate = 4;
  }
  uint8_t data = 1000 / rate - 1;
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x03到SMPLRT_DIV寄存器
  if (ret != HAL_OK) {
    return 1;
  }
  return MPU6050_Set_CONFIG(hi2c, rate / 2);
}

uint8_t MPU6050_Set_CONFIG(I2C_HandleTypeDef *hi2c, uint16_t dlpf_cfg)
{
  uint8_t data = 0;
  if (dlpf_cfg >= 188) {
    data = 1;
  } else if (dlpf_cfg >= 98) {
    data = 2;
  } else if (dlpf_cfg >= 42) {
    data = 3;
  } else if (dlpf_cfg >= 20) {
    data = 4;
  } else if (dlpf_cfg >= 10) {
    data = 5;
  } else {
    data = 6;
  }
  HAL_StatusTypeDef ret = HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR << 1, MPU6050_CONFIG, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);  // 写入0x03到CONFIG寄存器
  if (ret != HAL_OK) {
    return 1;
  }
  return 0;
}


/// init dmp
uint8_t mpu_dmp_init(void)
{
  uint8_t ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (ret != 0) {
    return 1;
  }
  ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  if (ret != 0) {
    return 1;
  }
  ret = mpu_set_sample_rate(MPU6050_DMP_SAMPLE_RATE);
  if (ret != 0) {
    return 1;
  }
  ret = dmp_load_motion_driver_firmware();
  if (ret != 0) {
    return 1;
  }

  ret = dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
  if (ret != 0) {
    return 1;
  }

  ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP
    | DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO
    | DMP_FEATURE_GYRO_CAL);
  if (ret != 0) {
    return 1;
  }

  ret = dmp_set_fifo_rate(MPU6050_DMP_SAMPLE_RATE);
  if (ret != 0) {
    return 1;
  }

  ret = run_self_test();
  if (ret != 0) {
    return 1;
  }

  ret = mpu_set_dmp_state(1);
  if (ret != 0) {
    return 1;
  }

  return 0;
}

uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx)
{
  uint16_t scalar = 0;

  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;

  return scalar;
}

uint16_t inv_row_2_scale(const signed char *row)
{
  unsigned short b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;      // error
  return b;
}

uint8_t run_self_test(void)
{
  int result;
	long gyro[3], accel[3]; 

  result = mpu_run_self_test(gyro, accel);
	if (result == 0x3) 
	{
		/* Test passed. We can trust the gyro data here, so let's push it down
		* to the DMP.
		*/
		float sens;
		unsigned short accel_sens;

		mpu_get_gyro_sens(&sens);

		gyro[0] = (long)(gyro[0] * sens);
		gyro[1] = (long)(gyro[1] * sens);
		gyro[2] = (long)(gyro[2] * sens);

		dmp_set_gyro_bias(gyro);
		mpu_get_accel_sens(&accel_sens);

		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;

		dmp_set_accel_bias(accel);
    
		return 0;
	}else {
    return 1;
  }
}

uint8_t MPU6050_dmp_get_data(float *pitch, float *roll, float *yaw)
{
  float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];

	if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more)) {
    return 1;  // FIFO读取失败
  }
	/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
	 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
	**/
	/*if (sensors & INV_XYZ_GYRO )
	send_packet(PACKET_TYPE_GYRO, gyro);
	if (sensors & INV_XYZ_ACCEL)
	send_packet(PACKET_TYPE_ACCEL, accel); */
	/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
	 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
	**/
	if (sensors&INV_WXYZ_QUAT) 
	{
		q0 = quat[0] / q30;	//q30格式转换为浮点数
		q1 = quat[1] / q30;
		q2 = quat[2] / q30;
		q3 = quat[3] / q30; 
		//计算得到俯仰角/横滚角/航向角
		*pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;	// pitch
		*roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
	} else {
    return 2;  // 没有四元数数据
  }
	return 0;
}
