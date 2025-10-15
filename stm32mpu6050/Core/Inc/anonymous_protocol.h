/**
 * @file anonymous_protocol.h
 * @brief Anonymous communication protocol implementation
 * @author Generated for STM32 MPU6050 Project
 * @date 2025
 */

#ifndef ANONYMOUS_PROTOCOL_H
#define ANONYMOUS_PROTOCOL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Protocol definitions */
#define PROTOCOL_HEADER          0xAA
#define PROTOCOL_TARGET_ADDR     0xFF
#define PROTOCOL_FUNCTION_ID     0x03
#define PROTOCOL_DATA_LENGTH     7
#define PROTOCOL_TOTAL_LENGTH    13  // Header + Addr + ID + Len + Data(7) + Checksum + AddCheck

/* Fusion status definitions */
#define FUSION_STATUS_OK         0x01
#define FUSION_STATUS_ERROR      0x00

/* DMP data structure */
typedef struct {
    float pitch;            // Pitch angle in degrees
    float roll;             // Roll angle in degrees
    float yaw;              // Yaw angle in degrees
    uint8_t fusion_status;  // Fusion status
} dmp_data_t;

/* Function prototypes */
uint8_t Protocol_Encode_MPU6050_Data(const dmp_data_t *dmp_data, uint8_t *buffer);
uint8_t Protocol_Calculate_Checksum(const uint8_t *data, uint8_t length);
uint8_t Protocol_Calculate_AddCheck(const uint8_t *data, uint8_t length);
void Protocol_Send_MPU6050_Data(UART_HandleTypeDef *huart, const dmp_data_t *dmp_data);
void Protocol_Debug_Print(UART_HandleTypeDef *huart, const dmp_data_t *dmp_data);
void Protocol_Send_Int16(UART_HandleTypeDef *huart, int16_t value);
void Protocol_Send_Hex(UART_HandleTypeDef *huart, uint8_t value);

#endif /* ANONYMOUS_PROTOCOL_H */
