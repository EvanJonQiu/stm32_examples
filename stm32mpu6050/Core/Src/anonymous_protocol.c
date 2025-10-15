/**
 * @file anonymous_protocol.c
 * @brief Anonymous communication protocol implementation
 * @author Generated for STM32 MPU6050 Project
 * @date 2025
 */

#include "anonymous_protocol.h"

/**
 * @brief Encode MPU6050 data into protocol format
 * @param dmp_data: Pointer to DMP data structure
 * @param buffer: Output buffer for encoded data
 * @retval Length of encoded data
 */
uint8_t Protocol_Encode_MPU6050_Data(const dmp_data_t *dmp_data, uint8_t *buffer)
{
    uint8_t index = 0;
    
    // 1. Frame header (0xAA)
    buffer[index++] = PROTOCOL_HEADER;
    
    // 2. Target address (0xFF)
    buffer[index++] = PROTOCOL_TARGET_ADDR;
    
    // 3. Function code (0x03)
    buffer[index++] = PROTOCOL_FUNCTION_ID;
    
    // 4. Data length (7)
    buffer[index++] = PROTOCOL_DATA_LENGTH;
    
    // 5. Data content
    // 5.1 ROL * 100 (int16) - Roll angle * 100
    int16_t roll_scaled = (int16_t)(dmp_data->roll * 100.0f);
    buffer[index++] = (uint8_t)(roll_scaled & 0xFF);        // Low byte
    buffer[index++] = (uint8_t)((roll_scaled >> 8) & 0xFF); // High byte
    
    // 5.2 PIT * 100 (int16) - Pitch angle * 100
    int16_t pitch_scaled = (int16_t)(dmp_data->pitch * 100.0f);
    buffer[index++] = (uint8_t)(pitch_scaled & 0xFF);        // Low byte
    buffer[index++] = (uint8_t)((pitch_scaled >> 8) & 0xFF); // High byte
    
    // 5.3 YAW * 100 (int16) - Yaw angle * 100
    int16_t yaw_scaled = (int16_t)(dmp_data->yaw * 100.0f);
    buffer[index++] = (uint8_t)(yaw_scaled & 0xFF);        // Low byte
    buffer[index++] = (uint8_t)((yaw_scaled >> 8) & 0xFF); // High byte
    
    // 5.4 FUSION_DATA (uint8) - Fusion status
    buffer[index++] = dmp_data->fusion_status;
    
    // 6. Calculate checksum (sum of all bytes from header to data end)
    uint8_t sumcheck = Protocol_Calculate_Checksum(buffer, index);
    buffer[index++] = sumcheck;
    
    // 7. Calculate additional checksum
    uint8_t addcheck = Protocol_Calculate_AddCheck(buffer, index - 1);
    buffer[index++] = addcheck;
    
    return index; // Total length = 13 bytes
}

/**
 * @brief Calculate checksum (sum of all bytes)
 * @param data: Data buffer
 * @param length: Length of data
 * @retval Checksum value
 */
uint8_t Protocol_Calculate_Checksum(const uint8_t *data, uint8_t length)
{
    uint8_t sumcheck = 0;
    for (uint8_t i = 0; i < length; i++) {
        sumcheck += data[i];
    }
    return sumcheck;
}

/**
 * @brief Calculate additional checksum
 * @param data: Data buffer
 * @param length: Length of data
 * @retval Additional checksum value
 */
uint8_t Protocol_Calculate_AddCheck(const uint8_t *data, uint8_t length)
{
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    
    for (uint8_t i = 0; i < length; i++) {
        sumcheck += data[i];
        addcheck += sumcheck;
    }
    
    return addcheck;
}

/**
 * @brief Send MPU6050 data via UART using protocol
 * @param huart: UART handle pointer
 * @param dmp_data: Pointer to DMP data structure
 * @retval None
 */
void Protocol_Send_MPU6050_Data(UART_HandleTypeDef *huart, const dmp_data_t *dmp_data)
{
    uint8_t protocol_buffer[PROTOCOL_TOTAL_LENGTH];
    uint8_t data_length;
    
    // Encode data into protocol format
    data_length = Protocol_Encode_MPU6050_Data(dmp_data, protocol_buffer);
    
    // Send data via UART
    HAL_UART_Transmit(huart, protocol_buffer, data_length, 1000);
}

/**
 * @brief Debug print protocol data
 * @param huart: UART handle pointer
 * @param dmp_data: Pointer to DMP data structure
 * @retval None
 */
void Protocol_Debug_Print(UART_HandleTypeDef *huart, const dmp_data_t *dmp_data)
{
    // Send debug info via UART
    HAL_UART_Transmit(huart, (uint8_t*)"Protocol Debug:\r\n", 17, 1000);
    
    // Print scaled angles
    int16_t pitch_scaled = (int16_t)(dmp_data->pitch * 100.0f);
    int16_t roll_scaled = (int16_t)(dmp_data->roll * 100.0f);
    int16_t yaw_scaled = (int16_t)(dmp_data->yaw * 100.0f);
    
    // Send pitch value
    HAL_UART_Transmit(huart, (uint8_t*)"Pitch: ", 7, 1000);
    Protocol_Send_Int16(huart, pitch_scaled);
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1000);
    
    // Send roll value
    HAL_UART_Transmit(huart, (uint8_t*)"Roll: ", 6, 1000);
    Protocol_Send_Int16(huart, roll_scaled);
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1000);
    
    // Send yaw value
    HAL_UART_Transmit(huart, (uint8_t*)"Yaw: ", 5, 1000);
    Protocol_Send_Int16(huart, yaw_scaled);
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1000);
    
    // Send protocol frame
    uint8_t protocol_buffer[PROTOCOL_TOTAL_LENGTH];
    uint8_t data_length = Protocol_Encode_MPU6050_Data(dmp_data, protocol_buffer);
    
    HAL_UART_Transmit(huart, (uint8_t*)"Frame: ", 7, 1000);
    for (uint8_t i = 0; i < data_length; i++) {
        Protocol_Send_Hex(huart, protocol_buffer[i]);
        HAL_UART_Transmit(huart, (uint8_t*)" ", 1, 1000);
    }
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n---\r\n", 8, 1000);
}

/**
 * @brief Send int16 value as string via UART
 * @param huart: UART handle pointer
 * @param value: int16 value to send
 * @retval None
 */
void Protocol_Send_Int16(UART_HandleTypeDef *huart, int16_t value)
{
    char buffer[8];
    uint8_t index = 0;
    
    if (value < 0) {
        HAL_UART_Transmit(huart, (uint8_t*)"-", 1, 1000);
        value = -value;
    }
    
    if (value == 0) {
        HAL_UART_Transmit(huart, (uint8_t*)"0", 1, 1000);
        return;
    }
    
    // Convert to string
    while (value > 0) {
        buffer[index++] = (value % 10) + '0';
        value /= 10;
    }
    
    // Send in reverse order
    for (int8_t i = index - 1; i >= 0; i--) {
        HAL_UART_Transmit(huart, (uint8_t*)&buffer[i], 1, 1000);
    }
}

/**
 * @brief Send hex value via UART
 * @param huart: UART handle pointer
 * @param value: uint8 value to send as hex
 * @retval None
 */
void Protocol_Send_Hex(UART_HandleTypeDef *huart, uint8_t value)
{
    char hex_chars[] = "0123456789ABCDEF";
    char hex_str[3];
    
    hex_str[0] = hex_chars[(value >> 4) & 0x0F];
    hex_str[1] = hex_chars[value & 0x0F];
    hex_str[2] = '\0';
    
    HAL_UART_Transmit(huart, (uint8_t*)hex_str, 2, 1000);
}
