/*************************** Dongguan-University of Technology -ACE**************************
 * @file    BMI088Middleware.cpp
 * @author  study-sheep
 * @version V1.0
 * @date    2024/9/29
 * @brief   BMI088中间文件
 ******************************************************************************
 * @verbatim
 * 
 * @attention
 * 
 * @version           time
 * v1.0   基础版本     2024-9-29   已测试
 ************************** Dongguan-University of Technology -ACE***************************/
#include "BMI088Middleware.hpp"
#include "real_main.hpp"

SPI_HandleTypeDef *BMI088_SPI;

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
