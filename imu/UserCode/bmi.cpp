//
// Created by 35877 on 2026/10/18.
//

#include "../Core/Inc/bmi.h"
#include <string.h>
#include <stdint.h>
#include "spi.h"

void bmi088_write_byte(uint8_t tx_data) {
    HAL_SPI_Transmit(&hspi1, &tx_data, 1, 1000);
}

void bmi088_read_byte(uint8_t *rx_data, uint8_t length) {
    if (rx_data == NULL || length == 0) return;
    HAL_SPI_Receive(&hspi1, rx_data, length, 1000);
}

void BMI088_ACCEL_NS_L() { HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET); }
void BMI088_ACCEL_NS_H() { HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET); }
void BMI088_GYRO_NS_L()  { HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET); }
void BMI088_GYRO_NS_H()  { HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET); }

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data) {
    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(data);
    BMI088_ACCEL_NS_H();
}

void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    uint8_t tx_data = reg | 0x80;
    uint8_t temp_buf[32];
    if (length > 31) return;

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();
    bmi088_write_byte(tx_data);
    bmi088_read_byte(temp_buf, length + 1);
    memcpy(rx_data, &temp_buf[1], length);
    BMI088_ACCEL_NS_H();
}

void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data) {
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();
    bmi088_write_byte(reg & 0x7F);
    bmi088_write_byte(tx_data);
    BMI088_GYRO_NS_H();
}

void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length) {
    uint8_t tx_data = reg | 0x80;
    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_L();
    bmi088_write_byte(tx_data);
    bmi088_read_byte(rx_data, length);
    BMI088_GYRO_NS_H();
}

uint8_t bmi088_init() {
    uint8_t dummy = 0, acc_id = 0, gyro_id = 0;

    BMI088_ACCEL_NS_H();
    BMI088_GYRO_NS_H();
    HAL_Delay(50);

    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(10);

    bmi088_accel_write_single_reg(0x7E, 0xB6); HAL_Delay(50);
    bmi088_gyro_write_single_reg(0x14, 0xB6);  HAL_Delay(50);

    bmi088_accel_read_reg(0x00, &dummy, 1);
    HAL_Delay(10);

    bmi088_accel_read_reg(0x00, &acc_id, 1);
    if (acc_id != 0x1E) return 0;
    bmi088_gyro_read_reg(0x00, &gyro_id, 1);
    if (gyro_id != 0x0F) return 0;

    bmi088_accel_write_single_reg(0x7C, 0x00); HAL_Delay(5);
    bmi088_accel_write_single_reg(0x7D, 0x04); HAL_Delay(5);
    bmi088_accel_write_single_reg(0x41, 0x01); HAL_Delay(1);
    bmi088_accel_write_single_reg(0x40, 0xAB); HAL_Delay(40);

    bmi088_gyro_write_single_reg(0x11, 0x00); HAL_Delay(30);
    bmi088_gyro_write_single_reg(0x0F, 0x00); HAL_Delay(1);
    bmi088_gyro_write_single_reg(0x10, 0x02); HAL_Delay(10);

    return 1;
}