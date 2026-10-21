//
// Created by 35877 on 2026/10/18.
//

#ifndef __BMI_H__
#define __BMI_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi1;

void bmi088_write_byte(uint8_t tx_data);
void bmi088_read_byte(uint8_t *rx_data, uint8_t length);

void BMI088_ACCEL_NS_L();
void BMI088_ACCEL_NS_H();
void BMI088_GYRO_NS_L();
void BMI088_GYRO_NS_H();

void bmi088_accel_write_single_reg(uint8_t reg, uint8_t data);
void bmi088_accel_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);
void bmi088_gyro_write_single_reg(uint8_t reg, uint8_t tx_data);
void bmi088_gyro_read_reg(uint8_t reg, uint8_t *rx_data, uint8_t length);

uint8_t bmi088_init();

#ifdef __cplusplus
}
#endif

#endif

