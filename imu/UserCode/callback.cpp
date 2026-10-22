//
// Created by 35877 on 2026/10/18.
//

#include "bmi.h"
#include "imu.h"

extern IMU my_imu;
extern volatile bool tim6_flag; 

extern "C" void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    if (hspi->Instance == SPI1) {
        my_imu.OnDMAComplete();
    }
}
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        tim6_flag = true;
    }
}