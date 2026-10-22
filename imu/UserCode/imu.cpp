//
// Created by 35877 on 2026/10/18.
//


#include "imu.h"
#include "../Core/Inc/bmi.h"
#include "math.h"
#define M_PI 3.14159265358979323846f

IMU my_imu;
const float R_imu[3][3] = {
    { 1.0f, 0.0f, 0.0f},
    { 0.0f, 1.0f, 0.0f},
    { 0.0f, 0.0f, 1.0f}
};

IMU::IMU() {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    gyro_bias[0] = 0.0f;
    gyro_bias[1] = 0.0f;
    gyro_bias[2] = 0.26f;
    
    tx_accel_buf[0] = 0x12 | 0x80;
    for(int i=1; i<8; i++) tx_accel_buf[i] = 0;

    tx_gyro_buf[0] = 0x02 | 0x80;
    for(int i=1; i<7; i++) tx_gyro_buf[i] = 0;
}

void IMU::gyro_calculate() {
    uint8_t rx_gyro_data[6];
    bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);

    int16_t gx = (int16_t)((rx_gyro_data[1] << 8) | rx_gyro_data[0]);
    int16_t gy = (int16_t)((rx_gyro_data[3] << 8) | rx_gyro_data[2]);
    int16_t gz = (int16_t)((rx_gyro_data[5] << 8) | rx_gyro_data[4]);

    float gyro_scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
    raw_data.gyro[0] = gx * gyro_scale;
    raw_data.gyro[1] = gy * gyro_scale;
    raw_data.gyro[2] = gz * gyro_scale;
}

void IMU::calibrate_gyro() {
    uint8_t gyr_buf[6];
    float sum[3] = {0.0f};
    int samples = 1000;

    for (int i = 0; i < samples; i++) {
        bmi088_gyro_read_reg(0x02, gyr_buf, 6);
        int16_t gx = (int16_t)((gyr_buf[1] << 8) | gyr_buf[0]);
        int16_t gy = (int16_t)((gyr_buf[3] << 8) | gyr_buf[2]);
        int16_t gz = (int16_t)((gyr_buf[5] << 8) | gyr_buf[4]);

        float gyro_scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
        sum[0] += (gx * gyro_scale) * R_imu[0][0] + (gy * gyro_scale) * R_imu[0][1] + (gz * gyro_scale) * R_imu[0][2];
        sum[1] += (gx * gyro_scale) * R_imu[1][0] + (gy * gyro_scale) * R_imu[1][1] + (gz * gyro_scale) * R_imu[1][2];
        sum[2] += (gx * gyro_scale) * R_imu[2][0] + (gy * gyro_scale) * R_imu[2][1] + (gz * gyro_scale) * R_imu[2][2];
        HAL_Delay(1);
    }
    gyro_bias[0] = sum[0] / (float)samples;
    gyro_bias[1] = sum[1] / (float)samples;
    gyro_bias[2] = sum[2] / (float)samples;
}

void IMU::start_dma_read() {
    if (dma_step != 0) {
        return; 
    }

    BMI088_GYRO_NS_H();
    BMI088_ACCEL_NS_L();

    dma_step = 1;
	if (HAL_SPI_TransmitReceive_DMA(&hspi1, tx_accel_buf, rx_accel_buf, 8) != HAL_OK) {
        dma_step = 0;
        BMI088_ACCEL_NS_H();
    }
}

void IMU::OnDMAComplete() {
    if (dma_step == 1) {
        BMI088_ACCEL_NS_H();
        BMI088_GYRO_NS_L();

        dma_step = 2;
        HAL_SPI_TransmitReceive_DMA(&hspi1, tx_gyro_buf, rx_gyro_buf, 7);
    }
    else if (dma_step == 2) {
        BMI088_GYRO_NS_H();
        dma_step = 0;
        dma_data_ready = true;
    }
}

void IMU::process_dma_data() {
	int16_t ax = (int16_t)((rx_accel_buf[3] << 8) | rx_accel_buf[2]);
    int16_t ay = (int16_t)((rx_accel_buf[5] << 8) | rx_accel_buf[4]);
    int16_t az = (int16_t)((rx_accel_buf[7] << 8) | rx_accel_buf[6]);

    float acc_scale = 9.8f * 6.0f / 32768.0f;
    raw_data.accel[0] = ax * acc_scale;
    raw_data.accel[1] = ay * acc_scale;
    raw_data.accel[2] = az * acc_scale;

	int16_t gx = (int16_t)((rx_gyro_buf[2] << 8) | rx_gyro_buf[1]);
	int16_t gy = (int16_t)((rx_gyro_buf[4] << 8) | rx_gyro_buf[3]);
	int16_t gz = (int16_t)((rx_gyro_buf[6] << 8) | rx_gyro_buf[5]);

    float gyro_scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
    raw_data.gyro[0] = gx * gyro_scale;
    raw_data.gyro[1] = gy * gyro_scale;
    raw_data.gyro[2] = gz * gyro_scale;

    sensorDataTransform();
}


void IMU::sensorDataTransform() {
    for (uint8_t i = 0; i < 3; i++) {
        sensor_data.gyro[i] = (raw_data.gyro[0] * R_imu[i][0] +
                               raw_data.gyro[1] * R_imu[i][1] +
                               raw_data.gyro[2] * R_imu[i][2]) - gyro_bias[i];

        sensor_data.accel[i] = (raw_data.accel[0] * R_imu[i][0] +
                                raw_data.accel[1] * R_imu[i][1] +
                                raw_data.accel[2] * R_imu[i][2]);
    }
}

void IMU::mahony_update(float dt) {
    float ax = sensor_data.accel[0];
    float ay = sensor_data.accel[1];
    float az = sensor_data.accel[2];

    float norm = sqrtf(ax*ax + ay*ay + az*az);

    if (norm == 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q2 * q3 + q0 * q1);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    gyro_bias[0] -= Ki * ex * dt;
    gyro_bias[1] -= Ki * ey * dt;
    gyro_bias[2] -= Ki * ez * dt;

    float gx = sensor_data.gyro[0] + Kp * ex;
    float gy = sensor_data.gyro[1] + Kp * ey;
    float gz = sensor_data.gyro[2] + Kp * ez;

    float q0_dot = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float q1_dot = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float q2_dot = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float q3_dot = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    q0 += q0_dot * dt;
    q1 += q1_dot * dt;
    q2 += q2_dot * dt;
    q3 += q3_dot * dt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;
}

void IMU::quaternion_to_euler(float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}


void IMU_Calibrate() {
    my_imu.calibrate_gyro();
}
bool IMU_IsDataReady() {
    return my_imu.dma_data_ready;
}
void IMU_ClearDataReady() {
    my_imu.dma_data_ready = false;
}
void IMU_ProcessData() {
    my_imu.process_dma_data();
}
void IMU_Update(float dt) {
    my_imu.mahony_update(dt);
}
void IMU_GetEuler(float *r, float *p, float *y) {
    my_imu.quaternion_to_euler(r, p, y);
}
void IMU_StartDMARead() {
    my_imu.start_dma_read();
}

