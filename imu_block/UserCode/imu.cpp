//
// Created by 35877 on 2026/10/18.
//


#include "imu.h"
#include <string.h>
#include <math.h>
#include "bmi.h"
#define M_PI 3.14159265358979323846f

IMU my_imu;
const float R_imu[3][3] = {
    { 1.0f, 0.0f, 0.0f},
    { 0.0f, 1.0f, 0.0f},
    { 0.0f, 0.0f, 1.0f}
};

IMU::IMU() {
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    memset(gyro_bias, 0, sizeof(gyro_bias));
}

void IMU::read_data() {
    bmi088_accel_read_reg(0x12, acc_buf, 6);
    int16_t ax = (int16_t)((acc_buf[1] << 8) | acc_buf[0]);
    int16_t ay = (int16_t)((acc_buf[3] << 8) | acc_buf[2]);
    int16_t az = (int16_t)((acc_buf[5] << 8) | acc_buf[4]);

    bmi088_gyro_read_reg(0x02, gyr_buf, 6);
    int16_t gx = (int16_t)((gyr_buf[1] << 8) | gyr_buf[0]);
    int16_t gy = (int16_t)((gyr_buf[3] << 8) | gyr_buf[2]);
    int16_t gz = (int16_t)((gyr_buf[5] << 8) | gyr_buf[4]);

    float acc_scale = 9.8f * 6.0f / 32768.0f;
    accel[0] = ax * acc_scale;
    accel[1] = ay * acc_scale;
    accel[2] = az * acc_scale;

    float gyro_scale = (2000.0f * M_PI / 180.0f) / 32768.0f;
    gyro[0] = gx * gyro_scale;
    gyro[1] = gy * gyro_scale;
    gyro[2] = gz * gyro_scale;

    for (uint8_t i = 0; i < 3; i++) {
        accel_body[i] = accel[0] * R_imu[i][0] + accel[1] * R_imu[i][1] + accel[2] * R_imu[i][2];
        gyro_body[i]  = (gyro[0] * R_imu[i][0] + gyro[1] * R_imu[i][1] + gyro[2] * R_imu[i][2]) - gyro_bias[i];
    }
}

void IMU::calibrate_gyro() {
    float sum[3] = {0.0f};
    int samples = 1000;

    for (int i = 0; i < samples; i++) {
        read_data();
        sum[0] += gyro_body[0];
        sum[1] += gyro_body[1];
        sum[2] += gyro_body[2];
        HAL_Delay(2);
    }
    gyro_bias[0] = sum[0] / (float)samples;
    gyro_bias[1] = sum[1] / (float)samples;
    gyro_bias[2] = sum[2] / (float)samples;
}

void IMU::update(float dt) {
    read_data();
    mahony_update(dt);
}

void IMU::mahony_update(float dt) {
    float ax = accel_body[0];
    float ay = accel_body[1];
    float az = accel_body[2];

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

    float gx = gyro_body[0] + Kp * ex;
    float gy = gyro_body[1] + Kp * ey;
    float gz = gyro_body[2] + Kp * ez;

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

void IMU::get_euler(float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * 180.0f / M_PI;
    *pitch = asinf(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / M_PI;
    *yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * 180.0f / M_PI;
}

extern "C" {
void IMU_Calibrate(void) {
    my_imu.calibrate_gyro();
}
void IMU_Update(float dt) {
    my_imu.update(dt);
}
void IMU_GetEuler(float *r, float *p, float *y) {
    my_imu.get_euler(r, p, y);
}
}