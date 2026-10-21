//
// Created by 35877 on 2026/10/18.
//

#ifndef __IMU_H__
#define __IMU_H__

#include "main.h"


class IMU {
public:
    IMU();
    void calibrate_gyro();
    void update(float dt);
    void get_euler(float *r, float *p, float *y);

private:
    void read_data();
    void mahony_update(float dt);

    float q0, q1, q2, q3;
    float gyro_bias[3];

    uint8_t acc_buf[6];
    uint8_t gyr_buf[6];

    float accel[3];
    float gyro[3];

    float accel_body[3];
    float gyro_body[3];

    const float Kp = 1.5f;
    const float Ki = 0.005f;
};
#ifdef __cplusplus
extern "C" {
#endif

void IMU_Calibrate(void);
void IMU_Update(float dt);
void IMU_GetEuler(float *r, float *p, float *y);

#ifdef __cplusplus
}
#endif
#endif

