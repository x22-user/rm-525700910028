//
// Created by 35877 on 2026/10/18.
//

#ifndef IMU_IMU__H
#define IMU_IMU__H
#include <stdint.h>

struct IMU_Data {
    float accel[3];
    float gyro[3];
};

class IMU {
public:
    IMU();

    void gyro_calculate();
    void calibrate_gyro();
    void start_dma_read();
    void OnDMAComplete();
    void process_dma_data();

    void sensorDataTransform();
    void mahony_update(float dt);
    void quaternion_to_euler(float *roll, float *pitch, float *yaw);

    volatile bool dma_data_ready = false;

private:
    float q0, q1, q2, q3;

    float gyro_bias[3];

    const float Kp = 1.0f;
    const float Ki = 0.002f;

    IMU_Data raw_data;
    IMU_Data sensor_data;

    uint8_t tx_accel_buf[8];
    uint8_t rx_accel_buf[8];
    uint8_t tx_gyro_buf[7];
    uint8_t rx_gyro_buf[7];
    uint8_t dma_step = 0;
};
#ifdef __cplusplus
extern "C" {
#endif


void IMU_Calibrate(void);
bool IMU_IsDataReady(void);
void IMU_ClearDataReady(void);
void IMU_ProcessData(void);
void IMU_Update(float dt);
void IMU_GetEuler(float *r, float *p, float *y);
void IMU_StartDMARead(void);

#ifdef __cplusplus
}
#endif
#endif //IMU_IMU__H

