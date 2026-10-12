//
// Created by 35877 on 2026/10/3.
//

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H
#include <cstdint>
#include "pid.h"

class Motor {
public:
    enum ControlMode { POSITION_SPEED, SPEED, TORQUE };
    explicit Motor(float ratio, ControlMode mode, PID& ppid, PID& spid);
    void handle();
    float calculateGravityFF(float angle);
    void setTxCurrent(float amperes,uint8_t motor_id);
    void canRxMsgCallback(const uint8_t rx_data[8]);
    float angle() const;
    float speedRpm() const; //转子转速
    float currentAmps() const; //转矩电流
    float temperatureC() const; //温度
    bool hasFeedback() const; //收到过帧？
    uint8_t* getTxData();

private:
    const float ratio_;
    ControlMode mode;
    PID& ppid_,spid_;
    float tar_angle,cur_angle,tar_speed,cur_speed;
    bool is_connected = false;

    float ecd_angle_ = 0; //机械角度：0-8191
    float currentA_ = 0;
    float tempC_ = 0;
    float angle_;
    float last_ecd_ = 0;
    float speedRpm_ = 0;
    uint8_t tx_data_[8]={};
    static constexpr uint16_t kEncoderRange =8192;
};

#endif //CAN_MOTOR_H
