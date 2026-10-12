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
    void setTarget(float target);
    void setTxCurrent(float amperes,uint8_t motor_id);
    void canRxMsgCallback(const uint8_t rx_data[8]);
    float angle() const;
    float speedRpm() const;
    float currentAmps() const;
    float temperatureC() const;
    bool hasFeedback() const;
    uint8_t* getTxData();

    float getTargetAngle() const;
    void resetPID();
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
