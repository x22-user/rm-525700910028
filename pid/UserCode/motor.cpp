//
// Created by 35877 on 2026/10/3.
//

#include "motor.h"
#include <stdint.h>
#include <math.h>

extern uint8_t stop_flag;
float output_current_ = 0;
constexpr float PI = 3.1415926535f;
const float KT = 0.18f;
const float GEAR_EFFICIENCY = 0.8f;
const float TORQUE_TO_CURRENT = 10000.0f / 10.0f;

Motor::Motor(float ratio, ControlMode mode, PID &ppid, PID &spid)
    : ratio_(ratio), mode(mode), ppid_(ppid), spid_(spid) {
    tar_angle = 0.0f;
    cur_angle = 0.0f;
    cur_speed = 0.0f;
    is_connected = true;
}

void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    uint16_t raw_angle_ = (static_cast<uint16_t>(rx_data[0]) << 8) | rx_data[1];
    int16_t raw_Rpm_ = static_cast<uint16_t>(rx_data[2]) << 8 | rx_data[3];
    int16_t raw_current = (static_cast<uint16_t>(rx_data[4]) << 8) | rx_data[5];
    uint8_t raw_temp_ = rx_data[6];
    ecd_angle_ = raw_angle_;
    speedRpm_ = raw_Rpm_;
    currentA_ = raw_current * 10.0f / 10000.0f;
    tempC_ = raw_temp_;

    if (!is_connected) {
        last_ecd_ = ecd_angle_;
        angle_ = 0.0f;
        is_connected = true;
    }
    int32_t diff = static_cast<int32_t>(ecd_angle_) - static_cast<int32_t>(last_ecd_);
    if (diff > kEncoderRange / 2) {
        diff -= kEncoderRange;
    } else if (diff < -static_cast<int32_t>(kEncoderRange / 2)) {
        diff += kEncoderRange;
    }
    float delta_angle_ = diff * 360.0f / 8192.0f;
    angle_ += delta_angle_ / ratio_;
    last_ecd_ = ecd_angle_;

    cur_angle = angle_;
    cur_speed = speedRpm_ / ratio_;
}

float Motor::angle() const {
    return angle_;
}

float Motor::speedRpm() const {
    return speedRpm_;
}

float Motor::currentAmps() const {
    return currentA_;
}

float Motor::temperatureC() const {
    return tempC_;
}

bool Motor::hasFeedback() const {
    return is_connected;
}
float Motor::getTargetAngle() const {
    return tar_angle;
}
void Motor::resetPID() {
    ppid_.reset(); spid_.reset();
}

void Motor::setTarget(float target) {
    if (mode == POSITION_SPEED) {
        tar_angle = target;
    } else if (mode == SPEED) {
        tar_speed = target;
    } else if (mode == TORQUE) {
        tar_angle = target;
    }
}

float Motor::calculateGravityFF(float angle) {
    float mass = 0.1f;
    float length = 0.065f;
    float g = 9.8f;
    float angle_rad=angle*PI/180.0f;
    float torque=mass*length*g*sinf(angle_rad);
    float torque_motor=torque/(KT*GEAR_EFFICIENCY*ratio_);
    return torque_motor;
}
void Motor::handle() {
    if (!is_connected) return;
    if (stop_flag==1) {
        setTxCurrent(0,1);
        ppid_.reset();
        spid_.reset();
        return;
    }

    if (mode==POSITION_SPEED) {   
        float gravityFeedforward=calculateGravityFF(cur_angle);
		tar_speed=ppid_.calc(tar_angle,cur_angle);
        output_current_=spid_.calc(tar_speed,cur_speed)+gravityFeedforward;
    }else if (mode==SPEED) {
		float gravityFeedforward=calculateGravityFF(cur_angle);
        output_current_=spid_.calc(tar_speed,cur_speed)+gravityFeedforward;
    }else if (mode==TORQUE) {
        output_current_=tar_angle;
    }
    setTxCurrent(output_current_,1);
}
void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    if (motor_id<1 || motor_id>4) {
        return;
    }
    if (amperes>10.0f) amperes=10.0f;
    if (amperes<-10.0f) amperes=-10.0f;
    int16_t raw=static_cast<int16_t>(amperes*10000.0f/10.0f);
    uint8_t idx=static_cast<uint8_t>(motor_id-1)*2;
    tx_data_[idx]=static_cast<uint8_t>((raw>>8)&0xFF);
    tx_data_[idx+1]=static_cast<uint8_t>(raw&0xFF);
}
uint8_t* Motor::getTxData() {
    return tx_data_;
}


