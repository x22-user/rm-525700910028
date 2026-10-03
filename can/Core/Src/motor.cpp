//
// Created by 35877 on 2026/10/3.
//

#include "motor.h"

void Motor::canRxMsgCallback(const uint8_t rx_data[8]) {
    uint16_t raw_angle_=(static_cast<uint16_t>(rx_data[0])<<8)|rx_data[1];
    int16_t raw_Rpm_=static_cast<uint16_t>(rx_data[2])<<8|rx_data[3];
    int16_t raw_current=(static_cast<uint16_t>(rx_data[4])<<8)|rx_data[5];
    uint8_t raw_temp_=rx_data[6];
    ecd_angle_=raw_angle_;
    speedRpm_=raw_Rpm_;
    currentA_=raw_current*20.0f/16384.0f;
    tempC_=raw_temp_;

    if (!received_) {
        last_ecd_=ecd_angle_;
        angle_=0.0f;
        received_=true;
    }
    int32_t diff=static_cast<int32_t>(ecd_angle_)-static_cast<int32_t>(last_ecd_);
    if (diff>kEncoderRange/2) {
        diff=-kEncoderRange;
    }else if (diff<-static_cast<int32_t>(kEncoderRange/2)) {
        diff+=kEncoderRange;
    }
    float delta_angle_= diff*360.0f/8192.0f;
    angle_+=delta_angle_/ratio;
    last_ecd_=ecd_angle_;
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
    return received_;
}

void Motor::setTxCurrent(float amperes, uint8_t motor_id) {
    if (motor_id<1 || motor_id>4) {
        return;
    }
    if (amperes>20.0f) amperes=20.0f;
    if (amperes<-20.0f) amperes=-20.0f;
    int16_t raw=static_cast<int16_t>(amperes*16384.0f/20.0f);
    uint8_t idx=static_cast<uint8_t>(motor_id-1)*2;
    tx_data_[idx]=static_cast<uint8_t>((raw>>8)&0xFF);
    tx_data_[idx+1]=static_cast<uint8_t>(raw&0xFF);
}

uint8_t* Motor::getTxData() {
    return tx_data_;
}
