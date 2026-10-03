//
// Created by 35877 on 2026/10/3.
//

#ifndef CAN_MOTOR_H
#define CAN_MOTOR_H
#include "stm32f4xx_hal_can.h"

class Motor {
public:
    explicit Motor(float ratio);

    void canRxMsgCallback(const uint8_t rx_data[8]);
    //CAN接收中断调用
    //读取数据
    float angle() const; //输出轴角度
    float speedRpm() const; //转子转速
    float currentAmps() const; //转矩电流
    float temperatureC() const; //温度
    bool hasFeedback() const; //收到过帧？

    void setTxCurrent(float amperes,uint8_t motor_id);
    uint8_t* getTxData();

private:
    const float ratio;

    float ecd_angle_ = 0; //机械角度：0-8191
    float speedRpm_ = 0;
    float currentA_ = 0;
    float tempC_ = 0;

    float angle_ = 0; //输出轴累计角度
    float last_ecd_ = 0;
    float received_ = false;

    uint8_t tx_data_[8]={};
    static constexpr uint16_t kEncoderRange =8192;
};

#endif //CAN_MOTOR_H
