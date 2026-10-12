//
// Created by 35877 on 2026/10/3.
//


#include "tim.h"
#include "can_user.h"
#include "motor.h"
volatile uint32_t interrupt_tick = 0;
constexpr uint32_t kMotor1FeedbackID =0x201;
static volatile uint32_t last_exti_tick = 0;
PID ppid(18.0f, 0.85f,0.7f, 10.0f, 100.0f, 0.1f);
PID spid(0.08f, 0.0f, 0.0f, 10.0f, 10000.0f, 0.05f);
Motor motor(36.0f, Motor::POSITION_SPEED, ppid, spid);

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    uint8_t data[8];
    if (hcan != &hcan1) return;
    if (HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,data)!=HAL_OK) return;
    if (rx_header.IDE==CAN_ID_STD && rx_header.RTR==CAN_RTR_DATA && rx_header.DLC==8 && rx_header.StdId==kMotor1FeedbackID) {
        motor.canRxMsgCallback(data);
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance != TIM6) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)>0) {
        motor.handle();
        HAL_CAN_AddTxMessage(&hcan1,&tx_header,motor.getTxData(),&can_tx_mailbox);
    }
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        uint32_t now = HAL_GetTick();
        if (now - last_exti_tick > 50) {
            last_exti_tick = now;
            float new_target = motor.getTargetAngle() + 10.0f;
            motor.setTarget(new_target);
        }
    }
}





