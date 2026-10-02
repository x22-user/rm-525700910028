//
// Created by 35877 on 2026/10/2.
//

#include "callback.h"

#include <string.h>

#include "usart.h"
#define BUFFER_SIZE 64
extern uint8_t rx_msg[BUFFER_SIZE];
extern uint8_t tx_msg[BUFFER_SIZE];

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//     if (huart==&huart1) {
//         HAL_UART_Transmit(&huart1, (uint8_t*)"Hello\r\n", 7, 1000);
//         if (rx_msg[0]=='R') {
//             HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
//         }else if (rx_msg[0]=='M') {
//             HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
//         }
//         HAL_UART_Receive_DMA(&huart1, rx_msg, 1);
//     }
// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart,uint16_t Size) {
//     if (huart==&huart1 ) {
//         if (Size>0) {
//             memcpy(tx_msg,rx_msg,Size);
//             HAL_UART_Transmit(&huart1,tx_msg,Size,1000);
//         }
//         HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_msg, 10);
//     }
// }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart==&huart1) {
        if (Size>0 && (HAL_UARTEx_GetRxEventType(huart)==HAL_UART_RXEVENT_IDLE
        || HAL_UARTEx_GetRxEventType(huart)==HAL_UART_RXEVENT_TC)) {
            memcpy(tx_msg,rx_msg,Size);
            HAL_UART_Transmit_IT(&huart1,tx_msg,Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_msg,BUFFER_SIZE);
    }
}
