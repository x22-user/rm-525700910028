//
// Created by 35877 on 2026/10/2.
//

#ifndef TRAINING_USART_CALLBACK_H
#define TRAINING_USART_CALLBACK_H

#endif //TRAINING_USART_CALLBACK_H

#include "usart.h"

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);