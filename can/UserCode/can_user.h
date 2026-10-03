//
// Created by 35877 on 2026/10/3.
//

#ifndef CAN_CAN_USER_H
#define CAN_CAN_USER_H
#include "can.h"

extern CAN_RxHeaderTypeDef rx_header;
extern CAN_TxHeaderTypeDef tx_header;
extern uint32_t can_tx_mailbox;
extern CAN_FilterTypeDef can_filter_config;

#endif //CAN_CAN_USER_H
