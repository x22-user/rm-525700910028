//
// Created by 35877 on 2026/10/3.
//

#include "can.h"
#include "stm32f4xx_hal.h"

CAN_RxHeaderTypeDef rx_header;
CAN_TxHeaderTypeDef tx_header = {
    .StdId = 0x200,
    .ExtId = 0x0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
};

uint32_t can_tx_mailbox;

CAN_FilterTypeDef can_filter_config = {
    .FilterIdHigh = 0x0000, //期望的 ID 值
    .FilterIdLow = 0x0000,
    .FilterMaskIdHigh = 0x0000, //掩码
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0, //过滤器的报文存到哪个接收 FIFO
    .FilterBank = 0, //选择过滤器组编号
    .FilterMode = CAN_FILTERMODE_IDMASK, //过滤器的工作模式（掩码模式、列表模式-直接列出若干个允许通过的 ID ）
    .FilterScale = CAN_FILTERSCALE_32BIT, //过滤器位宽
    .FilterActivation = ENABLE, //使能过滤器
    .SlaveStartFilterBank = 14
};


