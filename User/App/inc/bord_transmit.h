//
// Created by user on 2026/6/24.
//

#ifndef C_BOARD_FRAMEWORK_BORD_TRANSMIT_H
#define C_BOARD_FRAMEWORK_BORD_TRANSMIT_H

#include <stdint.h>

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vw;
    int16_t yaw_relative;  // deg * 100
    uint8_t mode;
    uint8_t cap_enable;
    uint8_t online_time;
} BoardCom_Rx_t;


#endif //C_BOARD_FRAMEWORK_BORD_TRANSMIT_H