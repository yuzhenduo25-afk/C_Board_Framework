//
// Created by user on 2026/6/24.
//

#ifndef C_BOARD_FRAMEWORK_BORD_TRANSMIT_H
#define C_BOARD_FRAMEWORK_BORD_TRANSMIT_H

#include "can.h"
#include "MY_define.h"
#include <stdint.h>

#define BOARD_COM_ID_GIMBAL_CMD   GIMBAL_kong
#define BOARD_COM_ID_CHASSIS_FB   CHASSIC_kong
#define BOARD_COM_ID_GIMBAL_MODE  0x233U
#define BOARD_COM_OFFLINE_TIME_MS 50U

typedef struct
{
    int16_t vx;
    int16_t vy;
    int16_t vw;
    int16_t yaw_relative;
    uint8_t mode;
    uint8_t cap_enable;
    uint16_t online_time;
} BoardCom_Rx_t;

typedef struct
{
    int16_t wheel1;
    int16_t wheel2;
    int16_t wheel3;
    int16_t wheel4;
    uint16_t online_time;
} BoardCom_ChassisFb_t;

extern BoardCom_Rx_t BoardCom_Rx;
extern BoardCom_ChassisFb_t BoardCom_ChassisFb;

void BoardCom_Init(void);
void BoardCom_SendGimbalCmd(CAN_HandleTypeDef *hcan,
                            int16_t vx,
                            int16_t vy,
                            int16_t vw,
                            int16_t yaw_relative,
                            uint8_t mode,
                            uint8_t cap_enable);
void BoardCom_SendChassisFb(CAN_HandleTypeDef *hcan,
                            int16_t wheel1,
                            int16_t wheel2,
                            int16_t wheel3,
                            int16_t wheel4);
void BoardCom_Decode(uint16_t std_id, const uint8_t data[8]);
void BoardCom_OfflineTick(void);
uint8_t BoardCom_IsGimbalOnline(void);
uint8_t BoardCom_IsChassisOnline(void);

#endif //C_BOARD_FRAMEWORK_BORD_TRANSMIT_H
