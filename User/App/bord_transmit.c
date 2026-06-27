//
// Created by user on 2026/6/24.
//

#include "inc/bord_transmit.h"
#include "can_bsp.h"
#include "MY_define.h"
#include <string.h>

BoardCom_Rx_t BoardCom_Rx;
BoardCom_ChassisFb_t BoardCom_ChassisFb;

static uint8_t board_com_seq;

static void BoardCom_PutInt16(uint8_t *data, uint8_t index, int16_t value)
{
    data[index] = (uint8_t)((uint16_t)value >> 8);
    data[index + 1U] = (uint8_t)value;
}

static int16_t BoardCom_GetInt16(const uint8_t *data, uint8_t index)
{
    return (int16_t)(((uint16_t)data[index] << 8) | data[index + 1U]);
}

void BoardCom_Init(void)
{
    memset(&BoardCom_Rx, 0, sizeof(BoardCom_Rx));
    memset(&BoardCom_ChassisFb, 0, sizeof(BoardCom_ChassisFb));
    BoardCom_Rx.online_time = BOARD_COM_OFFLINE_TIME_MS;
    BoardCom_ChassisFb.online_time = BOARD_COM_OFFLINE_TIME_MS;
}

void BoardCom_SendGimbalCmd(CAN_HandleTypeDef *hcan,
                            int16_t vx,
                            int16_t vy,
                            int16_t vw,
                            int16_t yaw_relative,
                            uint8_t mode,
                            uint8_t cap_enable)
{
    uint8_t data[8] = {0};

    BoardCom_PutInt16(data, 0, vx);
    BoardCom_PutInt16(data, 2, vy);
    BoardCom_PutInt16(data, 4, vw);
    BoardCom_PutInt16(data, 6, yaw_relative);
    canx_send_data(hcan, BOARD_COM_ID_GIMBAL_CMD, data);

    memset(data, 0, sizeof(data));
    data[0] = mode;
    data[1] = cap_enable;
    data[2] = board_com_seq++;
    canx_send_data(hcan, BOARD_COM_ID_GIMBAL_MODE, data);
}

void BoardCom_SendChassisFb(CAN_HandleTypeDef *hcan,
                            int16_t wheel1,
                            int16_t wheel2,
                            int16_t wheel3,
                            int16_t wheel4)
{
    uint8_t data[8] = {0};

    BoardCom_PutInt16(data, 0, wheel1);
    BoardCom_PutInt16(data, 2, wheel2);
    BoardCom_PutInt16(data, 4, wheel3);
    BoardCom_PutInt16(data, 6, wheel4);
    canx_send_data(hcan, BOARD_COM_ID_CHASSIS_FB, data);
}

void BoardCom_Decode(uint16_t std_id, const uint8_t data[8])
{
    switch (std_id)
    {
        case BOARD_COM_ID_GIMBAL_CMD:
            BoardCom_Rx.vx = BoardCom_GetInt16(data, 0);
            BoardCom_Rx.vy = BoardCom_GetInt16(data, 2);
            BoardCom_Rx.vw = BoardCom_GetInt16(data, 4);
            BoardCom_Rx.yaw_relative = BoardCom_GetInt16(data, 6);
            BoardCom_Rx.online_time = 0;
            break;

        case BOARD_COM_ID_GIMBAL_MODE:
            BoardCom_Rx.mode = data[0];
            BoardCom_Rx.cap_enable = data[1];
            BoardCom_Rx.online_time = 0;
            break;

        case BOARD_COM_ID_CHASSIS_FB:
            BoardCom_ChassisFb.wheel1 = BoardCom_GetInt16(data, 0);
            BoardCom_ChassisFb.wheel2 = BoardCom_GetInt16(data, 2);
            BoardCom_ChassisFb.wheel3 = BoardCom_GetInt16(data, 4);
            BoardCom_ChassisFb.wheel4 = BoardCom_GetInt16(data, 6);
            BoardCom_ChassisFb.online_time = 0;
            break;

        default:
            break;
    }
}

void BoardCom_OfflineTick(void)
{
    if (BoardCom_Rx.online_time < BOARD_COM_OFFLINE_TIME_MS)
    {
        BoardCom_Rx.online_time++;
    }

    if (BoardCom_ChassisFb.online_time < BOARD_COM_OFFLINE_TIME_MS)
    {
        BoardCom_ChassisFb.online_time++;
    }
}

uint8_t BoardCom_IsGimbalOnline(void)
{
    return (BoardCom_Rx.online_time < BOARD_COM_OFFLINE_TIME_MS);
}

uint8_t BoardCom_IsChassisOnline(void)
{
    return (BoardCom_ChassisFb.online_time < BOARD_COM_OFFLINE_TIME_MS);
}

