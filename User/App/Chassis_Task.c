#include "Chassis_Task.h"
#include "All_Init.h"
#include "Power_Ctrl.h"

uint8_t MOTOR_PID_Chassis_INIT(MOTOR_Typdef *MOTOR)
{
    float PID_S_1[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_1.PID_S, 16384.0f, 2000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit);

    float PID_S_2[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_2.PID_S, 16384.0f, 2000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit);

    float PID_S_3[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_3.PID_S, 16384.0f, 2000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit);

    float PID_S_4[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_4.PID_S, 16384.0f, 2000.0f,
             PID_S_4, 0, 0,
             0, 0, 0,
             Integral_Limit);
    return RUI_DF_READY;
}

uint8_t Chassis_Task(MOTOR_Typdef *MOTOR, User_Data_T *User_data,RUI_ROOT_STATUS_Typedef *Root,CONTAL_Typedef *CONTAL)
{
//电机上电保护
    static uint8_t PID_INIT = RUI_DF_ERROR;
    if (PID_INIT != RUI_DF_ERROR)
    {
        PID_INIT = MOTOR_PID_Chassis_INIT(MOTOR);
        return RUI_DF_ERROR;
    }
//目标值赋值
    MOTOR->DJI_3508_Chassis_1.DATA.Aim = CONTAL->BOTTOM.wheel1;
    MOTOR->DJI_3508_Chassis_2.DATA.Aim = CONTAL->BOTTOM.wheel2;
    MOTOR->DJI_3508_Chassis_3.DATA.Aim = CONTAL->BOTTOM.wheel3;
    MOTOR->DJI_3508_Chassis_4.DATA.Aim = CONTAL->BOTTOM.wheel4;
//PID计算
    PID_Calculate(&MOTOR->DJI_3508_Chassis_1.PID_S, (float)MOTOR->DJI_3508_Chassis_1.DATA.Speed_now, MOTOR->DJI_3508_Chassis_1.DATA.Aim );
    PID_Calculate(&MOTOR->DJI_3508_Chassis_2.PID_S, (float)MOTOR->DJI_3508_Chassis_2.DATA.Speed_now, MOTOR->DJI_3508_Chassis_2.DATA.Aim );
    PID_Calculate(&MOTOR->DJI_3508_Chassis_3.PID_S, (float)MOTOR->DJI_3508_Chassis_3.DATA.Speed_now, MOTOR->DJI_3508_Chassis_3.DATA.Aim );
    PID_Calculate(&MOTOR->DJI_3508_Chassis_4.PID_S, (float)MOTOR->DJI_3508_Chassis_4.DATA.Speed_now, MOTOR->DJI_3508_Chassis_4.DATA.Aim );

    DJI_Current_Ctrl(&hcan1, 0x200,
        (int16_t)MOTOR->DJI_3508_Chassis_1.PID_S.Output,
        (int16_t)MOTOR->DJI_3508_Chassis_2.PID_S.Output,
        (int16_t)MOTOR->DJI_3508_Chassis_3.PID_S.Output,
        (int16_t)MOTOR->DJI_3508_Chassis_4.PID_S.Output);

    return RUI_DF_READY;
}