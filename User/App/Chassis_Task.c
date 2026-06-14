#include "Chassis_Task.h"
#include "All_Init.h"
#include "Power_Ctrl.h"

uint8_t MOTOR_PID_Chassis_INIT(MOTOR_Typdef *MOTOR)
{
    float PID_F_1[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_1[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_1.PID_S, 16384.0f, 2000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit);

    float PID_F_2[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_2[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_2.PID_S, 16384.0f, 2000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit);

    float PID_F_3[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_3[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_3.PID_S, 16384.0f, 2000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit);

    float PID_F_4[3] = {   1.0f,   0.0f,   0.0f   };
    float PID_S_4[3] = {   20.0f,   0.0f,   0.0f   };

    PID_Init(&MOTOR->DJI_3508_Chassis_4.PID_S, 16384.0f, 2000.0f,
             PID_S_4, 0, 0,
             0, 0, 0,
             Integral_Limit);
    return RUI_DF_READY;
}

