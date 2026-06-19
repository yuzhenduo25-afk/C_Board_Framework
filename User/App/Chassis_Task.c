#include "Chassis_Task.h"
#include "All_Init.h"
#include <math.h>
static mecanumInit_typdef Mecanum_t;

uint8_t MOTOR_PID_Chassis_INIT(MOTOR_Typdef *MOTOR)
{
    float PID_S_1[3] = {   20.0f,   0.0f,   0.0f   };//待调
    float PID_S_2[3] = {   20.0f,   0.0f,   0.0f   };//待调
    float PID_S_3[3] = {   20.0f,   0.0f,   0.0f   };//待调
    float PID_S_4[3] = {   20.0f,   0.0f,   0.0f   };//待调

    PID_Init(&MOTOR->DJI_3508_Chassis_1.PID_S, 16384.0f, 2000.0f,
             PID_S_1, 0, 0,
             0, 0, 0,
             Integral_Limit);

    PID_Init(&MOTOR->DJI_3508_Chassis_2.PID_S, 16384.0f, 2000.0f,
             PID_S_2, 0, 0,
             0, 0, 0,
             Integral_Limit);

    PID_Init(&MOTOR->DJI_3508_Chassis_3.PID_S, 16384.0f, 2000.0f,
             PID_S_3, 0, 0,
             0, 0, 0,
             Integral_Limit);

    PID_Init(&MOTOR->DJI_3508_Chassis_4.PID_S, 16384.0f, 2000.0f,
             PID_S_4, 0, 0,
             0, 0, 0,
             Integral_Limit);
    return RUI_DF_READY;
}

uint8_t Chassis_Task(MOTOR_Typdef *MOTOR, User_Data_T *User_data,RUI_ROOT_STATUS_Typedef *Root,CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS)
{
//定义一些东西，后面要用
    CONTAL->CG.RELATIVE_ANGLE = MOTOR->m_dm4310_y_t.DATA.ralativeAngle;
    float th = MOTOR->m_dm4310_y_t.DATA.ralativeAngle / 57.3f;
    float c = cosf(th);
    float s = sinf(th);//待调（正负号）
//电机上电保护
    static uint8_t PID_INIT = RUI_DF_ERROR;
    if (PID_INIT != RUI_DF_ERROR)
    {
        PID_INIT = MOTOR_PID_Chassis_INIT(MOTOR);
        MecanumInit(&Mecanum_t);
        return RUI_DF_ERROR;
    }
//遥控值转成V
    float vx,vy,vw;
    vx = DBUS->Remote.CH2 *15;
    vy = DBUS->Remote.CH3 *15;
    vw = DBUS->Remote.CH0 *15;
//拨杆s1选择模式，1是小陀螺，2是底盘跟随，3是正常模式
    switch (DBUS->Remote.S1)
    {
        case 1:
            CONTAL->BOTTOM.VX = vx * c - vy * s;
            CONTAL->BOTTOM.VY = vx * s + vy * c;
            CONTAL->BOTTOM.VW = 300;//待调
            break;
        case 2:
            CONTAL->BOTTOM.VX = vx * c - vy * s;
            CONTAL->BOTTOM.VY = vx * s + vy * c;
            CONTAL->BOTTOM.VW = RUI_F_CHASSIS_PID(CONTAL->CG.RELATIVE_ANGLE, 1.0f, 0.0f,0.0f);
            break;
        default:
            CONTAL->BOTTOM.VX = vx;
            CONTAL->BOTTOM.VY = vy;
            CONTAL->BOTTOM.VW = vw;
    }
    float wheel_rpm[4];
//麦轮底盘解算
    MecanumResolve(wheel_rpm,CONTAL->BOTTOM.VX,CONTAL->BOTTOM.VY,CONTAL->BOTTOM.VW,&Mecanum_t);

    CONTAL->BOTTOM.wheel1 = wheel_rpm[0];
    CONTAL->BOTTOM.wheel2 = wheel_rpm[1];
    CONTAL->BOTTOM.wheel3 = wheel_rpm[2];
    CONTAL->BOTTOM.wheel4 = wheel_rpm[3];
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
//can发送
    DJI_Current_Ctrl(&hcan1, 0x200,
        (int16_t)MOTOR->DJI_3508_Chassis_1.PID_S.Output,
        (int16_t)MOTOR->DJI_3508_Chassis_2.PID_S.Output,
        (int16_t)MOTOR->DJI_3508_Chassis_3.PID_S.Output,
        (int16_t)MOTOR->DJI_3508_Chassis_4.PID_S.Output);//stdid待调，&hcan待调

    return RUI_DF_READY;
}