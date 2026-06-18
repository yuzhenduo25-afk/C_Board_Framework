#include "Gimbal_Task.h"
#include "All_Init.h"

uint8_t MOTOR_HEAD_PID_INIT(MOTOR_Typdef *MOTOR)
{
    float PID_Angle[3] = {  20.0f,   0.0f,   0.0f   };//待调
    float PID_Speed[3] = {  20.0f,   0.0f,   0.0f   };//待调

    PID_Init(&MOTOR->m_dm4310_y_t.PID_P, 16384.0f, 2000.0f,
             PID_Angle, 0, 0,
             0, 0, 0,
             Integral_Limit);

    PID_Init(&MOTOR->m_dm4310_y_t.PID_S, 16384.0f, 2000.0f,
             PID_Speed, 0, 0,
             0, 0, 0,
             Integral_Limit);

    PID_Init(&MOTOR->m_dm4310_p_t.PID_P, 16384.0f, 2000.0f,
             PID_Angle, 0, 0,
             0, 0, 0,
             Integral_Limit);

    PID_Init(&MOTOR->m_dm4310_p_t.PID_S, 16384.0f, 2000.0f,
             PID_Speed, 0, 0,
             0, 0, 0,
             Integral_Limit);
    return RUI_DF_READY;
}

uint8_t gimbal_task(MOTOR_Typdef *MOTOR, IMU_Data_t *IMU_Data,RUI_ROOT_STATUS_Typedef *Root,CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS)
{
//上电PID初始化
    static uint8_t PID_INIT = RUI_DF_ERROR;
    if (PID_INIT != RUI_DF_ERROR)
    {
        PID_INIT = MOTOR_HEAD_PID_INIT(MOTOR);
        return RUI_DF_ERROR;
    }
//定义目标值
    static float yaw_aim   = 0.0f;
    static float pitch_aim = 0.0f;
//遥控数值传给目标值
    yaw_aim   += DBUS->Remote.CH0 * 0.1f; // 待调
    pitch_aim += DBUS->Remote.CH3 * 0.1f; // 待调
//pitch限幅
    pitch_aim = MATH_Limit_float(CONTAL->HEAD.Pitch_MAX,CONTAL->HEAD.Pitch_MIN, pitch_aim);
//双环PID，外环反馈输入为IMU算出来的yaw/pitch，内环反馈输入为IMU算出来的角加速度。
    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_P, MOTOR->m_dm4310_y_t.DATA.ralativeAngle, yaw_aim);
    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_S, MOTOR->m_dm4310_y_t.DATA.Speed_now, MOTOR->m_dm4310_y_t.PID_P.Output);
    return RUI_DF_READY;
}