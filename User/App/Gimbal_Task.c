#include "Gimbal_Task.h"
#include "All_Init.h"

uint8_t MOTOR_HEAD_PID_INIT(MOTOR_Typdef *MOTOR)
{
    float PID_Angle[3] = {  20.0f,   0.0f,   0.0f   };//TODO待调
    float PID_Speed[3] = {  20.0f,   0.0f,   0.0f   };//TODO待调

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
//上电初始化
    static uint8_t PID_INIT = RUI_DF_ERROR;
    if (PID_INIT == RUI_DF_ERROR)
    {
        PID_INIT = MOTOR_HEAD_PID_INIT(MOTOR);
        return RUI_DF_ERROR;
    }
//定义目标值
    static float yaw_aim   = 0.0f;
    static float pitch_aim = 0.0f;
//遥控数值传给目标值
    yaw_aim   += DBUS->Remote.CH0 * 0.1f; // TODO 待填:yaw 灵敏度系数上车调
    pitch_aim += DBUS->Remote.CH3 * 0.1f; // TODO 待填:pitch 灵敏度系系数上车调
//pitch限幅(TODO 待填:Pitch_MAX/Pitch_MIN 待调)
    pitch_aim = MATH_Limit_float(CONTAL->HEAD.Pitch_MAX,CONTAL->HEAD.Pitch_MIN, pitch_aim);
//双环PID，外环反馈输入为IMU算出来的yaw/pitch，内环反馈输入为IMU算出来的角速度
    //yaw 外环用陀螺仪绝对连续�?YawTotalAngle(世界系锁�?小陀螺关�?,不能用编码器机械�?
    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_P, IMU_Data->YawTotalAngle, yaw_aim);
    PID_Calculate(&MOTOR->m_dm4310_y_t.PID_S, IMU_Data->gyro[2], MOTOR->m_dm4310_y_t.PID_P.Output);// TODO 待填:gyro yaw 先试[2]),上车看转 yaw 时哪个分量最大

    PID_Calculate(&MOTOR->m_dm4310_p_t.PID_P, IMU_Data->pitch, pitch_aim);
    PID_Calculate(&MOTOR->m_dm4310_p_t.PID_S, IMU_Data->gyro[0], MOTOR->m_dm4310_p_t.PID_P.Output);// TODO 待填:gyro pitch 先试[0]),上车待调

    DM_Motor_Send(&hcan2, 0x300,                        // TODO 待填:CAN 总线(hcan1/hcan2)一拖四报文 ID
                    MOTOR->m_dm4310_y_t.PID_S.Output,   // TODO 待填:确认 m1 对应 yaw 电机
                    MOTOR->m_dm4310_p_t.PID_S.Output,   // TODO 待填:确认 m2 对应 pitch 电机
                    0, 0);

    return RUI_DF_READY;
}