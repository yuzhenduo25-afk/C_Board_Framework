#ifndef __GIMBAL_TASK_H
#define __GIMBAL_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "IMU_Task.h"
#include "All_Init.h"
#include "WHW_IRQN.h"
#include "controller.h"

uint8_t MOTOR_HEAD_PID_INIT(MOTOR_Typdef *MOTOR);
uint8_t gimbal_task(MOTOR_Typdef *MOTOR, IMU_Data_t *IMU_Data, RUI_ROOT_STATUS_Typedef *Root, CONTAL_Typedef *CONTAL, DBUS_Typedef *DBUS);

#endif
