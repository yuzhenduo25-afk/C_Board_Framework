#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "DJI_Motor.h"
#include "DM_Motor.h"
#include "DBUS.h"
#include "MY_define.h"
#include "RUI_ROOT_INIT.h"
#include "Motors.h"
#include "Power_Ctrl.h"

uint8_t MOTOR_PID_Chassis_INIT(MOTOR_Typdef *MOTOR);
uint8_t Chassis_Task(MOTOR_Typdef *MOTOR, User_Data_T *User_data,RUI_ROOT_STATUS_Typedef *Root,CONTAL_Typedef *CONTAL,DBUS_Typedef *DBUS);

#endif
