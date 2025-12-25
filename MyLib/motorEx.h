#ifndef __MOTOREX_H__
#define __MOTOREX_H__

#include "motor.h"
#include "pid_old.h"

typedef struct
{
    RM3508_TypeDef motor;
    CAN_HandleTypeDef *hcan;
    uint32_t ID;
    float ex_velocity;
    float ex_torque;
    float ex_rad;
    float velocity;
    float torque;
    float output;
    PID2 vel_pid;
    PID2 torque_pid;
    uint8_t ready;
}Motor3508Ex_t;


uint32_t Motor3508Recv(Motor3508Ex_t* motor_ex,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf);



#endif
