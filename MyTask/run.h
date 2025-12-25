#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "go_motor.h"

#include "motorEx.h"

#include "ramp_simple.h"

typedef struct{
    GO_MotorHandle_t motor;
    float pos_offset;
    int8_t inv_motor;

    float exp_rad;
    float exp_omega;
    float exp_torque;

    float Kp;
    float Kd;
}Joint_t;

typedef struct{
    Joint_t joint[3];
    Motor3508Ex_t joint4;
}Leg_t;

#pragma pack(1)

typedef struct{
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
}MotorState_t;

typedef struct{
    float velocity;
    float torque;
}Motor3508State_t; 

typedef struct{
    MotorState_t joint[3];
    Motor3508State_t joint4; 
}LegState_t;

typedef struct{
    uint32_t pack_type;
    LegState_t leg[4];
}LegPack_t;

#pragma pack()


void MotorControlTask(void* param);
void MotorSendTask(void* param);
void MotorRecvTask(void* param);
void Motor3508Control(void *param);
void MotorContol_start_setup(void *param);

#endif
