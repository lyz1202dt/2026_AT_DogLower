#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motorEx.h"
#include "go_motor.h"

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
    Motor3508Ex_t wheel;
}Leg_t;

#pragma pack(1)

typedef struct{
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
}MotorTarget_t;

typedef struct{
    float omega;
    float torque;
}WheelTarget_t;

typedef struct{
    MotorTarget_t joint[3];
    WheelTarget_t wheel;
}LegTarget_t;

typedef struct{
    int pack_type;
    LegTarget_t leg[4];
}MotorTargetPack_t;



typedef struct {
    float X, Y, Z;
} Vector3D_Typedef_;

typedef struct {
  Vector3D_Typedef_ AngularVelocity;
  struct {
    float Yaw, Pitch, Roll;
  } Angle;
} JY61_Typedef_;

typedef struct{
    float rad;
    float omega;
    float torque;
}MotorState_t;

typedef struct{
    float omega;
    float torque;
}WheelState_t;

typedef struct{
    MotorState_t joint[3];
    WheelState_t wheel;
}LegState_t;

typedef struct{
    int pack_type;
    LegState_t leg[4];
    JY61_Typedef_ JY61_; 
}MotorStatePack_t;

#pragma pack()


void MotorControlTask(void* param);
void MotorSendTask(void* param);
void MotorRecvTask(void* param);
void WheelControlTask(void* param);
void UART6_ServiceTask(void *arg);
#endif
