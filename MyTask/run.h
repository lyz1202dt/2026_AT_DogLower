#ifndef __RUN_H__
#define __RUN_H__

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

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
    Joint_t joint1;
    Joint_t joint2;
    Joint_t joint3;
}Leg_t;

typedef struct{
    float rad;
    float omega;
    float torque;
    float kp;
    float kd;
}MotorState_t;

typedef struct{
    MotorState_t joint1;
    MotorState_t joint2;
    MotorState_t joint3;
}LegState_t;

typedef struct{
    int id;
    LegState_t leg;
}LegPack_t;


void MotorControlTask(void* param);
void MotorSendTask(void* param);
void MotorRecvTask(void* param);


uint8_t My_CDC_SendCb(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);
uint8_t My_CDC_RecvCb(uint8_t* Buf, uint32_t *Len);


#endif
