#ifndef __DM_H6215_H__
#define __DM_H6215_H__

#include <stdint.h>
#include <string.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    CAN_HandleTypeDef *hcan;
    uint32_t id;

    //电机当前的控制模式
    uint8_t mode;

    //TODO:电机状态
    float position;
    float velocity;
    float torque;

    //TODO:其它状态
    uint8_t state;
    uint8_t t_mos;
    uint8_t t_rotor;

    //发送缓冲区
    uint8_t send_buf[8];
}DMH6215_t;

void DMH6215_Init(DMH6215_t *motor,CAN_HandleTypeDef *hcan,uint32_t motor_id);

uint32_t DMH6215_MIT_Control(DMH6215_t *motor,float pos,float vel,float tor,float kp,float kd);

uint32_t DMH6215_PosVel_Control(DMH6215_t *motor,float pos,float vel);

uint32_t DMH6215_Vel_Control(DMH6215_t *motor,float vel);

uint32_t DMH6215_Disable(DMH6215_t *motor);

uint32_t DMH6215_Enable(DMH6215_t *motor);

uint32_t DMH6215_ClearError(DMH6215_t *motor);


uint32_t DMH6215_Recv_Handle(DMH6215_t *motor,CAN_HandleTypeDef *hcan,uint32_t id,uint8_t *buf);

#endif
