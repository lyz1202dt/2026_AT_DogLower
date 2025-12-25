#ifndef __RAMP_SIMPLE_H
#define __RAMP_SIMPLE_H

#include "main.h"

// 斜坡结构体定义
typedef struct {
    float current_value;      // 当前值
    float target_value;       // 目标值
    float step_size;          // 每次增加的步长
    uint8_t is_running;       // 是否正在运行
} SimpleRamp_t;


// 关节结构体 速度，力矩，角度定义
typedef struct { 
    SimpleRamp_t ramp_start[3];
}joint_ramp_t;

// 腿部结构体 4个关节定义
typedef struct { 
    joint_ramp_t joint_ramp_t[4];
}leg_ramp_t;


// 函数声明
uint8_t SimpleRamp_Update(SimpleRamp_t *ramp);
float SimpleRamp_GetValue(SimpleRamp_t *ramp);

#endif // __RAMP_SIMPLE_H