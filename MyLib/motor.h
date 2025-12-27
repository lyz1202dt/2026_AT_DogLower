/**
 * @file    motor.h
 * @author  yao
 * @date    1-May-2020
 * @brief   电机驱动模块头文件
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

//RMLIB_CPP_BEGIN

#include "CANDrive.h"

#define RM3508_LIMIT 16384  //!<@brief RM3508的输出限幅

/**
 * @brief RM3508电机数据结构体
 */
typedef struct {
    uint16_t MchanicalAngle;    //!<@brief 机械角度
    int16_t Speed;              //!<@brief 转速
    int16_t TorqueCurrent;      //!<@brief 转矩电流
    uint8_t temp;               //!<@brief 温度
    float Power;                //!<@brief 功率
    uint16_t LsatAngle;         //!<@brief 上一次的机械角度
    int16_t r;                  //!<@brief 圈数
    int32_t Angle;              //!<@brief 连续化机械角度 @warning 由于启动时角度不确定，启动时连续化角度可能有一圈的偏差
    float Angle_DEG;            //!<@brief 连续化角度制角度 @warning 由于启动时角度不确定，启动时连续化角度可能有一圈的偏差
    struct PowerCOF_s {
        float ss;               //!<@brief 速度平方项系数
        float sc;               //!<@brief 速度,转矩电流乘积项系数
        float cc;               //!<@brief 转矩电流平方项系数
        float constant;         //!<@brief 常量
    } PowerCOF;                 //!<@brief 计算功率所用的系数,由MATLAB拟合
} RM3508_TypeDef;

/**
 * @brief 设置RM3508功率计算参数
 * @param[out] Dst RM3510电机数据结构体指针
 * @param[in] cc 电流平方项系数
 * @param[in] sc 电流,转速乘积项系数
 * @param[in] ss 转速平方项系数
 * @param[in] constant 常数项
 */
static inline void RM3508_SetPowerCOF(RM3508_TypeDef *Dst, float cc, float sc, float ss, float constant) {
    Dst->PowerCOF.cc = cc;
    Dst->PowerCOF.sc = sc;
    Dst->PowerCOF.ss = ss;
    Dst->PowerCOF.constant = constant;
}

/**
 * @brief RM3508数据接收
 * @param[out] Dst RM3508电机数据结构体指针
 * @param[in] Data CAN数据帧指针
 */
void RM3508_Receive(RM3508_TypeDef *Dst, uint8_t *Data);



/**
 * @brief 发送电机控制信号
 * @param hcan CAN句柄
 * @param[in] StdId 标准帧ID
 * @param[in] Data 电机控制信号数组指针
 * @return HAL Status structures definition
 */
HAL_StatusTypeDef MotorSend(CAN_HandleTypeDef *hcan, uint32_t StdId, int16_t *Data);



#endif
