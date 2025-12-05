/**
 * @file Canbuffer.h
 * @author Liu yuanzhao
 * @date 31-Dec-2024
 * @brief CAN连续多个数据包以中断模式发送（适用于短时间高密度CAN数据包发送）
 * @note 请在STM32CubeMx中开启发送完成中断
 */

#ifndef __CANBUFFER_H__
#define __CANBUFFER_H__

#include "stm32f4xx_hal.h"
#include <string.h>

#define CAN1_BUFFER_SIZE 8
#define CAN2_BUFFER_SIZE 8


typedef struct
{
    CAN_TxHeaderTypeDef head;
    uint8_t buffer[8];
}CanDataPack;

uint8_t CAN_Transmit(CAN_HandleTypeDef *hcan,CAN_TxHeaderTypeDef *head,uint8_t *buf);
static void CAN_SendCompleteServe(CAN_HandleTypeDef *hcan);

__weak uint8_t CAN_Transmit_Error(CAN_HandleTypeDef *hcan,HAL_StatusTypeDef state);

#endif
