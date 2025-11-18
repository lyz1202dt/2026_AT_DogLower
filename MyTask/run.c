#include "run.h"
#include "usart.h"
#include "mylist.h"
#include "usb_trans.h"
#include "WatchDog2.h"
#include <string.h>

#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

RS485_t go_rs485bus;

QueueHandle_t cdc_recv_semphr;

LegPack_t legs_target = {.pack_type = 0x00};
LegPack_t legs_state = {.pack_type = 0x00};
Leg_t leg[4] = {
    {.joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &go_rs485bus}, .inv_motor = -1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 0.0f}},

    {.joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &go_rs485bus}, .inv_motor = -1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 0.0f}},

    {.joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = -6.75066614f},
     .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &go_rs485bus}, .inv_motor = -1, .pos_offset = -1.0258497f},
     .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 18.6955833f}},

    {.joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &go_rs485bus}, .inv_motor = -1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &go_rs485bus}, .inv_motor = 1, .pos_offset = 0.0f}}};

void MotorControlTask(void *param) // 将数据发送到电机，并从电机接收数据
{
    RS485Init(&go_rs485bus, &huart6, GPIOA, GPIO_PIN_4); // 初始化485总线管理器
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        for (int j = 0; j < 3; j++)
        {
            GoMotorSend(&leg[BACK_LEFT].joint[j].motor, leg[BACK_LEFT].joint[j].exp_torque / 6.33f * leg[BACK_LEFT].joint[j].inv_motor,
                        leg[BACK_LEFT].joint[j].exp_omega * 6.33f * leg[BACK_LEFT].joint[j].inv_motor,
                        leg[BACK_LEFT].joint[j].exp_rad * 6.33f * leg[BACK_LEFT].joint[j].inv_motor + leg[BACK_LEFT].joint[j].pos_offset,
                        leg[BACK_LEFT].joint[j].Kp, leg[BACK_LEFT].joint[j].Kd);
            GoMotorRecv(&leg[BACK_LEFT].joint[j].motor);
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
    }
}

uint32_t cnt = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
    if(size==sizeof(LegPack_t)&&((LegPack_t*)src)->pack_type==0x00)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        memcpy(&legs_target, src, sizeof(LegPack_t));
        xSemaphoreGive(cdc_recv_semphr);
    }
    cnt++;
    //HAL_UART_Transmit_DMA(&huart3, src, size);
}

void MotorSendTask(void *param) // 将电机的数据发送到PC上
{
    USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        for (int i = 0; i < 4; i++) // 填写数据并发送到PC
        {
            for (int j = 0; j < 3; j++)
            {
                legs_state.leg[i].joint[j].rad = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.rad - leg[i].joint[j].pos_offset) / 6.33f;
                legs_state.leg[i].joint[j].omega = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.velocity) / 6.33f;
                legs_state.leg[i].joint[j].torque = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.torque) * 6.33f;
            }
        }
        CDC_Transmit_FS((uint8_t*)&legs_state, sizeof(legs_state));
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
    }
}

void MotorRecvTask(void *param) // 从PC接收电机的期望值
{
    cdc_recv_semphr = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semphr, 0);
    while (1)
    {
        if (xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(50)) != pdPASS) // 发生超时，说明通讯断开
        {
            // TODO:通过LED显示，清零所有电机期望，电机进入阻尼模式，整狗进入安全模式
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    leg[i].joint[j].exp_omega = 0.0f;
                    leg[i].joint[j].exp_torque = 0.0f;
                    leg[i].joint[j].Kp = 0.0f;
                    leg[i].joint[j].Kd = 0.0f;
                }
            }
            continue;
        }

        // TODO:安全限幅并给到电机期望
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                leg[i].joint[j].exp_rad = legs_target.leg[i].joint[j].rad;
                leg[i].joint[j].exp_omega = legs_target.leg[i].joint[j].omega;
                leg[i].joint[j].exp_torque = legs_target.leg[i].joint[j].torque;
                leg[i].joint[j].Kp = legs_target.leg[i].joint[j].kp;
                leg[i].joint[j].Kd = legs_target.leg[i].joint[j].kd;
            }
        }
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        RS485SendIRQ_Handler(&go_rs485bus, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART6)
    {
        RS485RecvIRQ_Handler(&go_rs485bus, huart, size);
    }
}
