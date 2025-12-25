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

uint32_t err_timer_cnt=0;

RS485_t rs485bus;

QueueHandle_t cdc_recv_semphr;

LegPack_t legs_target = {.pack_type = 0x00};
LegPack_t legs_state = {.pack_type = 0x00};
Leg_t leg[4] = {
    {.joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .wheel={.hcan=&hcan1,.ID=0x201}},

    {.joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .wheel={.hcan=&hcan1,.ID=0x202}},

    {.joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .wheel={.hcan=&hcan1,.ID=0x203}},

    {.joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 0.0f},
     .wheel={.hcan=&hcan1,.ID=0x204
}}};

int32_t remain_time=0;
void MotorControlTask(void *param) // 将数据发送到电机，并从电机接收数据
{
    RS485Init(&rs485bus, &huart6, GPIOA, GPIO_PIN_4); // 初始化485总线管理器
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            for(int j=0;j<3;j++)
            {
                GoMotorSend(&leg[i].joint[j].motor, leg[i].joint[j].exp_torque / 6.33f * leg[i].joint[j].inv_motor,
                        leg[i].joint[j].exp_omega * 6.33f * leg[i].joint[j].inv_motor,
                        leg[i].joint[j].exp_rad * 6.33f * leg[i].joint[j].inv_motor + leg[i].joint[j].pos_offset,
                        leg[i].joint[j].Kp, leg[i].joint[j].Kd);
                GoMotorRecv(&leg[i].joint[j].motor);
            }
        }
		uint32_t temp=HAL_GetTick();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
		remain_time=HAL_GetTick()-temp;
    }
}

uint32_t current_size=0;
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
		current_size=size;
    //HAL_UART_Transmit_DMA(&huart3, src, size);
}

PID2 wheel_vel_pid[4];
float wheel_exp_vel[4],wheel_exp_torque[4];
int16_t can_send_buf[4];
void WheelControlTask(void* param)
{
    TickType_t last_wake_time=xTaskGetTickCount();
    leg[0].wheel.vel_pid.Kp=1.0f;
    leg[0].wheel.vel_pid.Ki=1.0f;
    leg[0].wheel.vel_pid.limit=100.0f;
    leg[0].wheel.vel_pid.output_limit=4.0f;
    leg[3].wheel.vel_pid=leg[2].wheel.vel_pid=leg[1].wheel.vel_pid=leg[0].wheel.vel_pid;
    while(1)
    {
        for(int i=0;i<4;i++)
        {
            PID_Control2(leg[i].wheel.motor.Speed*3.14159265f*2.0f/60.0f/19.0f,wheel_exp_vel[i],&leg[i].wheel.vel_pid);
            can_send_buf[i]=(int16_t)((leg[i].wheel.vel_pid.pid_out+wheel_exp_torque[i])/0.3f*(16384.0f/20.0f/0.3f));
        }
        MotorSend(&hcan1,0x200,can_send_buf);
        vTaskDelayUntil(&last_wake_time,2);
    }
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
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(8));
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
            wheel_exp_vel[i]=legs_target.leg[i].wheel.omega;
            wheel_exp_torque[i]=legs_target.leg[i].wheel.torque;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t buf[8];
    if(hcan->Instance==CAN1)
    {
        uint32_t id=CAN_Receive_DataFrame(hcan,buf);
        Motor3508Recv(&leg[0].wheel,hcan,id,buf);
        Motor3508Recv(&leg[1].wheel,hcan,id,buf);
        Motor3508Recv(&leg[2].wheel,hcan,id,buf);
        Motor3508Recv(&leg[3].wheel,hcan,id,buf);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART6)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
		err_timer_cnt=0;	//每接收一次，就清零
    }
}

uint32_t error_cnt=0;

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
			HAL_UART_DMAStop(huart);
        // 清除错误标志（HAL 已处理大部分，但 ORE 仍需要手动清）
      __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_OREFLAG(huart);

        // 3. 清除 huart->ErrorCode，否则 HAL 会认为错误仍然存在
        huart->ErrorCode = HAL_UART_ERROR_NONE;
			
			huart->RxState = HAL_UART_STATE_READY;
			
			RS485RecvIRQ_Handler(&rs485bus, huart, 0);
			error_cnt++;
    }
}
