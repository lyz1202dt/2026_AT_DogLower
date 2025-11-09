#include "run.h"
#include "usart.h"
#include "mylist.h"
#include "usbd_cdc_if.h"
#include "WatchDog2.h"
#include <string.h>

#define FRONT_LEFT  0
#define FRONT_RIGHT 1
#define BACK_LEFT   2
#define BACK_RIGHT  3

extern USBD_HandleTypeDef hUsbDeviceFS;

RS485_t go_rs485bus;

LegPack_t leg_target[4]={{.id=FRONT_LEFT},{.id=FRONT_RIGHT},{.id=BACK_LEFT},{.id=BACK_RIGHT}};
LegPack_t leg_state[4]={{.id=FRONT_LEFT},{.id=FRONT_RIGHT},{.id=BACK_LEFT},{.id=BACK_RIGHT}};
Leg_t leg[4]={
    {.joint1={.motor={.motor_id=0x01,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f},
     .joint2={.motor={.motor_id=0x02,.rs485=&go_rs485bus},.inv_motor=-1,.pos_offset=0.0f},
     .joint3={.motor={.motor_id=0x03,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f}},

    {.joint1={.motor={.motor_id=0x04,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f},
     .joint2={.motor={.motor_id=0x05,.rs485=&go_rs485bus},.inv_motor=-1,.pos_offset=0.0f},
     .joint3={.motor={.motor_id=0x06,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f}},

    {.joint1={.motor={.motor_id=0x01,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f},
     .joint2={.motor={.motor_id=0x02,.rs485=&go_rs485bus},.inv_motor=-1,.pos_offset=0.0f},
     .joint3={.motor={.motor_id=0x03,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f}},

    {.joint1={.motor={.motor_id=0x0A,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f},
     .joint2={.motor={.motor_id=0x0B,.rs485=&go_rs485bus},.inv_motor=-1,.pos_offset=0.0f},
     .joint3={.motor={.motor_id=0x0C,.rs485=&go_rs485bus},.inv_motor=1,.pos_offset=0.0f}}
};

void MotorControlTask(void* param)  //将数据发送到电机，并从电机接收数据
{
    RS485Init(&go_rs485bus,&huart1,GPIOB,GPIO_PIN_1);       //初始化485总线管理器
    TickType_t last_wake_time=xTaskGetTickCount();
    while(1)
    {
        GoMotorSend(&leg[BACK_LEFT].joint1.motor, leg[BACK_LEFT].joint1.exp_torque/6.33f*leg[BACK_LEFT].joint1.inv_motor,
            leg[BACK_LEFT].joint1.exp_omega*6.33f*leg[BACK_LEFT].joint1.inv_motor,
            leg[BACK_LEFT].joint1.exp_rad*6.33f*leg[BACK_LEFT].joint1.inv_motor+leg[BACK_LEFT].joint1.pos_offset,
            leg[BACK_LEFT].joint1.Kp, leg[BACK_LEFT].joint1.Kd);
        GoMotorRecv(&leg[BACK_LEFT].joint1.motor);
        GoMotorSend(&leg[BACK_LEFT].joint2.motor, leg[BACK_LEFT].joint2.exp_torque/6.33f*leg[BACK_LEFT].joint2.inv_motor,
            leg[BACK_LEFT].joint2.exp_omega*6.33f*leg[BACK_LEFT].joint2.inv_motor,
            leg[BACK_LEFT].joint2.exp_rad*6.33f*leg[BACK_LEFT].joint2.inv_motor+leg[BACK_LEFT].joint2.pos_offset,
            leg[BACK_LEFT].joint2.Kp, leg[BACK_LEFT].joint2.Kd);
        GoMotorRecv(&leg[BACK_LEFT].joint2.motor);
        GoMotorSend(&leg[BACK_LEFT].joint3.motor, leg[BACK_LEFT].joint3.exp_torque/6.33f*leg[BACK_LEFT].joint3.inv_motor,
            leg[BACK_LEFT].joint3.exp_omega*6.33f*leg[BACK_LEFT].joint3.inv_motor,
            leg[BACK_LEFT].joint3.exp_rad*6.33f*leg[BACK_LEFT].joint3.inv_motor+leg[BACK_LEFT].joint3.pos_offset,
            leg[BACK_LEFT].joint3.Kp, leg[BACK_LEFT].joint3.Kd);
        GoMotorRecv(&leg[BACK_LEFT].joint3.motor);
        vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(3));
    }
}


QueueHandle_t usb_send_semphr;
void MotorSendTask(void* param)     //将电机的数据发送到PC上
{
    usb_send_semphr=xSemaphoreCreateBinary();
    xSemaphoreTake(usb_send_semphr,0);
    TickType_t last_wake_time=xTaskGetTickCount();
    while(1)
    {
        for(int i=0;i<4;i++)    //填写数据并发送到PC
        {
            leg_state[i].leg.joint1.rad=leg[i].joint1.inv_motor*(leg[i].joint1.motor.state.rad-leg[i].joint1.pos_offset)/6.33f;
            leg_state[i].leg.joint2.rad=leg[i].joint2.inv_motor*(leg[i].joint2.motor.state.rad-leg[i].joint2.pos_offset)/6.33f;
            leg_state[i].leg.joint3.rad=leg[i].joint3.inv_motor*	(leg[i].joint3.motor.state.rad-leg[i].joint3.pos_offset)/6.33f;

            leg_state[i].leg.joint1.omega=leg[i].joint1.inv_motor*(leg[i].joint1.motor.state.velocity)/6.33f;
            leg_state[i].leg.joint2.omega=leg[i].joint2.inv_motor*(leg[i].joint2.motor.state.velocity)/6.33f;
            leg_state[i].leg.joint3.omega=leg[i].joint3.inv_motor*(leg[i].joint3.motor.state.velocity)/6.33f;
            
            leg_state[i].leg.joint1.torque=leg[i].joint1.inv_motor*(leg[i].joint1.motor.state.torque)*6.33f;
            leg_state[i].leg.joint2.torque=leg[i].joint2.inv_motor*(leg[i].joint2.motor.state.torque)*6.33f;
            leg_state[i].leg.joint3.torque=leg[i].joint3.inv_motor*(leg[i].joint3.motor.state.torque)*6.33f;

            CDC_Transmit_FS((uint8_t*)&leg_state[i],sizeof(LegPack_t));
            xSemaphoreTake(usb_send_semphr,pdMS_TO_TICKS(2));
        }
				vTaskDelayUntil(&last_wake_time,pdMS_TO_TICKS(3));
    }
}


QueueHandle_t usb_recv_queue;
void MotorRecvTask(void* param)     //从PC接收电机的期望值
{
    int leg_id=0;
    usb_recv_queue=xQueueCreate(8,sizeof(int));
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    while(1)
    {
        if(xQueueReceive(usb_recv_queue,&leg_id,pdMS_TO_TICKS(50))!=pdPASS)   //发生超时，说明通讯断开
        {
            //TODO:通过LED显示，清零所有电机期望，电机进入阻尼模式，整狗进入安全模式
            leg[leg_id].joint1.Kp=0.0f; //位置输出为0
            leg[leg_id].joint2.Kp=0.0f;
            leg[leg_id].joint3.Kp=0.0f;

            leg[leg_id].joint1.exp_torque=0.0f; //期望力矩前馈值为0
            leg[leg_id].joint2.exp_torque=0.0f;
            leg[leg_id].joint3.exp_torque=0.0f;

            leg[leg_id].joint1.exp_omega=0.0f;  //期望角速度为0
            leg[leg_id].joint2.exp_omega=0.0f;
            leg[leg_id].joint3.exp_omega=0.0f;

            continue;
        }

        //TODO:安全限幅并给到电机期望
        leg[leg_id].joint1.exp_rad=leg_target[leg_id].leg.joint1.rad;
        leg[leg_id].joint1.exp_omega=leg_target[leg_id].leg.joint1.omega;
        leg[leg_id].joint1.exp_torque=leg_target[leg_id].leg.joint1.torque;
        leg[leg_id].joint1.Kp=leg_target[leg_id].leg.joint1.kp;
        leg[leg_id].joint1.Kd=leg_target[leg_id].leg.joint1.kd;

        leg[leg_id].joint2.exp_rad=leg_target[leg_id].leg.joint2.rad;
        leg[leg_id].joint2.exp_omega=leg_target[leg_id].leg.joint2.omega;
        leg[leg_id].joint2.exp_torque=leg_target[leg_id].leg.joint2.torque;
        leg[leg_id].joint2.Kp=leg_target[leg_id].leg.joint2.kp;
        leg[leg_id].joint2.Kd=leg_target[leg_id].leg.joint2.kd;

        leg[leg_id].joint3.exp_rad=leg_target[leg_id].leg.joint3.rad;
        leg[leg_id].joint3.exp_omega=leg_target[leg_id].leg.joint3.omega;
        leg[leg_id].joint3.exp_torque=leg_target[leg_id].leg.joint3.torque;
        leg[leg_id].joint3.Kp=leg_target[leg_id].leg.joint3.kp;
        leg[leg_id].joint3.Kd=leg_target[leg_id].leg.joint3.kd;
    }
}

uint8_t My_CDC_SendCb(uint8_t *pbuf, uint32_t *Len, uint8_t epnum)
{
    BaseType_t temp;
    xSemaphoreGiveFromISR(usb_send_semphr,&temp);
    portYIELD_FROM_ISR(temp);
	return USBD_OK;
}

uint8_t recv_err_flag;
uint8_t My_CDC_RecvCb(uint8_t* Buf, uint32_t *Len)     //接收电机的期望并喂看门狗
{
    if(*Len!=sizeof(LegPack_t))
    {
        recv_err_flag=1;
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        return USBD_OK;
    }
    recv_err_flag=0;
    for(int i=0;i<4;i++)
    {
        if((*(uint32_t*)Buf)==leg_target[i].id)
        {
            memcpy(&leg_target[i],Buf+sizeof(uint32_t),sizeof(LegState_t));

            BaseType_t higherPriorityTaskWoken;
            xQueueSendFromISR(usb_recv_queue,&i,&higherPriorityTaskWoken);
            portYIELD_FROM_ISR(higherPriorityTaskWoken);

            break;
        }
    }
    return USBD_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		RS485SendIRQ_Handler(&go_rs485bus,huart);
	}
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if(huart->Instance==USART1)
	{
		RS485RecvIRQ_Handler(&go_rs485bus,huart,size);
	}
}

