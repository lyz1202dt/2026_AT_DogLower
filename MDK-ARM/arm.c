#include "FreeRTOS.h"
#include "RobStride.h"
#include "arm.h"
#include "can.h"
#include "CANDrive.h"
#include "motor.h"
#include "PID_old.h"
#include "main.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"

int allow=0;
extern int Temp_Servo_Target[4];
PID2 GM6020_pos;
PID2 GM6020_vel;
int16_t send_buf=0;
target_pack_t target_pack = {.pack_type = 0x00};
state_pack_t state_pack = {.pack_type = 0x00};
QueueHandle_t cdc_recv_semp;

TaskHandle_t servo_Serve_Handle;
void servo_Serve(void *argument)
{
	Temp_Servo_Target[0] = target_pack.servo1.up;
	Temp_Servo_Target[1]= target_pack.servo1.low;
	state_pack.servo2.low = target_pack.servo1.low;
	state_pack.servo2.up = target_pack.servo1.up;
	Servo_control();  
}

TaskHandle_t stride_Serve_Handle;
void stride_Serve(void *argument)
{
    vTaskDelay(2000);
    RobStrideInit(&state_pack.robstride01,&hcan2,0x01,RobStride_01);
    RobStrideSetMode(&state_pack.robstride01, RobStride_MotionControl);
    vTaskDelay(100);
    RobStrideEnable(&state_pack.robstride01);
    TickType_t last_wake = xTaskGetTickCount();
     for(;;)
     {
        RobStrideMotionControl(&state_pack.robstride01,0x01,target_pack.rob01.except_torque,
			 target_pack.rob01.except_pos,target_pack.rob01.except_omega,
			 target_pack.rob01.kp,target_pack.rob01.kd);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
     }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan2)
{
	uint8_t robstride_buf[8];
	uint8_t GM6020_buf[8];
	uint32_t ID = CAN_Receive_DataFrame(hcan2, robstride_buf);
	RobStrideRecv_Handle(&state_pack.robstride01, hcan2, ID, robstride_buf);
	GM6020_Receive(&state_pack.GM6020,GM6020_buf);
}

TaskHandle_t GM6020_Serve_Handle;
void GM6020_Serve(void *argument)
{
	
	 vTaskDelay(1000);
	 PID_Control2(target_pack.rob02.target_pos,state_pack.GM6020.Angle,&GM6020_pos);
	 target_pack.rob02.target_vel = GM6020_pos.pid_out;
	 PID_Control2(target_pack.rob02.target_pos,state_pack.GM6020.Speed,&GM6020_vel);
	 send_buf = (int16_t)GM6020_vel.pid_out;
	 for(;;)
	{
	 TickType_t last_wake = xTaskGetTickCount();
	 MotorSend(&hcan1,201,&send_buf);
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
	}
}

TaskHandle_t usb_cdc_Send_Handle;
void usb_cdc_Send(void *arguments)
{
	  USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
	if(allow==1)    {
				CDC_Transmit_FS((uint8_t*)&state_pack, sizeof(state_pack));
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
	}
}

TaskHandle_t usb_cdc_Receive_Handle;
void usb_cdc_Receive(void *argument)
{
    cdc_recv_semp = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semp, 0);
		vTaskDelay(1000);
	  allow=1;
}
void pid_init( 
	  PID2*pid,
	  float kp, 
		float ki, 
		float kd,  
		float limit,     
    float error_now,  
    float error_last,
    float error_inter,
    float pid_out,
    float output_limit)
{
	pid->Kp=kp;
	pid->Ki=ki;
  pid->Kd=kd;
  pid->limit=limit;
	pid->error_now=error_now;
	pid->error_last=error_last;
	pid->error_inter=error_inter;
	pid->pid_out=pid_out;
	pid->output_limit=output_limit;
}
uint32_t cur_size=0;
uint32_t count = 0;
void CDC_Receive_Cb(uint8_t *src, uint16_t size)
{
    if(size==sizeof(target_pack_t)&&((target_pack_t*)src)->pack_type==0x00)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        memcpy(&target_pack, src, sizeof(target_pack_t));
        xSemaphoreGive(cdc_recv_semp);
    }
        count++;
        cur_size=size;
}