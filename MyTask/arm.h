#ifndef __ARM_H_
#define __ARM_H_
//#include "arm.h"
#include "can.h"
#include "CANDrive.h"
#include "main.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "FreeRTOS.h"
#include "task.h"
#include "PID_old.h"
#include "RobStride.h"
#include "motor.h"
#include "task_init.h"
#include "kfc_grasp and release.h"

#define PAI 3.14159265358979323846f
extern TaskHandle_t servo_Serve_Handle;
extern TaskHandle_t stride_Serve_Handle;
extern TaskHandle_t GM6020_Serve_Handle;
extern TaskHandle_t usb_cdc_Send_Handle;
extern TaskHandle_t usb_cdc_Receive_Handle;
extern TaskHandle_t air_pump_Handle;
extern TaskHandle_t watch_dog_Handle;

typedef struct
{
    float except_torque;
    float except_pos;
    float except_omega;
    float kp;
	float kd; 
}Expect_Robstride;




//typedef struct
//{}
typedef struct
{
	float left_up;
	float left_low;
	float left_down;
	float right_up;
	float right_low;
	float right_down;
}servo;


#pragma pack(1)
typedef struct 
{
 int pack_type;
 int arm_pump_left;
 int arm_pump_right;
 servo servo1;
 Expect_Robstride rob01;
}target_pack_t;  

#pragma pack()

typedef struct
{
  PID2 GM6020_pos;
  PID2 GM6020_vel;
}GM6020_PID;

typedef struct 
{
	PID2 RobStride_pos;
	PID2 RobStride_vel;
}robstride_PID;

//typedef struct
//{
//	int pack_type;
//	servo servo2;
//	RobStride_t robstride01;
//	GM6020_TypeDef GM6020;
//	
//}state_pack_t;   //���͸���λ���Ľṹ��

void servo_Serve(void *argument);
void stride_Serve(void *argument);
void GM6020_Serve(void *argument);
void usb_cdc_Send(void *argument);
void usb_cdc_Receive(void *argument);
void CDC_Recv_Cb(uint8_t *src, uint16_t size);
void air_pump(void *argument);
void watch_dog(void *argument);

		
#endif
