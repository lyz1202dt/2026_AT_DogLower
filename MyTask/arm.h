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

typedef struct
{
    float except_torque;
    float except_pos;
    float except_omega;
    float kp;
	float kd; 
}Expect_Robstride;


typedef struct
{
	float target_pos;
	float target_vel;
	float kp;
	float kd;
	
}Expect_GM6020;



//typedef struct
//{}
typedef struct
{
	float up;
	float low;
}servo;


#pragma pack(1)
typedef struct 
{
 int pack_type;
 servo servo1;
 Expect_Robstride rob01;
 Expect_GM6020 rob02;
}target_pack_t;  

#pragma pack()

typedef struct
{
  PID2 GM6020_pos;
  PID2 GM6020_vel;
}GM6020_PID;
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

		
#endif
