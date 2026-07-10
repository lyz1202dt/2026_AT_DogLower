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
#include "RobStride2.h"
#include "motor.h"
#include "task_init.h"
#include "kfc_grasp and release.h"

#define PI 3.14159265358979323846f
#define ANGLE_270_RAD (270.0f * PI / 180.0f)
#define ANGLE_180_RAD (180.0f * PI / 180.0f)
#define PUMP_START 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, 1)
#define PUMP_OFF 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, 0)
#define AIR_VALUE_START 	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, 1)
#define AIR_VALUE_OFF 		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, 0)
#define READ_DISTANCE_PIN 	HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_13)

extern TaskHandle_t servo_Serve_Handle;
extern TaskHandle_t stride_Serve_Handle;
extern TaskHandle_t GM6020_Serve_Handle;
extern TaskHandle_t usb_cdc_Send_Handle;
extern TaskHandle_t usb_cdc_Receive_Handle;
extern TaskHandle_t air_pump_Handle;
extern TaskHandle_t watch_dog_Handle;
extern TaskHandle_t radiation_distance_Handle;


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
	float up;
	float middle;
	float down;
}servo;


#pragma pack(1)
typedef struct 
{
 int pack_type;
 int arm_pump;
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

#pragma pack(1)
typedef struct
{
	int pack_type;
	float red_distance;
}state_pack_t;   //���͸���λ���Ľṹ��
#pragma pack()

void servo_Serve(void *argument);
void stride_Serve(void *argument);
void GM6020_Serve(void *argument);
void usb_cdc_Send(void *argument);
void usb_cdc_Receive(void *argument);
void CDC_Recv_Cb(uint8_t *src, uint16_t size);
void air_pump(void *argument);
void watch_dog(void *argument);
void radiation_distance(void *argument);
void parse_vl53_data(char *buf, uint16_t len);

		
#endif
