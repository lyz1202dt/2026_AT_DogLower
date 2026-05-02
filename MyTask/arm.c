#include "arm.h"
#include "math.h"
#include <stdbool.h>
float b = 0;
int a=0;

TickType_t last_recv_tick =0;
bool first_packet_received = false;

GPIO_PinState gpio_pin_set;

volatile uint32_t robstride_last_rx_time = 0;//watchdog
volatile uint8_t watchdog_enable = 0;

//void RampToTarget();
int16_t current_output;
int allow=0;
extern int Temp_Servo_Target[4];
extern int32_t Servo_assignment[4];
float expect_ = 0;
//state_pack_t state_pack = {.pack_type = 0x01};
target_pack_t target_pack = {.pack_type = 0x01};
QueueHandle_t cdc_recv_semp;

 GM6020_TypeDef GM6020_state;
 RobStride_t robstride_state;

RobStride_t robstride01
={
.hcan = &hcan1,
.motor_id = 0x01,	
}
;
float arm_except = 0.0f;

float GM6020_forward_rad = 0.0f;
uint16_t last_cur = 0;  
uint16_t now_cur = 0;   
uint16_t cur_offset = 0; 
 uint8_t cur_read = 0;   
int32_t cur_round = 0;   

float current_vel = 0.0f;     
float expected_vel = 0.0f;    


uint16_t can_out[4];

float current_cur = 0.0f;

float MAX_VEL = 90.f;

GM6020_PID G_PID_SET= {
    .GM6020_pos = {
        .Kp = 10.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .limit = 0.0f,
        .output_limit = 16000.0f,
    },
    .GM6020_vel = {
        .Kp = 42.0f,
        .Ki = 0.0f,
        .Kd = 10.0f,
        .limit = 0.0f,
        .output_limit = 16000.0f,
    }
};

robstride_PID R_PID_SET={
    .RobStride_pos = {
        .Kp = 10.3f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .limit = 0,
        .output_limit = 100,
    },
     .RobStride_vel = {
        .Kp = 3.8f,
        .Ki = 1.0f,
        .Kd = 0.0f,
        .limit = 0.0f,
        .output_limit = 100,
    }
};

TaskHandle_t air_pump_Handle;
void air_pump(void *argument){
	for(;;){
//		gpio_pin_set = 1;
//		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, gpio_pin_set);
		if(target_pack.arm_pump == 1){
			 gpio_pin_set = 1;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, gpio_pin_set);
		}else if(target_pack.arm_pump == 0){
			 gpio_pin_set = 0;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7, gpio_pin_set);
		}
		
		vTaskDelay (5);
		
	}
}






TaskHandle_t servo_Serve_Handle;
void servo_Serve(void *argument)
{
	for(;;){
		
//		if(target_pack.servo1.up>0 && target_pack.servo1.up<1.5708)
//		{
//			Servo_assignment[0] = (int)(target_pack.servo1.up*700.0/PAI);
//			
//		}else if(target_pack.servo1.up>=1.5708 && target_pack.servo1.up<3.1415)
//		{
//			Servo_assignment[0] = (int)(target_pack.servo1.up*1400.0/PAI);
//			
//		}
		
		 Servo_assignment[0] = (int)(target_pack.servo1.up*1400.0/PAI);
		 Servo_assignment[1] = (int)(target_pack.servo1.low*1300.0/PAI);
		
		if(target_pack.servo1.low>1.8){
			Servo_assignment[1] = Servo_assignment[1] + 50;
		}
		
		
//    Servo_assignment[0] = (int)(target_pack.servo1.up*1300.0/PAI);
//    Servo_assignment[1]= (int)(target_pack.servo1.low*770.0/PAI*2.0);
////		if(Servo_assignment[0]>(2000.0f/270.0f*180.0f)){
////			Servo_assignment[0]=(2000.0f/270.0f*180.0f);
////		}
////		if(Servo_assignment[0]<0){
////			Servo_assignment[0]=0;
////		} 
////		if(Servo_assignment[1]>(2000.0f/270.0f*90.0f)){
////			Servo_assignment[1]=(2000.0f/270.0f*90.0f); 
////		}
////		if(Servo_assignment[1]<0){
////			Servo_assignment[1]=0;
////		}
//		if(Servo_assignment[0]>2000){
//			Servo_assignment[0]=2000 ;
//		}
//		if(Servo_assignment[0]<0){
//			Servo_assignment[0]=0; 
//		} 
//		if(Servo_assignment[1]>980){
//			Servo_assignment[1]=980; 
//		}
//		if(Servo_assignment[1]<0){
//			Servo_assignment[1]=0;
//		}
	  Servo_control();  
  	vTaskDelay (5);
	}
}

TaskHandle_t stride_Serve_Handle;
void stride_Serve(void *argument)
{
	
	vTaskDelay(5000);
    RobStrideInit(&robstride_state,&hcan1,0x02,RobStride_01);
    RobStrideSetMode(&robstride_state, RobStride_Torque);
    vTaskDelay(100);
    RobStrideEnable(&robstride_state);
	vTaskDelay(100);
	RobStrideResetAngle(&robstride_state);
    TickType_t last_wake = xTaskGetTickCount();
     for(;;)
     {
			 
     // arm_except=  target_pack.rob01.except_pos*1.50;
		
    //    RobStrideMotionControl(&robstride_state,0x02,target_pack.rob01.except_torque,
	// 		 arm_except,target_pack.rob01.except_omega*1.50,
	// 		 target_pack.rob01.kp,target_pack.rob01.kd);
 PID_Control2(robstride_state.state.rad,
             target_pack.rob01.except_pos,
             &R_PID_SET.RobStride_pos);
if(R_PID_SET.RobStride_pos.pid_out > 20.0f)  R_PID_SET.RobStride_pos.pid_out = 20.0f;
if(R_PID_SET.RobStride_pos.pid_out < -20.0f) R_PID_SET.RobStride_pos.pid_out = -20.0f;
PID_Control2(robstride_state.state.omega,
             R_PID_SET.RobStride_pos.pid_out, 
             &R_PID_SET.RobStride_vel);
if(R_PID_SET.RobStride_vel.pid_out > 60.0f)  R_PID_SET.RobStride_vel.pid_out = 60.0f;
if(R_PID_SET.RobStride_vel.pid_out < -60.0f) R_PID_SET.RobStride_vel.pid_out = -60.0f;
RobStrideTorqueControl(&robstride_state,R_PID_SET.RobStride_vel.pid_out);// R_PID_SET.RobStride_vel.pid_out
vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
     }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  	uint8_t CAN2_buffer[8];
   if(hcan->Instance == CAN2)
   {
		 	a = 1;
			 CAN_Receive_DataFrame(&hcan2,CAN2_buffer);
			 GM6020_Receive(&GM6020_state,CAN2_buffer);
			if(!cur_read)
	    {
		      cur_read = 1;
		      cur_offset = ((CAN2_buffer[1] & 0xff) | ((uint16_t)CAN2_buffer[0]<<8));
	     }
			  if(cur_read)
     {
          last_cur = now_cur;
          now_cur = ((CAN2_buffer[1] & 0xff) | ((uint16_t)CAN2_buffer[0]<<8));
		
         if((int16_t)(now_cur - last_cur + 1e-5) > 4000) cur_round--;
         else if((int16_t)(now_cur - last_cur +1e-5) < -4000) cur_round++;
       }
   }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	uint8_t CAN1_buffer[8];
     if(hcan->Instance == CAN1)
   {
    uint32_t ID = CAN_Receive_DataFrame(&hcan1,CAN1_buffer);
		RobStrideRecv_Handle(&robstride_state, &hcan1, ID, CAN1_buffer);
		 
		  //robstride_last_rx_time = xTaskGetTickCount();
   }
}


TaskHandle_t GM6020_Serve_Handle;
void GM6020_Serve(void *argument)//发给6020电机的角度要乘4.5
{

	 TickType_t last_wake = xTaskGetTickCount();
	 for(;;)
	{
   expect_=(GM6020_forward_rad * 180.0f / PAI)*4.5;
	 
 if(expect_>360 || expect_<-360)
{
    expect_ = fmod(expect_, 360.0f);
	
}

  if(expect_>70.0)
  {
    expect_ = 70.0;
	
  }
  if(expect_<-70.0)
	{
	   expect_ = -70.0;
	}


		current_cur =  now_cur * 360.0f / 8192.0f - cur_offset* 360.0f / 8192.0f; 

  PID_Control2(current_cur,expect_,&G_PID_SET.GM6020_pos);
		
  expected_vel = G_PID_SET.GM6020_pos.pid_out;
  if(expected_vel > MAX_VEL) expected_vel = MAX_VEL;
  if(expected_vel < -MAX_VEL) expected_vel = -MAX_VEL;

  PID_Control2( (float)GM6020_state.Speed,expected_vel,&G_PID_SET.GM6020_vel);//expected_vel

  current_output = (int16_t)G_PID_SET.GM6020_vel.pid_out;  
 

  if(current_output > 16384) current_output = 16384;
  if(current_output < -16384) current_output = -16384;
	
  can_out[1] = current_output;     // ��8λ
 
   MotorSend(&hcan2,0x1FE,(int16_t*)can_out);
   vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
	}
}

//TaskHandle_t usb_cdc_Send_Handle;
//void usb_cdc_Send(void *arguments)
//{

//    TickType_t last_wake_time = xTaskGetTickCount();
//	
//	for(;;){
//  
//		state_pack.GM6020.Angle_DEG=(now_cur * 360.0f / 8192.0f - cur_offset* 360.0f / 8192.0f)*PAI/180.0f;
//		if(allow==1)    {
//		CDC_Transmit_FS((uint8_t*)&state_pack, sizeof(state_pack));
//        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
//	}
//	}
//}
void CDC_Receive_Cb(uint8_t *src, uint16_t size);
TaskHandle_t usb_cdc_Receive_Handle;
void usb_cdc_Receive(void *argument)
{
	  USB_CDC_Init(CDC_Receive_Cb, NULL, NULL);
    cdc_recv_semp = xSemaphoreCreateBinary();
		vTaskDelay(1000);
	for(;;){
		

		taskENTER_CRITICAL();
		bool has_received = first_packet_received;
		TickType_t last_tick = last_recv_tick;
		taskEXIT_CRITICAL();

		if(has_received)
		{
			TickType_t now_recv_tick =  xTaskGetTickCount();
			if(now_recv_tick - last_tick >20)
			{
				R_PID_SET.RobStride_pos.Kp = 0;
//				R_PID_SET.RobStride_pos.Kd = 0;
				
			}
				
		}
		
    if(xSemaphoreTake(cdc_recv_semp, portMAX_DELAY) == pdTRUE)
    {
    GM6020_forward_rad = target_pack.rob02.target_pos;
    }
		
		vTaskDelay(5);
		allow=1;
	}
}

uint32_t cur_size=0;
uint32_t count = 0;
target_pack_t* pack = NULL;
void CDC_Receive_Cb(uint8_t *src, uint16_t size)
{
		pack = (target_pack_t*)src;
    if(size==sizeof(target_pack_t)&&((target_pack_t*)src)->pack_type==0x01)
    {
        
        memcpy(&target_pack, src, sizeof(target_pack_t));
				last_recv_tick = xTaskGetTickCount();
				first_packet_received = true;
        xSemaphoreGive(cdc_recv_semp);
    }
        count++;
        cur_size=size;
}


//TaskHandle_t watch_dog_Handle;
//void watch_dog(void *argument)
//{
//    TickType_t last_wake = xTaskGetTickCount();

//    // 上电延时5秒
//    vTaskDelay(pdMS_TO_TICKS(5000));

//    watchdog_enable = 1;  // 开启看门狗

//    for(;;)
//    {
//        uint32_t now = xTaskGetTickCount();

//        if(watchdog_enable && (TickType_t)(now - robstride_last_rx_time) > pdMS_TO_TICKS(20))
//        {
//            // 先断力
//            RobStrideTorqueControl(&robstride_state, 0);

//            // 锁死系统
//            taskDISABLE_INTERRUPTS();
//            while(1);
//        }

//        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(2));
//    }
//}

