#include "arm.h"
#include "math.h"
#include <stdbool.h>

#define VL53_RX_SIZE 128
float b = 0;
int a=0;

TickType_t last_recv_tick =0;
bool first_packet_received = false;//是否第一次接收上位机传来的数据的判断标志

GPIO_PinState gpio_pin_set;//高电平和低电平的宏定义

//void RampToTarget();
int16_t current_output;
int allow=0;//只有接收到了一次上位机发来的数据才允许给上位机发送数据
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern int Temp_Servo_Target[4];//舵机的期望占空比，应该用来接收上位机的期望
extern int32_t Servo_assignment[4];//舵机的预设置
float expect_ = 0;
state_pack_t state_pack = {.pack_type = 0x01};
target_pack_t target_pack = {.pack_type = 0x01};//用来接收上位机发来的数据包

uint8_t receiving_number = 0;

extern uint8_t vl53_rx_buf[VL53_RX_SIZE];
extern volatile uint16_t vl53_distance;
// volatile uint8_t ret;

//灵足电机的相关代码
RobStride_t robstride_state;

RobStride_t robstride01
={
.hcan = &hcan1,
.motor_id = 0x01,	
};


float arm_except = 0.0f;


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


//灵足电机pid
robstride_PID R_PID_SET={
    .RobStride_pos = {
        .Kp = 0.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .limit = 0,
        .output_limit = 40,
    },
     .RobStride_vel = {
        .Kp = 0.1f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .limit = 0.0f,
        .output_limit = 5,
    }
};

TaskHandle_t radiation_distance_Handle;
void radiation_distance(void *argument)
{
	vTaskDelay(pdMS_TO_TICKS(5000));
    for(;;)
    {
        state_pack.red_distance = (int)vl53_distance;     
        CDC_Transmit_FS((uint8_t *)&state_pack,sizeof(state_pack));  
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


TaskHandle_t air_pump_Handle;
void air_pump(void *argument){
	for(;;)
    {
		if(target_pack.arm_pump == 1)
        {	
			PUMP_START;
            AIR_VALUE_OFF;
		}
        else if(target_pack.arm_pump == 0)
        {
			PUMP_OFF;   
            AIR_VALUE_START;
		}
		
		vTaskDelay(5);
	}
}




TaskHandle_t servo_Serve_Handle;
void servo_Serve(void *argument)
{
	for(;;)
    {	
			// Servo_assignment[0] = (int)(target_pack.servo1.up*2000.0/ANGLE_270_RAD);	//PE9_TIM1_CH1_UP
			// Servo_assignment[1] = (int)(target_pack.servo1.middle*2000.0/ANGLE_270_RAD);	//PE11_TIM1_CH2_MIDDLE
			// Servo_assignment[3] = (int)(target_pack.servo1.down*2000.0/ANGLE_270_RAD);	//PE14_TIM1_CH4_DOWN
            Servo_assignment[0] = (int)(target_pack.servo1.up*2000.0/270);	    //PE9
			Servo_assignment[1] = (int)(target_pack.servo1.middle*2000.0/270);	//PE11
			Servo_assignment[3] = (int)(target_pack.servo1.down*2000.0/270);	//PE14
	        Servo_control();  

  	    vTaskDelay (5);
	}
}


TaskHandle_t stride_Serve_Handle;
void stride_Serve(void *argument)
{
	vTaskDelay(5000);
    RobStrideInit(&robstride_state,&hcan1,0x01,RobStride_EL05);
	vTaskDelay(100);
    RobStrideSetMode(&robstride_state, RobStride_Torque);
    vTaskDelay(100);
    RobStrideEnable(&robstride_state);
	vTaskDelay(100);
	RobStrideResetAngle(&robstride_state);
	
    TickType_t last_wake_time = xTaskGetTickCount();
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
        PID_Control2(robstride_state.state.omega,//0,
                    R_PID_SET.RobStride_pos.pid_out, 
                    &R_PID_SET.RobStride_vel);
        if(R_PID_SET.RobStride_vel.pid_out > 60.0f)  R_PID_SET.RobStride_vel.pid_out = 60.0f;
        if(R_PID_SET.RobStride_vel.pid_out < -60.0f) R_PID_SET.RobStride_vel.pid_out = -60.0f;
        RobStrideTorqueControl(&robstride_state,R_PID_SET.RobStride_vel.pid_out);// R_PID_SET.RobStride_vel.pid_out
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(4));
    }
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	uint8_t CAN1_buffer[8];
     if(hcan->Instance == CAN1)
   {
    uint32_t ID = CAN_Receive_DataFrame(&hcan1,CAN1_buffer);
		RobStrideRecv_Handle(&robstride_state, &hcan1, ID, CAN1_buffer);
		 
		 
   }
}



void CDC_Receive_Cb(uint8_t *src, uint16_t size);
TaskHandle_t usb_cdc_Receive_Handle;
void usb_cdc_Receive(void *argument)
{
	USB_CDC_Init(CDC_Receive_Cb, NULL, NULL);   
	vTaskDelay(1000);

	for(;;)
    {
		taskENTER_CRITICAL();
		bool has_received = first_packet_received;
		TickType_t last_tick = last_recv_tick;
		taskEXIT_CRITICAL();

		if(has_received)
		{
			TickType_t now_recv_tick =  xTaskGetTickCount();
			if(now_recv_tick - last_tick >20)
			{
				//R_PID_SET.RobStride_pos.Kp = 0;
				//R_PID_SET.RobStride_pos.Kd = 0;
				
			}	
		}	  
		vTaskDelay(5);	
	}
}

uint32_t cur_size=0;

void CDC_Receive_Cb(uint8_t *src, uint16_t size)
{
    if(size==sizeof(target_pack_t)&&((target_pack_t*)src)->pack_type==0x01)
    {
        memcpy(&target_pack, src, sizeof(target_pack_t));

		last_recv_tick = xTaskGetTickCount();
		first_packet_received = true;
        allow=1;
    }

    cur_size=size;
}



void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    if(huart->Instance == USART3)
    {
			
        vl53_rx_buf[Size] = '\0';

        char *p = strstr((char *)vl53_rx_buf, "d:");

        if(p != NULL)
        {
            int dist;

            if(sscanf(p, "d: %d", &dist) == 1)
            {
                vl53_distance = (uint16_t)dist;
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA( &huart3,vl53_rx_buf,VL53_RX_SIZE);
				__HAL_DMA_DISABLE_IT(huart3.hdmarx,DMA_IT_HT);
    }
}



