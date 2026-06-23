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
extern int Temp_Servo_Target[3];//舵机的期望占空比，应该用来接收上位机的期望
extern int32_t Servo_assignment[3];//舵机的预设置
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


// //灵足电机pid
// robstride_PID R_PID_SET={
//     .RobStride_pos = {
//         .Kp = 18.0f,//18
//         .Ki = 0.0f,
//         .Kd = 0.0f,
//         .limit = 0.0f,
//         .output_limit = 20,//20
//     },
//      .RobStride_vel = {
//         .Kp = 2.2f,//2.2
//         .Ki = 0.1f,//0.1
//         .Kd = 0.0f,
//         .limit = 7.0f,//7
//         .output_limit = 50,//50
//     }
// };


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

float Servo_offset_angle[3] = {59, 0, 0};
int32_t Servo_offset[3] = {0, 0, 0};
void servo_Serve(void *argument)
{
    int32_t Servo_assignment[3] = {0,0,0}; 
    uint8_t pwm_start = 0;
    target_pack.servo1.up = -1.03f;
    target_pack.servo1.middle = 0.0f;
    target_pack.servo1.down = 0.0f;

    // for(int i = 0;i < 3;i++)    //调试时使用,不调试时放到for循环之前只执行一次即可
    // {
    //     if(i == 1)
    //         Servo_offset[i] = Servo_offset_angle[i]*2000/180;
    //     else
    //         Servo_offset[i] = Servo_offset_angle[i]*2000/270;
    // }

	for(;;)
    {	
        for(int i = 0;i < 3;i++)    //调试时使用,不调试时放到for循环之前只执行一次即可
        {
            if(i == 1)
                Servo_offset[i] =  Servo_offset_angle[i]*2000/180;
            else
                Servo_offset[i] =  Servo_offset_angle[i]*2000/270;
        }

		Servo_assignment[0] = (int)(target_pack.servo1.up*2000.0/ANGLE_270_RAD);	    //PE9_TIM1_CH1_UP
		Servo_assignment[1] = (int)(target_pack.servo1.middle*2000.0/ANGLE_180_RAD);	//PE11_TIM1_CH2_MIDDLE
		Servo_assignment[2] = (int)(target_pack.servo1.down*2000.0/ANGLE_270_RAD);	    //PE14_TIM1_CH4_DOWN

        /*------以上为弧度控制，以下为角度控制------*/

        // Servo_assignment[0] = (int32_t)(target_pack.servo1.up*2000.0/270);	   
		// Servo_assignment[1] = (int32_t)(target_pack.servo1.middle*2000.0/180);
		// Servo_assignment[2] = (int32_t)(target_pack.servo1.down*2000.0/270);	

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500+Servo_offset[0]+Servo_assignment[0]);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,500+Servo_offset[1]+Servo_assignment[1]);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,500+Servo_offset[2]+Servo_assignment[2]); 
        if(!pwm_start)
       { 
            HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
            pwm_start = 1;
       }
  	    vTaskDelay(5);
	}
}

float offset = -0.95f; //机械臂灵足电机认为的零位对应的弧度
float stride_kp = 700.0f; 
float stride_kd = 4.0f; 
TaskHandle_t stride_Serve_Handle;
void stride_Serve(void *argument)
{
	vTaskDelay(5000);

    RobStrideInit(&robstride_state,&hcan1,0x02,RobStride_02);

    RobStrideSetMode(&robstride_state, RobStride_MotionControl);
	vTaskDelay(5);
	RobStrideSetMode(&robstride_state, RobStride_MotionControl);
	vTaskDelay(5);
	RobStrideSetMode(&robstride_state, RobStride_MotionControl);
    vTaskDelay(5);

    RobStrideEnable(&robstride_state);
	vTaskDelay(5);
	RobStrideEnable(&robstride_state);
	vTaskDelay(5);
	RobStrideEnable(&robstride_state);
    vTaskDelay(5);

  
	//RobStrideResetAngle(&robstride_state);
	
    TickType_t last_wake_time = xTaskGetTickCount();
    for(;;)
    {	 
     // arm_except=  target_pack.rob01.except_pos*1.50;
		
    //    RobStrideMotionControl(&robstride_state,0x02,target_pack.rob01.except_torque,
	// 		 arm_except,target_pack.rob01.except_omega*1.50,
	// 		 target_pack.rob01.kp,target_pack.rob01.kd);

        // PID_Control2(robstride_state.state.rad,
        //             target_pack.rob01.except_pos+offset,
        //             &R_PID_SET.RobStride_pos);
        // if(R_PID_SET.RobStride_pos.pid_out > 20.0f)  R_PID_SET.RobStride_pos.pid_out = 20.0f;
        // if(R_PID_SET.RobStride_pos.pid_out < -20.0f) R_PID_SET.RobStride_pos.pid_out = -20.0f;

        // PID_Control2(robstride_state.state.omega,
        //             R_PID_SET.RobStride_pos.pid_out, 
        //             &R_PID_SET.RobStride_vel);
        // if(R_PID_SET.RobStride_vel.pid_out > 60.0f)  R_PID_SET.RobStride_vel.pid_out = 60.0f;
        // if(R_PID_SET.RobStride_vel.pid_out < -60.0f) R_PID_SET.RobStride_vel.pid_out = -60.0f;

        // RobStrideTorqueControl(&robstride_state,R_PID_SET.RobStride_vel.pid_out);

        RobStrideMotionControl(&robstride_state,target_pack.rob01.except_torque,
            target_pack.rob01.except_pos+offset,
            target_pack.rob01.except_omega,
            stride_kp,
            stride_kd
        );

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



