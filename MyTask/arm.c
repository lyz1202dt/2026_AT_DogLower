#include "arm.h"
float b = 0;



int allow=0;
extern int Temp_Servo_Target[4];

int16_t current_output;
target_pack_t target_pack = {.pack_type = 0x01};
state_pack_t state_pack = {.pack_type = 0x01};
QueueHandle_t cdc_recv_semp;




RobStride_t robstride01
={
.hcan = &hcan1,
.motor_id = 0x02,	


}
;






//******GM6020电机有关数据定义*********************
float GM6020_forward_rad_ = 0.0;//调试用的数据

float GM6020_forward_rad = 0.0f;//上位机发来的期望弧度
uint16_t last_cur = 0;   //上一次读取的编码器原始值
uint16_t now_cur = 0;    //当前读取的编码器原始值
uint16_t cur_offset = 0; //电流编码器零偏值（用于校准）
uint8_t cur_read = 0;    //标志位，是否已经读取过电流编码器的值
int32_t cur_round = 0;   //电流编码器多圈计数（处理编码器溢出）

float current_vel = 0.0f;     //当前速度值 度每秒
float expected_vel = 0.0f;    //期望速度值，来自电流环输出


uint8_t can_out[8] = {0};//CAN发送数据缓冲区

float current_cur = 0.0f;//当前位置

float MAX_VEL = 90.f;//转动最大速度限制




Expect_GM6020_ GM6020_GO = {
    .GM6020_pos = {
        .Kp = 1.0f,
        .Ki = 0.0f,
        .Kd = 0.0f,
        .limit = 0.0f,
        .output_limit = 10000.0f,
    },
    .GM6020_vel = {
        .Kp = 10.0f,
        .Ki = 0.5f,
        .Kd = 0.0f,
        .limit = 0.0f,
        .output_limit = 10000.0f,
    }
};
	



	
	
	

 GM6020_TypeDef GM6020_state;

    

//*******************************

TaskHandle_t servo_Serve_Handle;
void servo_Serve(void *argument)
{
	for(;;){
	Temp_Servo_Target[0] = target_pack.servo1.up;
	Temp_Servo_Target[1]= target_pack.servo1.low;
	state_pack.servo2.low = target_pack.servo1.low;
	state_pack.servo2.up = target_pack.servo1.up;
	Servo_control();  
		vTaskDelay (5);
	}
}



TaskHandle_t stride_Serve_Handle;
void stride_Serve(void *argument)
{
    vTaskDelay(2000);
    RobStrideInit(&state_pack.robstride01,&hcan1,0x02,RobStride_01);
    RobStrideSetMode(&state_pack.robstride01, RobStride_MotionControl);
    vTaskDelay(100);
    RobStrideEnable(&state_pack.robstride01);
    TickType_t last_wake = xTaskGetTickCount();
     for(;;)
     {
        RobStrideMotionControl(&state_pack.robstride01,0x01,target_pack.rob01.except_torque,
			 target_pack.rob01.except_pos,target_pack.rob01.except_omega,
			 target_pack.rob01.kp,target_pack.rob01.kd);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
     }
}



//can的接收回调函数，用于接收云台GM6020电机和灵足电机
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t robstride_buf[8];
	  
    uint32_t ID;

    if(hcan->Instance == CAN2)
    {
			 CAN_Receive_DataFrame(&hcan2,CAN2_buff);
			 GM6020_Receive(&GM6020_state,CAN2_buff);
			if(!cur_read)//第一次读取编码器的值，用于零偏校准
	    {
		      cur_read = 1;//设置标志位，表示已读取过编码器的值
		      cur_offset = ((CAN2_buff[1] & 0xff) | ((uint16_t)CAN2_buff[0]<<8));
	     }
			  if(cur_read)//已初始化过编码器，开始正常读取和处理
      {
           last_cur = now_cur;//保存上一次编码器的值
           now_cur = ((CAN2_buff[1] & 0xff) | ((uint16_t)CAN2_buff[0]<<8)) ;//获取当前编码器的值
		        //编码器溢出检测和处理
          if((int16_t)(now_cur - last_cur + 1e-5) > 4000) cur_round--;
          else if((int16_t)(now_cur - last_cur +1e-5) < -4000) cur_round++;
        }
        
        
    }
//    else if(hcan->Instance == CAN1)
//    {
//        ID = CAN_Receive_DataFrame(hcan, robstride_buf);
//        RobStrideRecv_Handle(&state_pack.robstride01, hcan, ID,robstride_buf);
//    }
}



//控制GM6020电机任务

TaskHandle_t GM6020_Serve_Handle;
void GM6020_Serve(void *argument)
{

	
	 TickType_t last_wake = xTaskGetTickCount();
	 
  
	
	 for(;;)
	{
		
		
	  float expect_=GM6020_forward_rad * 180.0f / PAI;
		    
		if(expect_>360 || expect_<-360){
			
			
		}
		current_cur = cur_round * 360.0f + now_cur * 360.0f / 8192.0f - cur_offset* 360.0f / 8192.0f;      
  PID_Control2(current_cur,expect_,&GM6020_GO.GM6020_pos);
					
		current_vel = (float)state_pack.GM6020.Speed;  
		
  //expected_vel = GM6020_GO.GM6020_pos.pid_out;
			expected_vel = 0;
	if(expected_vel > MAX_VEL) expected_vel = MAX_VEL;
  if(expected_vel < -MAX_VEL) expected_vel = -MAX_VEL;

  PID_Control2(current_vel,0,&GM6020_GO.GM6020_vel);

  int16_t current_output = (int16_t)GM6020_GO.GM6020_vel.pid_out;  

  if(current_output > 16384) current_output = 16384;
  if(current_output < -16384) current_output = -16384;
	
  can_out[2] = (uint8_t)(current_output >> 8);     // 高8位
  can_out[3] = (uint8_t)(current_output & 0x00FF);   // 低8位、

  
  CAN_Send_StdDataFrame(&hcan2,0x1FE,can_out);
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
	}
}





//下位机发送给上位机的任务
TaskHandle_t usb_cdc_Send_Handle;
void usb_cdc_Send(void *arguments)
{
	 // USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
	
	for(;;){
		
//		state_pack.GM6020.Angle_DEG=(now_cur * 360.0f / 8192.0f - cur_offset* 360.0f / 8192.0f)*PAI/180.0f;
//		state_pack.GM6020.Speed=GM6020_state.Speed ;
//		state_pack.GM6020.TorqueCurrent=GM6020_state.TorqueCurrent ;
		state_pack.GM6020.Angle_DEG=2;
		state_pack.GM6020.Speed=2;
		state_pack.GM6020.TorqueCurrent=2;
		state_pack.servo2.up=2;
		state_pack.servo2.low=2;
		state_pack.robstride01.state .rad =2;
		state_pack.robstride01.state .omega  =2;
		state_pack.robstride01.state.torque  =2;
		
		
		if(allow==1)    {
				CDC_Transmit_FS((uint8_t*)&state_pack, sizeof(state_pack));
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
		
	}
	
	
	}
}

TaskHandle_t usb_cdc_Receive_Handle;
void usb_cdc_Receive(void *argument)
{
    cdc_recv_semp = xSemaphoreCreateBinary();
		vTaskDelay(1000);
	for(;;){
		
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