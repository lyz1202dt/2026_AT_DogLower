#include "arm.h"
#include "math.h"


//float up_test=0;
//float low_test=0;

int allow=0;
extern int Temp_Servo_Target[4];
extern int32_t Servo_assignment[4];

int16_t current_output;
target_pack_t target_pack = {.pack_type = 0x01};
state_pack_t state_pack = {.pack_type = 0x01};
QueueHandle_t cdc_recv_semp;

RobStride_t robstride01
={
.hcan = &hcan1,
.motor_id = 0x01,	
}
;


float GM6020_forward_rad = 0.0f;//��λ����������������
uint16_t last_cur = 0;   //��һ�ζ�ȡ�ı�����ԭʼֵ
uint16_t now_cur = 0;    //��ǰ��ȡ�ı�����ԭʼֵ
uint16_t cur_offset = 0; //������������ƫֵ������У׼��
uint8_t cur_read = 0;    //��־λ���Ƿ��Ѿ���ȡ��������������ֵ
int32_t cur_round = 0;   //������������Ȧ���������������������

float current_vel = 0.0f;     //��ǰ�ٶ�ֵ ��ÿ��
float expected_vel = 0.0f;    //�����ٶ�ֵ�����Ե��������


uint8_t can_out[8] = {0};//CAN�������ݻ�����

float current_cur = 0.0f;//��ǰλ��

float MAX_VEL = 90.f;//ת������ٶ�����

GM6020_PID PID_SET= {
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

TaskHandle_t servo_Serve_Handle;
void servo_Serve(void *argument)
{
	for(;;){
	Servo_assignment[0] = (int)(target_pack.servo1.up*1300.0/PAI);
	//Servo_assignment[0] = (int)(up_test*1300.0/PAI);
	Servo_assignment[1]= (int)(target_pack.servo1.low*620.0/PAI*2.0);
  //Servo_assignment[1]= (int)(low_test*620.0/PAI*2);
		if(Servo_assignment[0]>1300){
			Servo_assignment[0]=1300;
		}
		if(Servo_assignment[1]>620){
			Servo_assignment[1]=620; 
		}
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
    RobStrideInit(&state_pack.robstride01,&hcan1,0x01,RobStride_01);
    RobStrideSetMode(&state_pack.robstride01, RobStride_MotionControl);
    vTaskDelay(100);
    RobStrideEnable(&state_pack.robstride01);
    TickType_t last_wake = xTaskGetTickCount();
     for(;;)
     {

        RobStrideMotionControl(&state_pack.robstride01,0x01,NULL,
			 target_pack.rob01.except_pos,target_pack.rob01.except_omega,
			 target_pack.rob01.kp,target_pack.rob01.kd);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
     }
}

//can�Ľ��ջص����������ڽ�����̨GM6020�����������
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    
    if(hcan->Instance == CAN2)
    {
			 CAN_Receive_DataFrame(&hcan2,CAN2_buff);
			 GM6020_Receive(&GM6020_state,CAN2_buff);
			if(!cur_read)//��һ�ζ�ȡ��������ֵ��������ƫУ׼
	    {
		      cur_read = 1;//���ñ�־λ����ʾ�Ѷ�ȡ����������ֵ
		      cur_offset = ((CAN2_buff[1] & 0xff) | ((uint16_t)CAN2_buff[0]<<8));
	     }
			  if(cur_read)//�ѳ�ʼ��������������ʼ������ȡ�ʹ���
      {
           last_cur = now_cur;//������һ�α�������ֵ
           now_cur = ((CAN2_buff[1] & 0xff) | ((uint16_t)CAN2_buff[0]<<8)) ;//��ȡ��ǰ��������ֵ
		
          if((int16_t)(now_cur - last_cur + 1e-5) > 4000) cur_round--;
          else if((int16_t)(now_cur - last_cur +1e-5) < -4000) cur_round++;
        }
        
        
    }
   else if(hcan->Instance == CAN1)
   {
     CAN_Receive_DataFrame(&hcan1,CAN1_buff);
		RobStrideRecv_Handle(&state_pack.robstride01, &hcan1, 0x02, CAN1_buff);
   }
}

TaskHandle_t GM6020_Serve_Handle;
void GM6020_Serve(void *argument)
{
	 TickType_t last_wake = xTaskGetTickCount();
	 for(;;)
	{
float expect_=GM6020_forward_rad * 180.0f / PAI;
if(expect_>360 || expect_<-360)
{
    expect_ = fmod(expect_, 360.0f);
}
		current_cur = cur_round * 360.0f + now_cur * 360.0f / 8192.0f - cur_offset* 360.0f / 8192.0f; 

  PID_Control2(current_cur,expect_,&PID_SET.GM6020_pos);
		
  expected_vel = PID_SET.GM6020_pos.pid_out;
  if(expected_vel > MAX_VEL) expected_vel = MAX_VEL;
  if(expected_vel < -MAX_VEL) expected_vel = -MAX_VEL;

  PID_Control2( (float)state_pack.GM6020.Speed,expected_vel,&PID_SET.GM6020_vel);

   current_output = (int16_t)PID_SET.GM6020_vel.pid_out;  

  if(current_output > 16384) current_output = 16384;
  if(current_output < -16384) current_output = -16384;
	
  can_out[2] = (uint8_t)(current_output >> 8);     // ��8λ
  can_out[3] = (uint8_t)(current_output & 0x00FF);   // ��8λ��
     CAN_Send_StdDataFrame(&hcan2,0x1FE,can_out);
	 vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(5));
	}
}

TaskHandle_t usb_cdc_Send_Handle;
void usb_cdc_Send(void *arguments)
{
	 // USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
	
	for(;;){
  
		state_pack.GM6020.Angle_DEG=(now_cur * 360.0f / 8192.0f - cur_offset* 360.0f / 8192.0f)*PAI/180.0f;
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
    if(size==sizeof(target_pack_t)&&((target_pack_t*)src)->pack_type==0x01)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        memcpy(&target_pack, src, sizeof(target_pack_t));
        xSemaphoreGive(cdc_recv_semp);
    }
        count++;
        cur_size=size;
}