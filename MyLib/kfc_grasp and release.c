#include "main.h"

#include "freertos.h"
#include "task.h"
#include "kfc_grasp and release.h"
#include "math.h"

int Temp_Servo_Target[7] = {0,0,0,0,0,0,0}; 
int32_t Servo_assignment[7] = {0,0,0,0,0,0,0};        //ţPE9 PE11 PE13 PE14
int32_t Ramp_Value_Servo[7] = {50,50,50,50,50,50,50};
int32_t Servo_offset[7] = {270, 0, 0, 0, 270, 270, 270}; //϶һ90Ϊ1250


//��ؽڳ�ʼλ�ó��ϵ�ռ�ձȸ���1300ʱ��ת��180��

int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp )
{
    float buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    }		
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}

void Servo_control()    
{
	
	Temp_Servo_Target[0]=RAMP_self(Servo_assignment[0],Temp_Servo_Target[0],Ramp_Value_Servo[0]);
	Temp_Servo_Target[1]=RAMP_self(Servo_assignment[1],Temp_Servo_Target[1],Ramp_Value_Servo[1]);
	Temp_Servo_Target[2]=RAMP_self(Servo_assignment[2],Temp_Servo_Target[2],Ramp_Value_Servo[2]);
	Temp_Servo_Target[3]=RAMP_self(Servo_assignment[3],Temp_Servo_Target[3],Ramp_Value_Servo[3]);
	Temp_Servo_Target[4]=RAMP_self(Servo_assignment[4],Temp_Servo_Target[4],Ramp_Value_Servo[4]);
	Temp_Servo_Target[5]=RAMP_self(Servo_assignment[5],Temp_Servo_Target[5],Ramp_Value_Servo[5]);
	Temp_Servo_Target[6]=RAMP_self(Servo_assignment[6],Temp_Servo_Target[6],Ramp_Value_Servo[6]);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500+Servo_offset[0]+Temp_Servo_Target[0]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,500+Servo_offset[1]+Temp_Servo_Target[1]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,500+Servo_offset[2]+Temp_Servo_Target[2]);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,500+Servo_offset[3]+Temp_Servo_Target[3]);
 
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,500+Servo_offset[4]+Temp_Servo_Target[4]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,500+Servo_offset[5]+Temp_Servo_Target[5]);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,500+Servo_offset[6]+Temp_Servo_Target[6]);

}
