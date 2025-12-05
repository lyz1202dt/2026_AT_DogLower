#include "motorEx.h"


uint32_t Motor3508Recv(Motor3508Ex_t* motor_ex,CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t* buf)
{
    if(hcan->Instance==motor_ex->hcan->Instance&&ID==motor_ex->ID)
    {
        RM3508_Receive(&(motor_ex->motor),buf);
        return 1;
    }  
    return 0;
}





