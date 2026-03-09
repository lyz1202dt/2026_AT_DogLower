#include "FreeRTOS.h"
#include "task_init.h"
#include "main.h"
#include "arm.h"
#include "math.h"
//Expect_GM6020 GM6020_GO;
void task_init()
{
	vPortEnterCritical();

	xTaskCreate(servo_Serve,
        "servo_task",
         256,
         NULL,
         4,
         &servo_Serve_Handle);
	xTaskCreate(stride_Serve,
         "stride_task",
          256,
          NULL,
          4,
          &stride_Serve_Handle);
	xTaskCreate(GM6020_Serve,
        "GM6020_task",
         256,
         NULL,
         4,
         &GM6020_Serve_Handle);
//	xTaskCreate(usb_cdc_Send,
//        "usb_cdc_send_task",
//         512,
//         NULL,
//         2,
//         &usb_cdc_Send_Handle);
	xTaskCreate(usb_cdc_Receive,
        "usb_cdc__receive_task",
         512,
         NULL,
         2,
         &usb_cdc_Receive_Handle);
	      vPortExitCritical();
}
//void RampToTarget(float *val, float target, float step)//??
//{
//    float diff = target - *val;

//    if (fabsf(diff) < step)
//    {
//        *val = target;
//    }
//    else
//    {
//        *val += (diff > 0 ? step : -step);
//    }
//}