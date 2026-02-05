#include "FreeRTOS.h"
#include "task_init.h"
#include "main.h"
#include "arm.h"
void task_init()
	
{
	vPortEnterCritical();

	xTaskCreate(servo_Serve,
         "servo_task",
          256,
          NULL,
          2,
          &servo_Serve_Handle);
	xTaskCreate(stride_Serve,
         "stride_task",
          256,
          NULL,
          2,
          &stride_Serve_Handle);
	xTaskCreate(GM6020_Serve,
         "GM6020_task",
          256,
          NULL,
          2,
          &GM6020_Serve_Handle);
	xTaskCreate(usb_cdc_Send,
         "usb_cdc_send_task",
          256,
          NULL,
          2,
          &GM6020_Serve_Handle);
	xTaskCreate(usb_cdc_Receive,
         "usb_cdc__receive_task",
          256,
          NULL,
          2,
          &GM6020_Serve_Handle);
	      vPortExitCritical();
}