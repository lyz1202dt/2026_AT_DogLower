#include "FreeRTOS.h"
#include "task_init.h"
#include "main.h"
#include "arm.h"
#include "math.h"
//Expect_GM6020 GM6020_GO;
void task_init()
{
	vPortEnterCritical();
	
  	xTaskCreate(radiation_distance,
        "(radiation_distance",
         256,
         NULL,
         4,
         &radiation_distance_Handle);
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
          5,
          &stride_Serve_Handle);

	xTaskCreate(usb_cdc_Receive,
        "usb_cdc__receive_task",
         512,
         NULL,
         2,
         &usb_cdc_Receive_Handle);
				 
	xTaskCreate(air_pump,
        "air_pump_task",
         128,
         NULL,
         4,
         &air_pump_Handle);
				 
//	xTaskCreate(watch_dog,
//        "watch_dog_task",
//         128,
//         NULL,
//         4,
//         &watch_dog_Handle);
				 
	      vPortExitCritical();
}
