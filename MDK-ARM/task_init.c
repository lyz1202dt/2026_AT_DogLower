#include "task_init.h"
void task_init()
	
{
	vPortEnterCritical();
	xTaskCreate(duoji_Serve,
         "DuoJi_task",
          256,
          NULL,
          2,
          &duoji_Serve_Handle);
	xTaskCreate(stride_Serve,
         "stride_task",
          256,
          NULL,
          2,
          &stride_Serve_Handle);
	      vPortExitCritical();
}