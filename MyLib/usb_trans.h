#ifndef __USB_TRANS_H__
#define __USB_TRANS_H__

/**
 * @brief USB流式传输模块
 * @note 考虑到USB VCP的传输特性，提供类似于环形缓冲区的策略给接收和发送留出接口
 */

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define USB_SEND_BUF_SIZE  256
#define USB_RECV_BUF_SIZE  256

typedef struct{
    uint8_t* p_data;
    uint16_t size;
}USB_Trans_t;

typedef struct{
    uint8_t* p_data;
    uint16_t size;
    uint16_t timeout_ms;
    uint8_t use_idel_recv;
}USB_Recv_t;


__weak void USB_VCP_ReceiveIdel(uint16_t size);


#endif