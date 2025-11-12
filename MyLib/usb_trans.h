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

typedef void(*Send_finished_cb_t)(void* user_data);                               //发送完成回调
typedef void(*Recv_finished_cb_t)(uint8_t *src,uint16_t size);                    //数据包接收完成回调
typedef uint32_t (*Recv_match_cb_t)(uint32_t cur_desc,uint32_t exp_desc);         //数据包描述匹配回调(返回1时说明匹配成功，进入对应的接收完成回调)

typedef struct{
    uint16_t size;  //要发送的数据包长度
    uint8_t* data;  //要发送的数据包内容
    Send_finished_cb_t finished_cb;   //发送完成回调
    void* user_data;    //用户参数
}CDC_SendReq_t;

typedef struct{
    uint16_t size;  //要接收的数据包长度
    Recv_match_cb_t match_cb;         //数据包匹配回调
    Recv_finished_cb_t finished_cb;   //发送完成回调
    void* finished_cb_param;    //用户参数
}CDC_RecvReq_t;


#pragma pack(1)

typedef struct{
    uint32_t pack_id;      //包ID，表示除当前包以外还有的数据包个数，当该值为0时，表示一个数据包接收完成，需要开始拼包并触发用户回调
    uint8_t data[];
}CDC_Trans_t;

#pragma pack()


extern QueueHandle_t kUsbRecvQueue;
extern QueueHandle_t kUsbSendsemphr;
extern QueueHandle_t kUsbSendReqQueue;





void USB_CDC_Init(Recv_finished_cb_t recv_cb);

void USB_Send_Pack(CDC_SendReq_t *req,uint32_t time_out);

void CDC_TransCplt_Handler(void);
void CDC_RecvCplt_Handler(uint8_t* Buf, uint32_t *Len);


__weak void USB_CDC_SendTimeout(CDC_SendReq_t *req,CDC_Trans_t *trans);


#endif