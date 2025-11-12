#ifndef __USB_TRANS_H__
#define __USB_TRANS_H__


#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define USB_CDC_RECV_BUFFER_SIZE	512


typedef struct CDC_SendReq CDC_SendReq_t;
typedef struct CDC_RecvReq CDC_RecvReq_t;
typedef struct CDC_Trans_t CDC_Trans_t;


typedef void(*Send_finished_cb_t)(void* user_data);                               //发送完成回调
typedef void(*Send_Timeout_cb_t)(CDC_SendReq_t *req,CDC_Trans_t *trans);          //发送超时回调
typedef void(*Recv_finished_cb_t)(uint8_t *src,uint16_t size);                    //数据包接收完成回调


struct CDC_SendReq_t{
    uint16_t size;  //要发送的数据包长度
    uint8_t* data;  //要发送的数据包内容
    Send_finished_cb_t finished_cb;   //发送完成回调
    void* user_data;    //用户参数
};

struct CDC_RecvReq_t{
    uint16_t size;  //要接收的数据包长度
    Recv_finished_cb_t finished_cb;   //发送完成回调
    void* finished_cb_param;    //用户参数
};


#pragma pack(1)

struct CDC_Trans_t{
    uint32_t pack_id;      //包ID，表示除当前包以外还有的数据包个数，当该值为0时，表示一个数据包接收完成，需要开始拼包并触发用户回调
    uint8_t data[];
};

#pragma pack()


extern QueueHandle_t kUsbRecvQueue;
extern QueueHandle_t kUsbSendsemphr;
extern QueueHandle_t kUsbSendReqQueue;

void USB_CDC_Init(Recv_finished_cb_t recv_cb,Send_Timeout_cb_t send_timeout_cb);
void USB_Send_Pack(CDC_SendReq_t *req,uint32_t time_out);
void CDC_TransCplt_Handler(void);
void CDC_RecvCplt_Handler(uint8_t* Buf, uint32_t *Len);

__weak void USB_CDC_RecvBufferOverFlow(void);

#endif