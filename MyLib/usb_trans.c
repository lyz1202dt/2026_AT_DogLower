#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;


static uint8_t kUSBRecvBuf[USB_RECV_BUF_SIZE];
static uint16_t kUSBSendBufWriteIndex = 0;
static uint16_t kUSBRecvBufWriteIndex = 0;
static uint16_t kUSBSendBufReadIndex = 0;
static uint16_t kUSBRecvBufReadIndex = 0;
static uint8_t kUsbReadReturn = 0;
static uint8_t kBusyRecving = 0;

static QueueHandle_t kUsbRecvReqQueue;
static QueueHandle_t kUsbRecvsemphr;
static QueueHandle_t kUsbRecvReqSemphr;

TaskHandle_t kUsbSendTaskHandle;
TaskHandle_t kUsbRecvTaskHandle;

static uint8_t kUsbIsRecvIdel=0;
static uint8_t kUsbEnableIdelRecv=0;


static void USB_RecvTask(void *param)
{
    USB_Recv_t req;
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    while (1)
    {
        xQueueReceive(kUsbRecvReqQueue, &req, portMAX_DELAY);   //接收数据读取请求
        
        uint16_t readable_bytes;
        uint32_t wait_time=0;
        uint32_t statrt_time = xTaskGetTickCount();
        do
        {
            uint32_t delta_time=xTaskGetTickCount()-statrt_time;    //计算已经等待的时间
            if(delta_time<req.timeout_ms)
                req.timeout_ms=req.timeout_ms-delta_time;
            else
                req.timeout_ms=0;
            readable_bytes= kUSBRecvBufReadIndex >= kUSBRecvBufWriteIndex ? (kUSBRecvBufReadIndex - kUSBRecvBufWriteIndex - 1) : (USB_RECV_BUF_SIZE - (kUSBRecvBufWriteIndex - kUSBRecvBufReadIndex) - 1);  //计算可以被读取的字节数
        }while((xSemaphoreTake(kUsbRecvsemphr, req.timeout_ms)!=pdPASS)&&readable_bytes<req.size&&(!((kUsbIsRecvIdel||(readable_bytes>=req.size))&&kUsbEnableIdelRecv))); // 等待直到数据接收完成或数据接收超时或空闲事件发生

        if((kUsbIsRecvIdel||(readable_bytes>=req.size))&&kUsbEnableIdelRecv)    //由空闲中断主导
        {
            kUsbIsRecvIdel=0;
            USB_VCP_ReceiveIdel(readable_bytes>req.size?req.size:readable_bytes);
        }

        if(readable_bytes<req.size) //数据可以被完整读取
        {
            if(kUSBRecvBufWriteIndex > kUSBRecvBufReadIndex) // 写指针地址大于读指针地址
            {
                memcpy(req.p_data, &kUSBRecvBuf[kUSBRecvBufReadIndex], req.size);
                kUSBRecvBufReadIndex += req.size;
            }
            else // 写指针地址小于读指针地址
            {
                if (req.size + kUSBRecvBufReadIndex <= USB_RECV_BUF_SIZE) // 数据可以直接读取
                {
                    memcpy(req.p_data, &kUSBRecvBuf[kUSBRecvBufReadIndex], req.size);
                    kUSBRecvBufReadIndex += req.size;
                }
                else // 数据需要分两次读取
                {
                    uint16_t first_len = USB_RECV_BUF_SIZE - kUSBRecvBufReadIndex;
                    memcpy(req.p_data, &kUSBRecvBuf[kUSBRecvBufReadIndex], first_len);
                    uint16_t second_len = req.size - first_len;
                    memcpy(&req.p_data[first_len], &kUSBRecvBuf[0], second_len);
                    kUSBRecvBufReadIndex = second_len;
                }
            }
            kUsbReadReturn=1;
        }
        else
        {
            kUsbReadReturn=0;
        }
        xSemaphoreGive(kUsbRecvReqSemphr);  //通知应用层数据读取完成
    }
}


void USB_CDC_Recv_Handle(uint8_t *Buf, uint32_t *Len)
{
    uint32_t len = *Len;
    uint16_t free_byte = kUSBRecvBufReadIndex >= kUSBRecvBufWriteIndex ? (kUSBRecvBufReadIndex - kUSBRecvBufWriteIndex - 1) : (USB_RECV_BUF_SIZE - (kUSBRecvBufWriteIndex - kUSBRecvBufReadIndex) - 1);

    if (free_byte >= len) // 接收缓冲区空间不足，调用溢出中断
    {
        len=free_byte;
    }
    if (kUSBRecvBufWriteIndex > kUSBRecvBufReadIndex) // 写指针地址大于读指针地址
    {
        if (len + kUSBRecvBufWriteIndex <= USB_RECV_BUF_SIZE) // 数据可以直接写入
        {
            memcpy(&kUSBRecvBuf[kUSBRecvBufWriteIndex], Buf, len);
            kUSBRecvBufWriteIndex += len;
        }
        else // 数据需要分两次写入
        {
            uint16_t first_len = USB_RECV_BUF_SIZE - kUSBRecvBufWriteIndex;
            memcpy(&kUSBRecvBuf[kUSBRecvBufWriteIndex], Buf, first_len);
            uint16_t second_len = len - first_len;
            memcpy(&kUSBRecvBuf[0], &Buf[first_len], second_len);
            kUSBRecvBufWriteIndex = second_len;
        }
    }
    else // 写指针地址小于读指针地址
    {
        memcpy(&kUSBRecvBuf[kUSBRecvBufWriteIndex], Buf, len);
        kUSBRecvBufWriteIndex += len;
    }

    if(len<64)
        kUsbIsRecvIdel=1;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(kUsbRecvsemphr, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void USB_VCP_Init()
{
    kUsbRecvsemphr = xSemaphoreCreateBinary();
    kUsbRecvReqSemphr=xSemaphoreCreateBinary();

    kUsbRecvReqQueue = xQueueCreate(8, sizeof(USB_Recv_t));
    xTaskCreate(USB_RecvTask, "USB_DealTask", 256, NULL, 6, &kUsbRecvTaskHandle);
}

uint32_t USB_VCP_Send(uint8_t *Buf, uint16_t Len)
{
    CDC_Transmit_FS(Buf,Len);
    return 0;
}

uint32_t USB_VCP_Recv(uint8_t *data,uint16_t size,uint16_t timeout_ms)
{
    USB_Recv_t req={.p_data=data,.size=size,.timeout_ms=timeout_ms,.use_idel_recv=0};
    if(xQueueSend(kUsbRecvReqQueue,&req,0)!=pdPASS)   //接收请求队列满
        return 1;
    if(xSemaphoreTake(kUsbRecvReqSemphr,timeout_ms)!=pdPASS) //等待数据接收完成
        return 2;
    if(kUsbReadReturn==0) //数据读取超时
        return 3;
    return 0;
}

uint32_t USB_ResetSendBuffer()
{
    portENTER_CRITICAL();
    kUSBSendBufReadIndex=0;
    kUSBSendBufWriteIndex=0;
    portEXIT_CRITICAL();
    return 0;
}

uint32_t USB_ResetRecvBuffer()
{
    portENTER_CRITICAL();
    kUSBRecvBufReadIndex=0;
    kUSBRecvBufWriteIndex=0;
    portEXIT_CRITICAL();
    return 0;
}

uint32_t USB_Recv_Idel(uint8_t *data,uint16_t size)
{
    USB_Recv_t req={.p_data=data,.size=size,.timeout_ms=1000,.use_idel_recv=0};
    xQueueSend(kUsbRecvReqQueue,&req,0);
}

//USB空闲回调函数，当请求了空闲接收，在上位机发完一个小于64字节的CDC包后会调用该函数
__weak void USB_VCP_ReceiveIdel(uint16_t size)
{
    UNUSED(size);
}
