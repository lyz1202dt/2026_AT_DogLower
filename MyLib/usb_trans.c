#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
uint8_t _UserRxBufferFS[USB_CDC_RECV_BUFFER_SIZE];

QueueHandle_t kUsbRecvQueue;
QueueHandle_t kUsbSendsemphr;
QueueHandle_t kUsbSendReqQueue;

TaskHandle_t kUsbSendTaskHandle;
TaskHandle_t kUsbRecvTaskHandle;

static void USB_RecvTask(void *param)
{
    Recv_finished_cb_t USB_CDC_Recv_Cb = (Recv_finished_cb_t)param;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    uint32_t current_cdc_pack_size;
    uint32_t last_pack_id = 0;
    uint32_t buffer_index = 0;
    while (1)
    {
        xQueueReceive(kUsbRecvQueue, &current_cdc_pack_size, portMAX_DELAY);
        CDC_Trans_t *trans = (CDC_Trans_t *)(UserRxBufferFS);
        if (trans->pack_id) // 包ID不为0
        {
            if (last_pack_id <= trans->pack_id) // 如果当前包的ID大于等于之前包的ID，说明此时是全新的传输任务（上一个包被异常中断），将本次接收的数据包复制到用户缓冲区新位置
            {
                buffer_index = 0;
            }
            if (buffer_index + current_cdc_pack_size - sizeof(uint32_t) < USB_CDC_RECV_BUFFER_SIZE)
            {
                memcpy(_UserRxBufferFS + buffer_index, UserRxBufferFS + sizeof(uint32_t), current_cdc_pack_size - sizeof(uint32_t)); // 将接收的数据包拷贝到用户缓冲区
                buffer_index = buffer_index + current_cdc_pack_size - sizeof(uint32_t);
            }
            else
                USB_CDC_RecvBufferOverFlow();
        }
        else // 包ID为0，说明本次包传输任务正确完成
        {
            memcpy(_UserRxBufferFS + buffer_index, UserRxBufferFS + sizeof(uint32_t), current_cdc_pack_size - sizeof(uint32_t));
            buffer_index = buffer_index + current_cdc_pack_size - sizeof(uint32_t);
            USB_CDC_Recv_Cb(_UserRxBufferFS, buffer_index + 1); // 调用用户的数据接收中断函数
            buffer_index = 0;
        }
        last_pack_id = trans->pack_id;
    }
}

static void USB_SendTask(void *param)
{
    struct CDC_SendReq_t req;
    Send_Timeout_cb_t sendTimeoutCb = (Send_Timeout_cb_t)param;
    while (1)
    {
        xQueueReceive(kUsbSendReqQueue, &req, portMAX_DELAY);

        int pack_index = req.size / (64 - sizeof(uint32_t)); // 计算需要发的包的数量
        int index = 0;
        for (int i = pack_index; i >= 0; i--) // 填写发送缓冲区
        {
            CDC_Trans_t *trans = (CDC_Trans_t *)(UserTxBufferFS + i * 64);

            trans->pack_id = i;
            if (i == 0)
                memcpy(UserTxBufferFS + index * 64, &trans->data[index * 64], req.size % (64 - sizeof(uint32_t)));
            else
                memcpy(UserTxBufferFS + index * 64, &trans->data[index * 64], 64);
            index++;
        }
        xSemaphoreTake(kUsbSendsemphr, 0);    // 清空信号量
        for (int i = 0; i <= pack_index; i++) // 发送数据
        {
            CDC_Trans_t *trans = (CDC_Trans_t *)(UserTxBufferFS + i * 64);
            trans->pack_id = i;
            if (i == 0)
                USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS + i * 64, req.size % (64 - sizeof(uint32_t)));
            else
                USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS + i * 64, 64);

            USBD_CDC_TransmitPacket(&hUsbDeviceFS);
            if (xSemaphoreTake(kUsbSendsemphr, pdMS_TO_TICKS(5)) != pdPASS) // 清空信号量
            {
                if (sendTimeoutCb)
                    sendTimeoutCb(&req, trans);
                break;
            }
        }
        if (req.finished_cb) // 如果有定义回调函数，那么执行发送完成回调
            req.finished_cb(req.user_data);
    }
}

// 初始化USB CDC应用层
void USB_CDC_Init(Recv_finished_cb_t recv_cb, Send_Timeout_cb_t send_timeout_cb)
{
    kUsbRecvQueue = xQueueCreate(2, sizeof(uint32_t));
    kUsbSendsemphr = xSemaphoreCreateBinary();
    kUsbSendReqQueue = xQueueCreate(8, sizeof(struct CDC_SendReq_t));
    xTaskCreate(USB_SendTask, "usb_cdc_send", 128, send_timeout_cb, 6, &kUsbSendTaskHandle);
    xTaskCreate(USB_RecvTask, "usb_cdc_recv", 128, recv_cb, 6, &kUsbRecvTaskHandle);
}

void USB_Send_Pack(CDC_SendReq_t *req, uint32_t time_out)
{
    xQueueSend(kUsbSendReqQueue, req, pdMS_TO_TICKS(time_out));
}

void CDC_TransCplt_Handler(void)
{
    BaseType_t pxHigherPriorityTaskWoken;
    xSemaphoreGiveFromISR(kUsbSendsemphr, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void CDC_RecvCplt_Handler(uint8_t *Buf, uint32_t *Len)
{
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
    BaseType_t pxHigherPriorityTaskWoken;
    xQueueSendFromISR(kUsbRecvQueue, Len, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

__weak void USB_CDC_RecvBufferOverFlow(void)
{
}