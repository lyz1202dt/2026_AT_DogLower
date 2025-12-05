#include "Canbuffer.h"

CanDataPack can1sendbuffer[CAN1_BUFFER_SIZE];
uint8_t can1_send_point;
uint8_t can1_read_point;
uint8_t can1_data_num;

#ifdef CAN2
static CanDataPack can2sendbuffer[CAN2_BUFFER_SIZE];
static uint8_t can2_send_point;
static uint8_t can2_read_point;
static uint8_t can2_data_num;
#endif

/**
 * @brief 将要发送的CAN数据包放到发送缓冲区
 * @param hcan 该数据/遥控帧要使用的CAN句柄
 * @param head 数据/遥控帧 帧头
 * @param buf 数据帧所要携带的数据
 * @return 0表示发送成功，1表示缓冲区溢出
 */
uint8_t CAN_Transmit(CAN_HandleTypeDef *hcan,CAN_TxHeaderTypeDef *head,uint8_t *buf)
{
    uint32_t sfr=hcan->Instance->TSR;
    if (hcan->Instance==CAN1)
    {
        if (can1_data_num==CAN1_BUFFER_SIZE)  //缓冲区溢出
            return 1;
        __disable_irq(); //进入临界代码段，禁用全局中断
        memcpy(&can1sendbuffer[can1_send_point].head,head,sizeof(CAN_TxHeaderTypeDef));
		if(head->DLC&&head->RTR==CAN_RTR_DATA)
			memcpy(can1sendbuffer[can1_send_point].buffer,buf,head->DLC);
        can1_data_num=can1_data_num+1;
        can1_send_point=can1_send_point+1;
        if (can1_send_point==CAN1_BUFFER_SIZE)  //指针走出了数据缓冲区地址，回到头部
            can1_send_point=0;
        if (sfr & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))   //邮箱全空，并且中断请求标志位为0
			CAN_SendCompleteServe(hcan);
        __enable_irq();
    }
    else
    {
        #ifdef CAN2

        if (can2_data_num==CAN2_BUFFER_SIZE)  //缓冲区溢出
            return 1;
        __disable_irq(); //进入临界代码段，禁用全局中断
        memcpy(&can2sendbuffer[can2_send_point].head,head,sizeof(CAN_TxHeaderTypeDef));
		if(head->DLC&&head->RTR==CAN_RTR_DATA)
			memcpy(can2sendbuffer[can2_send_point].buffer,buf,head->DLC);
        can2_data_num=can2_data_num+1;
        can2_send_point=can2_send_point+1;
        if (can2_send_point==CAN2_BUFFER_SIZE)  //指针走出了数据缓冲区地址，回到头部
            can2_send_point=0;
        if (sfr & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2))
			CAN_SendCompleteServe(hcan);
        __enable_irq();

        #endif
    }
    return 0;
}


/**
 * @brief CAN数据发送函数
 */
static void CAN_SendCompleteServe(CAN_HandleTypeDef *hcan)
{
    uint32_t txMailbox;
    HAL_StatusTypeDef canState;

    __disable_irq();
    if (hcan->Instance==CAN1)
    {
        while(can1_data_num)        //重复执行直到发送缓冲区为空或者所有CAN邮箱均占满
        {
            uint32_t sfr=hcan->Instance->TSR;

            if((sfr&CAN_TSR_TME0)|(sfr&CAN_TSR_TME1)|(sfr&CAN_TSR_TME2))   //有空邮箱
                canState=HAL_CAN_AddTxMessage(hcan,&((can1sendbuffer+can1_read_point)->head),(can1sendbuffer+can1_read_point)->buffer,&txMailbox);
            else
                break;
            can1_data_num=can1_data_num-1;
            can1_read_point=can1_read_point+1;
            if(can1_read_point==CAN1_BUFFER_SIZE)
                can1_read_point=0;
        }
    }
    else
    {
        #ifdef CAN2
        while(can2_data_num)        //重复执行直到发送缓冲区为空或者所有CAN邮箱均占满
        {
            uint32_t sfr=hcan->Instance->TSR;

            if((sfr&CAN_TSR_TME0)|(sfr&CAN_TSR_TME1)|(sfr&CAN_TSR_TME2))   //有空邮箱
                canState=HAL_CAN_AddTxMessage(hcan,&((can2sendbuffer+can2_read_point)->head),(can2sendbuffer+can2_read_point)->buffer,&txMailbox);
            else
                break;
            can2_data_num=can2_data_num-1;
            can2_read_point=can2_read_point+1;
            if(can2_read_point==CAN2_BUFFER_SIZE)
                can2_read_point=0;
        }
        #endif
    }
    __enable_irq();
    if (canState!=HAL_OK)
        CAN_Transmit_Error(hcan,canState);
}

__weak uint8_t CAN_Transmit_Error(CAN_HandleTypeDef *hcan,HAL_StatusTypeDef state)    //返回0表示需要暂停发送，进行处理，返回非0值表示继续发送下一个数据包
{
    UNUSED(state);
    return 1;
}


void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_SendCompleteServe(hcan);
}
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_SendCompleteServe(hcan);
}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    CAN_SendCompleteServe(hcan);
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(CAN_Transmit_Error(hcan,0x02))   //传入0x02，表示发送取消
        CAN_SendCompleteServe(hcan);
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(CAN_Transmit_Error(hcan,0x02))   //传入0x02，表示发送取消
        CAN_SendCompleteServe(hcan);
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    if(CAN_Transmit_Error(hcan,0x02))   //传入0x02，表示发送取消
        CAN_SendCompleteServe(hcan);
}
