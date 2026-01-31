#include "dm_h6215.h"

#define P_MAX 12.5f
#define V_MAX 45.f
#define T_MAX 10.f

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts afloat to anunsigned int, given range and number ofbits
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DMH6215_Init(DMH6215_t *motor, CAN_HandleTypeDef *hcan, uint32_t motor_id)
{
    motor->hcan = hcan;
    motor->id = motor_id;
}

uint32_t DMH6215_MIT_Control(DMH6215_t *motor,float pos,float vel,float tor,float kp,float kd)
{
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    pos_tmp = float_to_uint(pos, -P_MAX, P_MAX, 16);
    vel_tmp = float_to_uint(vel, -V_MAX, V_MAX, 12);
    tor_tmp = float_to_uint(tor, -T_MAX, T_MAX, 12);
    kp_tmp = float_to_uint(kp, 0, 500, 12);
    kd_tmp = float_to_uint(kd, 0, 5, 12);

    CAN_TxHeaderTypeDef head = {
        .StdId = motor->id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
    };

    
    motor->send_buf[0] = (pos_tmp >> 8);
    motor->send_buf[1] = pos_tmp;
    motor->send_buf[2] = (vel_tmp >> 4);
    motor->send_buf[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    motor->send_buf[4] = kp_tmp;
    motor->send_buf[5] = (kd_tmp >> 4);
    motor->send_buf[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    motor->send_buf[7] = tor_tmp;
    uint32_t tx_mailbox;
    return HAL_CAN_AddTxMessage(motor->hcan, &head, motor->send_buf, &tx_mailbox);
}

uint32_t DMH6215_PosVel_Control(DMH6215_t *motor,float pos,float vel)
{
    return 0;
}

uint32_t DMH6215_Vel_Control(DMH6215_t *motor,float vel)
{
    return 0;
}

uint32_t DMH6215_Recv_Handle(DMH6215_t *motor, CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *buf)
{
    if (hcan->Instance != motor->hcan->Instance)
        return 0;
    if (id != motor->id)
        return 0;

        //DM_Motor->ID  = Data[0] & 0x0F;
        motor->state=buf[0]>>4;

		int pos_temp = (uint16_t)(((buf[1]) <<8) | (buf[2]));
		int vel_temp = (uint16_t)((buf[3]) <<4) | ((buf[4])>>4);
		int tor_temp = (uint16_t)((buf[4]&0xF) <<8) | ((uint16_t)(buf[5]));
		motor->torque=  uint_to_float(tor_temp,-T_MAX,T_MAX,12);
		motor->position=uint_to_float(pos_temp,-P_MAX,P_MAX,16);
        motor->velocity=uint_to_float(vel_temp,-V_MAX,V_MAX,12);
        motor->t_mos   = (float)(buf[6]);
	    motor->t_rotor = (float)(buf[7]);
    return 1;
}

uint32_t DMH6215_Enable(DMH6215_t *motor)
{
    CAN_TxHeaderTypeDef head = {
        .StdId = motor->id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
    };

    for(int i=0;i<7;i++)
        motor->send_buf[i]=0xFF;
    motor->send_buf[7]=0xFC;
    uint32_t tx_mailbox;
    return HAL_CAN_AddTxMessage(motor->hcan, &head, motor->send_buf, &tx_mailbox);
}

uint32_t DMH6215_Disable(DMH6215_t *motor)
{
    CAN_TxHeaderTypeDef head = {
        .StdId = motor->id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
    };

    for(int i=0;i<7;i++)
        motor->send_buf[i]=0xFF;
    motor->send_buf[7]=0xFD;
    uint32_t tx_mailbox;
    return HAL_CAN_AddTxMessage(motor->hcan, &head, motor->send_buf, &tx_mailbox);
}

uint32_t DMH6215_ClearError(DMH6215_t *motor)
{
    CAN_TxHeaderTypeDef head = {
        .StdId = motor->id,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = 8,
    };

    for(int i=0;i<7;i++)
        motor->send_buf[i]=0xFF;
    motor->send_buf[7]=0xFB;
    uint32_t tx_mailbox;
    return HAL_CAN_AddTxMessage(motor->hcan, &head, motor->send_buf, &tx_mailbox);
}
