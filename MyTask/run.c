#include "run.h"
#include "usart.h"
#include "mylist.h"
#include "usb_trans.h"
#include "WatchDog2.h"
#include <string.h>
#include "usbd_cdc_if.h"
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

// 添加错误统计结构
typedef struct {
    uint32_t total;
    uint32_t overrun;
    uint32_t frame;
    uint32_t noise;
    uint32_t parity;
    uint32_t last_error_time;
    uint32_t continuous_errors;
    uint32_t recovery_attempts;
    uint32_t last_recovery_time;
} ErrorStats_t;
ErrorStats_t error_stats = {0};
uint32_t error_cnt = 0;
uint32_t err_timer_cnt = 0;

uint32_t req_stop_transmit;

// 添加错误标志和重启接收标志
uint32_t last_error_time = 0;


RS485_t rs485bus;
QueueHandle_t cdc_recv_semphr;

MotorTargetPack_t legs_target = {.pack_type = 0x00};
MotorStatePack_t legs_state = {.pack_type = 0x00};
Leg_t leg[4] = {
    {.joint[0] = {.motor = {.motor_id = 0x01, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.02757889f},
     .joint[1] = {.motor = {.motor_id = 0x02, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.77129902f},
     .joint[2] = {.motor = {.motor_id = 0x03, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.47869856f},
     .wheel={.hcan=&hcan1,.ID=0x201}},

    {.joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 4.29111939f},
     .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.60819724f},
     .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.20990329f},
     .wheel={.hcan=&hcan1,.ID=0x202}},

    {.joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 4.09088597f},
     .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.67971121f},
     .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.661729104f},
     .wheel={.hcan=&hcan1,.ID=0x203}},

    {.joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.19752234f},
     .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.64519156f},
     .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.56194712f},
     .wheel={.hcan=&hcan1,.ID=0x204
}}};

float setup_offset[4][3];    //上电启动时的电机角度
uint32_t first_run=5;
void MotorControlTask(void *param) // 将数据发送到电机，并从电机接收数据
{
    
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {   
				int err_check=0;
        for (int i = 0; i < 4; i++)
        {
            for(int j=0;j<3;j++)
            {
                GoMotorSend(&leg[i].joint[j].motor, leg[i].joint[j].exp_torque / 6.33f * leg[i].joint[j].inv_motor,
                        leg[i].joint[j].exp_omega * 6.33f * leg[i].joint[j].inv_motor,
                        leg[i].joint[j].exp_rad * 6.33f * leg[i].joint[j].inv_motor + leg[i].joint[j].pos_offset+setup_offset[i][j],
                        leg[i].joint[j].Kp, leg[i].joint[j].Kd);
                err_check+=GoMotorRecv(&leg[i].joint[j].motor);
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
        if(req_stop_transmit)
        {
            req_stop_transmit=0;
            while(1)
                vTaskDelay(100);
        }
        if(err_check==12&&first_run)
            first_run--;
    }
}

uint32_t current_size=0;
uint32_t cnt = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
    if(size==sizeof(MotorTargetPack_t)&&((MotorTargetPack_t*)src)->pack_type==0x00)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        memcpy(&legs_target, src, sizeof(MotorTargetPack_t));
        xSemaphoreGive(cdc_recv_semphr);
    }
    cnt++;
        current_size=size;
    //HAL_UART_Transmit_DMA(&huart3, src, size);
}

PID2 wheel_vel_pid[4];
float wheel_exp_vel[4],wheel_exp_torque[4];
int16_t can_send_buf[4];
float inv_wheel[4]={-1.0f,1.0f,-1.0f,1.0f};
void WheelControlTask(void* param)
{
    TickType_t last_wake_time=xTaskGetTickCount();
    leg[0].wheel.vel_pid.Kp=0.55f;
    leg[0].wheel.vel_pid.Ki=0.04f;
    leg[0].wheel.vel_pid.limit=300.0f;
    leg[0].wheel.vel_pid.output_limit=4.0f;
    leg[3].wheel.vel_pid=leg[2].wheel.vel_pid=leg[1].wheel.vel_pid=leg[0].wheel.vel_pid;
    while(1)
    {
        for(int i=0;i<4;i++)
        {
            PID_Control2(leg[i].wheel.motor.Speed*3.14159265f*2.0f/60.0f/19.0f,wheel_exp_vel[i]*inv_wheel[i],&leg[i].wheel.vel_pid);
            float out_temp=((leg[i].wheel.vel_pid.pid_out+wheel_exp_torque[i]*inv_wheel[i])/0.3f*(16384.0f/20.0f/0.3f));
            if(out_temp>16384)
                out_temp=16384;
            else if(out_temp<-16384)
                out_temp=-16384;
            can_send_buf[i]=(int16_t)out_temp;
        }
        MotorSend(&hcan1,0x200,can_send_buf);
        vTaskDelayUntil(&last_wake_time,2);
    }
}

uint8_t allow_send=0;
void MotorSendTask(void *param) // 将电机的数据发送到PC上
{
    USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
    while (1)
    {
        for (int i = 0; i < 4; i++) // 填写数据并发送到PC
        {
            for (int j = 0; j < 3; j++)
            {
                legs_state.leg[i].joint[j].rad = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.rad - leg[i].joint[j].pos_offset-setup_offset[i][j]) / 6.33f;
                legs_state.leg[i].joint[j].omega = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.velocity) / 6.33f;
                legs_state.leg[i].joint[j].torque = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.torque) * 6.33f;
            }
            legs_state.leg[i].wheel.omega=leg[i].wheel.motor.Speed*3.14159265f*2.0f/60.0f/19.0f;
            legs_state.leg[i].wheel.torque=0.0f;    //TODO:根据反馈计算真实力矩
        }
				if(allow_send)    //电机数据准备好再发
					CDC_Transmit_FS((uint8_t*)&legs_state, sizeof(legs_state));
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

static uint32_t slope(float target,float *cur_target,float step_limit)
{
    float delta=target-*cur_target;
    uint32_t ret=0;

    if(delta>step_limit)
        delta=step_limit;
    else if(delta<-step_limit)
        delta=-step_limit;
    else    //本次结束后，应该就会到达目标值
        ret=1;

    *cur_target=*cur_target+delta;
    return ret;
}

float init_pos[4][3]={{0.0f,0.8f,0.0f},
                    {0.0f,-0.8f,0.0f},
                    {0.0f,0.8f,0.0f},
                    {0.0f,-0.8f,0.0f}};
static uint32_t DogReset(uint32_t time_out_ms)
{
    __disable_irq();    //更新电机初始关节角度和Kp，关闭中断确保绝对同步，防止狗腿震荡
    for (int i = 0; i < 4; i++)         //将狗腿状态读入当前目标值，设置狗腿期望为当前位置
    {
        leg[i].joint[0].exp_omega = 0.0f;
        leg[i].joint[0].exp_torque = 0.0f;
        leg[i].joint[0].Kp = 3.0f;
        leg[i].joint[0].Kd = 0.17f;
        leg[i].joint[0].exp_rad=legs_state.leg[i].joint[0].rad;
            
        leg[i].joint[1].exp_omega = 0.0f;
        leg[i].joint[1].exp_torque = 0.0f;
        leg[i].joint[1].Kp = 3.0f;
        leg[i].joint[1].Kd = 0.14f;
        leg[i].joint[1].exp_rad=legs_state.leg[i].joint[1].rad;

        leg[i].joint[2].exp_omega = 0.0f;
        leg[i].joint[2].exp_torque = 0.0f;
        leg[i].joint[2].Kp = 3.0f;
        leg[i].joint[2].Kd = 0.11f;
        leg[i].joint[2].exp_rad=legs_state.leg[i].joint[2].rad;
    }
        __enable_irq();
        //PID参数:[0]:kp=4.0,kd=0.24;[1]:kp=3.7,kd=0.24;[2]:kp=3,kd=0.11

    uint32_t finished_check;
    uint32_t current_time=0;
    do{
        finished_check=0;
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<3;j++)
                finished_check=finished_check+slope(init_pos[i][j],&leg[i].joint[j].exp_rad,0.001f);
        }
        vTaskDelay(5);
        current_time=current_time+5;
        if(current_time>time_out_ms)    //如果复位已经超时，那么返回0
            return 0;
    }while(finished_check!=12);     //完成校验不等于12，说明有电机没有完成复位

    return 1;   //返回1表示狗复位成功
}

void MotorRecvTask(void *param) // 从PC接收电机的期望值
{
    cdc_recv_semphr = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semphr, 0);
		vTaskDelay(1000);
    while(first_run)    //等待电机数据准备好
        vTaskDelay(1);
        //TODO:上电时电机角度在极点附近的处理
//        if(leg[0].joint[0].motor.state.rad>4.0f)
//            leg[0].joint[0].pos_offset=leg[0].joint[0].pos_offset+6.2831853f;
//        if(leg[1].joint[2].motor.state.rad<3.0f)
//            leg[1].joint[2].pos_offset=leg[1].joint[2].pos_offset-6.2831853f;
//        if(leg[2].joint[2].motor.state.rad<3.0f)
//            leg[2].joint[2].pos_offset=leg[2].joint[2].pos_offset-6.2831853f;
//        if(leg[3].joint[1].motor.state.rad<3.0f)
//            leg[3].joint[1].pos_offset=leg[3].joint[1].pos_offset-6.2831853f;
	for(int i=0;i<4;i++)
    {
        setup_offset[i][0]=leg[i].joint[0].motor.state.rad;
        setup_offset[i][1]=leg[i].joint[1].motor.state.rad;
        setup_offset[i][2]=leg[i].joint[2].motor.state.rad;
    }
		allow_send=1;		//允许发送数据
       
    while (1)
    {
        if (xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(50)) != pdPASS) // 发生超时，说明通讯断开
        {
            // TODO:通过LED显示，清零所有力矩，电机进入低阻尼模式，整狗进入安全模式
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    leg[i].joint[j].exp_omega = 0.0f;
                    leg[i].joint[j].exp_torque = 0.0f;
                    leg[i].joint[j].Kp = 0.0f;
                    leg[i].joint[j].Kd = 0.1f;
                }
                wheel_exp_torque[i]=0.0f;
                wheel_exp_vel[i]=0.0f;
            }
            continue;
        }

        // TODO:安全限幅并给到电机期望
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                leg[i].joint[j].exp_rad = legs_target.leg[i].joint[j].rad;
                leg[i].joint[j].exp_omega = legs_target.leg[i].joint[j].omega;
                leg[i].joint[j].exp_torque = legs_target.leg[i].joint[j].torque;
                leg[i].joint[j].Kp = legs_target.leg[i].joint[j].kp;
                leg[i].joint[j].Kd = legs_target.leg[i].joint[j].kd;
            }
            wheel_exp_vel[i]=legs_target.leg[i].wheel.omega;
            wheel_exp_torque[i]=legs_target.leg[i].wheel.torque;
        }
    }
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t buf[8];
    if(hcan->Instance==CAN1)
    {
        uint32_t id=CAN_Receive_DataFrame(hcan,buf);
        Motor3508Recv(&leg[0].wheel,hcan,id,buf);
        Motor3508Recv(&leg[1].wheel,hcan,id,buf);
        Motor3508Recv(&leg[2].wheel,hcan,id,buf);
        Motor3508Recv(&leg[3].wheel,hcan,id,buf);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        RS485SendIRQ_Handler(&rs485bus, huart);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART6)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
        err_timer_cnt=0;    //每接收一次，就清零
    }
}



void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint32_t now = HAL_GetTick();
        
        // 检查错误频率，但不进行复位
        if ((now - error_stats.last_error_time) < 10) { // 10ms内多次错误
            error_stats.continuous_errors++;
        } else {
            error_stats.continuous_errors = 0; // 重置连续错误计数
        }
        error_stats.last_error_time = now;
        
        // 先停止DMA传输，避免在清除错误标志时产生新的中断或错误
        HAL_UART_DMAStop(huart);
        
        // 重置HAL状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        huart->RxState = HAL_UART_STATE_READY;
        huart->gState = HAL_UART_STATE_READY;
        
        // 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
        uint32_t isrflags = READ_REG(huart->Instance->SR);
        
        // 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
        if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) {
            // 对于ORE、NE、FE错误，需要先读SR再读DR
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE
            
            // 统计具体的错误类型
            if (isrflags & USART_SR_ORE) {
                error_stats.overrun++;
            }
            if (isrflags & USART_SR_NE) {
                error_stats.noise++;
            }
            if (isrflags & USART_SR_FE) {
                error_stats.frame++;
            }
        }
        
        if (isrflags & USART_SR_PE) {
            // 奇偶校验错误只需读SR即可清除
            volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
            error_stats.parity++;
        }
        
        // 增加总错误计数
        error_stats.total++;
        error_cnt = error_stats.total; // 保持与原变量的兼容性
        
        last_error_time = now;
        
        RS485RecvIRQ_Handler(&rs485bus, huart, 0);
    }
}

uint64_t uart_reast = 0;
extern TaskHandle_t motor_control_task_handle;
static uint32_t action_stack1[128];
static uint32_t action_stack2[128];
static StaticTask_t task_block1;
static StaticTask_t task_block2;
static uint8_t choose_stack=0;
static TaskHandle_t task_handle=NULL;
SemaphoreHandle_t uart6ResetSem;
void UART6_ServiceTask(void *arg)
{
        uart6ResetSem = xSemaphoreCreateBinary();
        xSemaphoreTake(uart6ResetSem, 0);
     task_handle=xTaskCreateStatic(MotorControlTask,"MotorControl",128,NULL,5,&action_stack1[0],&task_block1);
    for (;;)
    {
        
        if (xSemaphoreTake(uart6ResetSem, portMAX_DELAY) == pdTRUE)
        {
            req_stop_transmit=1;
            while(req_stop_transmit)
                vTaskDelay(2);
            
            vTaskDelete(task_handle);
            __disable_irq();

            
            __HAL_DMA_DISABLE(huart6.hdmarx);
            __HAL_DMA_DISABLE(huart6.hdmatx);
            // 1. 停止 DMA，清 IDLE
            HAL_UART_DMAStop(&huart6);
            __HAL_UART_CLEAR_IDLEFLAG(&huart6);

            // 2. 调用 DeInit，自动触发 MspDeInit
            HAL_UART_DeInit(&huart6);

            // 3. 外设寄存器硬复位
            __HAL_RCC_USART6_FORCE_RESET();
            __HAL_RCC_USART6_RELEASE_RESET();

            // 4. 重新初始化 UART + DMA，MspInit 会自动执行
            MX_USART6_UART_Init();

            
            uart_reast++;
            __enable_irq();
            if(choose_stack==0)
            {
                choose_stack=1;
                task_handle=xTaskCreateStatic(MotorControlTask,"MotorControl",128,NULL,5,&action_stack2[0],&task_block2);
            }
            else
            {
                choose_stack=0;
                task_handle=xTaskCreateStatic(MotorControlTask,"MotorControl",128,NULL,5,&action_stack1[0],&task_block1);
            }
        }
    }
}