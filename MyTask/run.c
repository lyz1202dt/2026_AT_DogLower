#include "run.h"
#include "usart.h"
#include "mylist.h"
#include "usb_trans.h"
#include "WatchDog2.h"
#include <string.h>
#include "usbd_cdc_if.h"
#include "JY61.h"
#include "bezier.h"
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define BACK_LEFT 2
#define BACK_RIGHT 3

// 添加错误统计结构
typedef struct
{
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
uint32_t success_cnt = 0;
uint32_t reast_cnt = 0;
uint32_t err_timer_cnt = 0;
uint8_t data[11];
uint32_t req_stop_transmit;

//******有关遥控器需要的数据定义********

static BezierLine bezier = {.p1_x = 0.660634f, .p1_y = 0.131222f, .p2_x = 0.846154f, .p2_y = 0.556561f}; // 摇杆贝塞尔曲线参数
uint8_t remote_control_buf[12];
float filter_gate = 1.0f, last_v0, last_v1, last_omega, last_v3;
static const float filter_alpha = 0.2f;
float max_omega = 120.0f;
float max_forword_speed = 1.0f, max_backward_speed = 0.5f, max_speed = 0.4f;
float cur_dir = 0.0f;
float key1 = 0, key2 = 0;

RemotePack_t remotedata; // 接收遥控数据的结构体

extern QueueHandle_t remote_semaphore;

//**************************************

extern JY61_Typedef_ JY61_;
extern DMA_HandleTypeDef hdma_uart4_rx;
JY61_Typedef JY61;
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
     .wheel = {.hcan = &hcan1, .ID = 0x201}},

    {.joint[0] = {.motor = {.motor_id = 0x04, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 4.29111939f},
     .joint[1] = {.motor = {.motor_id = 0x05, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.60819724f},
     .joint[2] = {.motor = {.motor_id = 0x06, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.20990329f},
     .wheel = {.hcan = &hcan1, .ID = 0x202}},

    {.joint[0] = {.motor = {.motor_id = 0x07, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 4.09088597f},
     .joint[1] = {.motor = {.motor_id = 0x08, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.67971121f},
     .joint[2] = {.motor = {.motor_id = 0x09, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.661729104f},
     .wheel = {.hcan = &hcan1, .ID = 0x203}},

    {.joint[0] = {.motor = {.motor_id = 0x0A, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -4.19752234f},
     .joint[1] = {.motor = {.motor_id = 0x0B, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = 6.64519156f},
     .joint[2] = {.motor = {.motor_id = 0x0C, .rs485 = &rs485bus}, .inv_motor = 1, .pos_offset = -6.56194712f},
     .wheel = {.hcan = &hcan1, .ID = 0x204}}};

float setup_offset[4][3]; // 上电启动时的电机角度
uint32_t first_run = 5;
uint32_t watch_dog_id[24];
uint32_t reset_uart;

static void watchdog_cb(void *param)
{
    uint32_t user_data = (uint32_t)param;
    legs_state.watch_dog = legs_state.watch_dog | (0x0001 << user_data);
}

static void watchdog_reast(void *param)
{
    reset_uart = 1;
}

SemaphoreHandle_t uart6ResetSem;
uint64_t uart_reast = 0;
int err_check = 0;
void MotorControlTask(void *param) // 将数据发送到电机，并从电机接收数据
{
    TickType_t last_wake_time = xTaskGetTickCount();

    uart6ResetSem = xSemaphoreCreateBinary();
    xSemaphoreTake(uart6ResetSem, 0);
    legs_state.watch_dog = 0x0000;
    watch_dog_id[0] = AddWatchDog(watchdog_cb, 100, (void *)0, WATCHDOG_MODE_REPEAT);
    watch_dog_id[1] = AddWatchDog(watchdog_cb, 100, (void *)1, WATCHDOG_MODE_REPEAT);
    watch_dog_id[2] = AddWatchDog(watchdog_cb, 100, (void *)2, WATCHDOG_MODE_REPEAT);
    watch_dog_id[3] = AddWatchDog(watchdog_cb, 100, (void *)3, WATCHDOG_MODE_REPEAT);
    watch_dog_id[4] = AddWatchDog(watchdog_cb, 100, (void *)4, WATCHDOG_MODE_REPEAT);
    watch_dog_id[5] = AddWatchDog(watchdog_cb, 100, (void *)5, WATCHDOG_MODE_REPEAT);
    watch_dog_id[6] = AddWatchDog(watchdog_cb, 100, (void *)6, WATCHDOG_MODE_REPEAT);
    watch_dog_id[7] = AddWatchDog(watchdog_cb, 100, (void *)7, WATCHDOG_MODE_REPEAT);
    watch_dog_id[8] = AddWatchDog(watchdog_cb, 100, (void *)8, WATCHDOG_MODE_REPEAT);
    watch_dog_id[9] = AddWatchDog(watchdog_cb, 100, (void *)9, WATCHDOG_MODE_REPEAT);
    watch_dog_id[10] = AddWatchDog(watchdog_cb, 100, (void *)10, WATCHDOG_MODE_REPEAT);
    watch_dog_id[11] = AddWatchDog(watchdog_cb, 100, (void *)11, WATCHDOG_MODE_REPEAT);

    watch_dog_id[12] = AddWatchDog(watchdog_reast, 50, (void *)0, WATCHDOG_MODE_REPEAT);
    watch_dog_id[13] = AddWatchDog(watchdog_reast, 50, (void *)1, WATCHDOG_MODE_REPEAT);
    watch_dog_id[14] = AddWatchDog(watchdog_reast, 50, (void *)2, WATCHDOG_MODE_REPEAT);
    watch_dog_id[15] = AddWatchDog(watchdog_reast, 50, (void *)3, WATCHDOG_MODE_REPEAT);
    watch_dog_id[16] = AddWatchDog(watchdog_reast, 50, (void *)4, WATCHDOG_MODE_REPEAT);
    watch_dog_id[17] = AddWatchDog(watchdog_reast, 50, (void *)5, WATCHDOG_MODE_REPEAT);
    watch_dog_id[18] = AddWatchDog(watchdog_reast, 50, (void *)6, WATCHDOG_MODE_REPEAT);
    watch_dog_id[19] = AddWatchDog(watchdog_reast, 50, (void *)7, WATCHDOG_MODE_REPEAT);
    watch_dog_id[20] = AddWatchDog(watchdog_reast, 50, (void *)8, WATCHDOG_MODE_REPEAT);
    watch_dog_id[21] = AddWatchDog(watchdog_reast, 50, (void *)9, WATCHDOG_MODE_REPEAT);
    watch_dog_id[22] = AddWatchDog(watchdog_reast, 50, (void *)10, WATCHDOG_MODE_REPEAT);
    watch_dog_id[23] = AddWatchDog(watchdog_reast, 50, (void *)11, WATCHDOG_MODE_REPEAT);

    while (1)
    {
        err_check = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                // if(j!=1 && i!=0){
                GoMotorSend(&leg[i].joint[j].motor, leg[i].joint[j].exp_torque / 6.33f * leg[i].joint[j].inv_motor,
                            leg[i].joint[j].exp_omega * 6.33f * leg[i].joint[j].inv_motor,
                            leg[i].joint[j].exp_rad * 6.33f * leg[i].joint[j].inv_motor + leg[i].joint[j].pos_offset + setup_offset[i][j],
                            leg[i].joint[j].Kp, leg[i].joint[j].Kd);
                // err_check+=GoMotorRecv(&leg[i].joint[j].motor);

                int ret = GoMotorRecv(&leg[i].joint[j].motor);
                if (ret)
                {
                    FeedDog(watch_dog_id[i * 3 + j]);
                    FeedDog(watch_dog_id[i * 3 + j + 12]);
                    err_check++;
                    legs_state.watch_dog = legs_state.watch_dog & (~(0x0001 << (i * 3 + j)));
                }
                // }
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
        if (reset_uart)
        {
            HAL_UART_DMAStop(&huart6);
            vTaskDelay(2);
            HAL_UART_DeInit(&huart6);

            // 3. 外设寄存器硬复位
            __HAL_RCC_USART6_FORCE_RESET();
            __HAL_RCC_USART6_RELEASE_RESET();

            // 4. 重新初始化 UART + DMA，MspInit 会自动执行
            MX_USART6_UART_Init();
            reset_uart = 0;
            reast_cnt++;
        }

        if (err_check == 12 && first_run)
            first_run--;
    }
}

uint32_t current_size = 0;
uint32_t cnt = 0;
void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
    if (size == sizeof(MotorTargetPack_t) && ((MotorTargetPack_t *)src)->pack_type == 0x00)
    {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        memcpy(&legs_target, src, sizeof(MotorTargetPack_t));
        xSemaphoreGive(cdc_recv_semphr);
    }
    cnt++;
    current_size = size;
    // HAL_UART_Transmit_DMA(&huart3, src, size);
}

PID2 wheel_vel_pid[4];
float wheel_exp_vel[4], wheel_exp_torque[4];
int16_t can_send_buf[4];
float inv_wheel[4] = {-1.0f, 1.0f, -1.0f, 1.0f};
void WheelControlTask(void *param)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    leg[0].wheel.vel_pid.Kp = 0.55f;
    leg[0].wheel.vel_pid.Ki = 0.04f;
    leg[0].wheel.vel_pid.limit = 300.0f;
    leg[0].wheel.vel_pid.output_limit = 4.0f;
    leg[3].wheel.vel_pid = leg[2].wheel.vel_pid = leg[1].wheel.vel_pid = leg[0].wheel.vel_pid;
    while (1)
    {
        for (int i = 0; i < 4; i++)
        {
            PID_Control2(leg[i].wheel.motor.Speed * 3.14159265f * 2.0f / 60.0f / 19.0f, wheel_exp_vel[i] * inv_wheel[i], &leg[i].wheel.vel_pid);
            float out_temp = ((leg[i].wheel.vel_pid.pid_out + wheel_exp_torque[i] * inv_wheel[i]) / 0.3f * (16384.0f / 20.0f / 0.3f));
            if (out_temp > 16384)
                out_temp = 16384;
            else if (out_temp < -16384)
                out_temp = -16384;
            can_send_buf[i] = (int16_t)out_temp;
        }
        MotorSend(&hcan1, 0x200, can_send_buf);
        vTaskDelayUntil(&last_wake_time, 2);
    }
}
uint16_t count = 0;
uint8_t allow_send = 0;
JY61_Typedef_ JY61_2;
uint8_t flag_check = 1;
void MotorSendTask(void *param) // 将电机的数据发送到PC上
{
    USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
    TickType_t last_wake_time = xTaskGetTickCount();
    // vTaskDelay(500);
    while (1)
    {
        for (int i = 0; i < 4; i++) // 填写数据并发送到PC
        {
            for (int j = 0; j < 3; j++)
            {
                legs_state.leg[i].joint[j].rad = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.rad - leg[i].joint[j].pos_offset - setup_offset[i][j]) / 6.33f;
                legs_state.leg[i].joint[j].omega = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.velocity) / 6.33f;
                legs_state.leg[i].joint[j].torque = leg[i].joint[j].inv_motor * (leg[i].joint[j].motor.state.torque) * 6.33f;
            }
            legs_state.leg[i].wheel.omega = leg[i].wheel.motor.Speed * 3.14159265f * 2.0f / 60.0f / 19.0f;
            legs_state.leg[i].wheel.torque = 0.0f; // TODO:根据反馈计算真实力矩
        }

        JY61_2.AngularVelocity.X = JY61.AngularVelocity.X * 3.1415926 / 180.0f;
        JY61_2.AngularVelocity.Y = JY61.AngularVelocity.Y * 3.1415926 / 180.0f;
        JY61_2.AngularVelocity.Z = JY61.AngularVelocity.Z * 3.1415926 / 180.0f;

        JY61_2.Angle.Roll = JY61.Angle.Roll * 3.1415926 / 180.0f;
        JY61_2.Angle.Pitch = JY61.Angle.Pitch * 3.1415926 / 180.0f;
        JY61_2.Angle.Yaw = JY61.Angle.Yaw * 3.1415926 / 180.0f;

        if (JY61_2.AngularVelocity.X > 2.0f || JY61_2.AngularVelocity.X < -2.0f)
            flag_check = 0;
        else if (JY61_2.AngularVelocity.Y > 2.0f || JY61_2.AngularVelocity.Y < -2.0f)
            flag_check = 0;
        else if (JY61_2.AngularVelocity.Z > 2.0f || JY61_2.AngularVelocity.Z < -2.0f)
            flag_check = 0;
        else if (JY61_2.Angle.Roll > 1.0f || JY61_2.Angle.Roll < -1.0f)
            flag_check = 0;
        else if (JY61_2.Angle.Pitch > 1.0f || JY61_2.Angle.Pitch < -1.0f)
            flag_check = 0;
        else if (JY61_2.Angle.Yaw > 1.0f || JY61_2.Angle.Yaw < -1.0f)
            flag_check = 0;

        if (flag_check)
        {
            legs_state.JY61_ = JY61_2;
        }
        else
            flag_check = 1;
        if (allow_send) // 电机数据准备好再发
            CDC_Transmit_FS((uint8_t *)&legs_state, sizeof(legs_state));
        if (legs_state.watch_dog != 0x0000)
            count++;

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(5));
    }
}

static uint32_t slope(float target, float *cur_target, float step_limit)
{
    float delta = target - *cur_target;
    uint32_t ret = 0;

    if (delta > step_limit)
        delta = step_limit;
    else if (delta < -step_limit)
        delta = -step_limit;
    else // 本次结束后，应该就会到达目标值
        ret = 1;

    *cur_target = *cur_target + delta;
    return ret;
}

float init_pos[4][3] = {{0.0f, 0.8f, 0.0f},
                        {0.0f, -0.8f, 0.0f},
                        {0.0f, 0.8f, 0.0f},
                        {0.0f, -0.8f, 0.0f}};
static uint32_t DogReset(uint32_t time_out_ms)
{
    __disable_irq();            // 更新电机初始关节角度和Kp，关闭中断确保绝对同步，防止狗腿震荡
    for (int i = 0; i < 4; i++) // 将狗腿状态读入当前目标值，设置狗腿期望为当前位置
    {
        leg[i].joint[0].exp_omega = 0.0f;
        leg[i].joint[0].exp_torque = 0.0f;
        leg[i].joint[0].Kp = 3.0f;
        leg[i].joint[0].Kd = 0.17f;
        leg[i].joint[0].exp_rad = legs_state.leg[i].joint[0].rad;

        leg[i].joint[1].exp_omega = 0.0f;
        leg[i].joint[1].exp_torque = 0.0f;
        leg[i].joint[1].Kp = 3.0f;
        leg[i].joint[1].Kd = 0.14f;
        leg[i].joint[1].exp_rad = legs_state.leg[i].joint[1].rad;

        leg[i].joint[2].exp_omega = 0.0f;
        leg[i].joint[2].exp_torque = 0.0f;
        leg[i].joint[2].Kp = 3.0f;
        leg[i].joint[2].Kd = 0.11f;
        leg[i].joint[2].exp_rad = legs_state.leg[i].joint[2].rad;
    }
    __enable_irq();
    // PID参数:[0]:kp=4.0,kd=0.24;[1]:kp=3.7,kd=0.24;[2]:kp=3,kd=0.11

    uint32_t finished_check;
    uint32_t current_time = 0;
    do
    {
        finished_check = 0;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 3; j++)
                finished_check = finished_check + slope(init_pos[i][j], &leg[i].joint[j].exp_rad, 0.001f);
        }
        vTaskDelay(5);
        current_time = current_time + 5;
        if (current_time > time_out_ms) // 如果复位已经超时，那么返回0
            return 0;
    } while (finished_check != 12); // 完成校验不等于12，说明有电机没有完成复位

    return 1; // 返回1表示狗复位成功
}

void MotorRecvTask(void *param) // 从PC接收电机的期望值
{
    cdc_recv_semphr = xSemaphoreCreateBinary();
    xSemaphoreTake(cdc_recv_semphr, 0);
    vTaskDelay(1000);
    while (first_run) // 等待电机数据准备好
        vTaskDelay(1);
    // TODO:上电时电机角度在极点附近的处理
    //        if(leg[0].joint[0].motor.state.rad>4.0f)
    //            leg[0].joint[0].pos_offset=leg[0].joint[0].pos_offset+6.2831853f;
    //        if(leg[1].joint[2].motor.state.rad<3.0f)
    //            leg[1].joint[2].pos_offset=leg[1].joint[2].pos_offset-6.2831853f;
    //        if(leg[2].joint[2].motor.state.rad<3.0f)
    //            leg[2].joint[2].pos_offset=leg[2].joint[2].pos_offset-6.2831853f;
    //        if(leg[3].joint[1].motor.state.rad<3.0f)
    //            leg[3].joint[1].pos_offset=leg[3].joint[1].pos_offset-6.2831853f;
    for (int i = 0; i < 4; i++)
    {
        setup_offset[i][0] = leg[i].joint[0].motor.state.rad;
        setup_offset[i][1] = leg[i].joint[1].motor.state.rad;
        setup_offset[i][2] = leg[i].joint[2].motor.state.rad;
    }
    allow_send = 1;                                 // 允许发送数据
    xSemaphoreTake(cdc_recv_semphr, portMAX_DELAY); // 等待第一个数据帧到来
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
                wheel_exp_torque[i] = 0.0f;
                wheel_exp_vel[i] = 0.0f;
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
            wheel_exp_vel[i] = legs_target.leg[i].wheel.omega;
            wheel_exp_torque[i] = legs_target.leg[i].wheel.torque;
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t buf[8];
    if (hcan->Instance == CAN1)
    {
        uint32_t id = CAN_Receive_DataFrame(hcan, buf);
        Motor3508Recv(&leg[0].wheel, hcan, id, buf);
        Motor3508Recv(&leg[1].wheel, hcan, id, buf);
        Motor3508Recv(&leg[2].wheel, hcan, id, buf);
        Motor3508Recv(&leg[3].wheel, hcan, id, buf);
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
    if (huart->Instance == UART4)
    {
        JY61_Receive(&JY61, data, size);
        HAL_UART_DMAStop(&huart4);
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, data, sizeof(data));
        __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
    }
    else if (huart->Instance == UART5)
    {

        if (remote_control_buf[0] == 0xAA)
        {
            memcpy(&remotedata, remote_control_buf, 12);

            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(remote_semaphore, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_control_buf, sizeof(remote_control_buf));
    }
    else if (huart->Instance == USART6)
    {
        RS485RecvIRQ_Handler(&rs485bus, huart, size);
        success_cnt++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    volatile uint32_t temp=0;
    if (huart->Instance == USART6)
    {
        HAL_UART_DMAStop(huart);
        if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE))
            temp=huart->Instance->DR;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        __HAL_UART_FLUSH_DRREGISTER(huart);
        error_cnt++;
        UNUSED(temp);
        // RS485RecvIRQ_Handler(&rs485bus, huart, 0);
    }
    else if (huart->Instance == UART4)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, data, sizeof(data));
    }
    else if (huart->Instance == UART5)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        HAL_UART_DMAStop(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, remote_control_buf, sizeof(remote_control_buf));
    }
}

void UART5_RemotecontrolTask(void *param)
{

    TickType_t xLastWakeTime = xTaskGetTickCount(); // 获取当前 Tick
    const TickType_t xFrequency = pdMS_TO_TICKS(100);

    while (1)
    {

        float vel[3];
        // 阻塞等待 ISR 释放的信号量
        if (xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) != pdTRUE)
        { // 200ms未收到遥控器的数据，复位摇杆
            remotedata.rocker[0] = 0;
            remotedata.rocker[1] = 0;
            remotedata.rocker[2] = 0;
            remotedata.rocker[3] = 0;
        }

        __disable_irq();
        last_v0 = filter_gate * (((float)(remotedata.rocker[0])) / 2047.0f) + (1.0f - filter_gate) * last_v0;
        vel[0] = last_v0;
        last_v1 = filter_gate * (((float)(remotedata.rocker[1])) / 2047.0f) + (1.0f - filter_gate) * last_v1;
        vel[1] = last_v1;
        last_omega = (((float)(remotedata.rocker[2])) / 2047.0f) * max_omega * filter_gate + (1.0f - filter_gate) * last_omega;
        key1 = remotedata.key1;

        __enable_irq();

        legs_state.remote_cmd.omega = BezierTransform(last_omega, bezier); // 计算自转角速度
        legs_state.remote_cmd.omega = legs_state.remote_cmd.omega * M_PI / 180.0f;

        float rocker_val = (float)remotedata.rocker[3] / 2047.0f;
        last_v3 = filter_alpha * rocker_val + (1.0f - filter_alpha) * last_v3;
        vel[2] = last_v3 * 1.0f;

        // 归一化第四个摇杆值

        float model = sqrtf(vel[0] * vel[0] + vel[1] * vel[1]);
        if (model != 0.0f)
            cur_dir = atan2f(vel[1], vel[0]);

        model = BezierTransform(model, bezier);
        vel[1] = model * sinf(cur_dir);

        if (vel[1] >= 0)
        {
            vel[1] = vel[1] * max_forword_speed;
        }
        else if (vel[1] < 0)
        {
            vel[1] = vel[1] * max_backward_speed;
        };

        vel[0] = model * cosf(cur_dir) * max_speed;

        legs_state.remote_cmd.vy = vel[0];
        legs_state.remote_cmd.vx = vel[1];
        legs_state.remote_cmd.wheel_v = vel[2];

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
