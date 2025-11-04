#ifndef _KALMAN_H
#define _KALMAN_H

#include <stdint.h>      // 标准整数类型

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 一阶卡尔曼滤波器
 */
typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
} kalman_filter_t;

/**
  * @brief 初始化一个卡尔曼滤波器
  * @param[out] p 滤波器
  * @param[in] T_Q 系统噪声协方差
  * @param[in] T_R 测量噪声协方差
  */
void kalman_Init(kalman_filter_t *p, float T_Q, float T_R, float X);

/**
  * @brief 卡尔曼滤波器
  * @param[in] p 滤波器
  * @param[in] dat 待滤波信号
  * @retval 滤波后的信号
  */
float Kalman_Filter(kalman_filter_t *p, float dat);


#ifdef __cplusplus
}
#endif

/*
typedef struct {
    arm_matrix_instance_f32 F;  // 状态转移矩阵 (4x4)
    arm_matrix_instance_f32 H;  // 观测矩阵 (2x4)
    arm_matrix_instance_f32 Q;  // 过程噪声 (4x4)
    arm_matrix_instance_f32 R;  // 观测噪声 (2x2)
    arm_matrix_instance_f32 P;  // 状态协方差 (4x4)
    arm_matrix_instance_f32 K;  // 卡尔曼增益 (4x2)
    arm_matrix_instance_f32 X;  // 状态向量 (4x1)
    arm_matrix_instance_f32 X_pred;  // 预测状态 (4x1)
    arm_matrix_instance_f32 z;  // 观测向量 (2x1)

    // 临时变量
    arm_matrix_instance_f32 temp1_4x4;
    arm_matrix_instance_f32 temp2_4x4;
    arm_matrix_instance_f32 temp3_2x2;
    arm_matrix_instance_f32 I;

    // 内部数据存储
    float F_data[16], H_data[8], Q_data[16], R_data[4], P_data[16], K_data[8];
    float X_data[4], X_pred_data[4], z_data[2];
    float temp1_data[16], temp2_data[16], temp3_data[4], I_data[16];
} kalman2d_vector_t;


typedef struct {
    float32_t dt;

    // 状态 x (2x1)
    float32_t x_data[2];
    arm_matrix_instance_f32 x;

    // 协方差 P (2x2)
    float32_t P_data[4];
    arm_matrix_instance_f32 P;

    // 状态转移 F (2x2)
    float32_t F_data[4];
    arm_matrix_instance_f32 F;

    // 过程噪声 Q (2x2)
    float32_t Q_data[4];
    arm_matrix_instance_f32 Q;

    // 观测矩阵 H (2x2)
    float32_t H_data[4];
    arm_matrix_instance_f32 H;

    // 观测噪声 R (2x2)
    float32_t R_data[4];
    arm_matrix_instance_f32 R;

    // 临时变量（避免频繁分配）
    float32_t temp_data[16]; // 可做最大4x4中间矩阵
    arm_matrix_instance_f32 temp1;
    arm_matrix_instance_f32 temp2;
    arm_matrix_instance_f32 temp3;
} KalmanYawFilter;


void kalman2d_vector_init(kalman2d_vector_t *kf, float dt, float q, float r);
void kalman2d_vector_update(kalman2d_vector_t *kf, float x_meas, float y_meas, float dt, float q, float r);

void Kalman_SetConfidence(KalmanYawFilter* kf, float gyro_var, float wheel_var);
void KalmanCMSIS_Update(KalmanYawFilter* kf, float32_t gyro_omega, float32_t wheel_omega);
void KalmanCMSIS_Init(KalmanYawFilter* kf, float32_t dt);
*/
#endif
