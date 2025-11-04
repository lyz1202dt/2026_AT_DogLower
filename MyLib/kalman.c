#include "kalman.h"

void kalman_Init(kalman_filter_t *p, float T_Q, float T_R, float X) {
    p->X_last = X;
    p->P_last = T_R * 1000.f;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1.0f;
    p->X_mid = p->X_last;
}

float Kalman_Filter(kalman_filter_t *p, float dat) {
    p->X_mid = p->A * p->X_last;                    //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A * p->P_last + p->Q;             //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid / (p->P_mid + p->R);           //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid + p->kg * (dat - p->X_mid); //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1 - p->kg) * p->P_mid;              //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                           //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}
/*
void kalman2d_vector_init(kalman2d_vector_t *kf, float dt, float q, float r) {
    arm_mat_init_f32(&kf->F, 4, 4, kf->F_data);
    arm_mat_init_f32(&kf->H, 2, 4, kf->H_data);
    arm_mat_init_f32(&kf->Q, 4, 4, kf->Q_data);
    arm_mat_init_f32(&kf->R, 2, 2, kf->R_data);
    arm_mat_init_f32(&kf->P, 4, 4, kf->P_data);
    arm_mat_init_f32(&kf->K, 4, 2, kf->K_data);
    arm_mat_init_f32(&kf->X, 4, 1, kf->X_data);
    arm_mat_init_f32(&kf->X_pred, 4, 1, kf->X_pred_data);
    arm_mat_init_f32(&kf->z, 2, 1, kf->z_data);

    arm_mat_init_f32(&kf->temp1_4x4, 4, 4, kf->temp1_data);
    arm_mat_init_f32(&kf->temp2_4x4, 4, 4, kf->temp2_data);
    arm_mat_init_f32(&kf->temp3_2x2, 2, 2, kf->temp3_data);
    arm_mat_init_f32(&kf->I, 4, 4, kf->I_data);

    // F: 状态转移矩阵
    float F_tmp[16] = {
        1, dt, 0,  0,
        0, 1,  0,  0,
        0, 0,  1, dt,
        0, 0,  0, 1
    };
    memcpy(kf->F_data, F_tmp, sizeof(F_tmp));

    // H: 观测矩阵
    float H_tmp[8] = {
        1, 0, 0, 0,
        0, 0, 1, 0
    };
    memcpy(kf->H_data, H_tmp, sizeof(H_tmp));

    // Q: 过程噪声协方差
    memset(kf->Q_data, 0, sizeof(kf->Q_data));
    kf->Q_data[0] = q;
    kf->Q_data[5] = q;
    kf->Q_data[10] = q;
    kf->Q_data[15] = q;

    // R: 观测噪声协方差
    kf->R_data[0] = r;
    kf->R_data[3] = r;

    // P: 状态协方差矩阵
    for (int i = 0; i < 16; i++) kf->P_data[i] = 0.0f;
    kf->P_data[0] = 1.0f;
    kf->P_data[5] = 1.0f;
    kf->P_data[10] = 1.0f;
    kf->P_data[15] = 1.0f;

    // I: 单位矩阵
    memset(kf->I_data, 0, sizeof(kf->I_data));
    for (int i = 0; i < 4; i++) kf->I_data[i * 5] = 1.0f;

    // 初始状态为0
    memset(kf->X_data, 0, sizeof(kf->X_data));
}

void kalman2d_vector_update(kalman2d_vector_t *kf, float x_meas, float y_meas, float dt, float q, float r) {
	
	kf->Q_data[0]  = q;
    kf->Q_data[5]  = q;
    kf->Q_data[10] = q;
    kf->Q_data[15] = q;

    kf->R_data[0] = r;
    kf->R_data[3] = r;

	
    // Step 0: 动态更新 F 矩阵
    kf->F_data[1] = dt;   // F[0][1] = dt
    kf->F_data[14] = dt;  // F[2][3] = dt

    kf->z_data[0] = x_meas;
    kf->z_data[1] = y_meas;

    // 定义 H 的转置矩阵 H_T: 4x2
    arm_matrix_instance_f32 H_T;
    float H_T_data[8]; // 4*2
    arm_mat_init_f32(&H_T, 4, 2, H_T_data);
    arm_mat_trans_f32(&kf->H, &H_T);

    // Step 1: X_pred = F * X
    arm_mat_mult_f32(&kf->F, &kf->X, &kf->X_pred);

    // Step 2: P = F * P * F^T + Q
    arm_mat_mult_f32(&kf->F, &kf->P, &kf->temp1_4x4);
    arm_mat_trans_f32(&kf->F, &kf->temp2_4x4);  // temp2_4x4: 4x4
    arm_mat_mult_f32(&kf->temp1_4x4, &kf->temp2_4x4, &kf->temp1_4x4);
    arm_mat_add_f32(&kf->temp1_4x4, &kf->Q, &kf->P);  // New P

    // Step 3: S = H * P * H^T + R
    arm_matrix_instance_f32 S;
    float S_data[4]; // 2x2
    arm_mat_init_f32(&S, 2, 2, S_data);
    arm_matrix_instance_f32 temp_2x4;
    float temp_2x4_data[8]; // 2x4
    arm_mat_init_f32(&temp_2x4, 2, 4, temp_2x4_data);
    arm_mat_mult_f32(&kf->H, &kf->P, &temp_2x4);         // 2x4
    arm_mat_mult_f32(&temp_2x4, &H_T, &S);                // 2x4 * 4x2 = 2x2
    arm_mat_add_f32(&S, &kf->R, &S);                      // S = HPH^T + R

    // Step 4: K = P * H^T * S^-1
    arm_matrix_instance_f32 S_inv;
    float S_inv_data[4]; // 2x2
    arm_mat_init_f32(&S_inv, 2, 2, S_inv_data);
    arm_mat_inverse_f32(&S, &S_inv);  // S^-1

    arm_mat_mult_f32(&kf->P, &H_T, &kf->K);      // 4x4 * 4x2 = 4x2
    arm_mat_mult_f32(&kf->K, &S_inv, &kf->K);    // 4x2 * 2x2 = 4x2

    // Step 5: X = X_pred + K * (z - H * X_pred)
    arm_matrix_instance_f32 y;
    float y_data[2]; // 2x1
    arm_mat_init_f32(&y, 2, 1, y_data);
    arm_matrix_instance_f32 Hx;
    float Hx_data[2]; // 2x1
    arm_mat_init_f32(&Hx, 2, 1, Hx_data);
    arm_mat_mult_f32(&kf->H, &kf->X_pred, &Hx);     // H * X_pred
    arm_mat_sub_f32(&kf->z, &Hx, &y);               // y = z - Hx

    arm_matrix_instance_f32 K_y;
    float K_y_data[4]; // 4x1
    arm_mat_init_f32(&K_y, 4, 1, K_y_data);
    arm_mat_mult_f32(&kf->K, &y, &K_y);             // K * y
    arm_mat_add_f32(&kf->X_pred, &K_y, &kf->X);     // X = X_pred + K*y

    // Step 6: P = (I - K * H) * P
    arm_mat_mult_f32(&kf->K, &kf->H, &kf->temp1_4x4);   // K*H
    arm_mat_sub_f32(&kf->I, &kf->temp1_4x4, &kf->temp2_4x4); // I - K*H
    arm_mat_mult_f32(&kf->temp2_4x4, &kf->P, &kf->P);   // P = (I - K*H) * P
}


void KalmanCMSIS_Init(KalmanYawFilter* kf, float32_t dt) {
    kf->dt = dt;

    // 初始化矩阵
    arm_mat_init_f32(&kf->x, 2, 1, kf->x_data);
    arm_mat_init_f32(&kf->P, 2, 2, kf->P_data);
    arm_mat_init_f32(&kf->F, 2, 2, kf->F_data);
    arm_mat_init_f32(&kf->Q, 2, 2, kf->Q_data);
    arm_mat_init_f32(&kf->H, 2, 2, kf->H_data);
    arm_mat_init_f32(&kf->R, 2, 2, kf->R_data);

    arm_mat_init_f32(&kf->temp1, 2, 2, kf->temp_data);
    arm_mat_init_f32(&kf->temp2, 2, 2, kf->temp_data + 4);
    arm_mat_init_f32(&kf->temp3, 2, 2, kf->temp_data + 8);

    // 初始状态
    kf->x_data[0] = 0.0f;
    kf->x_data[1] = 0.0f;

    // 初始协方差
    kf->P_data[0] = 0.1f; kf->P_data[1] = 0.0f;
    kf->P_data[2] = 0.0f; kf->P_data[3] = 0.1f;

    // F
    kf->F_data[0] = 1.0f; kf->F_data[1] = dt;
    kf->F_data[2] = 0.0f; kf->F_data[3] = 1.0f;

    // Q
    kf->Q_data[0] = 1e-4f; kf->Q_data[1] = 0.0f;
    kf->Q_data[2] = 0.0f;  kf->Q_data[3] = 1e-3f;

    // H: 两个观测都观测 omega（状态第2项）
    kf->H_data[0] = 0.0f; kf->H_data[1] = 1.0f;
    kf->H_data[2] = 0.0f; kf->H_data[3] = 1.0f;

    // R: 观测噪声
    kf->R_data[0] = 0.02f; kf->R_data[1] = 0.0f;
    kf->R_data[2] = 0.0f;  kf->R_data[3] = 0.01f;
}


void KalmanCMSIS_Update(KalmanYawFilter* kf, float32_t gyro_omega, float32_t wheel_omega) {
    // === Prediction ===
    // x = F * x
    arm_mat_mult_f32(&kf->F, &kf->x, &kf->temp1);
    memcpy(kf->x.pData, kf->temp1.pData, sizeof(float32_t) * 2);

    // P = F * P * F^T + Q
    arm_mat_mult_f32(&kf->F, &kf->P, &kf->temp1);               // temp1 = F * P
    arm_mat_trans_f32(&kf->F, &kf->temp2);                      // temp2 = F^T
    arm_mat_mult_f32(&kf->temp1, &kf->temp2, &kf->temp3);       // temp3 = FPF^T
    arm_mat_add_f32(&kf->temp3, &kf->Q, &kf->P);                // P = temp3 + Q

    // === Measurement update ===
    float32_t z_data[2] = { gyro_omega, wheel_omega };
    arm_matrix_instance_f32 z;
    arm_mat_init_f32(&z, 2, 1, z_data);

    // y = z - H * x
    arm_mat_mult_f32(&kf->H, &kf->x, &kf->temp1);               // temp1 = H * x
    arm_mat_sub_f32(&z, &kf->temp1, &kf->temp2);                // temp2 = y

    // S = H * P * H^T + R
    arm_mat_mult_f32(&kf->H, &kf->P, &kf->temp1);               // temp1 = H * P
    arm_mat_trans_f32(&kf->H, &kf->temp3);                      // temp3 = H^T
    arm_mat_mult_f32(&kf->temp1, &kf->temp3, &kf->temp2);       // temp2 = HPH^T
    arm_mat_add_f32(&kf->temp2, &kf->R, &kf->temp1);            // temp1 = S

    // K = P * H^T * S^-1
    arm_mat_inverse_f32(&kf->temp1, &kf->temp2);                // temp2 = S^-1
    arm_mat_mult_f32(&kf->P, &kf->temp3, &kf->temp1);           // temp1 = P * H^T
    arm_mat_mult_f32(&kf->temp1, &kf->temp2, &kf->temp3);       // temp3 = K

    // x = x + K * y
    arm_mat_mult_f32(&kf->temp3, &kf->temp2, &kf->temp1);       // temp1 = K * y
    arm_mat_add_f32(&kf->x, &kf->temp1, &kf->x);

    // P = (I - K * H) * P
    float32_t I_data[4] = {1, 0, 0, 1};
    arm_matrix_instance_f32 I;
    arm_mat_init_f32(&I, 2, 2, I_data);

    arm_mat_mult_f32(&kf->temp3, &kf->H, &kf->temp1);           // temp1 = K * H
    arm_mat_sub_f32(&I, &kf->temp1, &kf->temp2);                // temp2 = I - KH
    arm_mat_mult_f32(&kf->temp2, &kf->P, &kf->P);               // P = (I - KH) * P
}

void Kalman_SetConfidence(KalmanYawFilter* kf, float gyro_var, float wheel_var) {
    // 观测噪声协方差矩阵 R 是 2x2 对角阵
    // [ gyro_var,      0
    //         0, wheel_var ]
    kf->R_data[0] = gyro_var;  // R[0][0]
    kf->R_data[1] = 0.0f;      // R[0][1]
    kf->R_data[2] = 0.0f;      // R[1][0]
    kf->R_data[3] = wheel_var; // R[1][1]
}
*/
