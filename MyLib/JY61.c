#include "JY61.h"
#include "run.h"
uint8_t Gyroscope_Init_count = 0;
//˳ʱ��Ϊ��
float Yaw_offset;
JY61_Typedef JY61;
extern MotorStatePack_t legs_state;



static uint8_t sum10(uint8_t *data) {
    uint8_t sum = 0;
    for (uint8_t i = 0; i < 10; i++)
        sum += data[i];
    return sum;
}

void JY61_Receive(JY61_Typedef* Gyro, uint8_t *data, uint8_t len) {
    for (uint8_t i = 0; i < len; i++) {
        if (data[i] == 0x55) { // ��ʶ��
            Angle_Pack_Typedef pack = *(Angle_Pack_Typedef *)(data + i);
            switch (pack.ID) {
                case 0x51://���ٶ�
                    if (pack.sum == sum10(data + i)) {
                        i += 9;//����������ݰ�����?
                        Gyro->Acceleration.X = pack.X / 32768.0f * 16;
                        Gyro->Acceleration.Y = pack.Y / 32768.0f * 16;
                        Gyro->Acceleration.Z = pack.Z / 32768.0f * 16;
                    }
                    break;
                case 0x52://���ٶ�
                    if (pack.sum == sum10(data + i)) {
                        i += 9;
                        Gyro->AngularVelocity.X = pack.X / 32768.0f * 2000;
                        Gyro->AngularVelocity.Y = pack.Y / 32768.0f * 2000;
                        Gyro->AngularVelocity.Z = pack.Z / 32768.0f * 2000;
                        legs_state.JY61_.AngularVelocity.X=Gyro->AngularVelocity.X*3.1415926/180.0f;
                        legs_state.JY61_.AngularVelocity.Y=Gyro->AngularVelocity.Y*3.1415926/180.0f;
                        legs_state.JY61_.AngularVelocity.Z=Gyro->AngularVelocity.Z*3.1415926/180.0f;
                    }
                    break;
                case 0x53://�Ƕ�
                    if (pack.sum == sum10(data + i)) {
                        i += 9;
                        Gyro->Angle.lastYaw = Gyro->Angle.Yaw;
                        Gyro->Angle.Roll = pack.X / 32768.0f * 180;
                        Gyro->Angle.Pitch = pack.Y / 32768.0f * 180;
                        Gyro->Angle.Yaw = pack.Z / 32768.0f * 180;
                        Gyro->Temp = pack.Temp / 340.0f + 36.53f;
                        float diff = Gyro->Angle.Yaw - Gyro->Angle.lastYaw;
                        legs_state.JY61_.Angle.Roll = Gyro->Angle.Roll*3.1415926/180.0f;
                        legs_state.JY61_.Angle.Pitch = Gyro->Angle.Pitch*3.1415926/180.0f;
                        legs_state.JY61_.Angle.Yaw = Gyro->Angle.Yaw*3.1415926/180.0f;

                        if (diff > 180)
                            Gyro->Angle.rand--;
                        else if (diff < -180)
                            Gyro->Angle.rand++;

                        Gyro->Angle.Multiturn = Gyro->Angle.Yaw + Gyro->Angle.rand * 360.0f - Yaw_offset;//˳ʱ��Ϊ��
                    }
                    break;
            }
			Gyroscope_Init_count = 1;
        }
    }
}
