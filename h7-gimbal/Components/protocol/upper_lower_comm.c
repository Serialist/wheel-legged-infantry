/**
 * @file upper_lower_comm.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-03-03
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "upper_lower_comm.h"

void ULComm_Aimbot_Cmd_Encode(ULComm_Aimbot_Cmd_t *data, uint8_t *buf)
{
}

bool ULComm_Aimbot_Cmd_Decode(uint8_t *buf, ULComm_Aimbot_Cmd_t *data)
{
    if (buf[0] == 0xff && buf[15] == 0x0d)
    {
        return false;
    }

    data->fire = (bool)buf[1];

    data->eyaw = buf[2] << 24 |
                 buf[3] << 16 |
                 buf[4] << 8 |
                 buf[5];

    data->epitch = buf[6] << 24 |
                   buf[7] << 16 |
                   buf[8] << 8 |
                   buf[9];

    data->ex = buf[10] << 24 |
               buf[11] << 16 |
               buf[12] << 8 |
               buf[13];
		
		return true;
}

// static void AutoAim_TxVCP(void)
// {
//     uint8_t robo_color = 100;

//     uint8_t buf[16];

//     buf[0] = 0xff;
//     buf[1] = 0x00;

//     buf[2] = ins.yaw <<

//              uint8_t IMU_Data[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};

//     Float_to_Byte(INS.Yaw, IMU_Data[YAW]);
//     Float_to_Byte(-INS.Pitch, IMU_Data[PITCH]);
//     Float_to_Byte(0, IMU_Data[2]);
//     if (Vision_Data.Vision_RobotID >= 10)
//         robo_color = Red;
//     else
//         robo_color = Blue;
//     uint8_t All_Data[16] = {0xff, 0x00,
//                             IMU_Data[2][0], IMU_Data[2][1], IMU_Data[2][2], IMU_Data[2][3],
//                             IMU_Data[PITCH][0], IMU_Data[PITCH][1], IMU_Data[PITCH][2], IMU_Data[PITCH][3],
//                             IMU_Data[YAW][0], IMU_Data[YAW][1], IMU_Data[YAW][2], IMU_Data[YAW][3], 0x00, 0x0d};
//     CDC_Transmit_FS(All_Data, sizeof(All_Data));
// }
