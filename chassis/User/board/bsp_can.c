/************************
 * @file bsp_can.c
 * @author Serialist (ba3pt@chd.edu.cn)
 * @brief
 * @version 0.1.0
 * @date 2025-10-30
 *
 * @copyright Copyright (c) VGD 2025
 *
 ************************/

#include "bsp_can.h"
#include "can.h"
#include "cmsis_os.h"
#include "wheel_legged_chassis.h"
#include "motor.h"

extern Wheel_Leg_Target_t set;

// AK_motor_fdb_t AK_motor[4]; // 伺服模式的数据

Motor_AK_RxData_t ak10[4];
DJI_RxData_Def_t m3508[2];

uint8_t can_rx_data[8];
void can_filter_init(void)
{

	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_message;
	if (hcan == &hcan1)
	{
		if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_message, can_rx_data) == HAL_OK) // 获得接收到的数据头和数据
		{
			switch (rx_message.StdId)
			{
			case HUB_R_ID:
				DJI_Motor_Receive(&m3508[RIGHT], can_rx_data);
				motor_status.receive_flag[WR] = true;
				break;

			case HUB_L_ID:
				DJI_Motor_Receive(&m3508[LEFT], can_rx_data);
				motor_status.receive_flag[WL] = true;
				break;

			case HIP_RF_ID:
				Motor_AK_MIT_Decode(&ak10[RF], can_rx_data, P_MIN, V_MAX, T_MAX);
				motor_status.receive_flag[RF] = true;
				break;

			case HIP_RB_ID:
				Motor_AK_MIT_Decode(&ak10[RB], can_rx_data, P_MIN, V_MAX, T_MAX);
				motor_status.receive_flag[RB] = true;
				break;

			case HIP_LF_ID:
				Motor_AK_MIT_Decode(&ak10[LF], can_rx_data, P_MIN, V_MAX, T_MAX);
				motor_status.receive_flag[LF] = true;
				break;

			case HIP_LB_ID:
				Motor_AK_MIT_Decode(&ak10[LB], can_rx_data, P_MIN, V_MAX, T_MAX);
				motor_status.receive_flag[LB] = true;
				break;

			default:
				break;
			}
		}
	}
	else if (hcan == &hcan2)
	{
		if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &rx_message, can_rx_data) == HAL_OK) // 获得接收到的数据头和数据
		{
		}
	}
}

// 古代的 ak10 mit 模式接受代码
// int id = can_rx_data[0]; // 驱动 ID 号
// int p_int = (can_rx_data[1] << 8) | (can_rx_data[2]);
// int v_int = (can_rx_data[3] << 4) | (can_rx_data[4] >> 4);
// int i_int = ((can_rx_data[4] & 0xF) << 8) | (can_rx_data[5]);
// int T_int = can_rx_data[6];
// float p = Uint_To_Float(p_int, P_MIN, P_MAX, 16);
// float v = Uint_To_Float(v_int, V_MIN, V_MAX, 12);
// float i = Uint_To_Float(i_int, -T_MAX, T_MAX, 12);
// float Temp = T_int;
// if (id == HIP_RF_ID)
// {
// 	motorAK10[RF].angle = p;
// 	motorAK10[RF].motor_ctrlspd = v;
// 	motorAK10[RF].motor_ctrltor = i;
// 	motorAK10[RF].motor_ctrltemp = Temp - 40;
// }
