/**
 * @file Gimbal_Task.c
 * @author Serialist (ba3pt@qq.com)
 * @brief
 * @version 0.1.0
 * @date 2026-02-24
 *
 * @copyright Copyright (c) Serialist 2026
 *
 */

#include "shoot.h"
#include "cmsis_os.h"
#include "PID.h"
#include "Motor.h"
#include "rm_motor.h"
#include "control.h"

RM_Motor_Fdb_t fr_motor[2];

Shoot_t shoot;

//                                KP KI KD Alpha Deadband I_MAX Output_MAX
static float feed_pid_param[7] = {0, 0, 0, 0, 0, 0, 0};
static float fr_l_pid_param[7] = {10.f, 0, 0, 0, 0, 0, 5000.f};
static float fr_r_pid_param[7] = {10.f, 0, 0, 0, 0, 0, 5000.f};

PID_Info_TypeDef feed_pid;
PID_Info_TypeDef fr_l_pid;
PID_Info_TypeDef fr_r_pid;

TickType_t shoot_task_tick = 0;

void Shoot_Task(void const *argument)
{
  uint8_t st_can_buf[8];
  FDCAN_TxHeaderTypeDef st_can_header = {
      .Identifier = 0x200,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = 8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0,
  };

  PID_Init(&feed_pid, PID_POSITION, feed_pid_param);
  PID_Init(&fr_l_pid, PID_POSITION, fr_l_pid_param);
  PID_Init(&fr_r_pid, PID_POSITION, fr_r_pid_param);

  shoot.target.fr = 0;

  for (;;)
  {
    shoot_task_tick = osKernelSysTick();

    shoot.output.fr[LEFT] = (int16_t)PID_Calculate(&fr_l_pid, -shoot.target.fr, fr_motor[LEFT].speed);
    shoot.output.fr[RIGHT] = (int16_t)PID_Calculate(&fr_r_pid, shoot.target.fr, fr_motor[RIGHT].speed);

    if (control.status != RBS_RUNNING)
    {
      shoot.output.fr[LEFT] = shoot.output.fr[RIGHT] = 0;
    }

    RM_Motor_Cmd_Encode(shoot.output.fr[LEFT], shoot.output.fr[RIGHT], 0, 0, st_can_buf);
    USER_FDCAN_Transmit(1, 0x200, st_can_buf);

    osDelay(1);
  }
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//

// float fr_set[2];
// float st_set;
// float fr_cmd[2];
// float st_cmd;
// PID_Info_TypeDef st_pid, fr_pid[2];
// void shoot_task(void const *pvParameters)
// {
//   shoot_init();
//   while (1)
//   {
//     GetFRMotorData(&shoot);
//     ShootMode_Set();
//     // Shoot_Heat_limit(&SHOOT_LIMIT, &shoot);
//     Shoot_Speed_Set(&shoot); // 拨盘摩擦轮设置
//     Shoot_Control();         // 拨盘摩擦轮控制
//     Sent_Shoot(fr_set[LEFT],
//                fr_set[RIGHT],
//                st_set,
//                0);
//     osDelay(1);
//   }
// }
// float _FRM_Error1 = 0, _FRM_Error2 = 0;
// static void GetFRMotorData(Shoot_t *data_fdb)
// {
//   GetRCData(&(data_fdb->rc_data));
//   Get_New_Rc_data(&shoot_new_rc_data);
//   GetFR1Motor(&(data_fdb->FR_motor_fdb[APP_FR1]));
//   GetFR2Motor(&(data_fdb->FR_motor_fdb[APP_FR2]));
//   GetSTMotor(&(data_fdb->ST_motor_fdb));
//   Get_FR_State(&(data_fdb->FR_state));
//   Get_New_rc_FR_State(&(data_fdb->new_rc_FR_state));
//   Get_vision_Data(&sh_vision_data);
//   Get_Referee_Data(&sh_referee);
//   shoot.FRM_VError[0] = ABS((float)(shoot.FR_motor_fdb[0].speed_rpm_fdb) / (float)(shoot.FR_motor_fdb[1].speed_rpm_fdb)) - 1;
//   shoot.FRM_VError[1] = ABS(shoot.FR_motor_fdb[0].speed_rpm_fdb + shoot.FR_motor_fdb[1].speed_rpm_fdb);
// }
// static void ShootMode_Set(void)
// {
//   if (robo.mode == true && VISION_FOLLOW == 1)
//   {
//     if (vision.flag == 0)
//       shoot.mode = FIRE_STOP;
//     else if (vision.flag == 1 &&
//              version.fire_flag != 1)
//     {
//       shoot.mode = FIRE_READY;
//     }
//     else if (vision.flag == 1 &&
//              version.fire_flag == 1)
//     {
//       shoot.mode = FIRE;
//     }
//     else
//     {
//       shoot.mode = FIRE_STOP;
//     }
//   }
//   else
//   {
//     if (gimbal.Climb_Mode != 0)
//       return;
//     if (rc_data.rc.ch[4] > 500)
//       shoot.mode = FIRE;
//     else if (rc_data.rc.ch[4] < -500)
//       shoot.mode = FIRE_REVERSAL;
//   }
// }
// int16_t reversal_angle = 0;
// int16_t shoot_back_flag = 0;
// int16_t jam_stop_time = 0;
// int16_t ecd_circle = 0;
// static void Shoot_Speed_Set(Shoot_t *speed)
// {
//   static uint16_t circ_count = 0;
//   if (!is_fire)
//   {
//     ammo_cnt = 0;
//   }
//   if (is_fire && shoot_ready)
//   {
//     fr1.speed = -FR_SPEED;
//     fr2.speed = FR_SPEED;
//     if (shoot.mode == CONTINUOUS)
//     {
//       st.speed = ST_CONTINUOUS_SPEED;
//     }
//     else if (shoot.mode == SINGLE)
//     {
//       static int16_t MST_LastEcd = 0, MST_CurrentEcd = 0;
//       MST_LastEcd = MST_CurrentEcd;
//       MST_CurrentEcd = speed->ST_motor_fdb.ecd_fdb;
//       if (abs(MST_LastEcd - MST_CurrentEcd > 1000))
//         circ_count++;
//       if (circ_count < 8)
//         st_set = SHOOT_MOTOR_SINGLE_SPEED_SET;
//       else
//         st_set = 0;
//     }
//   }
//   else
//   {
//     st_set = 0;
//   }
//   if (shoot.mode == EJECT)
//   {
//     // 退单的角度读取
//     speed->shoot_motor_angle.shoot_ecd_fdb_last = speed->ST_motor_fdb.ecd_fdb; // 结构体获得拨盘的编码器值，退弹时候限角度
//     GetSTMotor(&(speed->ST_motor_fdb));                                        // 更新返回的编码器值
//     speed->shoot_motor_angle.shoot_ecd_fdb_error = speed->ST_motor_fdb.ecd_fdb - speed->shoot_motor_angle.shoot_ecd_fdb_last;
//     // 退单的角度读取
//     if (speed->shoot_motor_angle.shoot_ecd_fdb_error <= -3000)
//       ecd_circle++;
//     if (ecd_circle <= 10)
//     {
//       st_set = -3000;
//     }
//     else
//     {
//       st_set = 0;
//       ecd_circle = 0;
//     }
//     speed->shoot_motor_angle.shoot_ecd_fdb_error = 0.0;
//   }//   //******无力模式*****//
//   if (shoot.mode == FIRE_STOP)
//   {
//     fr_set[LEFT] = 0;
//     fr_set[RIGHT] = 0;
//     st_set = 0;
//   }
// }
// 同济源码
// void source_code(void)
// {
//   float a = (float)(ext_robot_status.shooter_barrel_cooling_value);                                               // 冷却速度
//   float m = (float)(ext_robot_status.shooter_barrel_heat_limit - ext_power_heat_data.shooter_17mm_1_barrel_heat); // 热量上限
//   float d = 10.0f;
//   if (shoot_time == 0)
//   {
//     /**
//      * 方案二：根据热量上限和冷却决定射击策略，
//      * 计算得当射击时间为 m（热量上限）+ 1 * a（冷却速率）时，基本可以抹除冷却优先和爆发优的差距，即两者各级对应射速相近
//      * 当k增大时，差距射击频率差距主要体现在低等级（爆发高，冷却低），等级越高影响越小。爆发模式下各等级射频更加均匀且持续时间更长
//      * 冷却模式正好相反，低等级射频低，高等级射频高且持续时间短，可灵活选择 m + k * a
//      */
//     ShootTime = (m + 2 * a) * 10;
//     clapmf(&ShootTime, ShootTimeLower, ShootTimeUpper);
//     // 分级射速
//     if (m < 100)
//     {
//       shoot_speed = (10 * m - a - 3 * d) / (d * (ShootTime / 100.0f)) + a / d;
//     }
//     else
//     {
//       shoot_speed = (10 * m - a - 5 * d) / (d * (ShootTime / 100.0f)) + a / d;
//     }
//   }
//   else if (0 < shoot_time && shoot_time < ShootTime)
//   {
//     trigger_motor2006_data[0].target_speed = shoot_speed * 2 * PI / 7;
//     fn_Fp32Limit(&trigger_motor2006_data[0].target_speed, 0.0f, 18.0f);
//   }
//   else
//   {
//     trigger_motor2006_data[0].target_speed = (a / d) * 2 * PI / 7;
//     fn_Fp32Limit(&trigger_motor2006_data[0].target_speed, 0.0f, 18.0f);
//   }
//   if (shoot_time < ShootTime)
//   {
//     shoot_time++;
//   }
// }
// static void Shoot_Control(void)
// {
//   // float fr_fdb_avge = fabsf((fr_fdb[RIGHT].speed - fr_fdb[LEFT].speed)) * 0.5f;
//   // fr_cmd[LEFT] += PID_calc(&FRM_OffsetPID[0], -fr_fdb_avge, fr_fdb[LEFT].speed);
//   // fr_cmd[RIGHT] += PID_calc(&FRM_OffsetPID[1], fr_fdb_avge, fr_fdb[RIGHT].speed);
//   st_cmd = PID_calc(&st_pid, st_set, st_fdb.speed);
//   fr_cmd[LEFT] = PID_calc(&fr_pid[LEFT], fr_set[LEFT], fr_fdb[LEFT].speed);
//   fr_cmd[RIGHT] = PID_calc(&fr_pid[RIGHT], fr_set[RIGHT], fr_fdb[RIGHT].speed);
// }
// flaot st_pid_param[]={0.05,0.01,0.01,};
// static void shoot_init(void)
// {
//   fr_set[LEFT] = 1;
//   fr_set[RIGHT] = 1;
//   st_set = 1;
//   PID_Init(&st_pid, PID_POSITION, st_pid_param);
//   PID_Init(&fr_pid[LEFT], PID_POSITION, fr_pid_param);
//   PID_Init(&fr_pid[RIGHT], PID_POSITION, fr_pid_param);
// }
