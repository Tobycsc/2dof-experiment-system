#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "FreeRTOS.h"
#include "task.h"


#include "main.h"
#include "arm_math.h"
#include "usart.h"
#include "string.h"
//#include "referee.h"
#include "bsp_adc.h"

int ywjj = 0;


uint8_t yaw_mode = 0, yaw_mode_last = 0; //0:speed,1:angle
uint8_t pitch_mode = 0, pitch_mode_last = 0; //0:speed,1:angle

float yaw_pid_rate = 0.2f; //0.3f
float pitch_pid_rate = 0.25f;

float yaw_angle_err = 0, pitch_angle_err = 0;

float auto_aim_err_yaw = 0, auto_aim_err_pitch = 0;
int auto_aim_vx = 0, auto_aim_vz = 0;



float BattVotg;


uint32_t PwmL=1000,PwmR=1000;


void Gimbal_Task(void const* argument)
{

    vTaskDelay(200);

    while(1)
    {


				TIM1->CCR2=PwmL;
				TIM1->CCR3=PwmR;

        vTaskDelay(20);
    }
}









//void Gimbal_Motor_Init(void)
//{
//    const static fp32 yaw_motor_speed_pid[3] = {YAW_MOTOR_SPEED_PID_KP, YAW_MOTOR_SPEED_PID_KI, YAW_MOTOR_SPEED_PID_KD};
//    const static fp32 yaw_motor_angle_pid[3] = {YAW_MOTOR_ANGLE_PID_KP, YAW_MOTOR_ANGLE_PID_KI, YAW_MOTOR_ANGLE_PID_KD};
//    const static fp32 yaw_motor_auto_aim_pid[3] = {YAW_MOTOR_AUTO_AIM_PID_KP, YAW_MOTOR_AUTO_AIM_PID_KI, YAW_MOTOR_AUTO_AIM_PID_KD};

//    const static fp32 pitch_motor_speed_pid[3] = {PITCH_MOTOR_SPEED_PID_KP, PITCH_MOTOR_SPEED_PID_KI, PITCH_MOTOR_SPEED_PID_KD};
//    const static fp32 pitch_motor_angle_pid[3] = {PITCH_MOTOR_ANGLE_PID_KP, PITCH_MOTOR_ANGLE_PID_KI, PITCH_MOTOR_ANGLE_PID_KD};
//    const static fp32 pitch_motor_auto_aim_pid[3] = {PITCH_MOTOR_AUTO_AIM_PID_KP, PITCH_MOTOR_AUTO_AIM_PID_KI, PITCH_MOTOR_AUTO_AIM_PID_KD};

//    for(uint8_t i = 0; i < 2; i++)
//    {
//        gimbal_m6020[i].INS_speed = 0;
//        gimbal_m6020[i].INS_speed_set = 0;
//        gimbal_m6020[i].INS_angle = 0;
//        gimbal_m6020[i].INS_angle_set = 0;
//        gimbal_m6020[i].ENC_angle = 0;
//        gimbal_m6020[i].ENC_angle_actual = 0;
//        gimbal_m6020[i].ENC_angle_set = 0;
//        gimbal_m6020[i].give_current = 0;
//    }

////	chassis_move_init->chassis_INS_angle = get_INS_angle_point();
////	chassis_move_init->chassis_INS_gyro = get_gyro_data_point();
////	chassis_move_init->chassis_quat = get_INS_quat_point();
////	chassis_move_init->chassis_accel = get_accel_data_point();


//    PID_init(&gimbal_m6020[0].speed_pid, PID_POSITION, yaw_motor_speed_pid, YAW_MOTOR_SPEED_PID_MAX_OUT, YAW_MOTOR_SPEED_PID_MAX_IOUT);
//    PID_init(&gimbal_m6020[0].angle_pid, PID_POSITION, yaw_motor_angle_pid, YAW_MOTOR_ANGLE_PID_MAX_OUT, YAW_MOTOR_ANGLE_PID_MAX_IOUT);
//    PID_init(&gimbal_m6020[0].auto_aim_pid, PID_POSITION, yaw_motor_auto_aim_pid, YAW_MOTOR_AUTO_AIM_PID_MAX_OUT, YAW_MOTOR_AUTO_AIM_PID_MAX_IOUT);

//    PID_init(&gimbal_m6020[1].speed_pid, PID_POSITION, pitch_motor_speed_pid, PITCH_MOTOR_SPEED_PID_MAX_OUT, PITCH_MOTOR_SPEED_PID_MAX_IOUT);
//    PID_init(&gimbal_m6020[1].angle_pid, PID_POSITION, pitch_motor_angle_pid, PITCH_MOTOR_ANGLE_PID_MAX_OUT, PITCH_MOTOR_ANGLE_PID_MAX_IOUT);
//    PID_init(&gimbal_m6020[1].auto_aim_pid, PID_POSITION, pitch_motor_auto_aim_pid, PITCH_MOTOR_AUTO_AIM_PID_MAX_OUT, PITCH_MOTOR_AUTO_AIM_PID_MAX_IOUT);
//}

//void Gimbal_Motor_Data_Update(void)
//{
//    //yaw
//    //z2+y2
//    fp32 temp;

//    arm_sqrt_f32(bmi088_real_data.gyro[2]*bmi088_real_data.gyro[2] + bmi088_real_data.gyro[0]*bmi088_real_data.gyro[0], &temp);
//    if(bmi088_real_data.gyro[2] < 0)
//    {
//        temp = -temp;
//    }
//    //gimbal_m6020[0].INS_speed=temp*0.1f+gimbal_m6020[0].INS_speed*0.9f;
//    gimbal_m6020[0].INS_speed = bmi088_real_data.gyro[2];
//    gimbal_m6020[0].INS_angle = INS_angle_deg[0];
//    gimbal_m6020[0].ENC_angle = motor_measure_gimbal[0].ecd;

//    //pitch
//    //gimbal_m6020[1].INS_speed=bmi088_real_data.gyro[2]*0.1f+gimbal_m6020[1].INS_speed*0.9f;
//    gimbal_m6020[1].INS_speed = bmi088_real_data.gyro[1];
//    gimbal_m6020[1].INS_angle = INS_angle_deg[2];
//    gimbal_m6020[1].ENC_angle = motor_measure_gimbal[1].ecd;
//}

//void Yaw_Motor_Control(void)
//{
//    if(rc_ctrl.mouse.press_r && AutoAim_Data_Receive.Aimed_ID)
////	if((rc_ctrl.rc.s[0]==RC_SW_MID||rc_ctrl.rc.s[0]==RC_SW_UP)&&AutoAim_Data_Receive.Aimed_ID)
//    {
//        yaw_angle_err = AutoAim_Data_Receive.Yaw - gimbal_m6020[0].INS_angle;
//        if(yaw_angle_err > 180) yaw_angle_err -= 360;
//        else if(yaw_angle_err < -180) yaw_angle_err += 360;

//        gimbal_m6020[0].auto_aim_pid.Kp = yaw_pid_rate / (log((fabs(yaw_angle_err)) + 1.1f));
//        if(gimbal_m6020[0].auto_aim_pid.Kp > 20.0f) gimbal_m6020[0].auto_aim_pid.Kp = 20.0f;
//        if(gimbal_m6020[0].auto_aim_pid.Kp < 0.2f) gimbal_m6020[0].auto_aim_pid.Kp = 0.2f;

//        PID_calc(&gimbal_m6020[0].auto_aim_pid, yaw_angle_err, 0);
//        gimbal_m6020[0].INS_speed_set = -gimbal_m6020[0].auto_aim_pid.out - AutoAim_Data_Receive.Yaw_Omega * ANGLE_TO_RAD;
//        gimbal_m6020[0].INS_angle_set = AutoAim_Data_Receive.Yaw;
//        yaw_mode = yaw_mode_last = 1;
////		gimbal_m6020[0].INS_angle_set=gimbal_m6020[0].INS_angle;
//    }

//    else
//    {
//        yaw_mode_last = yaw_mode;
//        if((rc_ctrl.rc.ch[0] > 10 || rc_ctrl.rc.ch[0] < -10) || (rc_ctrl.mouse.x > 10 || rc_ctrl.mouse.x < -10))
//        {
//            yaw_mode = 0;
//        }
//        else
//        {
//            yaw_mode = 1;
//        }

//        if(chassis_follow_gimbal_changing == 1)
//        {
//            yaw_mode = 1;
//            yaw_mode_last = yaw_mode;
//        }

//        if(yaw_mode == 0)
//        {
//            if(rc_ctrl.mouse.x > 10 || rc_ctrl.mouse.x < -10)
//                gimbal_m6020[0].INS_speed_set = -(float)rc_ctrl.mouse.x * YAW_MOUSE_SEN * 0.03f + gimbal_m6020[0].INS_speed_set * 0.97f;
//            else
//                gimbal_m6020[0].INS_speed_set = -(float)rc_ctrl.rc.ch[0] / 660.0f * 3 * 0.1f + gimbal_m6020[0].INS_speed_set * 0.9f;
//        }
//        else if(yaw_mode == 1 && yaw_mode_last == 0)
//        {
//            gimbal_m6020[0].INS_angle_set = gimbal_m6020[0].INS_angle;
//        }

//        if(yaw_mode == 1)
//        {
//            yaw_angle_err = gimbal_m6020[0].INS_angle_set - gimbal_m6020[0].INS_angle;
//            if(yaw_angle_err > 180) yaw_angle_err -= 360;
//            else if(yaw_angle_err < -180) yaw_angle_err += 360;

//            PID_calc(&gimbal_m6020[0].angle_pid, yaw_angle_err, 0);
//            gimbal_m6020[0].INS_speed_set = -gimbal_m6020[0].angle_pid.out;
//        }
//    }
//    if(rc_ctrl.rc.s[1] == RC_SW_DOWN) //无力模式
//    {
//        PID_clear(&gimbal_m6020[0].angle_pid);
//        gimbal_m6020[0].INS_angle_set = gimbal_m6020[0].INS_angle;

//        float angle_err_1 = (CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO - gimbal_m6020[0].ENC_angle) / 22.755556f;
//        if(angle_err_1 > 180) angle_err_1 -= 360;
//        else if(angle_err_1 < -180) angle_err_1 += 360;
//        float angle_err_2 = (CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO + 4096 - gimbal_m6020[0].ENC_angle) / 22.755556f;
//        if(angle_err_2 > 180) angle_err_2 -= 360;
//        else if(angle_err_2 < -180) angle_err_2 += 360;

//        if(fabs(angle_err_2) < fabs(angle_err_1))angle_err_1 = angle_err_2;

//        PID_calc(&gimbal_m6020[0].angle_pid, angle_err_1, 0);
//        gimbal_m6020[0].INS_speed_set = -0.3f * gimbal_m6020[0].angle_pid.out;

//        PID_calc(&gimbal_m6020[0].speed_pid, 0.3f * motor_measure_gimbal[0].speed_rpm * 2 * 3.14f / 60.0f, 0.3f * gimbal_m6020[0].INS_speed_set);
//        gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out;
//        return;
//    }
//    PID_calc(&gimbal_m6020[0].speed_pid, gimbal_m6020[0].INS_speed, gimbal_m6020[0].INS_speed_set);

//    gimbal_m6020[0].give_current = gimbal_m6020[0].speed_pid.out;
//}


//void Pitch_Motor_Control(void)
//{
//    gimbal_m6020[1].ENC_angle_actual = (float)(((uint16_t)gimbal_m6020[1].ENC_angle + (8192 - 5077)) % 8192) / 8192.0f * 360.0f;
//    if(gimbal_m6020[1].ENC_angle_actual > 180)
//    {
//        gimbal_m6020[1].ENC_angle_actual -= 360;
//    }

//    if(rc_ctrl.mouse.press_r && AutoAim_Data_Receive.Aimed_ID)
////	if((rc_ctrl.rc.s[0]==RC_SW_MID||rc_ctrl.rc.s[0]==RC_SW_UP)&&AutoAim_Data_Receive.Aimed_ID)
//    {
//        pitch_angle_err = AutoAim_Data_Receive.Pitch - gimbal_m6020[1].INS_angle;
//        gimbal_m6020[1].auto_aim_pid.Kp = pitch_pid_rate / (log((fabs(pitch_angle_err)) + 1.1f));
//        if(gimbal_m6020[1].auto_aim_pid.Kp > 20.0f) gimbal_m6020[1].auto_aim_pid.Kp = 20.0f;
//        if(gimbal_m6020[1].auto_aim_pid.Kp < 0.2f) gimbal_m6020[1].auto_aim_pid.Kp = 0.2f;

//        PID_calc(&gimbal_m6020[1].auto_aim_pid, gimbal_m6020[1].INS_angle, AutoAim_Data_Receive.Pitch);
//        gimbal_m6020[1].INS_speed_set = gimbal_m6020[1].auto_aim_pid.out;
//        gimbal_m6020[1].INS_angle_set = AutoAim_Data_Receive.Pitch;
//        yaw_mode = yaw_mode_last = 1;
//    }
//    else
//    {
//        pitch_mode_last = pitch_mode;
//        if((rc_ctrl.rc.ch[1] > 5 || rc_ctrl.rc.ch[1] < -5) || (rc_ctrl.mouse.y > 10 || rc_ctrl.mouse.y < -10))
//        {
//            pitch_mode = 0;
//        }
//        else
//        {
//            pitch_mode = 1;
//        }

////		pitch_mode=0;

//        if(pitch_mode == 0)
//        {
//            if(rc_ctrl.mouse.y != 0)
//                gimbal_m6020[1].INS_speed_set = -(float)rc_ctrl.mouse.y * PITCH_MOUSE_SEN * 0.25f + gimbal_m6020[1].INS_speed_set * 0.75f;
//            else
//                gimbal_m6020[1].INS_speed_set = -(float)rc_ctrl.rc.ch[1] / 660.0f * 2;
//        }
//        else if(pitch_mode == 1 && pitch_mode_last == 0)
//        {
//            gimbal_m6020[1].INS_angle_set = gimbal_m6020[1].INS_angle;

//            //		gimbal_m6020[1].ENC_angle_set=gimbal_m6020[1].ENC_angle_actual;
//        }

//        if(pitch_mode == 1)
//        {
////			gimbal_m6020[1].INS_angle_set=60*arm_sin_f32(5*tt);
////			if(gimbal_m6020[1].INS_angle_set>16)gimbal_m6020[1].INS_angle_set=16;
////			if(gimbal_m6020[1].INS_angle_set<-20)gimbal_m6020[1].INS_angle_set=-20;

////			if(gimbal_m6020[1].INS_angle_set>0)gimbal_m6020[1].INS_angle_set=20;
////			if(gimbal_m6020[1].INS_angle_set<-0)gimbal_m6020[1].INS_angle_set=-20;

//            PID_calc(&gimbal_m6020[1].angle_pid, gimbal_m6020[1].INS_angle, gimbal_m6020[1].INS_angle_set);
//            gimbal_m6020[1].INS_speed_set = gimbal_m6020[1].angle_pid.out;

////			if(gimbal_m6020[1].ENC_angle>5600&&gimbal_m6020[1].INS_speed_set>0)gimbal_m6020[1].INS_speed_set=0;
//            //		PID_calc(&gimbal_m6020[1].angle_pid,gimbal_m6020[1].ENC_angle_actual,gimbal_m6020[1].ENC_angle_set);
//            //		gimbal_m6020[1].INS_speed_set=gimbal_m6020[1].angle_pid.out;
//        }
//    }

//    PID_calc(&gimbal_m6020[1].speed_pid, gimbal_m6020[1].INS_speed, gimbal_m6020[1].INS_speed_set);
//    gimbal_m6020[1].give_current = gimbal_m6020[1].speed_pid.out;
////	if(gimbal_m6020[1].ENC_angle>5700&&gimbal_m6020[1].speed_pid.out>0)gimbal_m6020[1].give_current=0.2*gimbal_m6020[1].speed_pid.out;
////	if(gimbal_m6020[1].ENC_angle<4650&&gimbal_m6020[1].speed_pid.out<0)gimbal_m6020[1].give_current=0.2*gimbal_m6020[1].speed_pid.out;
//}


