#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"



#define ROLL_ANGLE_PID_KP 70.0f//0.2f
#define ROLL_ANGLE_PID_KI 3.0f
#define ROLL_ANGLE_PID_KD 2000.0f//3.0f
#define ROLL_ANGLE_PID_MAX_OUT 10000.0f
#define ROLL_ANGLE_PID_MAX_IOUT 1000.0f

#define PITCH_ANGLE_PID_KP 150.0f//0.2f
#define PITCH_ANGLE_PID_KI 10.0f
#define PITCH_ANGLE_PID_KD 2500.0f//3.0f
#define PITCH_ANGLE_PID_MAX_OUT 10000.0f
#define PITCH_ANGLE_PID_MAX_IOUT 500.0f


typedef struct
{
	fp32 gimbal_pitch_real;
	fp32 gimbal_roll_real;
	
	fp32 gimbal_pitch_set;
	fp32 gimbal_roll_set;
	
	uint32_t PwmL;
	uint32_t PwmR;

	uint8_t enable_flag;  //enable:1     disable:0
	uint8_t mode_flag;    //self_ctrl:1  outside_ctrl:0
	uint8_t uartupdate_flag;

	
	pid_type_def roll_pid;
	pid_type_def pitch_pid;
	
	float board_temp;
	float batt_votage;

} gimbal_ctrl_t;


void Gimbal_Task(void const * argument);

void Gimbal_Init(void);
void Gimbal_Data_Update(void);
void KeyScan(void);
void SelfCtrl(void);
void MotorCtrl(void);

#endif
