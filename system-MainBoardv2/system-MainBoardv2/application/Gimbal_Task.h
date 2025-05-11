#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"



#define ROLL_ANGLE_PID_KP 80.0f//80.0f
#define ROLL_ANGLE_PID_KI 0.5f//0.5f
#define ROLL_ANGLE_PID_KD 4500.0f//4500.0f
#define ROLL_ANGLE_PID_MAX_OUT 10000.0f
#define ROLL_ANGLE_PID_MAX_IOUT 1000.0f

#define PITCH_ANGLE_PID_KP 70.0f//70.0f
#define PITCH_ANGLE_PID_KI 0.5f//0.5f
#define PITCH_ANGLE_PID_KD 3000.0f//3000.0f
#define PITCH_ANGLE_PID_MAX_OUT 10000.0f
#define PITCH_ANGLE_PID_MAX_IOUT 1000.0f

enum BATSTATE    //charger state enum
{
	BATT_CHARGING=1,BATT_FULL,BATT_BOOSTING  
};

typedef struct
{
	fp32 gimbal_pitch_real;  
	fp32 gimbal_roll_real;     //real angle data
	
	fp32 gimbal_pitch_set;
	fp32 gimbal_roll_set;      //self ctrl mode angle set
	
	pid_type_def roll_pid;
	pid_type_def pitch_pid;    //self ctrl pid
	
	float board_temp;          //read from MCU
	float batt_votage;         //1S Li-on
	enum BATSTATE batt_state;  //charger state
	 
	uint32_t PwmL;             
	uint32_t PwmR;             //motor PWM buffer  Range:0-20000  Freq:500khz

	uint8_t enable_flag;       //enable:1     disable:0   
	uint8_t mode_flag;         //self_ctrl:1  outside_ctrl:0
	uint8_t uartupdate_flag;   //uart updata state  1:system parameter ctrl   2:ctrl data input(outside_ctrl only)

	uint8_t uarttran_flag;   //uart transmit state  [7]:attitude   [6]:basic operating state(enable mode temp vottage charger state)   [5]:pid para

	


} gimbal_ctrl_t;







void Gimbal_Task(void const * argument);

void GimbalInit(void);
void GimbalDataUpdate(void);
void KeyScan(void);
void SelfCtrl(void);
void MotorCtrl(void);
void DataSend(void);



float u8Arry2float(uint8_t *data);
void float2u8Arry(uint8_t *u8Arry, float *floatdata);
int32_t u8Arry2int32(uint8_t* data);




#endif
