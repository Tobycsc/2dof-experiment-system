#ifndef _GIMBAL_TASK
#define _GIMBAL_TASK

#include "struct_typedef.h"
#include "pid.h"



#define ROLL_ANGLE_PID_KP 80.0f//80.0f
#define ROLL_ANGLE_PID_KI 0.5f//0.5f
#define ROLL_ANGLE_PID_KD 4500.0f//4500.0f
#define ROLL_ANGLE_PID_MAX_OUT 19000.0f
#define ROLL_ANGLE_PID_MAX_IOUT 1000.0f
#define ROLL_ANGLE_PID_TYPE PID_POSITION

#define ROLL_SPEED_PID_KP 80.0f//80.0f
#define ROLL_SPEED_PID_KI 0.5f//0.5f
#define ROLL_SPEED_PID_KD 4500.0f//4500.0f
#define ROLL_SPEED_PID_MAX_OUT 19000.0f
#define ROLL_SPEED_PID_MAX_IOUT 1000.0f
#define ROLL_SPEED_PID_TYPE PID_POSITION

#define PITCH_ANGLE_PID_KP 70.0f//70.0f
#define PITCH_ANGLE_PID_KI 0.5f//0.5f
#define PITCH_ANGLE_PID_KD 3000.0f//3000.0f
#define PITCH_ANGLE_PID_MAX_OUT 19000.0f
#define PITCH_ANGLE_PID_MAX_IOUT 1000.0f
#define PITCH_ANGLE_PID_TYPE PID_DELTA

#define PITCH_SPEED_PID_KP 70.0f//70.0f
#define PITCH_SPEED_PID_KI 0.5f//0.5f
#define PITCH_SPEED_PID_KD 3000.0f//3000.0f
#define PITCH_SPEED_PID_MAX_OUT 19000.0f
#define PITCH_SPEED_PID_MAX_IOUT 1000.0f
#define PITCH_SPEED_PID_TYPE PID_POSITION

#define MODE_DEFAULT 1
#define UARTTRAN_DEFAULT 0b00000001

#define INIT_FLAG 0 //when this flag is 1 , the para above will be progarm into the flash at downloading



enum BATSTATE    //charger state enum
{
	BATT_CHARGING=1,BATT_FULL,BATT_BOOSTING  
};

typedef struct
{
	float para_kp;
	float para_ki;
	float para_kd;
	float para_max_out;
	float para_max_iout;
	uint8_t pid_type;

} pid_paras_t;

typedef struct
{
	uint8_t enable_flag;       //enable:1     disable:0 
	uint16_t uartupdate_flag;   //uart updata state  1:ctrl data input(outside_ctrl only)

	uint16_t PwmL;             
	uint16_t PwmR;             //motor PWM buffer  Range:0-20000  Freq:500khz

	float board_temp;          //read from MCU
	float batt_votage;         //1S Li-on
	enum BATSTATE batt_state;  //charger state
	
	fp32 gimbal_pitch_real;  
	fp32 gimbal_roll_real;     //real angle data
	fp32 gimbal_pitchspeed_real;  
	fp32 gimbal_rollspeed_real;     //real speed data

	fp32 gimbal_pitch_set;
	fp32 gimbal_roll_set;      //self ctrl mode angle set
	fp32 gimbal_pitchspeed_set;
	fp32 gimbal_rollspeed_set;      //self ctrl mode speed set	
	
	
	pid_type_def roll_angle_pid;
	pid_type_def pitch_angle_pid;    
	pid_type_def roll_speed_pid;
	pid_type_def pitch_speed_pid;    //self ctrl pid
	
	
	
	pid_paras_t roll_angle_pid_para;
	pid_paras_t pitch_angle_pid_para;
	pid_paras_t roll_speed_pid_para;
	pid_paras_t pitch_speed_pid_para; //self ctrl pid para
	
	uint8_t mode_flag;         //self_ctrl:1  outside_ctrl:0

	uint8_t uarttran_flag;   //uart transmit state  [0]:attitude   [1]:basic operating state(enable mode temp vottage charger state)   [1]:pid para1   [3]:pid para2   [4]:pid para3   [5]:pid para4
	


} gimbal_ctrl_t;







void Gimbal_Task(void const * argument);

void GimbalInit(void);
void GimbalDataUpdate(void);
void KeyScan(void);
void SelfCtrl(void);
void OutsideCtrl(void);
void MotorCtrl(void);
void DataSend(void);



float u8Arry2float(uint8_t *data);
void float2u8Arry(uint8_t *u8Arry, float *floatdata);
int32_t u8Arry2int32(uint8_t* data);




#endif
