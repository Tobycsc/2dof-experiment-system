#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "FreeRTOS.h"
#include "task.h"


#include "main.h"
#include "arm_math.h"
#include "usart.h"
#include "string.h"
#include "bsp_adc.h"




float BattVotg;
uint8_t sw1_flag=0;

extern fp32 INS_angle_deg[3];
extern uint16_t ADC_buf[2];

gimbal_ctrl_t gimbal_ctrl;


void Gimbal_Task(void const* argument)
{
    Gimbal_Init();
    vTaskDelay(200);

    while(1)
    {
				Gimbal_Data_Update();
				KeyScan();
				if(gimbal_ctrl.mode_flag)
				{
					SelfCtrl();
				}
				else
				{
				}
				MotorCtrl();
        vTaskDelay(20);
    }
}









void Gimbal_Init(void)
{
    const static fp32 roll_angle_pid[3] = {ROLL_ANGLE_PID_KP, ROLL_ANGLE_PID_KI, ROLL_ANGLE_PID_KD};
    const static fp32 pitch_angle_pid[3] = {PITCH_ANGLE_PID_KP, PITCH_ANGLE_PID_KI, PITCH_ANGLE_PID_KD};


		gimbal_ctrl.enable_flag=0;
		gimbal_ctrl.mode_flag=1;
		gimbal_ctrl.PwmL=0;
		gimbal_ctrl.PwmR=0;
		gimbal_ctrl.gimbal_pitch_set=0;
		gimbal_ctrl.gimbal_roll_set=0;
		gimbal_ctrl.gimbal_pitch_real=0;
		gimbal_ctrl.gimbal_roll_real=0;
		gimbal_ctrl.uartupdate_flag=0;


    PID_init(&gimbal_ctrl.pitch_pid, PID_DELTA, pitch_angle_pid , ROLL_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);
    PID_init(&gimbal_ctrl.roll_pid, PID_POSITION, roll_angle_pid, PITCH_ANGLE_PID_MAX_OUT, ROLL_ANGLE_PID_MAX_IOUT);
		
}

void Gimbal_Data_Update(void)
{
		gimbal_ctrl.gimbal_pitch_real=90.0f+INS_angle_deg[1];
		gimbal_ctrl.gimbal_roll_real=INS_angle_deg[2];   //<- + 
	
		gimbal_ctrl.board_temp=((((float)(ADC_buf[0]) / 4096 * 3.3f)-0.76f)/0.0025f) + 25;
		gimbal_ctrl.batt_votage=(float)(ADC_buf[1]) / 4096 * 3.3f * 1.51f;
}


void KeyScan(void)
{
	
	if(sw1_flag)
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_SET)  
		{
			sw1_flag=0;
		}
	}
	else
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)==GPIO_PIN_RESET)  
		{
			if(gimbal_ctrl.enable_flag)
			{
				gimbal_ctrl.enable_flag=0;
			}
			else
			{
				gimbal_ctrl.enable_flag=1;
			}
			sw1_flag=1;
		}
	}
}

void SelfCtrl(void)
{
		if(gimbal_ctrl.enable_flag)
		{
			PID_calc(&gimbal_ctrl.pitch_pid, gimbal_ctrl.gimbal_pitch_real, gimbal_ctrl.gimbal_pitch_set);
			PID_calc(&gimbal_ctrl.roll_pid, gimbal_ctrl.gimbal_roll_real, gimbal_ctrl.gimbal_roll_set);
			gimbal_ctrl.PwmL=-gimbal_ctrl.roll_pid.out+gimbal_ctrl.pitch_pid.out;
			gimbal_ctrl.PwmR=gimbal_ctrl.roll_pid.out+gimbal_ctrl.pitch_pid.out;
		}
		else
		{
			gimbal_ctrl.PwmL=0;
			gimbal_ctrl.PwmR=0;
		}
    


}


void MotorCtrl(void)
{
	if(gimbal_ctrl.PwmL<0)
	{
		gimbal_ctrl.PwmL=0;
	}
	if(gimbal_ctrl.PwmR<0)
	{
		gimbal_ctrl.PwmR=0;
	}
	TIM1->CCR2=gimbal_ctrl.PwmL;
	TIM1->CCR3=gimbal_ctrl.PwmR;
}
