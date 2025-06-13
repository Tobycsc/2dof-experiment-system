#include "Gimbal_Task.h"
#include "INS_Task.h"
#include "bsp_flash.h"
#include "FreeRTOS.h"
#include "task.h"


#include "main.h"
#include "arm_math.h"
#include "usart.h"
#include "string.h"
#include "bsp_adc.h"

#include "usbd_cdc_if.h"



uint8_t ywj=0;

uint8_t sw1_flag=0;           
uint8_t sw2_flag=0;           
uint8_t sw3_flag=0;           //switch state buffer

uint16_t LED_cnt=0;           //for frequece calc

extern fp32 INS_angle_deg[3]; //IMU angle data
extern uint16_t ADC_buf[2];   //batt data buffer

gimbal_ctrl_t gimbal_ctrl;    //main data struct 


void Gimbal_Task(void const* argument)
{
    GimbalInit();
    vTaskDelay(200);

    while(1)
    {
				GimbalDataUpdate();
				
				if(gimbal_ctrl.mode_flag)
				{
					SelfCtrl();
					LED_cnt++;
					if(LED_cnt>=200)
					{
						LED_cnt=0;
						HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);
					}
				}
				else
				{
					
				}
				MotorCtrl();
				
				
				DataSend();
				
				if(ywj)
				{
					ywj=0;
					WriteAllPara();
				}
				
        vTaskDelay(5);
    }
}









void GimbalInit(void)
{

    ReadAllPara();

		gimbal_ctrl.enable_flag=0;
		gimbal_ctrl.mode_flag=1;
		gimbal_ctrl.PwmL=0;
		gimbal_ctrl.PwmR=0;
		gimbal_ctrl.gimbal_pitch_set=0;
		gimbal_ctrl.gimbal_roll_set=0;
		gimbal_ctrl.gimbal_pitch_real=0;
		gimbal_ctrl.gimbal_roll_real=0;
		gimbal_ctrl.uartupdate_flag=0;

	
		fp32 roll_angle_pid_para_t[3] = {ROLL_ANGLE_PID_KP,ROLL_ANGLE_PID_KI,ROLL_ANGLE_PID_KD};
		fp32 pitch_angle_pid_para_t[3] = {PITCH_ANGLE_PID_KP,PITCH_ANGLE_PID_KI,PITCH_ANGLE_PID_KD};
		
		roll_angle_pid_para_t[0]=gimbal_ctrl.roll_angle_pid_para.para_kp;
		roll_angle_pid_para_t[1]=gimbal_ctrl.roll_angle_pid_para.para_ki;
		roll_angle_pid_para_t[2]=gimbal_ctrl.roll_angle_pid_para.para_kd;
		

    PID_init(&gimbal_ctrl.pitch_angle_pid, PID_DELTA, pitch_angle_pid_para_t , ROLL_ANGLE_PID_MAX_OUT, PITCH_ANGLE_PID_MAX_IOUT);
    PID_init(&gimbal_ctrl.roll_angle_pid, PID_POSITION, roll_angle_pid_para_t, PITCH_ANGLE_PID_MAX_OUT, ROLL_ANGLE_PID_MAX_IOUT);
		
}

void GimbalDataUpdate(void)
{
	
		gimbal_ctrl.gimbal_pitch_real=90.0f+INS_angle_deg[1];
		gimbal_ctrl.gimbal_roll_real=INS_angle_deg[2];   //<- + 
	
		gimbal_ctrl.board_temp=((((float)(ADC_buf[0]) / 4096 * 3.3f)-0.76f)/0.0025f) + 25;
		gimbal_ctrl.batt_votage=(float)(ADC_buf[1]) / 4096 * 3.3f * 1.51f;
	
	  if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)&&HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))
		{
			gimbal_ctrl.batt_state=BATT_BOOSTING;
		}
		else if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))
		{
			gimbal_ctrl.batt_state=BATT_FULL;
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);
		}
		else
		{
			gimbal_ctrl.batt_state=BATT_CHARGING;
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);
		}
		
		KeyScan();
		
		
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
			PID_calc(&gimbal_ctrl.pitch_angle_pid, gimbal_ctrl.gimbal_pitch_real, gimbal_ctrl.gimbal_pitch_set);
			PID_calc(&gimbal_ctrl.roll_angle_pid, gimbal_ctrl.gimbal_roll_real, gimbal_ctrl.gimbal_roll_set);
			gimbal_ctrl.PwmL=-gimbal_ctrl.roll_angle_pid.out+gimbal_ctrl.pitch_angle_pid.out;
			gimbal_ctrl.PwmR=gimbal_ctrl.roll_angle_pid.out+gimbal_ctrl.pitch_angle_pid.out;
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

void DataSend(void)
{
	
	
	
	
	uint8_t justFloatTail[50];
	
	float2u8Arry(&justFloatTail[0],&gimbal_ctrl.gimbal_pitch_real);
	float2u8Arry(&justFloatTail[4],&gimbal_ctrl.gimbal_roll_real);
		
	justFloatTail[8]=0x00;
	justFloatTail[9]=0x00;
	justFloatTail[10]=0x80;
	justFloatTail[11]=0x7f;
	
	CDC_Transmit_FS(justFloatTail,12);

	
	
	
	

}























float u8Arry2float(uint8_t *data)
{
    uint8_t uc[4];
    float fa = 0;

    uc[0] = data[0];
    uc[1] = data[1];
    uc[2] = data[2];
    uc[3] = data[3];

    memcpy(&fa, uc, 4);
    return fa;
}

void float2u8Arry(uint8_t *u8Arry, float *floatdata)
{
    uint8_t farray[4];
    *(float *)farray = *floatdata;
    u8Arry[3] = farray[3];
    u8Arry[2] = farray[2];
    u8Arry[1] = farray[1];
    u8Arry[0] = farray[0];
}

int32_t u8Arry2int32(uint8_t* data)
{
    int32_t fa = 0;
    uint8_t uc[4];

    uc[3] = data[0];
    uc[2] = data[1];
    uc[1] = data[2];
    uc[0] = data[3];

    memcpy(&fa, uc, 4);
    return fa;
}