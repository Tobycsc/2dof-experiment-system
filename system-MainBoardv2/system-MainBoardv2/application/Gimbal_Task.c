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
uint16_t uart_cnt=0;           //for tran ferq ctrl

extern fp32 INS_angle_deg[3]; //IMU angle data
extern bmi088_real_data_t bmi088_real_data; //IMU data
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
				}
				else
				{
					OutsideCtrl();
				}
				MotorCtrl();				
				
				DataSend();
				
				LED_cnt++;
				if(LED_cnt>=200)
				{
					LED_cnt=0;
					HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);
				}
				
        vTaskDelay(5);
    }
}









void GimbalInit(void)
{
	
		fp32 roll_angle_pid_para_t[3];
		fp32 roll_speed_pid_para_t[3];
		fp32 pitch_angle_pid_para_t[3];	
		fp32 pitch_speed_pid_para_t[3];
		
		if(INIT_FLAG)
		{
			gimbal_ctrl.roll_angle_pid_para.para_kp=ROLL_ANGLE_PID_KP;
			gimbal_ctrl.roll_angle_pid_para.para_ki=ROLL_ANGLE_PID_KI;
			gimbal_ctrl.roll_angle_pid_para.para_kd=ROLL_ANGLE_PID_KD;
			gimbal_ctrl.roll_angle_pid_para.para_max_out=ROLL_ANGLE_PID_MAX_OUT;
			gimbal_ctrl.roll_angle_pid_para.para_max_iout=ROLL_ANGLE_PID_MAX_IOUT;
			gimbal_ctrl.roll_angle_pid_para.pid_type=ROLL_ANGLE_PID_TYPE;
			
			gimbal_ctrl.roll_speed_pid_para.para_kp=ROLL_SPEED_PID_KP;
			gimbal_ctrl.roll_speed_pid_para.para_ki=ROLL_SPEED_PID_KI;
			gimbal_ctrl.roll_speed_pid_para.para_kd=ROLL_SPEED_PID_KD;
			gimbal_ctrl.roll_speed_pid_para.para_max_out=ROLL_SPEED_PID_MAX_OUT;
			gimbal_ctrl.roll_speed_pid_para.para_max_iout=ROLL_SPEED_PID_MAX_IOUT;
			gimbal_ctrl.roll_speed_pid_para.pid_type=ROLL_SPEED_PID_TYPE;			
			
			gimbal_ctrl.pitch_angle_pid_para.para_kp=PITCH_ANGLE_PID_KP;
			gimbal_ctrl.pitch_angle_pid_para.para_ki=PITCH_ANGLE_PID_KI;
			gimbal_ctrl.pitch_angle_pid_para.para_kd=PITCH_ANGLE_PID_KD;
			gimbal_ctrl.pitch_angle_pid_para.para_max_out=PITCH_ANGLE_PID_MAX_OUT;
			gimbal_ctrl.pitch_angle_pid_para.para_max_iout=PITCH_ANGLE_PID_MAX_IOUT;
			gimbal_ctrl.pitch_angle_pid_para.pid_type=PITCH_ANGLE_PID_TYPE;			
			
			gimbal_ctrl.pitch_speed_pid_para.para_kp=PITCH_SPEED_PID_KP;
			gimbal_ctrl.pitch_speed_pid_para.para_ki=PITCH_SPEED_PID_KI;
			gimbal_ctrl.pitch_speed_pid_para.para_kd=PITCH_SPEED_PID_KD;
			gimbal_ctrl.pitch_speed_pid_para.para_max_out=PITCH_SPEED_PID_MAX_OUT;
			gimbal_ctrl.pitch_speed_pid_para.para_max_iout=PITCH_SPEED_PID_MAX_IOUT;
			gimbal_ctrl.pitch_speed_pid_para.pid_type=PITCH_SPEED_PID_TYPE;			
		
		
			gimbal_ctrl.mode_flag=MODE_DEFAULT;
			gimbal_ctrl.uarttran_flag=UARTTRAN_DEFAULT;
			
			WriteAllPara();
		}

		else
		{
			ReadAllPara();
		}

		gimbal_ctrl.enable_flag=0;
		gimbal_ctrl.PwmL=0;
		gimbal_ctrl.PwmR=0;
		gimbal_ctrl.gimbal_pitch_set=0;
		gimbal_ctrl.gimbal_roll_set=0;
		gimbal_ctrl.gimbal_pitch_real=0;
		gimbal_ctrl.gimbal_roll_real=0;
		gimbal_ctrl.uartupdate_flag=50000;

	
		roll_angle_pid_para_t[0]=gimbal_ctrl.roll_angle_pid_para.para_kp;
		roll_angle_pid_para_t[1]=gimbal_ctrl.roll_angle_pid_para.para_ki;
		roll_angle_pid_para_t[2]=gimbal_ctrl.roll_angle_pid_para.para_kd;
		
		roll_speed_pid_para_t[0]=gimbal_ctrl.roll_speed_pid_para.para_kp;
		roll_speed_pid_para_t[1]=gimbal_ctrl.roll_speed_pid_para.para_ki;
		roll_speed_pid_para_t[2]=gimbal_ctrl.roll_speed_pid_para.para_kd;
		
		pitch_angle_pid_para_t[0]=gimbal_ctrl.pitch_angle_pid_para.para_kp;
		pitch_angle_pid_para_t[1]=gimbal_ctrl.pitch_angle_pid_para.para_ki;
		pitch_angle_pid_para_t[2]=gimbal_ctrl.pitch_angle_pid_para.para_kd;
		
		pitch_speed_pid_para_t[0]=gimbal_ctrl.pitch_speed_pid_para.para_kp;
		pitch_speed_pid_para_t[1]=gimbal_ctrl.pitch_speed_pid_para.para_ki;
		pitch_speed_pid_para_t[2]=gimbal_ctrl.pitch_speed_pid_para.para_kd;


		PID_init(&gimbal_ctrl.roll_angle_pid, gimbal_ctrl.roll_angle_pid_para.pid_type, roll_angle_pid_para_t, gimbal_ctrl.roll_angle_pid_para.para_max_out, gimbal_ctrl.roll_angle_pid_para.para_max_iout);
		PID_init(&gimbal_ctrl.roll_speed_pid, gimbal_ctrl.roll_speed_pid_para.pid_type, roll_speed_pid_para_t, gimbal_ctrl.roll_speed_pid_para.para_max_out, gimbal_ctrl.roll_speed_pid_para.para_max_iout);
		PID_init(&gimbal_ctrl.pitch_angle_pid, gimbal_ctrl.pitch_angle_pid_para.pid_type, pitch_angle_pid_para_t, gimbal_ctrl.pitch_angle_pid_para.para_max_out, gimbal_ctrl.pitch_angle_pid_para.para_max_iout);
    PID_init(&gimbal_ctrl.pitch_speed_pid, gimbal_ctrl.pitch_speed_pid_para.pid_type, pitch_speed_pid_para_t , gimbal_ctrl.pitch_speed_pid_para.para_max_out, gimbal_ctrl.pitch_speed_pid_para.para_max_iout);
    
		
}

void GimbalDataUpdate(void)
{
	
		gimbal_ctrl.gimbal_pitch_real=90.0f+INS_angle_deg[1];
		gimbal_ctrl.gimbal_roll_real=INS_angle_deg[2];   //<- + 
	
		gimbal_ctrl.gimbal_pitchspeed_real=bmi088_real_data.gyro[0];
		gimbal_ctrl.gimbal_rollspeed_real=bmi088_real_data.gyro[2];
	
		gimbal_ctrl.board_temp=((((float)(ADC_buf[0]) / 4096 * 3.3f)-0.76f)/0.0025f) + 25;
		gimbal_ctrl.batt_votage=(float)(ADC_buf[1]) / 4096 * 3.3f * 1.51f;
	
	  if(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8)&&HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15))
		{
			gimbal_ctrl.batt_state=BATT_BOOSTING;
		}
		else if(!HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8))
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


void OutsideCtrl(void)
{
		if(!gimbal_ctrl.enable_flag)
		{
			gimbal_ctrl.PwmL=0;
			gimbal_ctrl.PwmR=0;
		}
		if(!gimbal_ctrl.uartupdate_flag)
		{
			gimbal_ctrl.enable_flag=0;
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
	  uint8_t temp;
	
		if((uart_cnt==15)||(uart_cnt==105))
		{
			temp = (0x02);
			temp &= gimbal_ctrl.uarttran_flag;
			if(temp) //debug data
			{
				uint8_t justFloatTail[20];
				uint8_t intarray[2];
				*(uint16_t *)intarray = gimbal_ctrl.PwmL;
				justFloatTail[0]=intarray[0];
				justFloatTail[1]=intarray[1];
				*(uint16_t *)intarray = gimbal_ctrl.PwmR;
				justFloatTail[2]=intarray[0];
				justFloatTail[3]=intarray[1];
				
				float2u8Arry(&justFloatTail[4],&gimbal_ctrl.gimbal_pitchspeed_real);
				float2u8Arry(&justFloatTail[8],&gimbal_ctrl.gimbal_rollspeed_real);
				
				justFloatTail[12]=0x00;
				justFloatTail[13]=0x00;
				justFloatTail[14]=0x81;
				justFloatTail[15]=0x7e;
				
				CDC_Transmit_FS(justFloatTail,16);
			}
		}
		
		if((uart_cnt==30)||(uart_cnt==120))
		{
			temp = (0x04);
			temp &= gimbal_ctrl.uarttran_flag;
			if(temp) //basic data
			{
				uint8_t justFloatTail[25];
				justFloatTail[0]=gimbal_ctrl.enable_flag;
				justFloatTail[1]=gimbal_ctrl.mode_flag;
				
				float2u8Arry(&justFloatTail[2],&gimbal_ctrl.gimbal_pitch_set);
				float2u8Arry(&justFloatTail[6],&gimbal_ctrl.gimbal_roll_set);
				float2u8Arry(&justFloatTail[10],&gimbal_ctrl.board_temp);
				float2u8Arry(&justFloatTail[14],&gimbal_ctrl.batt_votage);
				
				justFloatTail[18]=gimbal_ctrl.batt_state;
				justFloatTail[19]=gimbal_ctrl.uarttran_flag;
				
				justFloatTail[20]=0x00;
				justFloatTail[21]=0x00;
				justFloatTail[22]=0x7e;
				justFloatTail[23]=0x81;
				
				CDC_Transmit_FS(justFloatTail,24);
			}
		}
		
		if((uart_cnt==45)||(uart_cnt==135))
		{		
			temp = (0x08);
			temp &= gimbal_ctrl.uarttran_flag;
			if(temp) //roll angle pid para
			{
				uint8_t justFloatTail[28];
				
				float2u8Arry(&justFloatTail[0],&gimbal_ctrl.roll_angle_pid_para.para_kp);
				float2u8Arry(&justFloatTail[4],&gimbal_ctrl.roll_angle_pid_para.para_ki);
				float2u8Arry(&justFloatTail[8],&gimbal_ctrl.roll_angle_pid_para.para_kd);
				float2u8Arry(&justFloatTail[12],&gimbal_ctrl.roll_angle_pid_para.para_max_out);
				float2u8Arry(&justFloatTail[16],&gimbal_ctrl.roll_angle_pid_para.para_max_iout);	
				
				justFloatTail[20]=gimbal_ctrl.roll_angle_pid_para.pid_type;
				justFloatTail[21]=1;
				
				justFloatTail[22]=0x00;
				justFloatTail[23]=0x00;
				justFloatTail[24]=0x7f;
				justFloatTail[25]=0x80;
				
				CDC_Transmit_FS(justFloatTail,26);
			}
		}
		
		if((uart_cnt==60)||(uart_cnt==150))
		{					
			temp = (0x10);
			temp &= gimbal_ctrl.uarttran_flag;
			if(temp) //roll speed pid para
			{
				uint8_t justFloatTail[28];
				
				float2u8Arry(&justFloatTail[0],&gimbal_ctrl.roll_speed_pid_para.para_kp);
				float2u8Arry(&justFloatTail[4],&gimbal_ctrl.roll_speed_pid_para.para_ki);
				float2u8Arry(&justFloatTail[8],&gimbal_ctrl.roll_speed_pid_para.para_kd);
				float2u8Arry(&justFloatTail[12],&gimbal_ctrl.roll_speed_pid_para.para_max_out);
				float2u8Arry(&justFloatTail[16],&gimbal_ctrl.roll_speed_pid_para.para_max_iout);	
				
				justFloatTail[20]=gimbal_ctrl.roll_speed_pid_para.pid_type;
				justFloatTail[21]=1;
				
				justFloatTail[22]=0x00;
				justFloatTail[23]=0x00;
				justFloatTail[24]=0x7f;
				justFloatTail[25]=0x80;
				
				CDC_Transmit_FS(justFloatTail,26);
			}
		}
		
		if((uart_cnt==75)||(uart_cnt==165))
		{			
			temp = (0x20);
			temp &= gimbal_ctrl.uarttran_flag;
			if(temp) //pitch angle pid para
			{
				uint8_t justFloatTail[28];
				
				float2u8Arry(&justFloatTail[0],&gimbal_ctrl.pitch_angle_pid_para.para_kp);
				float2u8Arry(&justFloatTail[4],&gimbal_ctrl.pitch_angle_pid_para.para_ki);
				float2u8Arry(&justFloatTail[8],&gimbal_ctrl.pitch_angle_pid_para.para_kd);
				float2u8Arry(&justFloatTail[12],&gimbal_ctrl.pitch_angle_pid_para.para_max_out);
				float2u8Arry(&justFloatTail[16],&gimbal_ctrl.pitch_angle_pid_para.para_max_iout);	
				
				justFloatTail[20]=gimbal_ctrl.pitch_angle_pid_para.pid_type;
				justFloatTail[21]=1;
				
				justFloatTail[22]=0x00;
				justFloatTail[23]=0x00;
				justFloatTail[24]=0x7f;
				justFloatTail[25]=0x80;
				
				CDC_Transmit_FS(justFloatTail,26);
			}
		}
	
		if((uart_cnt==90)||(uart_cnt==180))
		{					
			temp = (0x40);
			temp &= gimbal_ctrl.uarttran_flag;
			if(temp) //pitch speed pid para
			{
				uint8_t justFloatTail[28];
				
				float2u8Arry(&justFloatTail[0],&gimbal_ctrl.pitch_speed_pid_para.para_kp);
				float2u8Arry(&justFloatTail[4],&gimbal_ctrl.pitch_speed_pid_para.para_ki);
				float2u8Arry(&justFloatTail[8],&gimbal_ctrl.pitch_speed_pid_para.para_kd);
				float2u8Arry(&justFloatTail[12],&gimbal_ctrl.pitch_speed_pid_para.para_max_out);
				float2u8Arry(&justFloatTail[16],&gimbal_ctrl.pitch_speed_pid_para.para_max_iout);	
				
				justFloatTail[20]=gimbal_ctrl.pitch_speed_pid_para.pid_type;
				justFloatTail[21]=1;
				
				justFloatTail[22]=0x00;
				justFloatTail[23]=0x00;
				justFloatTail[24]=0x7f;
				justFloatTail[25]=0x80;
				
				CDC_Transmit_FS(justFloatTail,26);
			}
		}
		
		temp = (0x01);
		temp &= gimbal_ctrl.uarttran_flag;		
		if(temp)
		{
			uint8_t justFloatTail[20];
			float2u8Arry(&justFloatTail[0],&gimbal_ctrl.gimbal_pitch_real);
			float2u8Arry(&justFloatTail[4],&gimbal_ctrl.gimbal_roll_real);
				
			justFloatTail[8]=0x00;
			justFloatTail[9]=0x00;
			justFloatTail[10]=0x80;
			justFloatTail[11]=0x7f;
			
			CDC_Transmit_FS(justFloatTail,12);
		}
	
		if(uart_cnt>200)
		{
			uart_cnt=0;
		}
		else
		{
			uart_cnt++;
		}
	
	

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