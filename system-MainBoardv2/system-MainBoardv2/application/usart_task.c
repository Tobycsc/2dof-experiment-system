#include "usart_task.h"
#include "main.h"
#include "usart.h"
#include "Gimbal_Task.h"
#include "bsp_flash.h"
#include "usbd_cdc_if.h"


extern uint32_t usb_recv_len;
extern uint8_t usb_data_rx_buf[50];

extern gimbal_ctrl_t gimbal_ctrl;


void pid_para_write(float kp,float ki,float kd,float max,float imax,uint8_t type,uint8_t num)
{
	switch(num)
	{
		case 1 :
		{
				gimbal_ctrl.roll_angle_pid_para.para_kp=kp;
				gimbal_ctrl.roll_angle_pid_para.para_ki=ki;
				gimbal_ctrl.roll_angle_pid_para.para_kd=kd;
				gimbal_ctrl.roll_angle_pid_para.para_max_out=max;
				gimbal_ctrl.roll_angle_pid_para.para_max_iout=imax;
				if(type)
				{
					gimbal_ctrl.roll_angle_pid_para.pid_type=PID_DELTA;				
				}
				else
				{
					gimbal_ctrl.roll_angle_pid_para.pid_type=PID_POSITION;
				}
			
				fp32 roll_angle_pid_para_t[3];
				roll_angle_pid_para_t[0]=gimbal_ctrl.roll_angle_pid_para.para_kp;
				roll_angle_pid_para_t[1]=gimbal_ctrl.roll_angle_pid_para.para_ki;
				roll_angle_pid_para_t[2]=gimbal_ctrl.roll_angle_pid_para.para_kd;
				
				PID_init(&gimbal_ctrl.roll_angle_pid, gimbal_ctrl.roll_angle_pid_para.pid_type, roll_angle_pid_para_t, gimbal_ctrl.roll_angle_pid_para.para_max_out, gimbal_ctrl.roll_angle_pid_para.para_max_iout);

				break;
		}
		case 2 :
		{
				gimbal_ctrl.roll_speed_pid_para.para_kp=kp;
				gimbal_ctrl.roll_speed_pid_para.para_ki=ki;
				gimbal_ctrl.roll_speed_pid_para.para_kd=kd;
				gimbal_ctrl.roll_speed_pid_para.para_max_out=max;
				gimbal_ctrl.roll_speed_pid_para.para_max_iout=imax;
				if(type)
				{
					gimbal_ctrl.roll_speed_pid_para.pid_type=PID_DELTA;				
				}
				else
				{
					gimbal_ctrl.roll_speed_pid_para.pid_type=PID_POSITION;
				}
			
				fp32 roll_speed_pid_para_t[3];
				roll_speed_pid_para_t[0]=gimbal_ctrl.roll_speed_pid_para.para_kp;
				roll_speed_pid_para_t[1]=gimbal_ctrl.roll_speed_pid_para.para_ki;
				roll_speed_pid_para_t[2]=gimbal_ctrl.roll_speed_pid_para.para_kd;
				
				PID_init(&gimbal_ctrl.roll_speed_pid, gimbal_ctrl.roll_speed_pid_para.pid_type, roll_speed_pid_para_t, gimbal_ctrl.roll_speed_pid_para.para_max_out, gimbal_ctrl.roll_speed_pid_para.para_max_iout);

				break;
		}
		case 3 :
		{
				gimbal_ctrl.pitch_angle_pid_para.para_kp=kp;
				gimbal_ctrl.pitch_angle_pid_para.para_ki=ki;
				gimbal_ctrl.pitch_angle_pid_para.para_kd=kd;
				gimbal_ctrl.pitch_angle_pid_para.para_max_out=max;
				gimbal_ctrl.pitch_angle_pid_para.para_max_iout=imax;
				if(type)
				{
					gimbal_ctrl.pitch_angle_pid_para.pid_type=PID_DELTA;				
				}
				else
				{
					gimbal_ctrl.pitch_angle_pid_para.pid_type=PID_POSITION;
				}
			
				fp32 pitch_angle_pid_para_t[3];
				pitch_angle_pid_para_t[0]=gimbal_ctrl.pitch_angle_pid_para.para_kp;
				pitch_angle_pid_para_t[1]=gimbal_ctrl.pitch_angle_pid_para.para_ki;
				pitch_angle_pid_para_t[2]=gimbal_ctrl.pitch_angle_pid_para.para_kd;
				
				PID_init(&gimbal_ctrl.pitch_angle_pid, gimbal_ctrl.pitch_angle_pid_para.pid_type, pitch_angle_pid_para_t, gimbal_ctrl.pitch_angle_pid_para.para_max_out, gimbal_ctrl.pitch_angle_pid_para.para_max_iout);

				break;
		}
		case 4 :
		{
				gimbal_ctrl.pitch_speed_pid_para.para_kp=kp;
				gimbal_ctrl.pitch_speed_pid_para.para_ki=ki;
				gimbal_ctrl.pitch_speed_pid_para.para_kd=kd;
				gimbal_ctrl.pitch_speed_pid_para.para_max_out=max;
				gimbal_ctrl.pitch_speed_pid_para.para_max_iout=imax;
				if(type)
				{
					gimbal_ctrl.pitch_speed_pid_para.pid_type=PID_DELTA;				
				}
				else
				{
					gimbal_ctrl.pitch_speed_pid_para.pid_type=PID_POSITION;
				}
			
				fp32 pitch_speed_pid_para_t[3];
				pitch_speed_pid_para_t[0]=gimbal_ctrl.pitch_speed_pid_para.para_kp;
				pitch_speed_pid_para_t[1]=gimbal_ctrl.pitch_speed_pid_para.para_ki;
				pitch_speed_pid_para_t[2]=gimbal_ctrl.pitch_speed_pid_para.para_kd;
				
				PID_init(&gimbal_ctrl.pitch_speed_pid, gimbal_ctrl.pitch_speed_pid_para.pid_type, pitch_speed_pid_para_t, gimbal_ctrl.pitch_speed_pid_para.para_max_out, gimbal_ctrl.pitch_speed_pid_para.para_max_iout);

				break;
		}
	}
}



void usart_task(void const* argument)
{


    vTaskDelay(300);
    while(1)
    {

				if(usb_recv_len)
				{
					gimbal_ctrl.uartupdate_flag=50000;
				}
				else
				{
					if(gimbal_ctrl.uartupdate_flag)
					{
						gimbal_ctrl.uartupdate_flag--;
					}
				}

				
				
				if(gimbal_ctrl.enable_flag)
				{
					if(!gimbal_ctrl.mode_flag)
					{
						if(usb_recv_len==8) // outside ctrl
						{
							if((usb_data_rx_buf[0]==0x72)&&(usb_data_rx_buf[1]==0x7d))
							{
								if((usb_data_rx_buf[6]==0x7e)&&(usb_data_rx_buf[7]==0x7d))
								{
										memcpy(&gimbal_ctrl.PwmL,&usb_data_rx_buf[2], 2);		
										memcpy(&gimbal_ctrl.PwmR,&usb_data_rx_buf[4], 2);
										usb_recv_len=0;
								}
							}
						}
					}
				}
				else
				{
					if(usb_recv_len==15) //basic para
					{
						if((usb_data_rx_buf[0]==0x70)&&(usb_data_rx_buf[1]==0x7f))
						{
							if((usb_data_rx_buf[13]==0x7b)&&(usb_data_rx_buf[14]==0x7f))
							{
								gimbal_ctrl.enable_flag=usb_data_rx_buf[2];
								gimbal_ctrl.mode_flag=usb_data_rx_buf[3];
								memcpy(&gimbal_ctrl.gimbal_roll_set,&usb_data_rx_buf[4], 4);
								memcpy(&gimbal_ctrl.gimbal_pitch_set,&usb_data_rx_buf[8], 4);
								gimbal_ctrl.uarttran_flag=usb_data_rx_buf[12];
								usb_recv_len=0;
							}
						}
					}				
					
					if(usb_recv_len==26) //pid para
					{
						if((usb_data_rx_buf[0]==0x71)&&(usb_data_rx_buf[1]==0x7e))
						{
							if((usb_data_rx_buf[24]==0x7c)&&(usb_data_rx_buf[25]==0x7e))
							{
								float kp,ki,kd,max,imax;
								uint8_t type,num;
								memcpy(&kp,&usb_data_rx_buf[2], 4);
								memcpy(&ki,&usb_data_rx_buf[6], 4);
								memcpy(&kd,&usb_data_rx_buf[10], 4);
								memcpy(&max,&usb_data_rx_buf[14], 4);
								memcpy(&imax,&usb_data_rx_buf[18], 4);
								type=usb_data_rx_buf[22];
								num=usb_data_rx_buf[23];
								if(num)
								{
									pid_para_write(kp,ki,kd,max,imax,type,num);
								}
								else
								{
									pid_para_write(kp,ki,kd,max,imax,type,1);
									pid_para_write(kp,ki,kd,max,imax,type,2);
									pid_para_write(kp,ki,kd,max,imax,type,3);
									pid_para_write(kp,ki,kd,max,imax,type,4);
								}
								usb_recv_len=0;
							}
						}
					}	
					
					if(usb_recv_len==8) // save to flash
					{
						if((usb_data_rx_buf[0]==0x77)&&(usb_data_rx_buf[1]==0x75))
						{
							if((usb_data_rx_buf[6]==0x71)&&(usb_data_rx_buf[7]==0x7e))
							{
								WriteAllPara();
								usb_recv_len=0;
							}
						}
					}	
				}
        vTaskDelay(2);

    }
}


