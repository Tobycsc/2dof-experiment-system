/**
  ******************************************************************************
  * @file    refereetask.h
  * @author  Karolance Future
  * @version V1.0.0
  * @date    2022/03/21
  * @brief   
  ******************************************************************************
  * @attention
	*
  ******************************************************************************
  */
	
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __REFEREETASK_H__
#define __REFEREETASK_H__
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "fifo.h"
#include "protocol.h"

#define REFEREE_USART_RX_BUF_LENGHT 512
#define REFEREE_FIFO_BUF_LENGTH     1024

/* ����ϵͳ����˫������ */
extern uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];

/* ����ϵͳ�������ݶ��� */
extern fifo_s_t Referee_FIFO;
extern uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
extern unpack_data_t Referee_Unpack_OBJ;

typedef struct
{
	uint8_t updata_flag,updata_flag1;
	
	uint8_t chassis_state;
	uint8_t aim_mode;
	uint8_t chassis_mode;
	
	uint16_t spd_lim,pwr_lim;
	

	_Bool gim_pwr;
	_Bool lock_state;
	_Bool magz_state;
	
	_Bool if_single;
	_Bool if_boost;
	_Bool if_leap;
	
} State_mach_t;

/* ��̬UI���ݱ��� */
extern uint8_t UI_AutoAim_Flag; //�Ƿ��������־λ
extern float   UI_Kalman_Speed; //������Ԥ���ٶ�
extern float   UI_Gimbal_Pitch; //��̨Pitch��Ƕ�
extern float   UI_Gimbal_Yaw;   //��̨Yaw��Ƕ�
extern uint8_t UI_Capacitance;  //����ʣ������
extern uint8_t UI_fric_is_on;   //Ħ�����Ƿ���

void Referee_IRQHandler(void);
void UI_init(void);
void StateFlagCtrl(void);
void StateMachine(void);

#ifdef __cplusplus
}
#endif

#endif /* __REFEREETASK_H__ */
