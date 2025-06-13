#include "bsp_flash.h"
#include "main.h"
#include "Gimbal_Task.h"
#include "string.h"

extern gimbal_ctrl_t gimbal_ctrl;

uint8_t WriteFloat(uint32_t addr,float *data)
{
		uint32_t DATA_32=*(uint32_t *)data;
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,addr, DATA_32) == HAL_OK) 
		{
			return 1;
		}
		else
		{
			return 0;
		}
}

void ReadFloat(uint32_t addr,float *data)
{
	uint8_t buf[4];
	*(uint32_t *)buf= *(__IO uint32_t *)addr;
	memcpy(data,buf,4);
}

uint8_t WriteAllPara(void)
{
		uint32_t PageError;
		FLASH_EraseInitTypeDef EraseInitStruct;
		

		HAL_FLASH_Unlock();
		__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

		EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector        = 11;
		EraseInitStruct.NbSectors     = 1;
		
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
		{
			HAL_FLASH_Lock();
			return 0;
		}
		
		WriteFloat(ADDR_ROLLANGLE_KP,&(gimbal_ctrl.roll_angle_pid_para.para_kp));
		WriteFloat(ADDR_ROLLANGLE_KI,&(gimbal_ctrl.roll_angle_pid_para.para_ki));
		WriteFloat(ADDR_ROLLANGLE_KD,&(gimbal_ctrl.roll_angle_pid_para.para_kd));
		WriteFloat(ADDR_ROLLANGLE_MAX,&(gimbal_ctrl.roll_angle_pid_para.para_max_out));
		WriteFloat(ADDR_ROLLANGLE_IMAX,&(gimbal_ctrl.roll_angle_pid_para.para_max_iout));
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,ADDR_ROLLANGLE_TYPE,gimbal_ctrl.roll_angle_pid_para.pid_type);

		HAL_FLASH_Lock();

		return 1;
}



void ReadAllPara(void)
{
	ReadFloat(ADDR_ROLLANGLE_KP,&(gimbal_ctrl.roll_angle_pid_para.para_kp));
	ReadFloat(ADDR_ROLLANGLE_KI,&(gimbal_ctrl.roll_angle_pid_para.para_ki));
	ReadFloat(ADDR_ROLLANGLE_KD,&(gimbal_ctrl.roll_angle_pid_para.para_kd));
	
}


































