#include "usart_task.h"
#include "main.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "Gimbal_Task.h"

#include "usbd_cdc_if.h"

/* Private define ------------------------------------------------------------*/
#define Referee_FIFOInit fifo_s_init
#define Max(a,b) ((a) > (b) ? (a) : (b))

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

#define Robot_ID_Current Robot_ID_Red_Infantry4
//#define Robot_ID_Current Robot_ID_Blue_Infantry4

/* Private variables ---------------------------------------------------------*/


extern gimbal_ctrl_t gimbal_ctrl;

void usart_task(void const* argument)
{


    vTaskDelay(300);
    while(1)
    {
			
				//CDC_Transmit_FS()
        vTaskDelay(10);

    }
}

uint16_t this_time_rx_len = 0;
//void Referee_IRQHandler(void)
//{
//    if(huart6.Instance->SR & UART_FLAG_RXNE)
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart6);
//    }
//    else if(USART6->SR & UART_FLAG_IDLE)
//    {
//        static uint16_t this_time_rx_len = 0;

//        __HAL_UART_CLEAR_PEFLAG(&huart6);

//        if((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
//            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//            fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
//            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//            fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
//        }
//    }
//}
