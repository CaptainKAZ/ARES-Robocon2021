#include "main.h"
#include "OPS-9_receive.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_rx;

fp32 pos_x=0;
fp32 pos_y=0;
fp32 zangle=0;
fp32 xangle=0;
fp32 yangle=0;
fp32 w_z=0;

//接收原始数据，为28个字节，给了56个字节长度，防止DMA传输越界
static uint8_t ops_rx_buf[2][OPS_BUFFER_LENGTH];
static void ops(volatile const uint8_t *ops_buf);

void UART7_IRQHandler(void)
{
    if(huart7.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart7);

    } else if (huart7.Instance->SR & UART_FLAG_IDLE)

        __HAL_UART_CLEAR_PEFLAG(&huart7);
        static uint16_t this_time_rx_len = 0;

        if ((hdma_usart7_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart7_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = OPS_BUFFER_LENGTH - hdma_usart7_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart7_rx.Instance->NDTR = OPS_BUFFER_LENGTH;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart7_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart7_rx);

            if(this_time_rx_len == OPS_FRAME_LENGTH)
            {
                ops(ops_rx_buf[0]);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart7_rx);

            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = OPS_BUFFER_LENGTH - hdma_usart7_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart7_rx.Instance->NDTR = OPS_BUFFER_LENGTH;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart7_rx);

            if(this_time_rx_len == OPS_FRAME_LENGTH)
            {
                ops(ops_rx_buf[1]);
            }
        }
}


static void ops(volatile const uint8_t *ops_buf)
{
    if (ops_buf == NULL)
    {
        return;
    }

    if ((ops_buf[0] | ops_buf[1] << 8) == 0x0a0f)
    {
        pos_x = ops_buf[2] | ops_buf[3] << 8 | ops_buf[4] << 16 | ops_buf[5] << 24;
        pos_y = ops_buf[6] | ops_buf[7] << 8 | ops_buf[8] << 16 | ops_buf[9] << 24;
        zangle = ops_buf[10] | ops_buf[11] << 8 | ops_buf[12] << 16 | ops_buf[13] << 24;
        xangle = ops_buf[14] | ops_buf[15] << 8 | ops_buf[16] << 16 | ops_buf[17] << 24;
        yangle = ops_buf[18] | ops_buf[19] << 8 | ops_buf[20] << 16 | ops_buf[21] << 24;
        w_z = ops_buf[22] | ops_buf[23] << 8 | ops_buf[24] << 16 | ops_buf[25] << 24;
    }
}


