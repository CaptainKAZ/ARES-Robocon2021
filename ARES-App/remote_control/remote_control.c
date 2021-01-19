#include "remote_control.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef  hdma_usart1_rx;

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

RC_ctrl_t rc_ctrl;

//接收原始数据，为25个字节，给了50个字节长度，防止DMA传输越界
static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

//串口中断
void USART1_IRQHandler(void)
{
    if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}

static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[1] | (sbus_buf[2] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[2] >> 3) | (sbus_buf[3] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[3] >> 6) | (sbus_buf[4] << 2) |          //!< Channel 2
                         (sbus_buf[5] << 10)) &0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[5] >> 1) | (sbus_buf[6] << 7)) & 0x07ff; //!< Channel 3                  
    rc_ctrl->rc.ch[4] = ((sbus_buf[6] >> 4) | (sbus_buf[7] << 4)) & 0x07ff;
    rc_ctrl->rc.ch[5] = ((sbus_buf[7] >> 7) | (sbus_buf[8] << 1) | (sbus_buf[9] << 9)) & 0x07ff;
    rc_ctrl->rc.ch[6] = ((sbus_buf[9] >> 2) | (sbus_buf[10] << 6)) & 0x07ff;
    rc_ctrl->rc.ch[7] = ((sbus_buf[10] >> 5) | (sbus_buf[11] << 3)) & 0x07ff;
    rc_ctrl->rc.ch[8] = (sbus_buf[12] | (sbus_buf[13] << 8)) & 0x07ff;
    rc_ctrl->rc.ch[9] = ((sbus_buf[13] >> 3) | (sbus_buf[14] << 5)) & 0x07ff;

    rc_ctrl->rc.ch[0] = (rc_ctrl->rc.ch[0]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[1] = (rc_ctrl->rc.ch[1]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[2] = (rc_ctrl->rc.ch[2]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[3] = (rc_ctrl->rc.ch[3]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[4] = (rc_ctrl->rc.ch[4]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[5] = (rc_ctrl->rc.ch[5]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[6] = (rc_ctrl->rc.ch[6]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[7] = (rc_ctrl->rc.ch[7]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[8] = (rc_ctrl->rc.ch[8]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
    rc_ctrl->rc.ch[9] = (rc_ctrl->rc.ch[9]-RC_CH_VALUE_OFFSET)/RC_CH_VALUE_MAX;
}
