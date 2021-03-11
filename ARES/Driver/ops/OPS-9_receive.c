#include "main.h"
#include "OPS-9_receive.h"
#include "String.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef  hdma_uart7_rx;

typedef struct {
  fp32 yaw;
  fp32 pitch;
  fp32 roll;
  fp32 x;
  fp32 y;
  fp32 w_z;
} OPS;

OPS OPSdata;

//接收原始数据，为28个字节，给了56个字节长度，防止DMA传输越界
static uint8_t ops_rx_buf[2][OPS_BUFFER_LENGTH];
static void    ops(volatile const uint8_t *ops_buf);

void ops_init(void) {
  //使能DMA串口接收
  SET_BIT(huart7.Instance->CR3, USART_CR3_DMAR);
  //使能串口空闲中断
  __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
  //停止DMA接收
  __HAL_DMA_DISABLE(&hdma_uart7_rx);
  //由于DMA是AHB总线主设备,和CPU有冲突，所以有可能一次指令不成功
  while (hdma_uart7_rx.Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(&hdma_uart7_rx);
  }
  //将DMA的外设地址设置为UART7的DR寄存器
  hdma_uart7_rx.Instance->PAR = (uint32_t) & (UART7->DR);
  //将DMA的缓冲区1地址设置为第一个BUFFER
  hdma_uart7_rx.Instance->M0AR = (uint32_t)(&ops_rx_buf[0]);
  //将DMA的缓冲区2地址设置为第二个BUFFER
  hdma_uart7_rx.Instance->M1AR = (uint32_t)(&ops_rx_buf[1]);
  //DMA接收长度为缓冲区大小
  hdma_uart7_rx.Instance->NDTR = OPS_BUFFER_LENGTH;
  //使能双缓冲区
  SET_BIT(hdma_uart7_rx.Instance->CR, DMA_SxCR_DBM);
  //使能DMA
  __HAL_DMA_ENABLE(&hdma_uart7_rx);
}

void UART7_IRQHandler(void) {
  if (huart7.Instance->SR & UART_FLAG_RXNE) //接收到数据
  {
    __HAL_UART_CLEAR_PEFLAG(&huart7);

    __HAL_UART_FLUSH_DRREGISTER(&huart7);

    return;

  } else if (huart7.Instance->SR & UART_FLAG_IDLE)

    __HAL_UART_CLEAR_PEFLAG(&huart7);
  static uint16_t this_time_rx_len = 0;

  //失效DMA
  __HAL_DMA_DISABLE(&hdma_uart7_rx);

  //get receive data length, length = set_data_length - remain_length
  //获取接收数据长度,长度 = 设定长度 - 剩余长度
  this_time_rx_len = OPS_BUFFER_LENGTH - hdma_uart7_rx.Instance->NDTR;

  //reset set_data_lenght
  //重新设定数据长度
  hdma_uart7_rx.Instance->NDTR = OPS_BUFFER_LENGTH;

  if ((hdma_uart7_rx.Instance->CR & DMA_SxCR_CT) == RESET) {
    /* Current memory buffer used is Memory 0 */

    //set memory buffer 1
    //设定缓冲区1
    hdma_uart7_rx.Instance->CR |= DMA_SxCR_CT;

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart7_rx);

    if (this_time_rx_len == OPS_FRAME_LENGTH) {
      ops(ops_rx_buf[0]);
    }
  } else {
    /* Current memory buffer used is Memory 1 */

    //set memory buffer 0
    //设定缓冲区0
    DMA1_Stream1->CR &= ~(DMA_SxCR_CT);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart7_rx);

    if (this_time_rx_len == OPS_FRAME_LENGTH) {
      ops(ops_rx_buf[1]);
    }
  }
}

static void ops(volatile const uint8_t *ops_buf) {
  if (ops_buf == NULL) {
    return;
  }

  if (ops_buf[0] == 0x0d && ops_buf[1] == 0x0a) {

    memcpy((void *)&OPSdata, (void *)&ops_buf[2], sizeof(OPSdata));
  }
}
