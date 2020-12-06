/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     motionfx_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    MotionFX任务
  * @version  0.1
  * @date     2020-12-05
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#include "main.h"
#include "motionfx_task.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "spi.h"
#include "motion_fx.h"
#include "mpu6500driver.h"
#include "ist8310driver.h"
#include "mpu6500comm.h"
#include "ist8310comm.h"
#include "string.h"

#define MPU6500_RX_BUF_DATA_OFFSET 1
#define IST8310_RX_BUF_DATA_OFFSET 16

MFX_output_t motionFX_output;

static MFX_input_t         motionFX_input;
static MFX_MagCal_output_t mag_cal_out;
static MFX_MagCal_input_t  mag_cal_in;
static MFX_knobs_t         knobs;
static TaskHandle_t        task_handle;
static mpu6500_real_data_t mpu6500_real_data;
static ist8310_real_data_t ist8310_real_data;
static uint8_t             SPI_RX_buf[SPI_BUF_SIZE];
static uint8_t             SPI_TX_buf[SPI_BUF_SIZE] = {MPU_INT_STATUS | MPU_SPI_READ_MSB};
static uint8_t             initialized              = 0;
static uint8_t             counter                  = 0;

static void MotionFX_get_input(MFX_input_t *MFX_input, mpu6500_real_data_t *mpu6500_real_data,
                               ist8310_real_data_t *ist8310_real_data) {
  MFX_input->acc[0]  = mpu6500_real_data->accel[0];
  MFX_input->acc[1]  = mpu6500_real_data->accel[1];
  MFX_input->acc[2]  = mpu6500_real_data->accel[2];
  MFX_input->gyro[0] = mpu6500_real_data->gyro[0];
  MFX_input->gyro[1] = mpu6500_real_data->gyro[1];
  MFX_input->gyro[2] = mpu6500_real_data->gyro[2];
  MFX_input->mag[0]  = ist8310_real_data->mag[0];
  MFX_input->mag[1]  = ist8310_real_data->mag[1];
  MFX_input->mag[2]  = ist8310_real_data->mag[2];
}

static void MotionFX_MagCal_get_input(MFX_MagCal_input_t *mag_cal_in, ist8310_real_data_t *ist8310_real_data) {
  mag_cal_in->mag[0]     = ist8310_real_data->mag[0];
  mag_cal_in->mag[1]     = ist8310_real_data->mag[1];
  mag_cal_in->mag[2]     = ist8310_real_data->mag[2];
  mag_cal_in->time_stamp = xTaskGetTickCount();
}

static void MotionFX_Init(void) {
  MotionFX_initialize();
  MotionFX_getKnobs(&knobs);
  knobs.start_automatic_gbias_calculation = 1; //设置为1，开机后需静置10s
  knobs.LMode                             = 1;
  knobs.modx                              = MFX_UPDATE_INTERVAL; //每调用几次推理调用一次更新
  knobs.output_type                       = MFX_ENGINE_OUTPUT_ENU;
  knobs.gyro_orientation[0]               = 'w';
  knobs.gyro_orientation[1]               = 's';
  knobs.gyro_orientation[2]               = 'u';
  knobs.acc_orientation[0]                = 'w';
  knobs.acc_orientation[1]                = 's';
  knobs.acc_orientation[2]                = 'u';
  knobs.mag_orientation[0]                = 'n';
  knobs.mag_orientation[1]                = 'w';
  knobs.mag_orientation[3]                = 'd';
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);
  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
  MotionFX_MagCal_init(MFX_UPDATE_INTERVAL, MFX_ENGINE_ENABLE);
}

static HAL_StatusTypeDef MPU_SPI_speed_change(SPI_HandleTypeDef *hspi) {
  __HAL_LOCK(hspi);
  hspi->Instance->CR1 &= 0xF7C7;
  hspi->Instance->CR1 |= SPI_BAUDRATEPRESCALER_8;
  hspi->Instance->CR1 |= SPI_DATASIZE_8BIT;
  hspi->Instance->CR1 |= 1 << 6;
  __HAL_UNLOCK(hspi);
  return HAL_OK;
}

void motionfx_task(void *pvParameters) {
  TickType_t last_wake_time;
  float      delta_time;
  //延迟等待系统稳定
  vTaskDelay(MOTIONFX_INIT_TIME);
  //初始化MPU6500
  while (mpu6500_init() != MPU6500_NO_ERROR) {
    ;
  }
  //初始化IST8310
  while (ist8310_init() != IST8310_NO_ERROR) {
    ;
  }
  //获得任务句柄
  task_handle = xTaskGetCurrentTaskHandle();
  //提高SPI传输速度
  MPU_SPI_speed_change(&hspi5);
  //启动MotionFX
  MotionFX_Init();
  initialized = 1;
  for (;;) {
    last_wake_time = xTaskGetTickCount();
    //等待数据传输完成
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
      ;
    counter++;
    MotionFX_MagCal_getParams(&mag_cal_out);
    mpu6500_get_data(SPI_RX_buf + MPU6500_RX_BUF_DATA_OFFSET, &mpu6500_real_data);
    ist8310_get_data(SPI_RX_buf + IST8310_RX_BUF_DATA_OFFSET, &ist8310_real_data);
    MotionFX_get_input(&motionFX_input, &mpu6500_real_data, &ist8310_real_data);
    delta_time = (fp32)(xTaskGetTickCount() - last_wake_time) / 1000.0f;
    MotionFX_propagate(&motionFX_output, &motionFX_input, &delta_time);
    if (counter == MFX_UPDATE_INTERVAL) {
      MotionFX_update(&motionFX_output, &motionFX_input, &delta_time, NULL);
      if (mag_cal_out.cal_quality < MFX_MAGCALGOOD) {
        MotionFX_MagCal_get_input(&mag_cal_in, &ist8310_real_data);
        MotionFX_MagCal_run(&mag_cal_in);
      }
      counter = 0;
    }
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_DRDY_Pin) {
    if (initialized) {
      mpu6500_SPI_NSS_L();
      HAL_SPI_TransmitReceive_DMA(&hspi5, SPI_TX_buf, SPI_RX_buf, SPI_BUF_SIZE);
    }
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI5) {
    mpu6500_SPI_NSS_H();
    //唤醒任务
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data) {
  memcpy(data, (void*)MFX_DATA_ADDRESS, dataSize);
  return 1;
}

char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data) {
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return 0;
  }
  for (uint8_t i = 0; i < dataSize; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, MFX_DATA_ADDRESS + i * 8, data[i]) != HAL_OK) {
      return 0;
    }
  }
  if (HAL_FLASH_Lock() != HAL_OK) {
    return 0;
  }
  return 1;
}
