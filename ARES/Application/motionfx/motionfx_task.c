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
#include "imu.h"
#include "imu_comm.h"
#include "string.h"
#include "mpu6500reg.h"
#include <stdio.h>
#include "user_lib.h"

#define MPU6500_RX_BUF_DATA_OFFSET 1
#define IST8310_RX_BUF_DATA_OFFSET 16

MFX_output_t motionFX_output;

static MFX_input_t         motionFX_input;
static MFX_MagCal_output_t mag_cal_out;
static MFX_MagCal_input_t  mag_cal_in;
static MFX_knobs_t         knobs;
static TaskHandle_t        task_handle;
static mpu_real_data_t     mpu6500_real_data;
static ist_real_data_t     ist8310_real_data;
static uint8_t             SPI_RX_buf[SPI_BUF_SIZE];
static uint8_t             SPI_TX_buf[SPI_BUF_SIZE] = {MPU_INT_STATUS | MPU_READ_MSB};
static uint8_t             initialized              = 0;
static uint8_t             counter                  = 2;

static void MotionFX_get_input(MFX_input_t *MFX_input, mpu_real_data_t *mpu6500_real_data,
                               ist_real_data_t *ist8310_real_data) {
  MFX_input->acc[0]  = mpu6500_real_data->accel[0];
  MFX_input->acc[1]  = mpu6500_real_data->accel[1];
  MFX_input->acc[2]  = mpu6500_real_data->accel[2];
  MFX_input->gyro[0] = mpu6500_real_data->gyro[0];
  MFX_input->gyro[1] = mpu6500_real_data->gyro[1];
  MFX_input->gyro[2] = mpu6500_real_data->gyro[2];
  MFX_input->mag[0]  = ist8310_real_data->mag[0] - mag_cal_out.hi_bias[0];
  MFX_input->mag[1]  = ist8310_real_data->mag[1] - mag_cal_out.hi_bias[1];
  MFX_input->mag[2]  = ist8310_real_data->mag[2] - mag_cal_out.hi_bias[2];
}

static void MotionFX_MagCal_get_input(MFX_MagCal_input_t *mag_cal_in, ist_real_data_t *ist8310_real_data) {
  mag_cal_in->mag[0]     = ist8310_real_data->mag[0] / 50;
  mag_cal_in->mag[1]     = ist8310_real_data->mag[1] / 50;
  mag_cal_in->mag[2]     = ist8310_real_data->mag[2] / 50;
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
  MotionFX_setKnobs(&knobs);
  MotionFX_enable_9X(MFX_ENGINE_DISABLE);
  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
}

static HAL_StatusTypeDef MPU_SPI_speed_change(SPI_HandleTypeDef *hspi) {
  __HAL_LOCK(hspi);
  hspi->Instance->CR1 &= 0xF7C7;
  hspi->Instance->CR1 |= ((uint16_t)0x0018);
  hspi->Instance->CR1 |= ((uint16_t)0x0000);
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
  while (mpu_init() != MPU6500_NO_ERROR) {
    ;
  }
  //初始化IST8310
  while (ist_init() != IST8310_NO_ERROR) {
    ;
  }
  //获得任务句柄
  task_handle = xTaskGetCurrentTaskHandle();
  //提高SPI传输速度
  MPU_SPI_speed_change(&hspi5);
  //启动MotionFX
  MotionFX_Init();
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);
  MotionFX_MagCal_init(10, MFX_ENGINE_ENABLE);
  initialized = 1;
  for (;;) {
    //等待外部中断中断唤醒任务
    // while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
    //    ;
    last_wake_time = xTaskGetTickCount();
    MotionFX_MagCal_getParams(&mag_cal_out);
    mpu_get_data(SPI_RX_buf + MPU6500_RX_BUF_DATA_OFFSET, &mpu6500_real_data);
    ist_get_data(SPI_RX_buf + IST8310_RX_BUF_DATA_OFFSET, &ist8310_real_data);
    MotionFX_get_input(&motionFX_input, &mpu6500_real_data, &ist8310_real_data);
    delta_time = 0.01f;
    MotionFX_propagate(&motionFX_output, &motionFX_input, &delta_time);
    MotionFX_update(&motionFX_output, &motionFX_input, &delta_time, NULL);
    if(counter-->0){
    MotionFX_MagCal_get_input(&mag_cal_in, &ist8310_real_data);
    MotionFX_MagCal_run(&mag_cal_in);
    printf("%f,%f,%f,%f\n", motionFX_output.quaternion_9X[0], motionFX_output.quaternion_9X[1],
             motionFX_output.quaternion_9X[2], motionFX_output.quaternion_9X[3]);
        counter=2;
    }
    vTaskDelayUntil(&last_wake_time, 10);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == MPU_DRDY_Pin) {
    if (initialized) {
      MPU_NSS_L();
      HAL_SPI_TransmitReceive_DMA(&hspi5, SPI_TX_buf, SPI_RX_buf, SPI_BUF_SIZE);
    }
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI5) {
    MPU_NSS_H();
    HAL_SPI_DMAStop(hspi);
    //唤醒任务
    //  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    //    static BaseType_t xHigherPriorityTaskWoken;
    //    vTaskNotifyGiveFromISR(task_handle, &xHigherPriorityTaskWoken);
    //    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    //  }
  }
}

char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data) {
  return 1;
  memcpy(data, (void *)MFX_DATA_ADDRESS, dataSize);
  return 1;
}

char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data) {
  return 1;
  if (HAL_FLASH_Unlock() != HAL_OK) {
    return 1;
  }
  for (uint8_t i = 0; i < dataSize; i++) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, MFX_DATA_ADDRESS + i * 8, data[i]) != HAL_OK) {
      return 1;
    }
  }
  if (HAL_FLASH_Lock() != HAL_OK) {
    return 1;
  }
  return 1;
}
