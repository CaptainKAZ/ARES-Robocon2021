/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     INS_task.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    惯性导航任务
  * @version  0.1
  * @date     2020-12-05
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */
#include "main.h"
#include "INS_task.h"
#include "cmsis_os.h"
#include "spi.h"
#include "imu.h"
#include "stopwatch.h"
#include "feedback_task.h"
#include "user_lib.h"
#include "PID.h"
#include "tim.h"
#include "flash_io.h"

MFX_output_t      motionFX_output;
TaskHandle_t      INS_task_handle;
INS_task_status_t INS_task_status;

static stopwatch_t     stopwatch;
static MFX_input_t     motionFX_input;
static MMC_Input_t     mag_cal_in;
static MFX_knobs_t     knobs;
static mpu_real_data_t mpu6500_real_data;
static ist_real_data_t ist8310_real_data;
static TickType_t      last_wake_time;
static float           delta_time;
static imu_cal_t       imu_cal;


static PID_Controller      temperature_pid;
static PID_ControllerParam temperature_pid_param = {
    .general.type = PIDPOS_CONTROLLER, .Int_type = BACK_CALCULATION_INT, .kB = 0.5, .kP = 3000, .kI = 300, .kD = 15, .max_Iout = 9000, .N = 10};
static fp32                temperature_O_Hlim      = 9999;
static fp32                temperature_O_Llim      = 0;
static fp32                temperature_I_loop_Llim = 0;
static fp32                temperature_I_loop_Hlim = 0;
static ControllerConstrain temperature_constrain   = {.I_loop_Hlim = &temperature_I_loop_Hlim,
                                                    .I_loop_Llim = &temperature_I_loop_Llim,
                                                    .O_Hlim      = &temperature_O_Hlim,
                                                    .O_Llim      = &temperature_O_Llim};
static fp32                imu_ctrl_tempretrue     = IMU_TEMPERATRUE;

static void MotionFX_get_input(MFX_input_t *MFX_input, mpu_real_data_t *mpu6500_real_data,
                               ist_real_data_t *ist8310_real_data) {
  MFX_input->acc[0]  = mpu6500_real_data->accel[0];
  MFX_input->acc[1]  = mpu6500_real_data->accel[1];
  MFX_input->acc[2]  = mpu6500_real_data->accel[2];
  MFX_input->gyro[0] = mpu6500_real_data->gyro[0];
  MFX_input->gyro[1] = mpu6500_real_data->gyro[1];
  MFX_input->gyro[2] = mpu6500_real_data->gyro[2];
  MFX_input->mag[0]  = ((ist8310_real_data->mag[0] - imu_cal.mag_cal.HI_Bias[0]) * imu_cal.mag_cal.SF_Matrix[0][0] +
                       (ist8310_real_data->mag[1] - imu_cal.mag_cal.HI_Bias[1]) * imu_cal.mag_cal.SF_Matrix[0][1] +
                       (ist8310_real_data->mag[2] - imu_cal.mag_cal.HI_Bias[2]) * imu_cal.mag_cal.SF_Matrix[0][2]) *
                      0.02f;
  MFX_input->mag[1] = ((ist8310_real_data->mag[0] - imu_cal.mag_cal.HI_Bias[0]) * imu_cal.mag_cal.SF_Matrix[1][0] +
                       (ist8310_real_data->mag[1] - imu_cal.mag_cal.HI_Bias[1]) * imu_cal.mag_cal.SF_Matrix[1][1] +
                       (ist8310_real_data->mag[2] - imu_cal.mag_cal.HI_Bias[2]) * imu_cal.mag_cal.SF_Matrix[1][2]) *
                      0.02f;
  MFX_input->mag[2] = ((ist8310_real_data->mag[0] - imu_cal.mag_cal.HI_Bias[0]) * imu_cal.mag_cal.SF_Matrix[2][0] +
                       (ist8310_real_data->mag[1] - imu_cal.mag_cal.HI_Bias[1]) * imu_cal.mag_cal.SF_Matrix[2][1] +
                       (ist8310_real_data->mag[2] - imu_cal.mag_cal.HI_Bias[2]) * imu_cal.mag_cal.SF_Matrix[2][2]) *
                      0.02f;
}

static void MotionMC_get_input(MMC_Input_t *mag_cal_in, ist_real_data_t *ist8310_real_data) {
  mag_cal_in->Mag[0]    = ist8310_real_data->mag[0];
  mag_cal_in->Mag[1]    = ist8310_real_data->mag[1];
  mag_cal_in->Mag[2]    = ist8310_real_data->mag[2];
  mag_cal_in->TimeStamp = xTaskGetTickCount();
}

static void MotionFX_Init(void) {
  MotionFX_initialize();
  MotionFX_getKnobs(&knobs);
  knobs.start_automatic_gbias_calculation = 1; //设置为1，开机后需静置10s
  knobs.MTime                             = 2.0f;
  //knobs.ATime                             = 0.7f;
  //knobs.FrTime                            = 0.66f;
  knobs.LMode                             = 1;
  knobs.modx                              = 1; //每调用几次推理调用一次更新
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

uint8_t load_cal_data(imu_cal_t *data) {
  if (((imu_cal_t *)FLASH_READ)->head == IMU_CAL_HEAD) {
    Flash_Get((uint8_t*)data, sizeof(imu_cal_t));
    return SUCCESS;
  } else {
    return FAIL;
  }
}

uint8_t save_cal_data(imu_cal_t *data) {
  return Flash_Save((uint8_t*)data, sizeof(imu_cal_t));
}

void INS_task(void *pvParameters) {
  //延迟等待系统稳定
  vTaskDelay(MOTIONFX_INIT_TIME);
  feedback_register(&motionFX_output.quaternion_9X[0], 0);
  feedback_register(&motionFX_output.quaternion_9X[1], 1);
  feedback_register(&motionFX_output.quaternion_9X[2], 2);
  feedback_register(&motionFX_output.quaternion_9X[3], 3);
  feedback_register(&mpu6500_real_data.temp, 4);
  feedback_register(&temperature_pid.out[0],5);
  feedback_register(&temperature_pid.Dout[0], 6);
  //初始化MPU6500
  while (mpu_init() != MPU6500_NO_ERROR) {
    ;
  }
  //初始化IST8310
  while (ist_init() != IST8310_NO_ERROR) {
    ;
  }
  //获得任务句柄
  INS_task_handle = xTaskGetCurrentTaskHandle();
  //提高SPI传输速度
  MPU_SPI_speed_change(&hspi5);
  //初始化PID
  PID_ControllerInit(&temperature_pid, &temperature_constrain, &temperature_pid_param, 0.05);
  //启动MotionFX
  MotionFX_Init();
  MotionMC_Initialize(10, 1);
  MotionFX_enable_9X(MFX_ENGINE_ENABLE);
  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
  MotionFX_MagCal_init(0, MFX_ENGINE_DISABLE);
  stopwatch_register(&stopwatch);
  if (load_cal_data(&imu_cal) == SUCCESS) {
    INS_task_status.imu_calibrated = 1;
    MotionMC_Initialize(0, 0);
    MotionFX_setGbias(imu_cal.gbias);
    MotionFX_getKnobs(&knobs);
    knobs.start_automatic_gbias_calculation = 0;
    MotionFX_setKnobs(&knobs);
  } else {
    INS_task_status.imu_calibrated  = 0;
    imu_cal.head                    = IMU_CAL_HEAD;
    imu_cal.mag_cal.SF_Matrix[0][0] = 1.0f;
    imu_cal.mag_cal.SF_Matrix[1][1] = 1.0f;
    imu_cal.mag_cal.SF_Matrix[2][2] = 1.0f;
  }
  INS_task_status.imu_initialized = 1;
  tic(&stopwatch);
  last_wake_time = xTaskGetTickCount();
  for (;;) {
    if (INS_task_status.imu_calibrated) {
      //等待外部中断中断唤醒任务
      while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
        ;
      imu_get_data(&mpu6500_real_data, &ist8310_real_data);
      if (xTaskGetTickCount() - last_wake_time >= 5) {
        __HAL_TIM_SET_COMPARE(
            &htim3, TIM_CHANNEL_2,
            controllerUpdate((Controller *)&temperature_pid, &imu_ctrl_tempretrue, &mpu6500_real_data.temp, NULL));
          last_wake_time = xTaskGetTickCount();
      }
    } else {
      vTaskDelayUntil(&last_wake_time, 10);
      imu_get_data(&mpu6500_real_data, &ist8310_real_data);
      MotionMC_get_input(&mag_cal_in, &ist8310_real_data);
      MotionMC_Update(&mag_cal_in);
      MotionMC_GetCalParams(&imu_cal.mag_cal);
      if (imu_cal.mag_cal.CalQuality == MMC_CALQSTATUSGOOD) {   
        if(xTaskGetTickCount()>=200000){
          INS_task_status.imu_calibrated = 1;
          MotionFX_getGbias((float *)&imu_cal.gbias);
          save_cal_data(&imu_cal);
          MotionMC_Initialize(0, 0);
        }
      }
      __HAL_TIM_SET_COMPARE(
          &htim3, TIM_CHANNEL_2,
          controllerUpdate((Controller *)&temperature_pid, &imu_ctrl_tempretrue, &mpu6500_real_data.temp, NULL));
      last_wake_time = xTaskGetTickCount();
    }
    MotionFX_get_input(&motionFX_input, &mpu6500_real_data, &ist8310_real_data);
    delta_time = stopwatch_disable(&stopwatch);
    tic(&stopwatch);
    MotionFX_propagate(&motionFX_output, &motionFX_input, &delta_time);
    MotionFX_update(&motionFX_output, &motionFX_input, &delta_time, NULL);
  }
}

char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data) { return 1; }

char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data) { return 1; }

char MotionMC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data) { return 1; }

char MotionMC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data) { return 1; }
