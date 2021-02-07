/**
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  *                                 ___     ____   ______ _____
  *                                /   |   / __ \ / ____// ___/
  *                               / /| |  / /_/ // __/   \__ \
  *                              / ___ | / _, _// /___  ___/ / 
  *                             /_/  |_|/_/ |_|/_____/ /____/ 
  *                        Association of Robotics Engineers at SUSTech
  * 
  * @file     mpu6500driver.c
  * @author   Zou Yuanhao (11810102@mail.sustech.edu.cn)
  * @brief    MPU6500驱动
  * @version  0.1
  * @date     2020-12-06
  * 
  * ****************************(C) COPYRIGHT 2020 ARES@SUSTech****************************
  */

#include "imu.h"
#include "imu_comm.h"
#include "mpu6500reg.h"
#include "ist8310reg.h"

#define IST8310_WHO_AM_I_VALUE 0x10
#define DEVICE_ID MPU6500_ID

#ifndef NULL
#define NULL 0
#endif

#define MAG_SEN 0.006f //转换成 uT/50

//  陀螺仪原始数据转换成dps 陀螺仪范围可以在h文件中修改
#ifdef MPU6500_GYRO_RANGE_2000
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_2000
#define GYRO_SEN 0.06103515625f
#elif defined(MPU6500_GYRO_RANGE_1000)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_1000
#define GYRO_SEN 0.030517578125f
#elif defined(MPU6500_GYRO_RANGE_500)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_500
#define GYRO_SEN 0.0152587890625f
#elif defined(MPU6500_GYRO_RANGE_250)
#define MPU_GYRO_RANGLE MPU_GYRO_RANGLE_250
#define GYRO_SEN 0.00762939453125f
#else
#error "Please set the right range of gyro (2000 , 1000, 500 or 250)"
#endif

//  加速度计原始数据转换成g 加速度计范围可以在h文件中修改
#ifdef MPU6500_ACCEL_RANGE_2G
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_2G
#define ACCEL_SEN 0.00006103515625f
#elif defined(MPU6500_ACCEL_RANGE_4G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_4G
#define ACCEL_SEN 0.0001220703125f
#elif defined(MPU6500_ACCEL_RANGE_8G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_8G
#define ACCEL_SEN 0.000244140625f
#elif defined(MPU6500_ACCEL_RANGE_16G)
#define MPU_ACCEL_RANGLE MPU_ACCEL_RANGLE_16G
#define ACCEL_SEN 0.00048828125f
#else
#error "Please set the right range of accel (16G , 8G, 4G or 2G)"
#endif

// 0x01 means 4 mg, 0xFF means 1020mg, set 0x90 means 144mg
#define WOM_THR_SET 0x0F

#define MPU6500_TEMPERATURE_FACTOR 0.002f
#define MPU6500_TEMPERATURE_OFFSET 23.0f

#define MPU_INIT_REG_NUM 14
#define IST_INIT_REG_NUM 4

static const uint8_t mpu_init_reg_data_error[MPU_INIT_REG_NUM][3] = {
    {MPU_PWR_MGMT_1,
     (((~MPU_DEVICE_RESET) & (~MPU_SLEEP) & (~MPU_CYCLE) & (~MPU_GYRO_STANDBY) & (~MPU_TEMP_DISABLE)) & (MPU_CLKSEL_INTERNAL)),
     PWR_MGMT_1_ERROR},

    {MPU_PWR_MGMT_2,
     (((~MPU_DISABLE_XA) & (~MPU_DISABLE_YA) & (~MPU_DISABLE_ZA) & (~MPU_DISABLE_XG) & (~MPU_DISABLE_YG) & (~MPU_DISABLE_ZG)) &
      (MPU_LP_WAKE_1_25_HZ)),
     PWR_MGMT_2_ERROR},

    {MPU_SMPLRT_DIV, MPU_SMPLRT_DIV_1, SMPLRT_DIV_ERROR},

    {MPU_CONFIG, ((~MPU_FIFO_MODE_OFF_REPLACE_OLD_DATA) & (MPU_EXT_SYNC_DISABLE | MPU_DLPF_CFG_1_SET)), CONFIG_ERROR},

    {MPU_GYRO_CONFIG, (((~MPU_XG_SELF_TEST_SET) & (~MPU_YG_SELF_TEST_SET) & (~MPU_ZG_SELF_TEST_SET)) & (MPU_GYRO_RANGLE)),
     GYRO_CONFIG_ERROR},

    {MPU_ACCEL_CONFIG, (((~MPU_XA_SELF_TEST_SET) & (~MPU_YA_SELF_TEST_SET) & (~MPU_ZA_SELF_TEST_SET)) & (MPU_ACCEL_RANGLE)),
     ACCEL_CONFIG_ERROR},

    {MPU_ACCEL_CONFIG_2, (MPU_ACCEL_FCHOICE_B_0_SET | MPU_A_DLPL_CFG_2_SET), ACCEL_CONFIG_2_ERROR},

    {MPU_WOM_THR, WOM_THR_SET, MOT_DETECT_CTRL_ERROR},

    {MPU_I2C_MST_CTRL,
     (((~MPU_MULT_MST_EN) & (~MPU_SLV_3_FIFO_EN) & (~MPU_I2C_MST_P_NSR)) & (MPU_WAIT_FOR_ES_EN) & (MPU_I2C_MST_CLK_400_KHZ)),
     I2C_MST_CTRL_ERROR},

    {MPU_INTBP_CFG,
     ((~MPU_INTBP_ACTL) & (~MPU_INTBP_OPEN) & (~MPU_LATCH_INT_EN) & (MPU_INT_ANYRD_2CLEAR) & (~MPU_ACTL_FSYNC) &
      (~MPU_FSYNC_INT_MODE_EN)) &
         (MPU_BYPASS_EN),
     INTBP_CFG_ERROR},

    {MPU_INT_ENABLE, (((~MPU_FIFO_OVERFLOW_EN) & (~MPU_FSYNC_INT_EN)) & (MPU_RAW_RDY_EN)), INT_ENABLE_ERROR},

    {MPU_I2C_MST_DELAY_CTRL, MPU_I2C_SLV0_DLY_EN, I2C_MST_DELAY_CTRL_ERROR},

    {MPU_MOT_DETECT_CTRL, MPU_ACCEL_INTEL_EN | MPU_ACCEL_INTEL_MODE_COMPARE, MOT_DETECT_CTRL_ERROR},

    {MPU_USER_CTRL,
     (((~MPU_DMP_EN) & (~MPU_FIFO_MODE_EN) & (~MPU_DMP_RST) & (~MPU_FIFO_RST) & (~MPU_I2C_MST_RST) & (~MPU_SIG_COND_RST)) &
      (MPU_I2C_MST_EN | MPU_I2C_IF_DIS)),
     USER_CTRL_ERROR},

};
static const uint8_t ist8310_write_reg_data_error[IST_INIT_REG_NUM][3] = {
    {0x0B, 0x08, 0x01}, {0x41, 0x09, 0x02}, {0x42, 0xC0, 0x03}, {0x0A, 0x0B, 0x03}};

uint8_t mpu_init(void) {
  uint8_t res       = 0;
  uint8_t wait_time = 1;
  uint8_t sleepTime = 50;

  uint8_t writeNum = 0;

  //check commiunication is normal
  mpu_read_reg(MPU_WHO_AM_I);
  HAL_Delay(wait_time);
  mpu_read_reg(MPU_WHO_AM_I);
  HAL_Delay(wait_time);

  mpu_write_reg(MPU_PWR_MGMT_1, MPU_DEVICE_RESET);
  HAL_Delay(sleepTime);

  //check commiunication is normal after reset
  mpu_read_reg(MPU_WHO_AM_I);
  HAL_Delay(wait_time);
  mpu_read_reg(MPU_WHO_AM_I);
  HAL_Delay(wait_time);

  //read the register "WHO AM I"
  res = mpu_read_reg(MPU_WHO_AM_I);
  HAL_Delay(wait_time);
  if (res != DEVICE_ID) {
    return NO_Sensor;
  }

  //set mpu6500 sonsor config and check
  for (writeNum = 0; writeNum < MPU_INIT_REG_NUM; writeNum++) {
    mpu_write_reg(mpu_init_reg_data_error[writeNum][0], mpu_init_reg_data_error[writeNum][1]);
    HAL_Delay(wait_time);
    res = mpu_read_reg(mpu_init_reg_data_error[writeNum][0]);
    HAL_Delay(wait_time);
    if (res != mpu_init_reg_data_error[writeNum][1]) {
      return mpu_init_reg_data_error[writeNum][2];
    }
  }
  // NO error
  return MPU6500_NO_ERROR;
}

uint8_t ist_init(void) {
  static const uint8_t wait_time = 2;
  static const uint8_t sleepTime = 50;
  uint8_t              res       = 0;
  uint8_t              writeNum  = 0;

  IST_RST_L();
  HAL_Delay(sleepTime);
  HAL_Delay(sleepTime);
  IST_RST_H();

  res = ist_read_reg(IST8310_WHO_AM_I);
  if (res != IST8310_WHO_AM_I_VALUE) {
    return IST8310_NO_SENSOR;
  }

  //set mpu6500 sonsor config and check
  for (writeNum = 0; writeNum < IST_INIT_REG_NUM; writeNum++) {
    ist_write_reg(ist8310_write_reg_data_error[writeNum][0], ist8310_write_reg_data_error[writeNum][1]);
    HAL_Delay(wait_time);
    res = ist_read_reg(ist8310_write_reg_data_error[writeNum][0]);
    HAL_Delay(wait_time);
    if (res != ist8310_write_reg_data_error[writeNum][1]) {
      return ist8310_write_reg_data_error[writeNum][2];
    }
  }
  ist_auto_comm();

  return IST8310_NO_ERROR;
}

void mpu_get_data(uint8_t *status_buf, mpu_real_data_t *mpu6500_real_data) {
  // check point null
  if (status_buf == NULL || mpu6500_real_data == NULL) {
    return;
  }

  if ((*status_buf) & MPU_INT_WOM_INT) {
    mpu6500_real_data->status |= (uint8_t)(1 << MPU_MOT_BIT);
  }
  if (mpu6500_real_data->status & (1 << MPU_MOT_BIT)) {
    static uint8_t motion_time = 0;
    motion_time++;
    if (motion_time > 10) {
      motion_time = 0;
      mpu6500_real_data->status &= ~(1 << MPU_MOT_BIT);
    }
  }

  if ((*status_buf) & MPU_RAW_RDY_INT) {
    int16_t temp_imu_data = 0;
    mpu6500_real_data->status |= (1 << MPU_DATA_READY_BIT);

    temp_imu_data               = (int16_t)((status_buf[1]) << 8) | status_buf[2];
    mpu6500_real_data->accel[0] = temp_imu_data * ACCEL_SEN;
    temp_imu_data               = (int16_t)((status_buf[3]) << 8) | status_buf[4];
    mpu6500_real_data->accel[1] = temp_imu_data * ACCEL_SEN;
    temp_imu_data               = (int16_t)((status_buf[5]) << 8) | status_buf[6];
    mpu6500_real_data->accel[2] = temp_imu_data * ACCEL_SEN;

    temp_imu_data           = (int16_t)((status_buf[7]) << 8) | status_buf[8];
    mpu6500_real_data->temp = temp_imu_data * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;

    temp_imu_data              = (int16_t)((status_buf[9]) << 8) | status_buf[10];
    mpu6500_real_data->gyro[0] = temp_imu_data * GYRO_SEN;
    temp_imu_data              = (int16_t)((status_buf[11]) << 8) | status_buf[12];
    mpu6500_real_data->gyro[1] = temp_imu_data * GYRO_SEN;
    temp_imu_data              = (int16_t)((status_buf[13]) << 8) | status_buf[14];
    mpu6500_real_data->gyro[2] = temp_imu_data * GYRO_SEN;
  }
}

void ist_get_data(uint8_t *status_buf, ist_real_data_t *ist8310_real_data) {
  if (status_buf[0] & 0x01) {
    int16_t temp_ist8310_data = 0;
    ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

    temp_ist8310_data         = (int16_t)((status_buf[2] << 8) | status_buf[1]);
    ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data         = (int16_t)((status_buf[4] << 8) | status_buf[3]);
    ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data         = (int16_t)((status_buf[6] << 8) | status_buf[5]);
    ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
  } else {
    ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
  }
}

void mpu_read_gyro(fp32 gyro[3]) {
  uint8_t buf[6];
  int16_t temp_imu_data = 0;
  mpu_read_regs(MPU_GYRO_XOUT_H, buf, 6);

  temp_imu_data = (int16_t)((buf[0]) << 8) | buf[1];
  gyro[0]       = temp_imu_data * GYRO_SEN;
  temp_imu_data = (int16_t)((buf[2]) << 8) | buf[3];
  gyro[1]       = temp_imu_data * GYRO_SEN;
  temp_imu_data = (int16_t)((buf[4]) << 8) | buf[5];
  gyro[2]       = temp_imu_data * GYRO_SEN;
}

void mpu_read_accel(fp32 accel[3]) {
  uint8_t buf[6];
  int16_t temp_imu_data = 0;
  mpu_read_regs(MPU_ACCEL_XOUT_H, buf, 6);

  temp_imu_data = (int16_t)((buf[0]) << 8) | buf[1];
  accel[0]      = temp_imu_data * ACCEL_SEN;
  temp_imu_data = (int16_t)((buf[2]) << 8) | buf[3];
  accel[1]      = temp_imu_data * ACCEL_SEN;
  temp_imu_data = (int16_t)((buf[4]) << 8) | buf[5];
  accel[2]      = temp_imu_data * ACCEL_SEN;
}

void mpu_read_temp(fp32 *temperature) {
  uint8_t buf[2];
  int16_t temp_imu_data = 0;
  mpu_read_regs(MPU_TEMP_OUT_H, buf, 2);

  temp_imu_data = (int16_t)((buf[0]) << 8) | buf[1];

  *temperature = temp_imu_data * MPU6500_TEMPERATURE_FACTOR + MPU6500_TEMPERATURE_OFFSET;
}

void ist_read_mag(fp32 mag[3]) {
  uint8_t buf[6];
  int16_t temp_ist8310_data = 0;
  ist_read_regs(0x02, buf, 6);

  temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
  mag[0]            = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
  mag[1]            = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
  mag[2]            = MAG_SEN * temp_ist8310_data;
}
