#ifndef IMU_H
#define IMU_H
#include "main.h"
#define MPU6500

#define GYRO_OFFSET_KP 0.0003f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大

#define MPU_DATA_READY_BIT 0 //陀螺仪数据准备
#define MPU_MOT_BIT 1        //mpu6500 运动检测
#define IST8310_DATA_READY_BIT 2
#define SPI_BUF_SIZE 23
#define MPU_DATA_OFFSET 1
#define IST_DATA_OFFSET 16

typedef enum {
  MPU6500_NO_ERROR         = 0x00,
  NO_Sensor                = 0x80,
  PWR_MGMT_1_ERROR         = 0x01,
  PWR_MGMT_2_ERROR         = 0x02,
  SMPLRT_DIV_ERROR         = 0x03,
  CONFIG_ERROR             = 0x04,
  INTBP_CFG_ERROR          = 0x05,
  GYRO_CONFIG_ERROR        = 0x06,
  ACCEL_CONFIG_ERROR       = 0x07,
  ACCEL_CONFIG_2_ERROR     = 0x08,
  I2C_MST_CTRL_ERROR       = 0x09,
  USER_CTRL_ERROR          = 0x0A,
  INT_ENABLE_ERROR         = 0x0B,
  I2C_MST_DELAY_CTRL_ERROR = 0x0C,
  MOT_DETECT_CTRL_ERROR    = 0x0D,
  WOM_THR_ERROR            = 0x0E
} mpu_errno;

#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

//以下定义设置mpu6500 数据范围，需要用哪个范围，取消注释，将其他范围注释掉
//#define MPU6500_ACCEL_RANGE_16G
//#define MPU6500_ACCEL_RANGE_8G
#define MPU6500_ACCEL_RANGE_4G
//#define MPU6500_ACCEL_RANGE_2G

//以下定义设置mpu6500 数据范围，需要用哪个范围，取消注释，将其他范围注释掉
//#define MPU6500_GYRO_RANGE_2000
//#define MPU6500_GYRO_RANGE_1000
#define MPU6500_GYRO_RANGE_500
//#define MPU6500_GYRO_RANGE_250

//陀螺仪数据结构体
typedef struct mpu_real_data_t {
  uint8_t status;
  fp32    accel[3];
  fp32    temp;
  fp32    gyro[3];
} mpu_real_data_t;

typedef struct ist_real_data_t {
  uint8_t status;
  fp32    mag[3];
} ist_real_data_t;

extern uint8_t SPI_RX_buf[SPI_BUF_SIZE];
extern uint8_t SPI_TX_buf[SPI_BUF_SIZE];

extern uint8_t mpu_init(void);
extern void    imu_get_data(mpu_real_data_t *mpu6500_real_data, ist_real_data_t *ist8310_real_data);
extern uint8_t ist_init(void);
extern void    ist_get_data(uint8_t *status_buf, ist_real_data_t *mpu6500_real_data);
extern void    ist_read_mag(fp32 mag[3]);
extern void    imu_DMA_read(void);
#endif
