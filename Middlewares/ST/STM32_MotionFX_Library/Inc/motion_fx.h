/**
  ******************************************************************************
  * @file    motion_fx.h
  * @author  MEMS Application Team
  * @version V2.4.1
  * @date    10-February-2020
  * @brief   Header for motion_fx module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ********************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_FX_H_
#define _MOTION_FX_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_FX MOTION_FX
  * @{
  */


/** @defgroup MOTION_FX_Exported_Types MOTION_FX_Exported_Types
 * @{
 */
/* Exported constants --------------------------------------------------------*/
#define MFX_NUM_AXES    3
#define MFX_QNUM_AXES   4

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MFX_ENGINE_DISABLE = 0,
  MFX_ENGINE_ENABLE = 1
} MFX_engine_state_t;

typedef enum
{
  MFX_ENGINE_OUTPUT_NED = 0,
  MFX_ENGINE_OUTPUT_ENU = 1
} MFX_engine_output_ref_sys;

typedef struct
{
  float ATime;                              /* merge rate to the accel */
  float MTime;                              /* merge rate to the mag */
  float FrTime;                             /* merge rate to the accel when external accelerations occours */
  unsigned char LMode;                      /* gyro bias learn mode: 1-static learning, 2-dynamic learning */
  float gbias_mag_th_sc_6X;                 /* 6 axes scaler for the gyro bias mag threshold nominal */
  float gbias_acc_th_sc_6X;                 /* 6 axes scaler for the gyro bias acc threshold nominal */
  float gbias_gyro_th_sc_6X;                /* 6 axes scaler for the gyro bias gyro threshold nominal */
  float gbias_mag_th_sc_9X;                 /* 9 axes scaler for the gyro bias mag threshold nominal */
  float gbias_acc_th_sc_9X;                 /* 9 axes scaler for the gyro bias acc threshold nominal */
  float gbias_gyro_th_sc_9X;                /* 9 axes scaler for the gyro bias gyro threshold nominal */
  unsigned char modx;                       /* setting to indicate the decimation, set to 1 in smartphone/tablet, set to >=1 in embedded solutions */
  char acc_orientation[MFX_QNUM_AXES];      /* accelerometer data orientation */
  char gyro_orientation[MFX_QNUM_AXES];     /* gyroscope data orientation */
  char mag_orientation[MFX_QNUM_AXES];      /* magnetometer data orientation */
  MFX_engine_output_ref_sys output_type;    /* 0: NED, 1: ENU */
  int start_automatic_gbias_calculation;
} MFX_knobs_t;

typedef struct
{
  float mag[MFX_NUM_AXES];                  /* Calibrated mag [uT]/50 */
  float acc[MFX_NUM_AXES];                  /* Acceleration in [g] */
  float gyro[MFX_NUM_AXES];                 /* Angular rate [dps] */
} MFX_input_t;

typedef struct
{
  float rotation_9X[MFX_NUM_AXES];            /* 9 axes yaw, pitch and roll */
  float quaternion_9X[MFX_QNUM_AXES];         /* 9 axes quaternion */
  float gravity_9X[MFX_NUM_AXES];             /* 9 axes device frame gravity */
  float linear_acceleration_9X[MFX_NUM_AXES]; /* 9 axes device frame linear acceleration */
  float heading_9X;                           /* 9 axes heading */
  float headingErr_9X;                        /* 9 axes heading error in deg */
  float rotation_6X[MFX_NUM_AXES];            /* 6 axes yaw, pitch and roll */
  float quaternion_6X[MFX_QNUM_AXES];         /* 6 axes quaternion */
  float gravity_6X[MFX_NUM_AXES];             /* 6 axes device frame gravity */
  float linear_acceleration_6X[MFX_NUM_AXES]; /* 6 axes device frame linear acceleration */
  float heading_6X;                           /* 6 axes heading */
  float headingErr_6X;                        /* 6 axes heading error in deg */
} MFX_output_t;

typedef enum
{
  MFX_MAGCALUNKNOWN = 0,
  MFX_MAGCALPOOR,
  MFX_MAGCALOK,
  MFX_MAGCALGOOD
} MFX_MagCal_quality_t;

typedef struct {
  float mag[MFX_NUM_AXES];                  /* Uncalibrated mag [uT]/50 */
  int time_stamp;                           /* Timestamp [ms] */
} MFX_MagCal_input_t;

typedef struct {
  float hi_bias[3];                         /* Hard iron offset array [uT]/50 */
  MFX_MagCal_quality_t cal_quality;         /* Calibration quality factor */
} MFX_MagCal_output_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_FX_Exported_Functions MOTION_FX_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionFX engine
 * @param  none
 * @retval none
 */
void MotionFX_initialize(void);

/**
 * @brief  Set the internal knobs
 * @param  knobs knobs structure
 * @retval None
 */
void MotionFX_setKnobs(MFX_knobs_t *knobs);

/**
 * @brief  Get the current internal knobs
 * @param  knobs knobs structure
 * @retval None
 */
void MotionFX_getKnobs(MFX_knobs_t *knobs);

/**
 * @brief  Get the status of the 6 axes library
 * @retval 1 if enabled, 0 if disabled
 */
MFX_engine_state_t MotionFX_getStatus_6X(void);

/**
 * @brief  Get the status of the 9 axes library
 * @retval 1 if enabled, 0 if disabled
 */
MFX_engine_state_t MotionFX_getStatus_9X(void);

/**
 * @brief  Enable or disable the 6 axes function (ACC + GYRO)
 * @param  enable 1 to enable, 0 to disable
 * @retval none
 */
void MotionFX_enable_6X(MFX_engine_state_t enable);

/**
 * @brief  Enable or disable the 9 axes function (ACC + GYRO + MAG)
 * @param  enable 1 to enable, 0 to disable
 * @retval none
 */
void MotionFX_enable_9X(MFX_engine_state_t enable);

/**
 * @brief  Set the initial gbias
 * @param  gbias pointer to a float array containing the 3 gbias values
 * @retval none
 */
void MotionFX_setGbias(float *gbias);

/**
 * @brief  Get the initial gbias
 * @param  pointer to a float array containing the 3 gbias values
 * @retval none
 */
void MotionFX_getGbias(float *gbias);

/**
 * @brief  Run the Kalman filter update
 * @param  data_out pointer to the MFX_output_t structure
 * @param  data_in pointer to the MFX_input_t structure
 * @param  eml_deltatime delta time between two propagate calls [sec]
 * @param  eml_q_update set to NULL
 * @retval none
 */
void MotionFX_update(MFX_output_t *data_out, MFX_input_t *data_in, float *eml_deltatime, float *eml_q_update);

/**
 * @brief  Run the Kalman filter propagate
 * @param  data_out pointer to the MFX_output_t structure
 * @param  data_in pointer to the MFX_input_t structure
 * @param  eml_deltatime delta time between two propagate calls [sec]
 * @retval none
 */
void MotionFX_propagate(MFX_output_t *data_out, MFX_input_t *data_in, float *eml_deltatime);

/**
 * @brief  Initialize magnetic calibration library
 * @param  sampletime  period in milliseconds [ms] between the update function call
 * @param  enable  enable (1) or disable (0) library
 * @retval none
 */
void MotionFX_MagCal_init(int sampletime, unsigned short int enable);

/**
 * @brief  Run magnetic calibration algorithm
 * @param  data_in  structure containing input data
 * @retval none
 */
void MotionFX_MagCal_run(MFX_MagCal_input_t *data_in);

/**
 * @brief  Get magnetic calibration parameters
 * @param  data_out  structure containing output data
 * @retval none
 */
void MotionFX_MagCal_getParams(MFX_MagCal_output_t *data_out);

/**
 * @brief  Get the library version
 * @param  version pointer to an array of 35 char
 * @retval Number of characters in the version string
 */
uint8_t MotionFX_GetLibVersion(char *version);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* _MOTION_FX_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
