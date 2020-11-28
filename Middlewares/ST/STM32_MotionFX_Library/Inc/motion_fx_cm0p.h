/**
  ******************************************************************************
  * @file    motion_fx_cm0p.h
  * @author  MEMS Application Team
  * @version V2.4.1
  * @date    10-February-2020
  * @brief   Header for motion_fx_cm0p module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
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
#ifndef _MOTION_FX_CM0P_H_
#define _MOTION_FX_CM0P_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_FX_CM0P MOTION_FX_CM0P
  * @{
  */


/** @defgroup MOTION_FX_CM0P_Exported_Types MOTION_FX_CM0P_Exported_Types
 * @{
 */
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef enum
{
  MFX_CM0P_ENGINE_DISABLE = 0,
  MFX_CM0P_ENGINE_ENABLE = 1
} MFX_CM0P_engine_state_t;

typedef struct
{
  float mag[3];                  /* Calibrated mag [uT]/50 */
  float acc[3];                  /* Acceleration in [g] */
  float gyro[3];                 /* Angular rate [dps] */
} MFX_CM0P_input_t;

typedef struct
{
  float quaternion_9X[4];         	/* 9 axes quaternion */
  float gravity_9X[3];             	/* 9 axes device frame gravity */
  float linear_acceleration_9X[3]; 	/* 9 axes device frame linear acceleration */
  float rotation_9X[3];            	/* 9 axes yaw, pitch and roll */
  float quaternion_6X[4];         	/* 6 axes quaternion */
  float gravity_6X[3];             	/* 6 axes device frame gravity */
  float linear_acceleration_6X[3]; 	/* 6 axes device frame linear acceleration */
  float rotation_6X[3];            	/* 6 axes yaw, pitch and roll */
} MFX_CM0P_output_t;

typedef enum
{
  MFX_CM0P_MAGCALUNKNOWN = 0,
  MFX_CM0P_MAGCALPOOR,
  MFX_CM0P_MAGCALOK,
  MFX_CM0P_MAGCALGOOD
} MFX_CM0P_MagCal_quality_t;

typedef struct {
  float mag[3];                         /* Uncalibrated mag [uT]/50 */
} MFX_CM0P_MagCal_input_t;

typedef struct {
  float hi_bias[3];                         /* Hard iron offset array [uT]/50 */
  MFX_CM0P_MagCal_quality_t cal_quality;    /* Calibration quality factor */
} MFX_CM0P_MagCal_output_t;

/**
  * @}
  */

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_FX_CM0P_Exported_Functions MOTION_FX_CM0P_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize the MotionFX engine
 * @param  none
 * @retval none
 */
void MotionFX_CM0P_initialize(void);

/**
 * @brief  Set sensor orientation, default orientation is ENU (x - east, y - north, z - up)
 * @param  acc_orientation string with reference to set
 * @param  gyr_orientation string with reference to set
 * @param  mag_orientation string with reference to set
 * @retval none
 */
void MotionFX_CM0P_setOrientation(const char acc_orientation[4], const char gyro_orientation[4], const char mag_orientation[4]);

/**
 * @brief  Get the status of the 6 axes library
 * @param  none
 * @retval 1 if enabled, 0 if disabled
 */
MFX_CM0P_engine_state_t MotionFX_CM0P_getStatus_6X(void);

/**
 * @brief  Get the status of the 9 axes library
 * @param  none
 * @retval 1 if enabled, 0 if disabled
 */
MFX_CM0P_engine_state_t MotionFX_CM0P_getStatus_9X(void);

/**
 * @brief  Get the status of the euler angles calculation
 * @param  none
 * @retval 1 if enabled, 0 if disabled
 */
MFX_CM0P_engine_state_t MotionFX_CM0P_getStatus_euler(void);

/**
 * @brief  Get the status of the gyroscope calibration
 * @param  none
 * @retval 1 if enabled, 0 if disabled
 */
MFX_CM0P_engine_state_t MotionFX_CM0P_getStatus_gbias(void);

/**
 * @brief  Enable or disable the 6 axes function (ACC + GYRO)
 * @param  enable 1 to enable, 0 to disable
 * @retval none
 */
void MotionFX_CM0P_enable_6X(MFX_CM0P_engine_state_t enable);

/**
 * @brief  Enable or disable the 9 axes function (ACC + GYRO + MAG)
 * @param  enable 1 to enable, 0 to disable
 * @retval none
 */
void MotionFX_CM0P_enable_9X(MFX_CM0P_engine_state_t enable);

/**
 * @brief  Enable or disable euler angles calculation
 * @param  1 to enable, 0 to disable
 * @retval none
 */
void MotionFX_CM0P_enable_euler(MFX_CM0P_engine_state_t enable);

/**
 * @brief  Enable or disable gyroscope calibration
 * @param  1 to enable, 0 to disable
 * @retval none
 */
void MotionFX_CM0P_enable_gbias(MFX_CM0P_engine_state_t enable);

/**
 * @brief  Set the initial gbias
 * @param  gbias pointer to a float array containing the 3 gbias values
 * @retval none
 */
void MotionFX_CM0P_setGbias(float *gbias);

/**
 * @brief  Get the initial gbias
 * @param  pointer to a float array containing the 3 gbias values
 * @retval none
 */
void MotionFX_CM0P_getGbias(float *gbias);

/**
 * @brief  This function runs one step of the sensor fusion algorithm
 * @param  data_out pointer to the MFX_CM0P_output structure
 * @param  data_in pointer to the MFX_CM0P_input structure
 * @param  deltatime delta time between two propagate calls [sec]
 * @retval none
 */
void MotionFX_CM0P_update(MFX_CM0P_output_t *data_out, MFX_CM0P_input_t *data_in, float deltatime);

/**
 * @brief  Initialize the compass calibration library
 * @param  sampletime  period in milliseconds [ms] between the update function call
 * @param  enable  enable (1) or disable (0) library
 * @retval none
 */
void MotionFX_CM0P_MagCal_init(int sampletime, unsigned short int enable);

/**
 * @brief  Run magnetic calibration algorithm
 * @param  data_in  structure containing input data
 * @retval none
 */
void MotionFX_CM0P_MagCal_run(MFX_CM0P_MagCal_input_t *data_in);

/**
 * @brief  Get magnetic calibration parameters
 * @param  data_out  structure containing output data
 * @retval none
 */
void MotionFX_CM0P_MagCal_getParams(MFX_CM0P_MagCal_output_t *data_out);

/**
  * @brief  Get the library version
  * @param  version pointer to an array of 35 char
  * @retval Number of characters in the version string
  */
uint8_t MotionFX_CM0P_GetLibVersion(char *version);

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

#endif /* _MOTION_FX_CM0P_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
