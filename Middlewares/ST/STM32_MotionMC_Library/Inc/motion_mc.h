/**
  ******************************************************************************
  * @file    motion_mc.h
  * @author  MEMS Application Team
  * @version V2.3.1
  * @date    19-June-2020
  * @brief   Header for motion_mc module
  ******************************************************************************
  * @attention
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
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTION_MC_H_
#define _MOTION_MC_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/** @addtogroup MIDDLEWARES
  * @{
  */

/** @defgroup MOTION_MC MOTION_MC
  * @{
  */

/** @defgroup MOTION_MC_Exported_Types MOTION_MC_Exported_Types
  * @{
  */
/* Exported types ------------------------------------------------------------*/

typedef enum
{
  MMC_CALQSTATUSUNKNOWN = 0,
  MMC_CALQSTATUSPOOR,
  MMC_CALQSTATUSOK,
  MMC_CALQSTATUSGOOD
} MMC_CalQuality_t;

typedef struct {
  float Mag[3];  /* Magnetometer sensor output [uT] */
  int TimeStamp; /* Timestamp [ms] */
} MMC_Input_t;

typedef struct {
  float HI_Bias[3];            /* Hard iron offset array [uT] */
  float SF_Matrix[3][3];       /* Scale factor correction matrix [-] */
  MMC_CalQuality_t CalQuality; /* Calibration quality factor */
} MMC_Output_t;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/** @defgroup MOTION_MC_Exported_Functions MOTION_MC_Exported_Functions
 * @{
 */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Initialize the MotionMC engine
  * @param  sampletime  period in milliseconds [ms] between the update function call
  * @param  enable  Enable (1) or disable (0) library
  * @retval none
  */
void MotionMC_Initialize(int sampletime, unsigned short int enable);

/**
  * @brief  Run Magnetic Calibration algorithm
  * @param  data_in  Structure containing input data
  * @retval none
  */
void MotionMC_Update(MMC_Input_t *data_in);

/**
  * @brief  Get the magnetic compensation for hard/soft iron
  * @param  data_out  Structure containing output data
  * @retval none
  */
void MotionMC_GetCalParams(MMC_Output_t *data_out);

/**
  * @brief  Get the library version
  * @param  version  pointer to an array of 35 char
  * @retval Number of characters in the version string
  */
uint8_t MotionMC_GetLibVersion(char *version);

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

#endif /* _MOTION_MC_H_ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
