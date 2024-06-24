/**
 ******************************************************************************
 * @file    6step_user_config.h
 * @author  IPC Agrate
 * @brief   Header file for 6step_pwm_interface.c file
  ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __6STEP_USER_CONFIG_H
#define __6STEP_USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "6step_core.h"
  

MC_FuncStatus_t MC_Core_ConfigureUserAdc(MC_Handle_t* pMc, uint32_t *pTrigTimer, uint16_t TrigTimerChannel, uint8_t NumberOfUserChannels);
MC_FuncStatus_t MC_Core_ConfigureUserAdcChannel(MC_Handle_t* pMc, uint32_t* pAdc, uint32_t AdcChannel, uint32_t SamplingTime, MC_UserMeasurements_t UserMeasurement);
MC_FuncStatus_t MC_Core_ConfigureUserButton(MC_Handle_t* pMc, uint16_t ButtonPin, uint16_t ButtonDebounceTimeMs);
MC_FuncStatus_t MC_Core_AssignUserMeasurementToSpeedDutyCycleCommand(MC_Handle_t* pMc, MC_UserMeasurements_t UserMeasurement);
uint32_t MC_Core_GetCommandFromAdcMeasurement(MC_Handle_t *pMc, uint32_t CommandMin, uint32_t CommandMax);
MC_FuncStatus_t MC_Core_InitAdc(MC_Handle_t *pMc);
MC_FuncStatus_t MC_Core_SetAdcUserTrigTime(MC_Handle_t* pMc, uint32_t DutyCycleToSet);

#ifdef __cplusplus
}
#endif

#endif /* __6STEP_USER_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/