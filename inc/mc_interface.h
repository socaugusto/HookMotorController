/**
  ******************************************************************************
  * @file    mc_interface.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains all definitions and functions prototypes for the
  *          MC Interface component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCInterface
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_INTERFACE_H
#define __MC_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "state_machine.h"
#include "6step_core.h"
/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCInterface
  * @{
  */
/** @defgroup MC_INTERFACE_Exported_Functions_Prototypes
  * @{
  */
uint32_t MC_Core_GetSpeed(MC_Handle_t* pMc);
MC_Status_t MC_Core_GetStatus(MC_Handle_t* pMc);
MC_FuncStatus_t MC_Core_SetDirection(MC_Handle_t* pMc, uint32_t DirectionToSet);
MC_FuncStatus_t MC_Core_SetSpeed(MC_Handle_t *pMc, uint32_t SpeedToSet);
MC_FuncStatus_t MC_Core_SetStartupDutyCycle(MC_Handle_t* pMc, uint32_t DutyCycleToSet);
MC_FuncStatus_t MC_Core_Start(MC_Handle_t *pMc);
MC_FuncStatus_t MC_Core_Stop(MC_Handle_t *pMc);
uint32_t MC_Core_GetGateDriverPwmFreq(MC_Handle_t* pMc);
void MC_Core_Error(MC_Handle_t *pMc);
uint32_t MC_Core_GetFaultState( MC_Handle_t * pMc );
uint8_t MC_Core_GetState( MC_Handle_t * pMc );
/**
  * @} end MC_INTERFACE_Exported_Functions_Prototypes
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __MC_INTERFACE_H */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/

