/**
  ******************************************************************************
  * @file    6step_SetFunctions.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides service firmware function used to prepare 
  *          the handle to the NextStep application
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
  */

/* Includes ------------------------------------------------------------------*/

#include "6step_SetFunctions_3pwm_6pwm.h"



extern void MC_Core_EnableChannelsHfPwmsAll(uint32_t *pHfTimer);
extern void MC_Core_GenerateComEvent(uint32_t *pHfTimer);
extern void MC_Core_StartHfPwms(uint32_t *pHfTimer, MC_EN_DRIVER_t enable);
extern MC_FuncStatus_t MC_Core_SetHFPWM(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);

inline void MC_Core_SetPWM_Enable(MC_Handle_t* pMc)
  {
    MC_Core_EnableChannelsHfPwmsAll(pMc->phf_timer);
    MC_Core_StartHfPwms(pMc->phf_timer, pMc->enDRIVER);    
    MC_Core_GenerateComEvent(pMc->phf_timer);
  }
  
inline MC_FuncStatus_t MC_Core_SetHFPWM_3pwm(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
  {
    return MC_FUNC_OK;
  }

inline MC_FuncStatus_t MC_Core_SetHFPWM_6pwm(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
  {
    return MC_Core_SetHFPWM(pMc, pMc->hf_timer_pulse_value_max, pMc->step_pos_next);
  }

