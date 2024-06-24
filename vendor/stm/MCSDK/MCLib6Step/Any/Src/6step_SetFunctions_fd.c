/**
  ******************************************************************************
  * @file    6step_SetFunctions.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware function used to prepare 
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

#include "6step_SetFunctions_std_fd_qs.h"


extern MC_FuncStatus_t MC_Core_SixStepTable(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
extern MC_FuncStatus_t MC_Core_SixStepTableFastDemag(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);

extern void MC_Core_ResetPolarityHfPwm(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
extern void MC_Core_EnableUpdateEvent(uint32_t *pHfTimer);


inline MC_FuncStatus_t MC_Core_SetHFPWM(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
  {
  if (pMc->bemf.demagn_value == pMc->bemf.const_bemf_demagn_counter)
  {
    return MC_Core_SixStepTable(pMc, PulseValue, StepNumber);
  }
  else
  {
    return MC_Core_SixStepTableFastDemag(pMc, PulseValue, StepNumber);
  }  
  
  }


inline void MC_Core_SetEnableUpdateEvent(uint32_t *pHfTimer)
  {
  MC_Core_EnableUpdateEvent(pHfTimer);
  }
  
inline void MC_Core_SetResetPolarityHfPwm(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity)
 {
    MC_Core_ResetPolarityHfPwm(pHfTimer, ch_polarity, Nch_polarity);
 }
  

