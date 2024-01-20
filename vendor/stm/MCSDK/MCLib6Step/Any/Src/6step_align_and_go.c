/**
  ******************************************************************************
  * @file    6step_align_and_go.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides functions for alignment process 
  *
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

#include "6step_align_and_go.h"

extern void MC_Core_StartAdcIt(uint32_t *pAdc);
extern void MC_Core_DisableIrq(void);
extern void MC_Core_EnableIrq(void);
extern void MC_Core_EnableItBreak(uint32_t *pTimer);
extern void MC_Core_StartLfTimerIt(uint32_t *pLfTimer);
extern void MC_Core_SetPrescalerTimer(uint32_t *pTimer, uint16_t PrescalerValue);
extern void MC_Core_SetPeriodTimer(uint32_t *pTimer, uint16_t PeriodValue);
extern void MC_Core_RampTuning(MC_Handle_t *pMc, uint16_t speed_incr);
extern void MC_Core_SetPWM_Enable(MC_Handle_t* pMc);

extern uint32_t SysClockFrequency;


extern MC_FuncStatus_t MC_Core_SixStepTableMidAlign(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);


/**
  * @brief  MC_Core_Alignment
  * @param[in] pMc motor control handle
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_Alignment(MC_Handle_t *pMc)
{  
/* ALIGNMENT MIDWAY BEGIN 2 */
  uint8_t alignment_step = pMc->const_alignment_step + pMc->const_startup_direction;
  if (alignment_step == 7) 
  {
    alignment_step = 1;
  }    
  if ((pMc->align_index) == 0)
  {
    MC_Core_DisableIrq();
    MC_Core_SixStepTableMidAlign(pMc, pMc->hf_timer_pulse_value_max, alignment_step);
    MC_Core_SetPWM_Enable(pMc);    //inline fuction to differentiate between 3pwm & 6pwm
    MC_Core_EnableIrq();
  }
  if ((pMc->align_index) == 1)
  {
    MC_Core_EnableItBreak(pMc->phf_timer);
  }  
  (pMc->align_index)++;
  if((pMc->align_index) >= (pMc->alignment_time))
  {
    pMc->status = MC_STARTUP;
    MC_Core_RampCalc(pMc);
    MC_Core_StartLfTimerIt(pMc->plf_timer);
    pMc->align_index = 0;
  }
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_RampCalc
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_RampCalc(MC_Handle_t* pMc)
{
  if (pMc->speed_fdbk_filtered == 0)
  {
    int16_t lf_prescaler_value;
    lf_prescaler_value = pMc->bemf.const_run_lf_timer_prescaler;
    pMc->lf_timer_prescaler = lf_prescaler_value;
    MC_Core_SetPrescalerTimer(pMc->plf_timer, lf_prescaler_value);
    pMc->speed_fdbk_filtered += pMc->acceleration;
    pMc->lf_timer_period = (10*SysClockFrequency)/((uint32_t) (pMc->motor_charac.pole_pairs*pMc->speed_fdbk_filtered*((pMc->lf_timer_prescaler)+1)));
    MC_Core_SetPeriodTimer(pMc->plf_timer, pMc->lf_timer_period);
  }
  else
  {
    if (pMc->speed_fdbk_filtered < pMc->speed_target_command)
    {
      if (pMc->align_index >=  pMc->speed_update)
      {
        MC_Core_RampTuning(pMc, pMc->const_startup_acceleration);
        pMc->speed_fdbk_filtered += pMc->const_startup_acceleration;
        pMc->align_index = 0;
      }
      else  pMc->align_index++;
    }
    else
    {
      pMc->status = MC_VALIDATION;
      MC_Core_StartAdcIt(pMc->bemf.padc);
      for (uint32_t i = 0; i < pMc->adc_user.number_of_channels; i++)
      {
        MC_Core_StartAdcIt(pMc->adc_user.padc[i]);
      }
    }
  }
  return MC_FUNC_OK;
}
