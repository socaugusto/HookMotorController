/**
  ******************************************************************************
  * @file    6step_synch_sl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Functions used for sensorless control 
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

#include "6step_synch_sl.h"

extern uint32_t SysClockFrequency;
extern uint16_t MC_Core_GetTimerPrescaler(uint32_t *pTimer);

/**
  * @brief  MC_Core_LfTimerPeriodFilter
  *         Call this function only during MC_RUN state
  * @retval  output pulse value to set
  */
uint16_t MC_Core_LfTimerPeriodFilter(MC_Handle_t *pMc)
{
  uint32_t period_sum = 0;
  uint16_t period = 0;
  
  pMc->lf_timer_period_array[pMc->lf_timer_period_array_index] = pMc->lf_timer_period;

  if (pMc->lf_timer_period_array_completed == FALSE)
  {  
    for(int16_t i = pMc->lf_timer_period_array_index; i >= 0; i--)
    {
      period_sum = period_sum + pMc->lf_timer_period_array[i];
    }  
    (pMc->lf_timer_period_array_index)++;
    period = (uint16_t) (period_sum / pMc->lf_timer_period_array_index);
    if(pMc->lf_timer_period_array_index >= MC_SPEED_ARRAY_SIZE) 
    {
      pMc->lf_timer_period_array_index = 0;
      pMc->lf_timer_period_array_completed = TRUE;
    }
  }
  else
  {
    for(int16_t i = (MC_SPEED_ARRAY_SIZE - 1); i >= 0; i--)
    {
      period_sum = period_sum + pMc->lf_timer_period_array[i];
    }      
    (pMc->lf_timer_period_array_index)++;
    if(pMc->lf_timer_period_array_index >= MC_SPEED_ARRAY_SIZE)
    {
      pMc->lf_timer_period_array_index = 0;
    }
    period = period_sum >> RUN_SPEED_ARRAY_SHIFT;
  }
  
  return period;
}





/**
  * @brief  MC_Core_SpeedCompute
  *         Call this function only during MC_RUN state
  * @retval  output pulse value to set
  */
uint32_t MC_Core_SpeedCompute(MC_Handle_t *pMc)
{
  uint16_t prescaler = MC_Core_GetTimerPrescaler(pMc->plf_timer);
  uint16_t period = MC_Core_LfTimerPeriodFilter(pMc);
  return ((SysClockFrequency * 10) / ((++period) * (++prescaler) * (pMc->motor_charac.pole_pairs)));
}



  

