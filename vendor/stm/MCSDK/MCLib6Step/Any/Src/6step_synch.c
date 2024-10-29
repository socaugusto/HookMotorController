/**
  ******************************************************************************
  * @file    6step_sync_sl.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware service function used to prepare 
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

#include "6step_synch.h"

extern uint32_t SysClockFrequency;

extern uint32_t MC_Core_SpeedCompute(MC_Handle_t *pMc);
extern uint16_t MC_Core_GetTimerPeriod(uint32_t *pTimer);
extern void MC_Core_SetPeriodTimer(uint32_t *pTimer, uint16_t PeriodValue);
extern void MC_Core_Error(MC_Handle_t *pMc);
extern void MC_Core_GenerateComEvent(uint32_t *pHfTimer);
extern void MC_Core_SelectAdcChannel(uint32_t *pAdcItToBeDisabled, uint32_t *pAdcItToBeEnabled, uint32_t AdcChannel, uint32_t SamplingTime);
extern void MC_Core_SetDutyCyclePwmForAdcTrig(uint32_t *pTimer, uint32_t Channel, uint16_t PulseValue);
extern MC_FuncStatus_t MC_Core_RampCalc(MC_Handle_t* pMc);
extern MC_FuncStatus_t MC_Core_SetHFPWM_3pwm(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
extern MC_FuncStatus_t MC_Core_SetHFPWM_6pwm(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
extern void MC_Core_SetEnableUpdateEvent(uint32_t *pHfTimer);
extern void MC_Core_SetResetPolarityHfPwm(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
extern void MC_Core_SetPWMON(MC_Handle_t* pMc);
extern MC_FuncStatus_t MC_Core_SixStepTable(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
extern void MC_Core_SetDutyCycleRefPwm(uint32_t *pRefTimer, uint16_t PulseValue);
extern void MC_Core_SetDutyCycleHfPwmForStepN(uint32_t *pHfTimer, uint16_t PulseValue, uint8_t StepNumber);
extern void MC_Core_SetTimerCounter(uint32_t *pTimer,  uint32_t Counter);

/**
  * @brief  MC_Core_PrepareNextStep
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_PrepareNextStep(MC_Handle_t* pMc)
{
  (pMc->step_prepared)++;
  pMc->bemf.demagn_counter = pMc->bemf.const_bemf_demagn_counter;

  if (pMc->status == MC_RUN)
  {
    if (pMc->speed_fdbk_filtered > (pMc->bemf.run_speed_thres_demag))
    {
      pMc->bemf.demagn_value = pMc->bemf.const_run_demagn_delay_min;
    }
    else
    {
      pMc->bemf.demagn_value = (pMc->bemf.demagn_coefficient) / (pMc->speed_fdbk_filtered);
    }
  }
  else
  {
    pMc->bemf.demagn_value = pMc->bemf.const_validation_demagn_delay;
  }
  if(pMc->direction == 0)
  {
    if(pMc->step_pos_next == 6)
    {
      pMc->step_pos_next = 1;
    }
    else
    {
      pMc->step_pos_next++;
    }
  }
  else
  {
    if(pMc->step_pos_next <= 1)
    {
      pMc->step_pos_next = 6;
    }
    else
    {
      pMc->step_pos_next--;
    }
  }
  (pMc->adc_user.channel_index)++;
  if (pMc->adc_user.channel_index >= pMc->adc_user.number_of_channels)
  {
    pMc->adc_user.channel_index = 0;
  }
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_HallPrepareNextStep
  * @retval  Function Status
  */

MC_FuncStatus_t MC_Core_HallPrepareNextStep(MC_Handle_t* pMc)
{
  if (pMc->status != MC_ALIGNMENT)
  {
    pMc->step_position = pMc->step_pos_next;
    if (pMc->lf_timer_period == MC_LF_TIMER_MAX_PERIOD)
    {
      (pMc->hall.stay_in_run_lf_cnt)++;
      if ((pMc->hall.stay_in_run_lf_max - pMc->hall.stay_in_run_lf_cnt) == 0)
      {
        pMc->status = MC_LF_TIMER_FAILURE;
        MC_Core_Error(pMc);
        return MC_FUNC_FAIL;
      }
    }
    pMc->speed_fdbk_filtered = MC_Core_SpeedCompute(pMc);
    pMc->lf_timer_period = MC_LF_TIMER_MAX_PERIOD;
    pMc->step_prepared=0;
  }     
  return MC_FUNC_OK;
} 


/**
  * @brief  MC_Core_NextStep
  * @param[in] pMc motor control handle
  * @param[in] HfTimerCounterSnapshot value of the HF timer counter at the beginning of the LF timer callback
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_NextStep(MC_Handle_t *pMc, uint16_t HfTimerCounterSnapshot)
{
  pMc->lf_timer_period = MC_Core_GetTimerPeriod(pMc->plf_timer);
  (pMc->step_change)++;
  switch (pMc->status)
  {
    case MC_RUN:
    {
      if (pMc->lf_timer_period == MC_LF_TIMER_MAX_PERIOD)
      {
        pMc->status = MC_LF_TIMER_FAILURE;
        MC_Core_Error(pMc);
        return MC_FUNC_FAIL;
      }
      else
      {
        pMc->speed_fdbk_filtered = MC_Core_SpeedCompute(pMc);
        MC_Core_SetPeriodTimer(pMc->plf_timer, MC_LF_TIMER_MAX_PERIOD);
      }
    }
    break;
    case MC_STARTUP:
    {
       pMc->lf_timer_period = (10*SysClockFrequency)/((uint32_t) (pMc->motor_charac.pole_pairs*pMc->speed_fdbk_filtered*((pMc->lf_timer_prescaler)+1)));
	   MC_Core_SetPeriodTimer(pMc->plf_timer, pMc->lf_timer_period);
    }
    break;
    case MC_VALIDATION:
    {
      if (pMc->steps < pMc->bemf.const_validation_steps_max)
      {
        (pMc->steps)++;
      }
      else
      {
        pMc->status = MC_VALIDATION_FAILURE;
        MC_Core_Error(pMc);
        return MC_FUNC_FAIL;
      }
    }
    break;
    default:
    break;
  }
  if (pMc->step_prepared == 0)
  {
    pMc->bemf.zero_crossing_events = 0;
    MC_Core_SetResetPolarityHfPwm(pMc->phf_timer, pMc->channel_polarity, pMc->Nchannel_polarity);
    MC_Core_PrepareNextStep(pMc);
    MC_Core_SetHFPWM_6pwm(pMc, pMc->hf_timer_pulse_value_max, pMc->step_pos_next);    
  }
  MC_Core_SetHFPWM_3pwm(pMc, pMc->hf_timer_pulse_value_max, pMc->step_pos_next);
  pMc->bemf.demagn_counter = pMc->bemf.const_bemf_demagn_counter;
  MC_Core_GenerateComEvent(pMc->phf_timer);
  MC_Core_SetTimerCounter(pMc->phf_timer, pMc->hf_timer_period);
  pMc->step_position = pMc->step_pos_next;
  MC_Core_SetEnableUpdateEvent(pMc->phf_timer);
  switch (pMc->step_position)
  { 
    case 1:
    case 4:
    {
      pMc->bemf.current_adc_channel = pMc->bemf.adc_channel[2]; 
    }  
    break;      
    case 2:
    case 5:
    {
      pMc->bemf.current_adc_channel = pMc->bemf.adc_channel[1];   
    }  
    break;      
    case 3:
    case 6:
    {
      pMc->bemf.current_adc_channel = pMc->bemf.adc_channel[0];      
    }     
    break;      
  } 
  if (pMc->status >= MC_VALIDATION)
  {
    MC_Core_SelectAdcChannel(pMc->adc_user.padc[pMc->adc_user.channel_index], pMc->bemf.padc, pMc->bemf.current_adc_channel, pMc->bemf.sampling_time);
    MC_Core_SetPWMON(pMc);
    MC_Core_SetDutyCyclePwmForAdcTrig(pMc->bemf.ptrig_timer, pMc->bemf.trig_timer_channel, pMc->bemf.trig_time);
  }
  pMc->step_prepared = 0;
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_HallNextStep
  * @param[in] pMc motor control handle
  * @param[in] HfTimerCounterSnapshot value of the HF timer counter at the beginning of the LF timer callback
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_HallNextStep(MC_Handle_t *pMc, uint16_t HfTimerCounterSnapshot)
{
  if (pMc->pref_timer != NULL)
  {
    int32_t pulse_ref = pMc->pulse_value - HfTimerCounterSnapshot;
    if (pulse_ref < 0) pulse_ref = pMc->pulse_value;
    else pulse_ref += pMc->pulse_value;
    MC_Core_SetPeriodTimer(pMc->pref_timer, (pMc->hf_timer_period << 1) - HfTimerCounterSnapshot);
    MC_Core_SetDutyCycleRefPwm(pMc->pref_timer, pulse_ref);
  }
  int32_t pulse_hf = pMc->hf_timer_pulse_value_max - HfTimerCounterSnapshot;
  if (pulse_hf < 0) pulse_hf = pMc->hf_timer_pulse_value_max;
  else pulse_hf += pMc->hf_timer_pulse_value_max;
  MC_Core_SixStepTable(pMc, pMc->hf_timer_pulse_value_max, pMc->step_pos_next);
  MC_Core_SetDutyCycleHfPwmForStepN(pMc->phf_timer, pMc->hf_timer_pulse_value_max , pMc->step_pos_next);
  MC_Core_SetTimerCounter(pMc->phf_timer, pMc->hf_timer_period);
  if (pMc->pref_timer != NULL)
  {
  MC_Core_SetPeriodTimer(pMc->pref_timer, pMc->hf_timer_period);
  MC_Core_SetDutyCycleRefPwm(pMc->pref_timer, pMc->pulse_value);
  }
  return MC_FUNC_OK;
}


/**
  * @brief  MC_Core_SetAdcBemfTrigTime
  * @param[in] DutyCycleToSet Duty cycle in 1/1024 of PWM period
  * @retval Function Status
  */
MC_FuncStatus_t MC_Core_SetAdcBemfTrigTime(MC_Handle_t* pMc, uint32_t DutyCycleToSet)
{
  pMc->bemf.trig_time = (uint16_t)((pMc->bemf.trig_timer_period*DutyCycleToSet) >> MC_DUTY_CYCLE_SCALING_SHIFT);  
  return MC_FUNC_OK; 
}


/**
  * @brief  MC_Core_LfTimerPeriodCompute
  *         Check events and compute time for the commutation to next step
  * @param[in]  pMc
  * @param[in]  CounterSnaphot
  * @param[in]  BemfIsIncreasing
  * @retval Function Status
  */

MC_FuncStatus_t MC_Core_LfTimerPeriodCompute(MC_Handle_t* pMc, uint16_t CounterSnaphot, uint8_t BemfIsIncreasing)
{
  {
    if (pMc->status == MC_VALIDATION)
    {
      if (pMc->bemf.over_threshold_events > pMc->bemf.const_validation_bemf_events_max)
      {
        pMc->status = MC_VALIDATION_BEMF_FAILURE;
        MC_Core_Error(pMc);
      }
      if (BemfIsIncreasing != 0)
      {
        pMc->bemf.zero_crossing_events++;
        pMc->bemf.over_threshold_events = 0;      
      }
      else
      {
        pMc->bemf.over_threshold_events++;
      }  
      if (pMc->bemf.zero_crossing_events >= pMc->bemf.const_validation_zero_cross_number)
      {
        pMc->status = MC_RUN;
        pMc->pid_parameters.integral_term_sum = ((pMc->pulse_value) << (pMc->pid_parameters.scaling_shift));
        pMc->pid_parameters.previous_speed = pMc->speed_fdbk_filtered;
        pMc->bemf.zero_crossing_events = 0;
      }
    }
    else
    {      
      MC_Core_SetPeriodTimer(pMc->plf_timer, CounterSnaphot + (uint16_t)(((uint32_t)((pMc->bemf.zcd_to_comm) * (pMc->lf_timer_period))) >> 9));
    }
  }
  return MC_FUNC_OK; 
}
