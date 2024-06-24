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

#include "6step_SetFunctions_vm_cm.h"

extern void MC_Core_ResetBemfGpio(MC_Handle_t* pMc);
extern void MC_Core_SetBemfGpio(MC_Handle_t* pMc);


inline void MC_Core_SetPWMON(MC_Handle_t* pMc)
  {
    /* PWM ON sensing BEGIN 5 */
    if ((pMc->bemf.pwm_on_sensing_enabled == 0) && (pMc->pulse_value > pMc->bemf.pwm_on_sensing_en_thres))
    {
      (pMc->bemf.pwm_on_sensing_enabled)++;
      pMc->bemf.trig_time = pMc->bemf.pwm_on_sensing_trig_time;
      pMc->bemf.adc_threshold_up = pMc->bemf.const_adc_threshold_up_on;
      pMc->bemf.adc_threshold_down = pMc->bemf.const_adc_threshold_down_on;
      MC_Core_ResetBemfGpio(pMc);
    }
    else if ((pMc->bemf.pwm_on_sensing_enabled == 1) && (pMc->pulse_value < pMc->bemf.pwm_on_sensing_dis_thres))
    {
      pMc->bemf.pwm_on_sensing_enabled = 0;
      pMc->bemf.trig_time = pMc->bemf.pwm_off_sensing_trig_time;
      pMc->bemf.adc_threshold_up = pMc->bemf.const_adc_threshold_up;
      pMc->bemf.adc_threshold_down = pMc->bemf.const_adc_threshold_down;
      MC_Core_SetBemfGpio(pMc);
    }
    /* PWM ON sensing END 5 */
  }
  


inline void MC_Core_RampTuning(MC_Handle_t *pMc, uint16_t speed_incr)
 {
    int16_t tmp;
    tmp = (pMc->hf_timer_period*(pMc->duty_cycle_ramp_residual >> MC_DUTY_CYCLE_SCALING_SHIFT))>> MC_DUTY_CYCLE_SCALING_SHIFT;
    pMc->duty_cycle_ramp_residual += (speed_incr * pMc->duty_cycle_ramp_factor);
    pMc->pulse_command = pMc->startup_reference + tmp;
    pMc->pulse_value = pMc->pulse_command;
    pMc->hf_timer_pulse_value_max = pMc->pulse_value;

}
