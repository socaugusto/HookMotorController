
/**
  ******************************************************************************
  * @file    mc_interface.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features
  *          of the MC Interface component of the Motor Control SDK:
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
#include "mc_config.h"
#include "mc_interface.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MCInterface Motor Control Interface
  * @brief MC Interface component of the Motor Control SDK
  *
  * @todo Document the MC Interface "module".
  *
  * @{
  */
/** @defgroup MC_INTERFACE_Private_Variables
  * @{
  */

/**
  * @} end MC_INTERFACE_Private_Variables
  */

/** @defgroup MC_INTERFACE_Private_Functions
  * @{
*/

/**
  * @brief  MC_Core_GetSpeed
  * @param[in] pMc motor control handle
  * @retval pMc->speed_fdbk_filtered
  */
uint32_t MC_Core_GetSpeed(MC_Handle_t* pMc)
{
  return pMc->speed_fdbk_filtered;
}

/**
  * @brief  MC_Core_GetStatus
  * @param[in] pMc motor control handle
  * @retval pMc->status
  */
__weak MC_Status_t MC_Core_GetStatus(MC_Handle_t* pMc)
{
  return pMc->status;
}

/**
  * @brief  MC_Core_SetDirection
  * @param[in] pMc motor control handle
  * @param[in] DirectionToSet
  * @retval Function Status
  */
__weak MC_FuncStatus_t MC_Core_SetDirection(MC_Handle_t* pMc, uint32_t DirectionToSet)
{
  pMc->direction = DirectionToSet;
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_SetSpeed
  * @param[in] SpeedToSet
  * @retval Function Status
  */
__weak MC_FuncStatus_t MC_Core_SetSpeed(MC_Handle_t *pMc, uint32_t SpeedToSet)
{
  pMc->speed_target_value = pMc->speed_target_command;
  pMc->speed_target_command = SpeedToSet;
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_SetStartupDutyCycle
  * @param[in] DutyCycleToSet Duty cycle in 1/1024 of PWM period
  * @retval Function Status
  */
__weak MC_FuncStatus_t MC_Core_SetStartupDutyCycle(MC_Handle_t* pMc, uint32_t DutyCycleToSet)
{
  pMc->startup_reference = (uint16_t)((DutyCycleToSet * (pMc->hf_timer_period)) >> MC_DUTY_CYCLE_SCALING_SHIFT);
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_Start
  * @param  None
  * @retval Function Status
  */
__weak MC_FuncStatus_t MC_Core_Start(MC_Handle_t *pMc)
{
  if (pMc->status == MC_STOP)
  {
    pMc->button_user.enabled = 0;
    pMc->uw_tick_cnt = 0;
    pMc->pulse_value = pMc->startup_reference;
    pMc->hf_timer_pulse_value_max = pMc->pulse_value;
//    for (uint32_t i = 0; i < pMc->adc_user.number_of_channels; i++)
//    {
//      MC_Core_CalibrateAdc(pMc->adc_user.padc[i]);
//    }
//	if (pMc->adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(pMc->adc_user.ptrig_timer, pMc->adc_user.trig_timer_channel, pMc->adc_user.trig_time);
	if (pMc->pref_timer != NULL)
    {
      MC_Core_SetDutyCycleRefPwm(pMc->pref_timer, pMc->ref_timer_period);
      MC_Core_StartRefPwm(pMc->pref_timer);
    }
    pMc->status = MC_ALIGNMENT;
    MC_Core_GetHallStatus(pMc);
    MC_Core_HallConfigureCommutationEvent(pMc->phf_timer, pMc->plf_timer);
    MC_Core_EnableUpdateEvent(pMc->phf_timer);
//    for (uint32_t i = 0; i < pMc->adc_user.number_of_channels; i++)
//    {
//      MC_Core_StartAdcIt(pMc->adc_user.padc[i]);
//    }
//    if (pMc->adc_user.number_of_channels != 0) MC_Core_SelectAdcChannel(pMc->adc_user.padc[pMc->adc_user.channel_index], pMc->adc_user.padc[pMc->adc_user.channel_index], pMc->adc_user.channel[pMc->adc_user.channel_index], pMc->adc_user.sampling_time[pMc->adc_user.channel_index]);
 }
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_Stop
  * @param  None
  * @retval Function Status
  */
__weak MC_FuncStatus_t MC_Core_Stop(MC_Handle_t *pMc)
{
  MC_Core_DisableIrq();
  MC_Core_StopLfTimerIt(pMc->plf_timer);
  MC_Core_StopHfPwms(pMc->phf_timer, pMc->enDRIVER);
  MC_Core_SetDutyCycleHfPwmForStepN(pMc->phf_timer, 0, pMc->step_position);
//  for (uint32_t i = 0; i < pMc->adc_user.number_of_channels; i++)
//  {
//    MC_Core_StopAdcIt(pMc->adc_user.padc[i]);
//  }
  if (pMc->pref_timer != NULL)
  {
    MC_Core_StopRefPwm(pMc->pref_timer);
  }
  pMc->status = MC_STOP;
  pMc->button_user.enabled = 0;
  MC_Core_EnableIrq();

  /* Motor parameters and private variable initialisation */
  MC_Core_Reset(pMc);

  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_GetGateDriverPwmFreq
  * @param[in] pMc motor control handle
  * @retval gate driver frequency in Hz
  */
__weak uint32_t MC_Core_GetGateDriverPwmFreq(MC_Handle_t* pMc)
{
  return pMc->gate_driver_frequency;
}

/**
  * @brief  MC_Core_Error
  * @param[in] pMc pointer to the motor control handle
  * @retval None
  */
__weak void MC_Core_Error(MC_Handle_t *pMc)
{
  MC_Core_DisableIrq();
  MC_Core_StopLfTimerIt(pMc->plf_timer);
  MC_Core_StopHfPwms(pMc->phf_timer, pMc->enDRIVER);
  MC_Core_SetDutyCycleHfPwmForStepN(pMc->phf_timer, 0, pMc->step_position);
  MC_Core_EnableIrq();
}

uint32_t MC_Core_GetFaultState( MC_Handle_t * pMc )
{
  uint32_t retVal = MC_NO_ERROR;
  switch ( pMc->status )
  {
    case MC_IDLE:
    case MC_STOP:
    case MC_ALIGNMENT:
    case MC_STARTUP:
    case MC_VALIDATION:
    case MC_RUN:
      retVal = (int32_t) 0;
      break;
    case MC_SPEEDFBKERROR:
      retVal = MC_SPEED_FDBK;
      break;
    case MC_OVERCURRENT:
      retVal = MC_BREAK_IN;
      break;
    case MC_VALIDATION_FAILURE:
      retVal = MC_START_UP;
      break;
    case MC_VALIDATION_BEMF_FAILURE:
      retVal = (MC_SW_ERROR | MC_SPEED_FDBK);
      break;
    case MC_VALIDATION_HALL_FAILURE:
      retVal = (MC_SW_ERROR | MC_SPEED_FDBK);
      break;
    case MC_LF_TIMER_FAILURE:
      retVal = MC_FOC_DURATION;
      break;
    default:
      retVal = MC_SW_ERROR;
	  break;
  }

  return retVal;
}

uint8_t MC_Core_GetState( MC_Handle_t * pMc )
{
  uint8_t retVal;

  switch ( pMc->status )
    {
    case MC_IDLE:
      retVal = IDLE;
      break;
    case MC_STOP:
      retVal = STOP;
      break;
    case MC_ALIGNMENT:
      retVal = ALIGNMENT;
      break;
    case MC_STARTUP:
      retVal = START;
      break;
    case MC_VALIDATION:
      retVal = START_RUN;
      break;
    case MC_RUN:
      retVal = RUN;
      break;
    case MC_SPEEDFBKERROR:
    case MC_OVERCURRENT:
    case MC_VALIDATION_FAILURE:
    case MC_VALIDATION_BEMF_FAILURE:
    case MC_VALIDATION_HALL_FAILURE:
    case MC_LF_TIMER_FAILURE:
    case MC_ADC_CALLBACK_FAILURE:
      retVal = FAULT_OVER;
      break;
    default:
      retVal = FAULT_NOW;
	  break;
	}

    return retVal;
}

/**
  * @} end MC_INTERFACE_Exported_Functions
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
