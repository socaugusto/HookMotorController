/**
 ******************************************************************************
 * @file    6step_hall_synch.c
 * @author  IPC Agrate
 * @brief   This file provides all the 6-step library functions
 * for hall sensors voltage mode with speed loop
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

#include "6step_synch_hall.h"

/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{ 
  */

/** @addtogroup MC_6STEP_LIB
  * @{
  */

/** @defgroup MC_6STEP_VM_HALL_SYNCH
  * @brief 6step core module
  * @{
  */ 

/** @defgroup MC_6STEP_VM_HALL_SYNCH_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup MC_6STEP_HALL_SYNCH_Private_Defines
  * @{
  */  
  /*!< Motor control validation parameters */

#define VALIDATION_HALL_STATUS_DIRECT0_STEP1 ((uint8_t)  5)
#define VALIDATION_HALL_STATUS_DIRECT0_STEP2 ((uint8_t)  4)
#define VALIDATION_HALL_STATUS_DIRECT0_STEP3 ((uint8_t)  6)
#define VALIDATION_HALL_STATUS_DIRECT0_STEP4 ((uint8_t)  2)
#define VALIDATION_HALL_STATUS_DIRECT0_STEP5 ((uint8_t)  3)
#define VALIDATION_HALL_STATUS_DIRECT0_STEP6 ((uint8_t)  1)

#define VALIDATION_HALL_STATUS_DIRECT1_STEP1 ((uint8_t)  2) 
#define VALIDATION_HALL_STATUS_DIRECT1_STEP2 ((uint8_t)  3)
#define VALIDATION_HALL_STATUS_DIRECT1_STEP3 ((uint8_t)  1)
#define VALIDATION_HALL_STATUS_DIRECT1_STEP4 ((uint8_t)  5)
#define VALIDATION_HALL_STATUS_DIRECT1_STEP5 ((uint8_t)  4)
#define VALIDATION_HALL_STATUS_DIRECT1_STEP6 ((uint8_t)  6)

/**
  * @}  end
  */ 
/** @defgroup MC_6STEP_HALL_SYNCH_Private_Macros
* @{
*/ 
/**
  * @}
  */ 

/** @defgroup MC_6STEP_HALL_SYNCH_Private_Functions_Prototypes
  * @{
  */
uint16_t MC_Core_LfTimerPeriodFilter(MC_Handle_t *pMc);
extern void MC_Core_GetHallStatus(MC_Handle_t *pMc);
extern void MC_Core_SetCompareHallTimer(uint32_t *pHallTimer, uint16_t CommutationDelay);
extern uint16_t MC_Core_GetTimerPrescaler(uint32_t *pTimer);
extern void MC_Core_Error(MC_Handle_t *pMc);
extern uint16_t MC_Core_GetTimerCaptureCompare(uint32_t *pTimer);
extern void MC_Core_SetDutyCycleRefPwm(uint32_t *pRefTimer, uint16_t PulseValue);
extern void MC_Core_StartHfPwms(uint32_t *pHfTimer, MC_EN_DRIVER_t enable);
extern void MC_Core_EnableItBreak(uint32_t *pTimer);
extern void MC_Core_GenerateComEvent(uint32_t *pHfTimer);

/**
  * @}
  */ 

/** @defgroup MC_6STEP_VM_HALL_SYNCH_Private_Variables
  * @{
  */
extern uint32_t SysClockFrequency;
extern MC_Handle_t Motor_Device1;

/**
  * @} end MC_6STEP_HALL_SYNCH_Private_Variables
  */

/** @defgroup MC_6STEP_HALL_SYNCH_Private_Functions
  * @{
  */

/**
  * @} end MC_6STEP_HALL_SYNCH_Private_Functions
  */ 

/** @defgroup MC_6STEP_HALL_SYNCH_Exported_Functions
  * @{
  */


/**
  * @brief  MC_Core_HallStatusToStep
  *         Find the current step according to the hall status
  * @param[in] pMc motor control handle
  * @retval Function Status
  */
MC_FuncStatus_t MC_Core_HallStatusToStep(MC_Handle_t *pMc)
{
  if (pMc->direction == 0)    
  {
    switch (pMc->hall.status)
    {
      case VALIDATION_HALL_STATUS_DIRECT0_STEP1:
      {
        pMc->step_pos_next = 1;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT0_STEP2:
      {
        pMc->step_pos_next = 2;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT0_STEP3:
      {
        pMc->step_pos_next = 3;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT0_STEP4:
      {
        pMc->step_pos_next = 4;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT0_STEP5:
      {
        pMc->step_pos_next = 5;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT0_STEP6:
      {
        pMc->step_pos_next = 6;
      }
      break;
      default:
      {
        return MC_FUNC_FAIL;
      }
      break;
    }
  }
  else
  {
    switch (pMc->hall.status)
    {
      case VALIDATION_HALL_STATUS_DIRECT1_STEP1:
      {
        pMc->step_pos_next = 1;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT1_STEP2:
      {
        pMc->step_pos_next = 2;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT1_STEP3:
      {
        pMc->step_pos_next = 3;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT1_STEP4:
      {
        pMc->step_pos_next = 4;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT1_STEP5:
      {
        pMc->step_pos_next = 5;
      }
      break;
      case VALIDATION_HALL_STATUS_DIRECT1_STEP6:
      {
        pMc->step_pos_next = 6;
      }
      break;
      default:
      {
        return MC_FUNC_FAIL;
      }
      break;
    }
  }
  return MC_FUNC_OK; 
}
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

uint32_t MC_Core_SpeedCompute(MC_Handle_t *pMc)
{
  uint32_t prescaler = (uint32_t) MC_Core_GetTimerPrescaler(pMc->plf_timer);
  uint32_t period = (uint32_t) MC_Core_LfTimerPeriodFilter(pMc);
  return ((SysClockFrequency * 10) / ((++period) * (++prescaler) * (pMc->motor_charac.pole_pairs)));
}


/**
  * @brief  MC_Core_NextStepScheduling
  * @param[in] pMc motor control handle
  * @retval  Function Status
  */

MC_FuncStatus_t MC_Core_NextStepScheduling(MC_Handle_t *pMc)
{
  pMc->lf_timer_period = MC_Core_GetTimerCaptureCompare(pMc->plf_timer);
  MC_Core_GetHallStatus(pMc);
  if (MC_Core_HallStatusToStep(pMc) != MC_FUNC_OK)
    {
      pMc->status = MC_VALIDATION_HALL_FAILURE;
      MC_Core_Error(pMc);  
      return MC_FUNC_FAIL;
    }
  if (pMc->direction == 0)
    {
      if ((pMc->step_pos_next != (pMc->step_position+1)) && ((pMc->step_pos_next != 1) || (pMc->step_position != 6)))
      {
        if (pMc->step_pos_next != pMc->step_position)
        {  
          pMc->hall.commutation_errors++;
          if (pMc->hall.commutation_errors > pMc->hall.commutation_errors_max)
          {
            pMc->status = MC_RUN_WRONG_STEP_FAILURE;
            MC_Core_Error(pMc);  
            return MC_FUNC_FAIL;
          }          
        }
      }
      else
      {  
        pMc->hall.commutation_errors = 0;
        pMc->hall.stay_in_run_lf_cnt = 0;
      }
    }
    else if ((pMc->step_pos_next != (pMc->step_position-1)) && ((pMc->step_pos_next != 6) || (pMc->step_position != 1)))
    {
      if (pMc->step_pos_next != pMc->step_position)
      {  
        pMc->hall.commutation_errors++;
        if (pMc->hall.commutation_errors > pMc->hall.commutation_errors_max)
        {
          pMc->status = MC_RUN_WRONG_STEP_FAILURE;
          MC_Core_Error(pMc);  
          return MC_FUNC_FAIL;
        }        
      }
    }
    else
    {  
      pMc->hall.commutation_errors = 0;
      pMc->hall.stay_in_run_lf_cnt = 0;
    }  
  if (pMc->status == MC_ALIGNMENT) pMc->status = MC_RUN;
  MC_Core_SetCompareHallTimer(pMc->plf_timer, pMc->hall.commutation_delay);
  pMc->step_prepared++;
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_AlignmentToCurrentStep
  * @param[in] pMc motor control handle
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_AlignmentToCurrentStep(MC_Handle_t *pMc)
{
  MC_Core_GetHallStatus(pMc);
  if (MC_Core_HallStatusToStep(pMc) != MC_FUNC_OK)
  {
    pMc->status = MC_VALIDATION_HALL_FAILURE;
    MC_Core_Error(pMc);  
    return MC_FUNC_FAIL;
  }
  if (pMc->align_index == 0)
  {
    (pMc->align_index)++;
    if (pMc->pref_timer != NULL)
	{
		MC_Core_SetDutyCycleRefPwm(pMc->pref_timer, pMc->startup_reference);
	}
	else
	{
		pMc->pulse_value = pMc->startup_reference;
		pMc->hf_timer_pulse_value_max = pMc->pulse_value;
	}
    MC_Core_SixStepTable(pMc, pMc->hf_timer_pulse_value_max, pMc->step_pos_next);
    pMc->step_prepared = pMc->step_pos_next;
    MC_Core_StartHfPwms(pMc->phf_timer, pMc->enDRIVER);
    MC_Core_EnableItBreak(pMc->phf_timer);
    MC_Core_GenerateComEvent(pMc->phf_timer);
    return MC_FUNC_OK;
  }
  else if (pMc->align_index <= pMc->alignment_time) 
  {
    (pMc->align_index)++;
    return MC_FUNC_OK;
  }
  else if (pMc->hall.alignement_step_change != 0)
  {
    pMc->hall.alignement_step_change = 0;
    pMc->align_index = 1;
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
    MC_Core_SixStepTable(pMc, pMc->hf_timer_pulse_value_max, pMc->step_pos_next);
    MC_Core_GenerateComEvent(pMc->phf_timer);
    return MC_FUNC_OK;
  }
  else
  {
    pMc->status = MC_VALIDATION_FAILURE;
    MC_Core_Error(pMc);  
    return MC_FUNC_FAIL;
  }
}

 
/**
  * @} end MC_6STEP_HALL_SYNCH_Private_Functions
  */ 

/**
  * @}  end MC_6STEP_HALL_SYNCH
  */ 

/**
  * @}  end MC_6STEP_LIB
  */

/**
  * @}  end MIDDLEWARES
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
 