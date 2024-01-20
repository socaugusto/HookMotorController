/**
 ******************************************************************************
 * @file    6step_PID.c
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

#include "6step_PID.h"


/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{ 
  */

/** @addtogroup MC_6STEP_LIB
  * @{
  */

/** @defgroup MC_6STEP_CONTROL_LOOP
  * @brief 6step core module
  * @{
  */ 

/** @defgroup MC_6STEP_CONTROL_LOOP_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_CONTROL_LOOP_Private_Defines
  * @{
  */


    
/**
  * @}
  */ 

/** @defgroup MC_6STEP_CONTROL_LOOP_Private_Macros
* @{
*/ 
/**
  * @}
  */ 

/** @defgroup MC_6STEP_CONTROL_LOOP_Private_Functions_Prototypes
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_CONTROL_LOOP_Private_Variables
  * @{
  */

/**
  * @} end MC_6STEP_CONTROL_LOOP_Private_Variables
  */

/** @defgroup MC_CONTROL_LOOP_Private_Functions
  * @{
  */

/**
  * @} end MC_6STEP_CONTROL_LOOP_Private_Functions
  */ 


/** @defgroup MC_6STEP_CORE_Exported_Functions
  * @{
  */

/**
  * @brief  MC_Core_SpeedControl
  * @retval  output pulse value to set
  */
uint16_t MC_Core_SpeedControl(MC_Handle_t *pMc)
{
  int32_t proportional_term = 0, derivative_term = 0, output = 0;
  int32_t speed_error;
  
  /* Error computation */
  speed_error = (pMc->speed_target_value) - (pMc->speed_fdbk_filtered);
  
  /* Proportional term computation */
  proportional_term = speed_error * (pMc->pid_parameters.kp);
    
  /* Integral term computation */
  pMc->pid_parameters.integral_term_sum += speed_error * (pMc->pid_parameters.ki);
  if (pMc->pid_parameters.integral_term_sum > (int32_t)((pMc->pid_parameters.maximum_output) << (pMc->pid_parameters.scaling_shift)))
  {
    pMc->pid_parameters.integral_term_sum = ((pMc->pid_parameters.maximum_output) << (pMc->pid_parameters.scaling_shift));
  } 
  else if (pMc->pid_parameters.integral_term_sum < (int32_t)((pMc->pid_parameters.minimum_output) << (pMc->pid_parameters.scaling_shift)))
  {
    pMc->pid_parameters.integral_term_sum = ((pMc->pid_parameters.minimum_output) << (pMc->pid_parameters.scaling_shift));
  }
  
  /* Derivative computation */
  derivative_term = ((pMc->pid_parameters.previous_speed) - (pMc->speed_fdbk_filtered)) * (pMc->pid_parameters.kd);
  pMc->pid_parameters.previous_speed = pMc->speed_fdbk_filtered;

  output =
    ((proportional_term + (pMc->pid_parameters.integral_term_sum) + derivative_term) >> (pMc->pid_parameters.scaling_shift));
  
  if (output > (pMc->pid_parameters.maximum_output))
  {
    output = (pMc->pid_parameters.maximum_output);
  }
  else if (output < (pMc->pid_parameters.minimum_output))
  {
    output = (pMc->pid_parameters.minimum_output);
  }
  
  return (uint16_t) output;
}


/**
  * @} end MC_6STEP_CONTROL_LOOP_Exported_Functions
  */ 

/**
  * @}  end MC_6STEP_CONTROL_LOOP
  */ 

/**
  * @}  end MC_6STEP_LIB
  */

/**
  * @}  end MIDDLEWARES
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/