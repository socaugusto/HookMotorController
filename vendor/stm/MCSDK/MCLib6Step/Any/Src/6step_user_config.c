/**
 ******************************************************************************
 * @file    6step_user_config.c
 * @author  IPC Agrate
 * @brief   This file provide user config functions
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

#include "6step_user_config.h"


/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{ 
  */

/** @addtogroup MC_6STEP_LIB
  * @{
  */

/** @defgroup MC_6STEP_USER_CONFIG
  * @brief 6step conf module
  * @{
  */

/** @defgroup MC_6STEP_USER_CONFIG_Exported_Functions
  * @{
  */

/** @defgroup MC_6STEP_USER_CONFIG_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_USER_CONFIG_Private_Defines
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_USER_CONFIG_Private_Macros
* @{
*/ 
/**
  * @}
  */ 

/** @defgroup MC_6STEP_USER_CONFIG_Private_Functions_Prototypes
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_USER_CONFIG_Private_Variables
  * @{
  */
extern MC_Handle_t Motor_Device1;
/**
  * @} end MC_6STEP_USER_CONFIG_Private_Variables
  */

/** @defgroup MC_6STEP_USER_CONFIG_Private_Functions
  * @{
  */

void MC_Core_SetAdcSamplingTime(uint32_t *pAdc, uint32_t AdcChannel, uint32_t SamplingTime);
uint8_t MC_Core_GetAdcResolution(MC_Handle_t *pMc, uint32_t *pAdc);

/**
  * @} end MC_6STEP_USER_CONFIG_Private_Functions
  */ 

/** @defgroup MC_6STEP_USER_CONFIG_Exported_Functions
  * @{
  */
/**
  * @brief  MC_Core_ConfigureUserAdc
  * @param[in] pMc motor control handle
  * @param[in] pTrigTimer pointer on the handle of the timer used to trig the ADC
  * @param[in] TrigTimerChannel Channel of the timer used to trig the ADC
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_ConfigureUserAdc(MC_Handle_t* pMc, uint32_t *pTrigTimer, uint16_t TrigTimerChannel, uint8_t NumberOfUserChannels)
{
  if (pTrigTimer == NULL)
  {
    return MC_FUNC_FAIL;
  }
  pMc->adc_user.ptrig_timer = pTrigTimer;
  pMc->adc_user.trig_timer_period = pMc->hf_timer_period;
  pMc->adc_user.trig_timer_channel = TrigTimerChannel;
  if (NumberOfUserChannels <= NUMBER_OF_USER_ADC_CHANNELS)
  {
    pMc->adc_user.number_of_channels = NumberOfUserChannels;
  }
  else
  {
    return MC_FUNC_FAIL;
  }
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_ConfigureUserAdcChannel
  * @param[in] pMc motor control handle
  * @param[in] pAdc pointer on the ADC to be selected
  * @param[in] AdcChannel ADC channel to be selected
  * @param[in] SamplingTime ADC sampling time to be selected
  * @param[in] UserMeasurement User measurement to map to ADC channel
  * @retval None
  */
MC_FuncStatus_t MC_Core_ConfigureUserAdcChannel(MC_Handle_t* pMc, uint32_t* pAdc, uint32_t AdcChannel, uint32_t SamplingTime, MC_UserMeasurements_t UserMeasurement)
{
  if ((UserMeasurement == MC_USER_MEAS_1) || (UserMeasurement == MC_USER_MEAS_2) || (UserMeasurement == MC_USER_MEAS_3) || (UserMeasurement == MC_USER_MEAS_4) || (UserMeasurement == MC_USER_MEAS_5))
  {
    pMc->adc_user.padc[UserMeasurement] = pAdc;
    pMc->adc_user.channel[UserMeasurement] = AdcChannel;
    pMc->adc_user.sampling_time[UserMeasurement] = SamplingTime;
    MC_Core_SetAdcSamplingTime(pAdc, AdcChannel, SamplingTime);
    return MC_FUNC_OK;
  }
  else
  {
    return MC_FUNC_FAIL;
  }
}

MC_FuncStatus_t MC_Core_ConfigureUserButton(MC_Handle_t* pMc, uint16_t ButtonPin, uint16_t ButtonDebounceTimeMs)
{
  pMc->button_user.gpio_pin = ButtonPin;
  pMc->button_user.debounce_time_ms = ButtonDebounceTimeMs;
  return MC_FUNC_OK;
}


/**
  * @brief  MC_Core_AssignUserMeasurementToSpeedDutyCycleCommand
  * @param[in] pMc motor control handle
  * @param[in] UserMeasurement User measurement used to build the speed command or the duty cycle command
  * @retval None
  */

MC_FuncStatus_t MC_Core_AssignUserMeasurementToSpeedDutyCycleCommand(MC_Handle_t* pMc, MC_UserMeasurements_t UserMeasurement)
{
  if (pMc == NULL)
  {
    return MC_FUNC_FAIL;
  }
  pMc->adc_user.spd_dc_command.channel_index = UserMeasurement;
  return MC_FUNC_OK;
}

/**
  * @brief  MC_Core_GetCommandFromAdcMeasurement
  * @param[in] pMc motor control handle
  * @param[in] CommandMin minimum possible value for the command to be returned
  * @param[in] CommandMax maximum possible value for the command to be returned
  * @retval  Speed command
  */
uint32_t MC_Core_GetCommandFromAdcMeasurement(MC_Handle_t *pMc, uint32_t CommandMin, uint32_t CommandMax)
{
  uint32_t tmp = 0;
  pMc->adc_user.spd_dc_command.filtered = (pMc->adc_user.spd_dc_command.filter_coef * pMc->adc_user.spd_dc_command.filtered + pMc->adc_user.measurement[pMc->adc_user.spd_dc_command.channel_index]) >> pMc->adc_user.spd_dc_command.filter_shift;
  if ((pMc->adc_user.spd_dc_command.filtered > (pMc->adc_user.spd_dc_command.current + pMc->adc_user.spd_dc_command.change_threshold))||\
      ((pMc->adc_user.spd_dc_command.filtered + pMc->adc_user.spd_dc_command.change_threshold) < pMc->adc_user.spd_dc_command.current))
  {
    pMc->adc_user.spd_dc_command.current = pMc->adc_user.spd_dc_command.filtered;
    if (pMc->reference_to_be_updated == 0) (pMc->reference_to_be_updated)++;
  }
  tmp = (((pMc->adc_user.spd_dc_command.current * CommandMax) >> pMc->adc_user.resolution) + (CommandMin >> 1));
  if (tmp > CommandMax)
  {
    tmp = CommandMax;
  }
  else if (tmp < CommandMin)
  {
    tmp = CommandMin;
  }  
  return tmp;
}

/**
  * @brief  MC_Core_InitAdc
  *         One time initialization of the ADC parameters inside the motor control structure
  * @param[in] pMc motor control handle
  * @retval Function Status
  */
MC_FuncStatus_t MC_Core_InitAdc(MC_Handle_t *pMc)
{
  pMc->adc_user.resolution = MC_Core_GetAdcResolution(pMc, pMc->adc_user.padc[0]);
  /* POTENTIOMETER BEGIN 3 */
#if (POTENTIOMETER_INTERFACE != 0)
  for (uint8_t i = 1; i < pMc->adc_user.number_of_channels; i++)
  {
    if (pMc->adc_user.resolution != MC_Core_GetAdcResolution(pMc, pMc->adc_user.padc[i]))
    {
      return MC_FUNC_FAIL;
    }
  }
  pMc->adc_user.spd_dc_command.change_threshold = MC_ADC_TO_COMMAND_THRESHOLD;
  pMc->adc_user.spd_dc_command.filter_shift = MC_ADC_TO_COMMAND_FILTER_SHIFT;
  pMc->adc_user.spd_dc_command.filter_coef = MC_ADC_TO_COMMAND_FILTER_COEF;
#endif
  /* POTENTIOMETER END 3 */
  return MC_FUNC_OK; 
}

/**
  * @brief  MC_Core_SetAdcUserTrigTime
  * @param[in] DutyCycleToSet Duty cycle in 1/1024 of PWM period
  * @retval Function Status
  */
MC_FuncStatus_t MC_Core_SetAdcUserTrigTime(MC_Handle_t* pMc, uint32_t DutyCycleToSet)
{
  pMc->adc_user.trig_time = (uint16_t)((pMc->adc_user.trig_timer_period*DutyCycleToSet) >> MC_DUTY_CYCLE_SCALING_SHIFT);
  return MC_FUNC_OK; 
}

/**
  * @} end MC_6STEP_USER_CONFIG_Exported_Functions
  */ 


/**
  * @}  end MC_6STEP_USER_CONFIG
  */ 

/**
  * @}  end MC_6STEP_LIB
  */

/**
  * @}  end MIDDLEWARES
  */ 