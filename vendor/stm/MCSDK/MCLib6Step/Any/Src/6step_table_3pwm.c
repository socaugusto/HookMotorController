/**
  ******************************************************************************
  * @file    6step_table_3pwm.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware function that 
  *          sets the HF pwm duty cycles for each step   
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
  
#include "6step_table_3pwm.h"


/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{ 
  */

/** @addtogroup MC_6STEP_LIB
  * @{
  */

/** @defgroup MC_6STEP_SL_CM_SPDLP_3PWM
  * @brief 6step module, sensors less, current mode, speed loop, 3 pwm gate driver interface.
  * @{
  */

/** @defgroup MC_6STEP_SL_CM_SPDLP_3PWM_Imported_Variables
  * @{
  */
extern uint32_t SysClockFrequency;

/**
  * @}
  */

/** @defgroup MC_6STEP_SL_CM_SPDLP_3PWM_Private_Defines
  * @{
  */

/**
  * @}
  */     
    
/** @defgroup MC_6STEP_SL_CM_SPDLP_3PWM_Private_Functions_Prototypes
  * @{
  */


extern void MC_Core_SetDutyCycleHfPwms(uint32_t *pHfTimer, uint16_t PulseValue1, uint16_t PulseValue2, uint16_t PulseValue3);
extern void MC_Core_EnableInputsHfPwmsStep14(MC_Handle_t *pMc);
extern void MC_Core_EnableInputsHfPwmsStep25(MC_Handle_t *pMc);
extern void MC_Core_EnableInputsHfPwmsStep36(MC_Handle_t *pMc);

/**
  * @}
  */

/** @defgroup MC_6STEP_SL_CM_SPDLP_3PWM_Private_Functions
  * @brief 6step core functions defined in this module
  * @{
  */

/**
  * @brief  Set the HF pwm duty cycles for each step
  * @param(in]  StepNumber
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_SixStepTable(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
{
  switch (StepNumber)
  {
    case 1:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, PulseValue, 0, 0);
      MC_Core_EnableInputsHfPwmsStep14(pMc);
    }
    break;
    case 2:      
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, PulseValue, 0, 0);
      MC_Core_EnableInputsHfPwmsStep25(pMc);
    }
    break;
    case 3:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, PulseValue, 0);
      MC_Core_EnableInputsHfPwmsStep36(pMc);
    }
    break;
    case 4:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, PulseValue, 0);
      MC_Core_EnableInputsHfPwmsStep14(pMc);
    }
    break;
    case 5:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, 0 ,PulseValue);
      MC_Core_EnableInputsHfPwmsStep25(pMc);    
    }
    break;
    case 6:      
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, 0 ,PulseValue);
      MC_Core_EnableInputsHfPwmsStep36(pMc);
    }
    break;
  } 
  return MC_FUNC_OK;
}

/* ALIGNMENT MIDWAY BEGIN 3 */
/**
  * @brief  Set the HF pwm duty cycles for each phase to perform mid alignment
  * @param(in]  StepNumber
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_SixStepTableMidAlign(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
{
  switch (StepNumber)
  {
    case 1:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, PulseValue, 0, PulseValue);
    }
    break;
    case 2:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, PulseValue, 0, 0);
    }
    break;
    case 3:
    {
     MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, PulseValue, PulseValue, 0);
    }
    break;
    case 4:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, PulseValue, 0);
    }
    break;
    case 5:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, PulseValue, PulseValue);
    }
    break;
    case 6:
    {
      MC_Core_SetDutyCycleHfPwms(pMc->phf_timer, 0, 0, PulseValue);
    }
    break;
  }
  return MC_FUNC_OK;
}
/* ALIGNMENT MIDWAY END 3 */









/**
  * @}  end MC_6STEP_SL_CM_SPDLP_3PWM
  */ 

/**
  * @}  end MC_6STEP_LIB
  */

/**
  * @}  end MIDDLEWARES
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

