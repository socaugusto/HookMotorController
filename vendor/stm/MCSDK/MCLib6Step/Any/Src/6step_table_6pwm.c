/**
  ******************************************************************************
  * @file    6step_table_6pwm.c
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
  
#include "6step_table_6pwm.h"

   
/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{ 
  */

/** @addtogroup MC_6STEP_LIB
  * @{
  */

/** @defgroup MC_6STEP_CORE 
  * @brief 6step core module
  * @{
  */ 

/** @defgroup MC_6STEP_CORE_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_CORE_Private_Defines
  * @{
  */

/**
  * @}
  */ 

/** @defgroup MC_6STEP_CORE_Private_Macros
* @{
*/ 
/**
  * @}
  */ 

/** @defgroup MC_6STEP_CORE_Private_Functions_Prototypes
  * @{
  */
extern void MC_Core_SetDutyCycleHfPwmU(uint32_t *pHfTimer, uint16_t PulseValue);
extern void MC_Core_SetDutyCycleHfPwmV(uint32_t *pHfTimer, uint16_t PulseValue);
extern void MC_Core_SetDutyCycleHfPwmW(uint32_t *pHfTimer, uint16_t PulseValue);
extern void MC_Core_EnableChannelsHfPwmsStep14(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep25(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep36(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep1(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep2(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep3(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep4(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep5(uint32_t *pHfTimer);
extern void MC_Core_EnableChannelsHfPwmsStep6(uint32_t *pHfTimer);
extern void MC_Core_DisableUpdateEvent(uint32_t *pHfTimer);
extern void  MC_Core_EnableChannelsHfPwmsStep1FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
extern void  MC_Core_EnableChannelsHfPwmsStep3FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
extern void  MC_Core_EnableChannelsHfPwmsStep5FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
/**
  * @}
  */ 

/** @defgroup MC_6STEP_CORE_Private_Variables
  * @{
  */
//extern MC_Handle_t Motor_Device1;

/**
  * @} end MC_6STEP_CORE_Private_Variables
  */

/** @defgroup MC_6STEP_CORE_Private_Functions
  * @{
  */



/**
  * @} end MC_6STEP_CORE_Private_Functions
  */ 

/** @defgroup MC_6STEP_CORE_Exported_Functions
  * @{
  */

/**
  * @brief  Set the HF pwm duty cycles for each step
  * @param[in] pMc motor control handle
  * @param(in]  PulseValue
  * @param(in]  StepNumber
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_SixStepTable(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
{
  switch (StepNumber)
  {
    case 1:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_EnableChannelsHfPwmsStep14(pMc->phf_timer);
    }
    break;
    case 2:
    {
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep25(pMc->phf_timer);
    }
    break;
    case 3:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
      MC_Core_EnableChannelsHfPwmsStep36(pMc->phf_timer);
    }
    break;
    case 4:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep14(pMc->phf_timer);
    }
    break;
    case 5:
    {
       MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
       MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
       MC_Core_EnableChannelsHfPwmsStep25(pMc->phf_timer);
    }
    break;
    case 6:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep36(pMc->phf_timer);
    }
    break;
  }
  return MC_FUNC_OK;
}

/**
  * @brief  Set the HF pwm duty cycles for each step
  * @param[in] pMc motor control handle
  * @param(in]  PulseValue
  * @param(in]  StepNumber
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_SixStepTableQuasiSyncRect(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
{
  switch (StepNumber)
  {
    case 1:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_EnableChannelsHfPwmsStep1(pMc->phf_timer);
    }
    break;
    case 2:
    {
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep2(pMc->phf_timer);
    }
    break;
    case 3:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
      MC_Core_EnableChannelsHfPwmsStep3(pMc->phf_timer);
    }
    break;
    case 4:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep4(pMc->phf_timer);
    }
    break;
    case 5:
    {
       MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
       MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
       MC_Core_EnableChannelsHfPwmsStep5(pMc->phf_timer);
    }
    break;
    case 6:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep6(pMc->phf_timer);
    }
    break;
  }
  return MC_FUNC_OK;
}

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
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
    }
    break;
    case 2:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0); 
    }
    break;
    case 3:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
    }
    break;
    case 4:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
  
    }
    break;
    case 5:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
    }
    break;
    case 6:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
    }
    break;
  }
  return MC_FUNC_OK;
}

/**
  * @brief  Set the HF pwm duty cycles for each step
  * @param[in] pMc motor control handle
  * @param(in]  PulseValue
  * @param(in]  StepNumber
  * @retval  Function Status
  */
MC_FuncStatus_t MC_Core_SixStepTableFastDemag(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber)
{ 
  uint32_t PWM_ch_polarity, PWM_Nch_polarity;
  if (pMc->channel_polarity == LL_TIM_OCPOLARITY_LOW) PWM_ch_polarity = LL_TIM_OCPOLARITY_HIGH;
  else PWM_ch_polarity = LL_TIM_OCPOLARITY_LOW;
  if (pMc->Nchannel_polarity == LL_TIM_OCPOLARITY_LOW) PWM_Nch_polarity = LL_TIM_OCPOLARITY_HIGH;
  else PWM_Nch_polarity = LL_TIM_OCPOLARITY_LOW;
  MC_Core_DisableUpdateEvent(pMc->phf_timer);
  switch (StepNumber)
  {
    case 1:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep1FastDemag(pMc->phf_timer, PWM_ch_polarity, PWM_Nch_polarity);
    }
    break;
    case 2:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
      MC_Core_EnableChannelsHfPwmsStep25(pMc->phf_timer);
    }
    break;
    case 3:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep3FastDemag(pMc->phf_timer, PWM_ch_polarity, PWM_Nch_polarity);
    }
    break;
    case 4:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, PulseValue);   
      MC_Core_EnableChannelsHfPwmsStep14(pMc->phf_timer); 
    }
    break;
    case 5:
    {
      MC_Core_SetDutyCycleHfPwmU(pMc->phf_timer, PulseValue);     
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, 0);
      MC_Core_EnableChannelsHfPwmsStep5FastDemag(pMc->phf_timer, PWM_ch_polarity, PWM_Nch_polarity);    
    }
    break;
    case 6:
    {
      MC_Core_SetDutyCycleHfPwmV(pMc->phf_timer, 0);
      MC_Core_SetDutyCycleHfPwmW(pMc->phf_timer, PulseValue);
      MC_Core_EnableChannelsHfPwmsStep36(pMc->phf_timer); 
    }
    break;
  }
  return MC_FUNC_OK;
}

/**
  * @} end MC_6STEP_CORE_Exported_Functions
  */ 

/**
  * @}  end MC_6STEP_CORE
  */ 

/**
  * @}  end MC_6STEP_LIB
  */

/**
  * @}  end MIDDLEWARES
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

