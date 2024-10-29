/**
  ******************************************************************************
  * @file    6step_SetFunctions.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Declarations of service function that prepares the handle  
  *          to the NextStep application
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __6STEP_SET_FUNCTIONS_3PWM_6PWM_H
#define __6STEP_SET_FUNCTIONS_3PWM_6PWM_H

/* Includes ------------------------------------------------------------------*/


#include "6step_core.h"
   
void MC_Core_SetPWM_Enable(MC_Handle_t* pMc);
MC_FuncStatus_t MC_Core_SetHFPWM_3pwm(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
MC_FuncStatus_t MC_Core_SetHFPWM_6pwm(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);

#endif
