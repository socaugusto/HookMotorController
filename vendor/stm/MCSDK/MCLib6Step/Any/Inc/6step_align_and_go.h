/**
  ******************************************************************************
  * @file    6step_align_and_go.h 
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
#ifndef __6STEP_ALIG_AND_GO_H
#define __6STEP_ALIG_AND_GO_H

/* Includes ------------------------------------------------------------------*/
#include "6step_core.h"


#define MC_NUMBER_OF_STEPS_IN_6STEP_ALGO  ((uint8_t)  6)
#define MC_SECONDS_PER_MINUTE             ((uint8_t)  60)





/** @defgroup MC_6STEP_SL_VM_SPDLP_Private_Functions_Prototypes
  * @{
  */
MC_FuncStatus_t MC_Core_Alignment(MC_Handle_t *pMc);
MC_FuncStatus_t MC_Core_RampCalc(MC_Handle_t* pMc);


#endif
