/**
  ******************************************************************************
  * @file    6step_synch_sl.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Declarations of function that prepares the handle  
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
#ifndef __6STEP_SYNC_SL_H
#define __6STEP_SYNC_SL_H

/* Includes ------------------------------------------------------------------*/


#include "6step_core.h"
   
MC_FuncStatus_t MC_Core_PrepareNextStep(MC_Handle_t* pMc);
MC_FuncStatus_t MC_Core_NextStep(MC_Handle_t *pMc, uint16_t HfTimerCounterSnapshot);
MC_FuncStatus_t MC_Core_HallPrepareNextStep(MC_Handle_t* pMc);
MC_FuncStatus_t MC_Core_HallNextStep(MC_Handle_t *pMc, uint16_t HfTimerCounterSnapshot);
MC_FuncStatus_t MC_Core_SetAdcBemfTrigTime(MC_Handle_t* pMc, uint32_t DutyCycleToSet);
MC_FuncStatus_t MC_Core_LfTimerPeriodCompute(MC_Handle_t* pMc, uint16_t CounterSnaphot, uint8_t BemfIsIncreasing);

#endif
