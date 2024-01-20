/**
  ******************************************************************************
  * @file    6step_6pwm_table.h 
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   Declaration of 6step_table function
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
#ifndef __6STEP_6PWM_TABLE_H
#define __6STEP_6PWM_TABLE_H

/* Includes ------------------------------------------------------------------*/

#include "6step_core.h"
MC_FuncStatus_t MC_Core_SixStepTable(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
MC_FuncStatus_t MC_Core_SixStepTableQuasiSyncRect(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
MC_FuncStatus_t MC_Core_SixStepTableMidAlign(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);
MC_FuncStatus_t MC_Core_SixStepTableFastDemag(MC_Handle_t *pMc, uint16_t PulseValue, uint8_t StepNumber);


#endif

