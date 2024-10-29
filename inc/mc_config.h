/**
 ******************************************************************************
 * @file    mc_config.h
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Motor Control Subsystem components configuration and handler
 *          structures declarations.
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

#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

#include "6step_core.h"
#include "6step_PID.h"
#include "6step_synch.h"
#include "hardware_conf.h"
#include "motor_conf.h"
#include "pmsm_motor_parameters.h"
#include "state_machine.h"
#include "mc_interface.h"
#include "6step_synch_hall.h"
#include "6step_table_6pwm.h"
#include "6step_service_f0xx.h"

extern MC_Handle_t Motor_Device1;
extern MC_Handle_t *pMCI;

MC_Handle_t *MC_Core_GetMotorControlHandle(uint8_t MotorDeviceId);
MC_FuncStatus_t MC_Core_SpeedRegulatorReset(MC_Handle_t *pMc);
MC_FuncStatus_t MC_Core_Init(MC_Handle_t *pMc);
MC_FuncStatus_t MC_Core_Reset(MC_Handle_t *pMc);
#endif /* __MC_CONFIG_H */
/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
