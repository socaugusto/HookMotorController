/**
  ******************************************************************************
  * @file    mc_configuration_registers.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides project configuration information registers.
  *
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "mc_configuration_registers.h"
#include "register_interface.h"
#include "parameters_conversion.h"

/* TODO : Replace board names by data coming from data model*/
char CTL_BOARD[] = "Nucleo G431RE";
char M1_PWR_BOARD[] = "IHM07 - Three shunt version";
char M1_MOTOR_NAME [] = "Shinano motor";

GlobalConfig_reg_t globalConfig_reg =
{
  .SDKVersion =  SDK_VERSION,
  .MotorNumber =  1 ,
};

MotorConfig_reg_t M1_MotorConfig_reg =
{
  .maxMechanicalSpeed = 4550,
  .maxMotorCurrent = 0xA, /* Information not available yet */
  .maxVoltageSupply = 45,
  .minVoltageSupply = 8,
  .driveType = DRIVE_TYPE_M1,
};

FOCFwConfig_reg_t M1_FOCConfig_reg =
{
  .primarySensor = (uint8_t) PRIM_SENSOR_M1,
  .auxiliarySensor = (uint8_t) AUX_SENSOR_M1,
  .topology = (uint8_t) TOPOLOGY_M1,
  .FOCRate = (uint8_t) FOC_RATE_M1,
  .PWMFrequency = (uint32_t) PWM_FREQ_M1,
  .MediumFrequency = (uint16_t) MEDIUM_FREQUENCY_TASK_RATE,
  .configurationFlag1 = (uint16_t) configurationFlag1_M1,
  .configurationFlag2 = (uint16_t) configurationFlag2_M1
};

char * PWR_BOARD_NAME[NBR_OF_MOTORS] = {M1_PWR_BOARD};
char * MOTOR_NAME[NBR_OF_MOTORS] = {M1_MOTOR_NAME};
FOCFwConfig_reg_t* FOCConfig_reg[NBR_OF_MOTORS]={ &M1_FOCConfig_reg };
MotorConfig_reg_t* MotorConfig_reg[NBR_OF_MOTORS]={ &M1_MotorConfig_reg };

/************************ (C) COPYRIGHT 2021 STMicroelectronics *****END OF FILE****/
