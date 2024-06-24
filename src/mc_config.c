
/**
 ******************************************************************************
 * @file    mc_config.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Motor Control Subsystem components configuration and handler structures.
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
#include "main.h"
#include "mc_type.h"
#include "parameters_conversion.h"
#include "mc_interface.h"
#include "mc_config.h"

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */
#define MC_REGULATOR_MIN_MAX_SCALING_DIV ((uint16_t)1000)
/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

MC_Handle_t Motor_Device1 =
    {
        .phf_timer = (uint32_t *)TIM1,
        .plf_timer = (uint32_t *)TIM2,
        .pref_timer = NULL,
        .channel_polarity = LL_TIM_OCPOLARITY_HIGH,
        .Nchannel_polarity = LL_TIM_OCPOLARITY_HIGH,
        .hall.HALL_IOs[0].HALLx_GPIO_Port = M1_HALL_H1_GPIO_Port,
        .hall.HALL_IOs[0].HALLx_GPIO_Pin = M1_HALL_H1_Pin,
        .hall.HALL_IOs[1].HALLx_GPIO_Port = M1_HALL_H2_GPIO_Port,
        .hall.HALL_IOs[1].HALLx_GPIO_Pin = M1_HALL_H2_Pin,
        .hall.HALL_IOs[2].HALLx_GPIO_Port = M1_HALL_H3_GPIO_Port,
        .hall.HALL_IOs[2].HALLx_GPIO_Pin = M1_HALL_H3_Pin,
        .enPWMx[0].enPWMx_GPIO_Port = NULL,
        .enPWMx[0].enPWMx_GPIO_Pin = (uint16_t)((uint32_t)NULL),
        .enPWMx[1].enPWMx_GPIO_Port = NULL,
        .enPWMx[1].enPWMx_GPIO_Pin = (uint16_t)((uint32_t)NULL),
        .enPWMx[2].enPWMx_GPIO_Port = NULL,
        .enPWMx[2].enPWMx_GPIO_Pin = (uint16_t)((uint32_t)NULL),
        .enDRIVER.enDriver_GPIO_Port = NULL,
};

uint8_t NumberOfDevices = 0;
MC_Handle_t *pMcCoreArray[NUMBER_OF_DEVICES];
uint32_t SysClockFrequency = 0;

MC_FuncStatus_t MC_Core_SetPulse(MC_Handle_t *pMc, uint16_t *pPulse, uint32_t Period, uint32_t DutyCycleToSet);
MC_FuncStatus_t MC_Core_SetGateDriverPwmFreq(MC_Handle_t *pMc, uint32_t FrequencyHzToSet);
MC_FuncStatus_t MC_Core_SpeedFeedbackReset(MC_Handle_t *pMc);
MC_FuncStatus_t MC_Core_InitAdcUser(MC_Handle_t *pMc);

uint16_t pidKp = PID_KP;
uint16_t pidKi = PID_KI;
uint16_t pidKd = PID_KD;
uint16_t pidScaling = PID_SCALING_SHIFT;
uint16_t pidOutMin = PID_OUTPUT_MIN;
uint16_t pidOutMax = PID_OUTPUT_MAX;

/* USER CODE BEGIN Additional include */

/* USER CODE END Additional include */

/* USER CODE BEGIN Additional define */

/* USER CODE END Additional define */

/**
 * @brief  MC_Core_GetMotorControlHandle
 *         Initializes the motor control
 * @param[in] MotorDeviceId
 * @retval motor control handle
 */
MC_Handle_t *MC_Core_GetMotorControlHandle(uint8_t MotorDeviceId)
{
  if (NumberOfDevices != 0)
  {
    if (MotorDeviceId < NUMBER_OF_DEVICES)
    {
      return pMcCoreArray[MotorDeviceId];
    }
    else
    {
      return NULL;
    }
  }
  else
  {
    return NULL;
  }
}

/**
 * @brief  MC_Core_Init
 *         Initializes the motor control
 * @param[in] pMc motor control handle
 * @retval Function Status
 */
MC_FuncStatus_t MC_Core_Init(MC_Handle_t *pMc)
{
  pMc->hf_timer_period = MC_Core_GetTimerPeriod(pMc->phf_timer);
  if (pMc->pref_timer != NULL)
  {
    pMc->ref_timer_period = pMc->hf_timer_period;
  }
  /* Check the motor control handle and the mandatory timers */
  if ((pMc == NULL) || (pMc->plf_timer == NULL) || (pMc->phf_timer == NULL))
  {
    return MC_FUNC_FAIL;
  }
  else
  {
    if (NumberOfDevices < NUMBER_OF_DEVICES)
    {
      pMc->id = NumberOfDevices;
      pMcCoreArray[NumberOfDevices] = pMc;
      NumberOfDevices++;
    }
    else
    {
      return MC_FUNC_FAIL;
    }
  }

  /* Set Device initial State */
  pMc->status = MC_IDLE;

  //  MC_Core_GetTemperatureCalibrationData(pMc);
  //  if (MC_Core_InitAdcUser(pMc) != MC_FUNC_OK)
  //  {
  //    return MC_FUNC_FAIL;
  //  }

  /* Motor characteristics initialisation */
  pMc->motor_charac.pole_pairs = POLE_PAIR_NUM;

  /* Motor parameters and private variable initialisation */
  if (MC_Core_Reset(pMc) != MC_FUNC_OK)
  {
    return MC_FUNC_FAIL;
  }
  SysClockFrequency = MC_Core_GetSysClockFrequency();
  MC_Core_SetGateDriverPwmFreq(pMc, MC_Core_GetTimerFrequency(pMc->phf_timer));

  /* Set Device State at the end of initialization */
  pMc->status = MC_STOP;
  pMc->const_startup_direction = STARTUP_DIRECTION;
  pMc->direction = STARTUP_DIRECTION;
  return MC_FUNC_OK;
}

/**
 * @brief  MC_Core_Reset
 * @retval  Function Status
 */
MC_FuncStatus_t MC_Core_Reset(MC_Handle_t *pMc)
{
  /* Motor parameters reset */

  uint16_t startup_duty_cycle_value = STARTUP_DUTY_CYCLE;
  uint32_t sys_clk_period_ns = 1000000000 / MC_Core_GetSysClockFrequency();
  pMc->hall.commutation_errors = 0;
  pMc->hall.commutation_errors_max = RUN_COMMUTATION_ERRORS_MAX;
  pMc->hall.commutation_delay = RUN_COMMUTATION_DELAY;
  pMc->hall.alignement_step_change = ALIGNMENT_FORCE_STEP_CHANGE;
  pMc->hall.stay_in_run_lf_cnt = 0;
  pMc->hall.stay_in_run_lf_max = (RUN_STAY_WHILE_STALL_MS * 1000) / ((MC_LF_TIMER_MAX_PERIOD * sys_clk_period_ns * (MC_Core_GetTimerPrescaler(pMc->plf_timer) + 1)) / 1000);
  pMc->adc_user.trig_time = (uint16_t)((pMc->adc_user.trig_timer_period * USER_ADC_TRIG_TIME) >> MC_DUTY_CYCLE_SCALING_SHIFT);
  pMc->alignment_time = ALIGNMENT_TIME;
  pMc->control_loop_time = RUN_CONTROL_LOOP_TIME;
  pMc->step_position = 0;

  /* Private variables reset */
  pMc->uw_tick_cnt = 0;
  pMc->align_index = 0;
  pMc->lf_timer_period = MC_LF_TIMER_MAX_PERIOD;
  pMc->step_prepared = 0;
  pMc->tick_cnt = 0;
  pMc->adc_user.channel_index = 0;

  /* POTENTIOMETER BEGIN 3 */
  /* Reset the filter for the command computed from the ADC USER measurement */
  pMc->adc_user.spd_dc_command.filtered = 0;
  /* POTENTIOMETER END  3 */

  /* Store the duty cycle to be programmed in MC_ALIGNMENT state */
  /* PWM INTERFACE BEGIN 4 */
  MC_Core_SetPulse(pMc, &(pMc->startup_reference), pMc->hf_timer_period, startup_duty_cycle_value);
  /* PWM INTERFACE END 4 */

  /* Speed variables reset */
  MC_Core_SpeedFeedbackReset(pMc);

  /* Speed PID regulator parameters */
  MC_Core_SpeedRegulatorReset(pMc);

  return MC_FUNC_OK;
}

/**
 * @brief  MC_Core_MC_Core_SetGateDriverPwmFreq
 * @param[in] pMc motor control handle
 * @param[in] FrequencyHzToSet frequency in Hz to be set
 * @retval Function Status
 */
MC_FuncStatus_t MC_Core_SetGateDriverPwmFreq(MC_Handle_t *pMc, uint32_t FrequencyHzToSet)
{
  if (pMc->status != MC_STOP)
  {
    MC_Core_Stop(pMc);
  }
  pMc->hf_timer_period = (SysClockFrequency / ((FrequencyHzToSet << MC_Core_GetTimerCounterMode(pMc->phf_timer)) * (MC_Core_GetTimerPrescaler(pMc->phf_timer) + 1)) - 1);
  if (pMc->hf_timer_period != 0)
  {
    pMc->gate_driver_frequency = FrequencyHzToSet;
    MC_Core_SetPeriodTimer(pMc->phf_timer, pMc->hf_timer_period);
    if (pMc->pref_timer != NULL)
    {
      pMc->ref_timer_period = pMc->hf_timer_period;
      MC_Core_SetPeriodTimer(pMc->pref_timer, pMc->hf_timer_period);
    }
    if (pMc->bemf.ptrig_timer != NULL)
    {
      pMc->bemf.trig_timer_period = pMc->hf_timer_period;
      MC_Core_SetPeriodTimer(pMc->bemf.ptrig_timer, pMc->hf_timer_period);
    }
    pMc->adc_user.trig_timer_period = pMc->hf_timer_period;
    if (Motor_Device1.adc_user.number_of_channels != 0)
      MC_Core_SetPeriodTimer(pMc->adc_user.ptrig_timer, pMc->hf_timer_period);
    MC_Core_Reset(pMc);
    return MC_FUNC_OK;
  }
  else
  {
    return MC_FUNC_FAIL;
  }
}

/**
 * @brief  MC_Core_SpeedFeedbackReset
 * @param[in] pMc motor control handle
 * @retval  Function Status
 */
MC_FuncStatus_t MC_Core_SpeedFeedbackReset(MC_Handle_t *pMc)
{
  pMc->speed_fdbk_filtered = 0;
  pMc->lf_timer_period_array_completed = FALSE;
  pMc->lf_timer_period_array_index = 0;
  for (int16_t i = MC_SPEED_ARRAY_SIZE - 1; i >= 0; i--)
  {
    pMc->lf_timer_period_array[i] = MC_LF_TIMER_MAX_PERIOD;
  }
  return MC_FUNC_OK;
}

/**
 * @brief  MC_Core_SpeedRegulatorReset
 * @retval  Function Status
 */
MC_FuncStatus_t MC_Core_SpeedRegulatorReset(MC_Handle_t *pMc)
{
  pMc->speed_target_value = STARTUP_SPEED_TARGET;
  pMc->speed_target_command = RUN_SPEED_TARGET;
  pMc->pid_parameters.kp = pidKp;
  pMc->pid_parameters.ki = ((pidKi) * (pMc->control_loop_time));
  pMc->pid_parameters.kd = ((pidKd) / (pMc->control_loop_time));
  pMc->pid_parameters.scaling_shift = pidScaling;
  pMc->pid_parameters.minimum_output = ((pidOutMin) * (pMc->hf_timer_period)) / MC_REGULATOR_MIN_MAX_SCALING_DIV;
  pMc->pid_parameters.maximum_output = ((pidOutMax) * (pMc->hf_timer_period)) / MC_REGULATOR_MIN_MAX_SCALING_DIV;
  pMc->pid_parameters.integral_term_sum = 0;
  pMc->pid_parameters.previous_speed = 0;
  return MC_FUNC_OK;
}

/**
 * @brief  MC_Core_SetPulse
 * @param[in] pPulse pointer to the pulse command to update
 * @param[in] Period Period to use with the DutyCycleToSet to compute the pulse command
 * @param[in] DutyCycleToSet Duty cycle in 1/1024 of PWM period
 * @retval Function Status
 */
MC_FuncStatus_t MC_Core_SetPulse(MC_Handle_t *pMc, uint16_t *pPulse, uint32_t Period, uint32_t DutyCycleToSet)
{
  (void)(pMc);
  *pPulse = (uint16_t)((DutyCycleToSet * Period) >> MC_DUTY_CYCLE_SCALING_SHIFT);
  return MC_FUNC_OK;
}

/**
 * @brief  MC_Core_InitAdcUser
 *         One time initialization of the ADC parameters inside the motor control structure
 * @param[in] pMc motor control handle
 * @retval Function Status
 */
MC_FuncStatus_t MC_Core_InitAdcUser(MC_Handle_t *pMc)
{
  pMc->adc_user.padc[0] = (uint32_t *)ADC1;
  pMc->adc_user.channel[0] = LL_ADC_CHANNEL_9;
  pMc->adc_user.sampling_time[0] = 6;
  MC_Core_SetAdcSamplingTime(pMc->adc_user.padc[0], pMc->adc_user.channel[0], pMc->adc_user.sampling_time[0]);
  pMc->adc_user.padc[1] = (uint32_t *)ADC1;
  pMc->adc_user.channel[1] = LL_ADC_CHANNEL_9;
  pMc->adc_user.sampling_time[1] = 2;
  MC_Core_SetAdcSamplingTime(pMc->adc_user.padc[1], pMc->adc_user.channel[1], pMc->adc_user.sampling_time[1]);
  pMc->adc_user.padc[2] = (uint32_t *)ADC1;
  pMc->adc_user.channel[2] = LL_ADC_CHANNEL_5;
  pMc->adc_user.sampling_time[2] = 2;
  MC_Core_SetAdcSamplingTime(pMc->adc_user.padc[2], pMc->adc_user.channel[2], pMc->adc_user.sampling_time[2]);
  pMc->adc_user.ptrig_timer = (uint32_t *)TIM1;
  pMc->adc_user.trig_timer_channel = LL_TIM_CHANNEL_CH4;
  pMc->adc_user.trig_timer_period = pMc->hf_timer_period;
  pMc->adc_user.number_of_channels = 3;
  pMc->adc_user.resolution = MC_Core_GetAdcResolution(pMc, pMc->adc_user.padc[0]);
  return MC_FUNC_OK;
}

/* USER CODE BEGIN Additional configuration */

/* USER CODE END Additional configuration */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
