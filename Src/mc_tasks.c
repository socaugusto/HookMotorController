
/**
  ******************************************************************************
  * @file    mc_tasks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition
  *
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"

#include "mc_config.h"
#include "mc_interface.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private define */
/* Private define ------------------------------------------------------------*/
#define CHARGE_BOOT_CAP_MS  10
#define CHARGE_BOOT_CAP_MS2 10
#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0
#define STOPPERMANENCY_MS  400
#define STOPPERMANENCY_MS2 400
#define CHARGE_BOOT_CAP_TICKS  (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS)/ 1000)
#define CHARGE_BOOT_CAP_TICKS2 (uint16_t)((SYS_TICK_FREQUENCY * CHARGE_BOOT_CAP_MS2)/ 1000)
#define OFFCALIBRWAITTICKS     (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000)
#define OFFCALIBRWAITTICKS2    (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS2)/ 1000)
#define STOPPERMANENCY_TICKS   (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS)/ 1000)
#define STOPPERMANENCY_TICKS2  (uint16_t)((SYS_TICK_FREQUENCY * STOPPERMANENCY_MS2)/ 1000)

/* Un-Comment this macro define in order to activate the smooth
   braking action on over voltage */
/* #define  MC.SMOOTH_BRAKING_ACTION_ON_OVERVOLTAGE */

/* USER CODE END Private define */
/* Private variables----------------------------------------------------------*/

extern MC_Handle_t Motor_Device1;

static volatile uint16_t hMFTaskCounterM1 = 0;
static volatile uint16_t hBootCapDelayCounterM1 = 0;
static volatile uint16_t hStopPermanencyCounterM1 = 0;

uint8_t bMCBootCompleted = 0;

/* USER CODE BEGIN Private Variables */



/* USER CODE END Private Variables */

/* Private functions ---------------------------------------------------------*/
MC_FuncStatus_t MC_Core_MediumFrequencyTask(void);
void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount);
bool TSK_ChargeBootCapDelayHasElapsedM1(void);
void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount);
bool TSK_StopPermanencyTimeHasElapsedM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */


/* USER CODE END Private Functions */
/**
  * @brief  It initializes the whole MC core according to user defined
  *         parameters.
  * @param  pMCIList pointer to the vector of MCInterface objects that will be
  *         created and initialized. The vector must have length equal to the
  *         number of motor drives.
  * @retval None
  */
__weak void MCboot( MC_Handle_t* pMCIList[] )
{
  /* USER CODE BEGIN MCboot 0 */
  (void)(pMCIList);
  /* USER CODE END MCboot 0 */

  bMCBootCompleted = 0;
  if (MC_Core_Init(&Motor_Device1) != MC_FUNC_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN MCboot 1 */

  /* USER CODE END MCboot 1 */

  ASPEP_start (&aspepOverUartA);
  STLNK_init (&STLNK);

  /* USER CODE BEGIN MCboot 2 */

  /* USER CODE END MCboot 2 */

  bMCBootCompleted = 1;
}

/**
 * @brief Runs all the Tasks of the Motor Control cockpit
 *
 * This function is to be called periodically at least at the Medium Frequency task
 * rate (It is typically called on the Systick interrupt). Exact invokation rate is
 * the Speed regulator execution rate set in the Motor Contorl Workbench.
 *
 * The following tasks are executed in this order:
 *
 * - Medium Frequency Tasks of each motors
 * - Safety Task
 * - Power Factor Correction Task (if enabled)
 * - User Interface task.
 */
__weak void MC_RunMotorControlTasks(void)
{
  if ( bMCBootCompleted ) {
    /* ** Medium Frequency Tasks ** */
    MC_Scheduler();

    /* Safety task is run after Medium Frequency task so that
     * it can overcome actions they initiated if needed. */
    TSK_SafetyTask();

  }
}

/**
 * @brief  Executes the Medium Frequency Task functions for each drive instance.
 *
 * It is to be clocked at the Systick frequency.
 */
__weak void MC_Scheduler(void)
{
/* USER CODE BEGIN MC_Scheduler 0 */

/* USER CODE END MC_Scheduler 0 */

  if (bMCBootCompleted == 1)
  {
    if(hMFTaskCounterM1 > 0u)
    {
      hMFTaskCounterM1--;
    }
    else
    {
      MC_Core_MediumFrequencyTask();

      MCP_Over_UartA.rxBuffer = MCP_Over_UartA.pTransportLayer->fRXPacketProcess ( MCP_Over_UartA.pTransportLayer,  &MCP_Over_UartA.rxLength);
      if (MCP_Over_UartA.rxBuffer)
      {
        /* Synchronous answer */
        if (MCP_Over_UartA.pTransportLayer->fGetBuffer (MCP_Over_UartA.pTransportLayer, (void **) &MCP_Over_UartA.txBuffer, MCTL_SYNC))
        {
          MCP_ReceivedPacket(&MCP_Over_UartA);
          MCP_Over_UartA.pTransportLayer->fSendPacket (MCP_Over_UartA.pTransportLayer, MCP_Over_UartA.txBuffer, MCP_Over_UartA.txLength, MCTL_SYNC);
        }
        else
        {
        /* no buffer available to build the answer ... should not occur */
        }
      }
      MCP_Over_STLNK.rxBuffer = MCP_Over_STLNK.pTransportLayer->fRXPacketProcess ( MCP_Over_STLNK.pTransportLayer,  &MCP_Over_STLNK.rxLength);
      if (MCP_Over_STLNK.rxBuffer)
      {
        /* Synchronous answer */
        if (MCP_Over_STLNK.pTransportLayer->fGetBuffer (MCP_Over_STLNK.pTransportLayer, (void **) &MCP_Over_STLNK.txBuffer, MCTL_SYNC))
        {
          MCP_ReceivedPacket(&MCP_Over_STLNK);
          MCP_Over_STLNK.pTransportLayer->fSendPacket (MCP_Over_STLNK.pTransportLayer, MCP_Over_STLNK.txBuffer, MCP_Over_STLNK.txLength, MCTL_SYNC);
        }
        else
        {
        /* no buffer available to build the answer ... should not occur */
        }
      }

      /* USER CODE BEGIN MC_Scheduler 1 */

      /* USER CODE END MC_Scheduler 1 */
      hMFTaskCounterM1 = MF_TASK_OCCURENCE_TICKS;
    }
    if(hBootCapDelayCounterM1 > 0u)
    {
      hBootCapDelayCounterM1--;
    }
    if(hStopPermanencyCounterM1 > 0u)
    {
      hStopPermanencyCounterM1--;
    }
  }
  else
  {
  }
  /* USER CODE BEGIN MC_Scheduler 2 */

  /* USER CODE END MC_Scheduler 2 */
}

/**
  * @brief Executes medium frequency periodic Motor Control tasks
  *
  * This function performs some of the control duties on Motor 1 according to the
  * present state of its state machine. In particular, duties requiring a periodic
  * execution at a medium frequency rate (such as the speed controller for instance)
  * are executed here.
  */
__weak MC_FuncStatus_t MC_Core_MediumFrequencyTask()
{
  switch (Motor_Device1.status)
  {
    case MC_RUN:
    {
       Motor_Device1.speed_target_value = Motor_Device1.speed_target_command;
       MC_Core_DisableIrq();
       Motor_Device1.pulse_value = MC_Core_SpeedControl(&Motor_Device1);
      Motor_Device1.hf_timer_pulse_value_max = Motor_Device1.pulse_value;
		MC_Core_SetDutyCycleHfPwmForStepN(Motor_Device1.phf_timer, Motor_Device1.hf_timer_pulse_value_max, Motor_Device1.step_position);
      MC_Core_EnableIrq();
		if (Motor_Device1.adc_user.number_of_channels != 0) MC_Core_SetDutyCyclePwmForAdcTrig(Motor_Device1.adc_user.ptrig_timer, Motor_Device1.adc_user.trig_timer_channel, Motor_Device1.adc_user.trig_time);
  }
	break;
    case MC_ALIGNMENT:
    {
        MC_Core_AlignmentToCurrentStep(&Motor_Device1);
    }
    break;
	default:
	break;
  }
  return MC_FUNC_OK;
}

/**
  * @brief  Process the ADC measurement
  * @param[in] pAdc pointer to the ADC
  * @param[in] LfTimerCounterSnapshot value of the LF timer counter at the beginning of the ADC callback
  * @param[in] AdcMeasurement the ADC conversion result, aka the ADC measurement
  * @retval  Function Status
  */
MC_FuncStatus_t  MC_Core_ProcessAdcMeasurement(uint32_t* pAdc, uint16_t LfTimerCounterSnapshot, uint16_t AdcMeasurement)
{
  (void)(LfTimerCounterSnapshot);
  (void)(AdcMeasurement);

  uint8_t tmp = Motor_Device1.adc_user.channel_index;
  if (Motor_Device1.adc_user.number_of_channels != 0)
  {

	if (pAdc == Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index])
	{
		/* Process user measurement */
		Motor_Device1.adc_user.measurement[Motor_Device1.adc_user.channel_index] = AdcMeasurement;
	}

	(Motor_Device1.adc_user.channel_index)++;
	if (Motor_Device1.adc_user.channel_index == Motor_Device1.adc_user.number_of_channels)
	{
		Motor_Device1.adc_user.channel_index = 0;
	}
	MC_Core_SelectAdcChannel(Motor_Device1.adc_user.padc[tmp], Motor_Device1.adc_user.padc[Motor_Device1.adc_user.channel_index], Motor_Device1.adc_user.channel[Motor_Device1.adc_user.channel_index], Motor_Device1.adc_user.sampling_time[Motor_Device1.adc_user.channel_index]);
  }

  return MC_FUNC_OK;
}

/**
  * @brief  It set a counter intended to be used for counting the delay required
  *         for drivers boot capacitors charging of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetChargeBootCapDelayM1(uint16_t hTickCount)
{
   hBootCapDelayCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the time required to charge boot
  *         capacitors of motor 1 has elapsed
  * @param  none
  * @retval bool true if time has elapsed, false otherwise
  */
__weak bool TSK_ChargeBootCapDelayHasElapsedM1(void)
{
  bool retVal = false;
  if (hBootCapDelayCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  It set a counter intended to be used for counting the permanency
  *         time in STOP state of motor 1
  * @param  hTickCount number of ticks to be counted
  * @retval void
  */
__weak void TSK_SetStopPermanencyTimeM1(uint16_t hTickCount)
{
  hStopPermanencyCounterM1 = hTickCount;
}

/**
  * @brief  Use this function to know whether the permanency time in STOP state
  *         of motor 1 has elapsed
  * @param  none
  * @retval bool true if time is elapsed, false otherwise
  */
__weak bool TSK_StopPermanencyTimeHasElapsedM1(void)
{
  bool retVal = false;
  if (hStopPermanencyCounterM1 == 0)
  {
    retVal = true;
  }
  return (retVal);
}

/**
  * @brief  Executes safety checks (e.g. bus voltage and temperature) for all drive instances.
  *
  * Faults flags are updated here.
  */
__weak void TSK_SafetyTask(void)
{
  /* USER CODE BEGIN TSK_SafetyTask 0 */

  /* USER CODE END TSK_SafetyTask 0 */
  if (bMCBootCompleted == 1)
  {
    TSK_SafetyTask_PWMOFF(M1);
  /* USER CODE BEGIN TSK_SafetyTask 1 */

  /* USER CODE END TSK_SafetyTask 1 */
  }
}

/**
  * @brief  Safety task implementation if  MC.ON_OVER_VOLTAGE == TURN_OFF_PWM
  * @param  bMotor Motor reference number defined
  *         \link Motors_reference_number here \endlink
  * @retval None
  */
__weak void TSK_SafetyTask_PWMOFF(uint8_t bMotor)
{
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 0 */
  (void)(bMotor);

  /* USER CODE END TSK_SafetyTask_PWMOFF 0 */
  /* USER CODE BEGIN TSK_SafetyTask_PWMOFF 3 */

  /* USER CODE END TSK_SafetyTask_PWMOFF 3 */
}

/**
  * @brief  Puts the Motor Control subsystem in in safety conditions on a Hard Fault
  *
  *  This function is to be executed when a general hardware failure has been detected
  * by the microcontroller and is used to put the system in safety condition.
  */
__weak void TSK_HardwareFaultTask(void)
{
  /* USER CODE BEGIN TSK_HardwareFaultTask 0 */

  /* USER CODE END TSK_HardwareFaultTask 0 */
  /* USER CODE BEGIN TSK_HardwareFaultTask 1 */

  /* USER CODE END TSK_HardwareFaultTask 1 */
}

__weak void UI_HandleStartStopButton_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  MC_Status_t status;
  status = MC_Core_GetStatus(&Motor_Device1);
  if (status == MC_RUN)
  {
    MC_Core_Stop(&Motor_Device1);
    MC_Core_Reset(&Motor_Device1);
  }
  else if (status == MC_STOP)
  {
		MC_Core_SetDirection(&Motor_Device1, 0);	
    MC_Core_Start(&Motor_Device1);
  }
/* USER CODE END START_STOP_BTN */
}

__weak void UI_HandleStartStopButton1_cb (void)
{
/* USER CODE BEGIN START_STOP_BTN */
  MC_Status_t status;
  status = MC_Core_GetStatus(&Motor_Device1);
  if (status == MC_RUN)
  {
    MC_Core_Stop(&Motor_Device1);
    MC_Core_Reset(&Motor_Device1);
  }
  else if (status == MC_STOP)
  {
		MC_Core_SetDirection(&Motor_Device1, 1);	
    MC_Core_Start(&Motor_Device1);
  }
/* USER CODE END START_STOP_BTN */
}

 /**
  * @brief  Locks GPIO pins used for Motor Control to prevent accidental reconfiguration
  */
__weak void mc_lock_pins (void)
{
LL_GPIO_LockPin(M1_VBUS_SENSE_GPIO_Port, M1_VBUS_SENSE_Pin);
LL_GPIO_LockPin(M1_CURRENT_SENSE_GPIO_Port, M1_CURRENT_SENSE_Pin);
LL_GPIO_LockPin(M1_HALL_H2_GPIO_Port, M1_HALL_H2_Pin);
LL_GPIO_LockPin(M1_HALL_H3_GPIO_Port, M1_HALL_H3_Pin);
LL_GPIO_LockPin(M1_HALL_H1_GPIO_Port, M1_HALL_H1_Pin);
LL_GPIO_LockPin(M1_PWM_UH_GPIO_Port, M1_PWM_UH_Pin);
LL_GPIO_LockPin(M1_PWM_VH_GPIO_Port, M1_PWM_VH_Pin);
LL_GPIO_LockPin(M1_OCP_GPIO_Port, M1_OCP_Pin);
LL_GPIO_LockPin(M1_PWM_VL_GPIO_Port, M1_PWM_VL_Pin);
LL_GPIO_LockPin(M1_PWM_WH_GPIO_Port, M1_PWM_WH_Pin);
LL_GPIO_LockPin(M1_PWM_WL_GPIO_Port, M1_PWM_WL_Pin);
LL_GPIO_LockPin(M1_PWM_UL_GPIO_Port, M1_PWM_UL_Pin);
}
/* USER CODE BEGIN mc_task 0 */

/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
