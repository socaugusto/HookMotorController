/**
 ******************************************************************************
 * @file    6step_core.h
 * @author  IPC Rennes
 * @brief   Header file for 6step_core.c file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __6STEP_SERVICE_H
#define __6STEP_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "6step_core.h"

/**
  * @} end MC_6STEP_CORE_Exported_Types
  */
  

/** @defgroup MC_6STEP_CORE_Exported_Functions_Prototypes
  * @{
  */
void MC_Core_GetTemperatureCalibrationData(MC_Handle_t *pMc);
void MC_Core_CalibrateAdc(uint32_t *pAdc);
void MC_Core_ConfigureCommutationEvent(uint32_t *pHfTimer, uint32_t *pLfTimer);
void MC_Core_HallConfigureCommutationEvent(uint32_t *pHfTimer, uint32_t *pLfTimer);
void MC_Core_DisableIrq(void);
void MC_Core_DisableUpdateEvent(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep1FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
void MC_Core_EnableChannelsHfPwmsStep3FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
void MC_Core_EnableChannelsHfPwmsStep5FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
void MC_Core_EnableChannelsHfPwmsStep1(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep2(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep3(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep4(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep5(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep6(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep14(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep25(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsStep36(uint32_t *pHfTimer);
void MC_Core_EnableChannelsHfPwmsAll(uint32_t *pHfTimer);
void MC_Core_EnableInputsHfPwmsStep14(uint8_t MotorDeviceId);
void MC_Core_EnableInputsHfPwmsStep25(uint8_t MotorDeviceId);
void MC_Core_EnableInputsHfPwmsStep36(uint8_t MotorDeviceId);
void MC_Core_EnableIrq(void);
void MC_Core_EnableItBreak(uint32_t *pTimer);
void MC_Core_EnableUpdateEvent(uint32_t *pHfTimer);
void MC_Core_GenerateComEvent(uint32_t *pHfTimer);
void MC_Core_GenerateUpdateEvent(uint32_t *pTimer);
uint8_t MC_Core_GetAdcResolution(MC_Handle_t *pMc, uint32_t *pAdc);
uint32_t MC_Core_GetTimerCounterMode(uint32_t *pTimer);
uint32_t MC_Core_GetTimerFrequency(uint32_t *pTimer);
void MC_Core_GetHallStatus(MC_Handle_t *pMc);
uint32_t MC_Core_GetSysClockFrequency(void);
uint16_t MC_Core_GetTimerCaptureCompare(uint32_t *pTimer);
uint16_t MC_Core_GetTimerCounter(uint32_t *pTimer);
uint16_t MC_Core_GetTimerPeriod(uint32_t *pTimer);
uint16_t MC_Core_GetTimerPrescaler(uint32_t *pTimer);
void MC_Core_SetCompareHallTimer(uint32_t *pHallTimer, uint16_t CommutationDelay);
void MC_Core_ResetBemfGpio(MC_Handle_t* pMc);
void MC_Core_SetBemfGpio(MC_Handle_t* pMc);
void MC_Core_SetDutyCyclePwmForAdcTrig(uint32_t *pTimer, uint32_t Channel, uint16_t PulseValue);
void MC_Core_SetDutyCycleHfPwmForStepN(uint32_t *pHfTimer, uint16_t PulseValue, uint8_t StepNumber);
void MC_Core_SetDutyCycleHfPwmU(uint32_t *pHfTimer, uint16_t PulseValue);
void MC_Core_SetDutyCycleHfPwmV(uint32_t *pHfTimer, uint16_t PulseValue);
void MC_Core_SetDutyCycleHfPwmW(uint32_t *pHfTimer, uint16_t PulseValue);
void MC_Core_SetDutyCycleRefPwm(uint32_t *pRefTimer, uint16_t PulseValue);
void MC_Core_SetPeriodTimer(uint32_t *pTimer, uint16_t PeriodValue);
void MC_Core_SetTimerCounter(uint32_t *pTimer,  uint32_t Counter);
void MC_Core_ResetPolarityHfPwm(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity);
void MC_Core_SetPrescalerTimer(uint32_t *pTimer, uint16_t PrescalerValue);
void MC_Core_SelectAdcChannel(uint32_t *pAdcItToBeDisabled, uint32_t *pAdcItToBeEnabled, uint32_t AdcChannel, uint32_t SamplingTime);
void MC_Core_SelectAdcChannelDuringCallback(uint32_t *pAdcItToBeDisabled, uint32_t *pAdcItToBeEnabled, uint32_t AdcChannel, uint32_t SamplingTime);
void MC_Core_SetAdcSamplingTime(uint32_t *pAdc, uint32_t AdcChannel, uint32_t SamplingTime);
void MC_Core_StartAdcIt(uint32_t *pAdc);
void MC_Core_StartHfPwms(uint32_t *pHfTimer, MC_EN_DRIVER_t enable);
void MC_Core_StartLfTimerIt(uint32_t *pLfTimer);
void MC_Core_StartRefPwm(uint32_t *pRefTimer);
void MC_Core_StopAdcIt(uint32_t *pAdc);
void MC_Core_StopHfPwms(uint32_t *pHfTimer, MC_EN_DRIVER_t enable);
void MC_Core_StopLfTimerIt(uint32_t *pLfTimer);
void MC_Core_StopHallTimerIt(uint32_t *pLfTimer);
void MC_Core_StopRefPwm(uint32_t *pRefTimer);
/**
  * @} end MC_6STEP_CORE_Exported_FunctionsPrototype
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

#ifdef __cplusplus
}
#endif

#endif /* __6STEP_CORE_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/