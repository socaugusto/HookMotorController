/**
  ******************************************************************************
  * @file    6step_service.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides all the 6-step library core functions 
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
#include "6step_service_f0xx.h"

/*******************************************************************************
                   LL Driver Interface (6-step library -> HAL/LL)
*******************************************************************************/
/**
  * @brief  MC_Core_GetTemperatureCalibrationData
  * @param[in] pMc pointer to the motor control handle
  * @retval None
  */
void MC_Core_GetTemperatureCalibrationData(MC_Handle_t *pMc)
{ /* Get temperature calibration data */
  pMc->adc_user.ts_cal_1_temp_deg_c = TEMPSENSOR_CAL1_TEMP;
  pMc->adc_user.ts_cal_2_temp_deg_c = TEMPSENSOR_CAL2_TEMP;
  pMc->adc_user.ts_cal_1 = *((uint16_t*) (TEMPSENSOR_CAL1_ADDR));
  pMc->adc_user.ts_cal_2 = *((uint16_t*) (TEMPSENSOR_CAL2_ADDR));
  pMc->adc_user.vrefint_cal = *((uint16_t*) (VREFINT_CAL_ADDR));
}

/**
  * @brief  MC_Core_CalibrateAdc
  * @param[in] pAdc pointer to the ADC
  * @retval None
  */
void MC_Core_CalibrateAdc(uint32_t *pAdc)
{
  if (pAdc != NULL)
  {
	uint32_t adc_sampling_time = LL_ADC_GetSamplingTimeCommonChannels((ADC_TypeDef *)pAdc);
	LL_ADC_SetSamplingTimeCommonChannels((ADC_TypeDef *)pAdc, LL_ADC_SAMPLINGTIME_239CYCLES_5);
	LL_ADC_Disable((ADC_TypeDef *)pAdc);
	while (LL_ADC_IsDisableOngoing((ADC_TypeDef *)pAdc));
	LL_ADC_StartCalibration( (ADC_TypeDef *)pAdc );
	while ((LL_ADC_IsCalibrationOnGoing((ADC_TypeDef *)pAdc) == SET) ||
			(LL_ADC_REG_IsConversionOngoing((ADC_TypeDef *)pAdc) == SET) ||
			(LL_ADC_REG_IsStopConversionOngoing((ADC_TypeDef *)pAdc) == SET) ||
			(LL_ADC_IsDisableOngoing((ADC_TypeDef *)pAdc) == SET))
	{
		/* wait */
	}
	/* Enables the ADC peripheral */
	LL_ADC_Enable( (ADC_TypeDef *)pAdc );
	/* Wait ADC Ready */
	while ( LL_ADC_IsActiveFlag_ADRDY( (ADC_TypeDef *)pAdc ) == RESET )
	{
    /* wait */
	}
	LL_ADC_SetSamplingTimeCommonChannels((ADC_TypeDef *)pAdc, adc_sampling_time);
  }
}

/**
  * @brief  MC_Core_ConfigureCommutationEvent
  *         This function is mandatory to allow the update the configuration of 
  *         the timer with the preloaded values at each generated event
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] pLfTimer pointer to the LF timer acting as the hall interface
  * @retval None
  */
void MC_Core_ConfigureCommutationEvent(uint32_t *pHfTimer, uint32_t *pLfTimer)
{
  /* Select the Capture Compare preload feature */
  LL_TIM_CC_EnablePreload((TIM_TypeDef *) pHfTimer);
  /* Select the Commutation event source */
  LL_TIM_CC_SetUpdate((TIM_TypeDef *) pHfTimer,LL_TIM_CCUPDATESOURCE_COMG_ONLY);
}

/**
  * @brief  MC_Core_HallConfigureCommutationEvent
  *         This function is mandatory to allow the update the configuration of 
  *         the timer with the preloaded values at each generated event
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] pLfTimer pointer to the LF timer acting as the hall interface
  * @retval None
  */
void MC_Core_HallConfigureCommutationEvent(uint32_t *pHfTimer, uint32_t *pLfTimer)
{
  /* Select the Capture Compare preload feature */
  LL_TIM_CC_EnablePreload((TIM_TypeDef *) pHfTimer);
  /* Select the Commutation event source */
  LL_TIM_CC_SetUpdate((TIM_TypeDef *) pHfTimer,LL_TIM_CCUPDATESOURCE_COMG_AND_TRGI);
  /* Enable the Commutation Interrupt Request */
  LL_TIM_EnableIT_COM((TIM_TypeDef *) pHfTimer);
    /* Enable the Capture Compare 2 */
  LL_TIM_EnableIT_CC2((TIM_TypeDef *) pLfTimer);
  /* Enable the Capture Compare 1 */
  LL_TIM_EnableIT_CC1((TIM_TypeDef *) pLfTimer);
  /* Enable the Input Capture channel 1 */
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pLfTimer, LL_TIM_CHANNEL_CH1);
  /* Enable the Peripheral */
  LL_TIM_EnableCounter((TIM_TypeDef *) pLfTimer);
}

/**
  * @brief  MC_Core_DisableIrq
  * @retval None
  */
void MC_Core_DisableIrq(void)
{
  __disable_irq();
}

/**
  * @brief  MC_Core_DisableUpdateEvent
  * @param[in] pHfTimer pointer to the HF timer
  * @retval None
  */
void MC_Core_DisableUpdateEvent(uint32_t *pHfTimer)
{
  LL_TIM_DisableUpdateEvent((TIM_TypeDef *) pHfTimer);
}

/**
  * @brief   MC_Core_EnableChannelsHfPwmsStep1FastDemag
  * @retval None
  */
void  MC_Core_EnableChannelsHfPwmsStep1FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity)
{  
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1N, Nch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2N, Nch_polarity);
}

/**
  * @brief   MC_Core_EnableChannelsHfPwmsStep3FastDemag
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep3FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2N, Nch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3N, Nch_polarity);
}

/**
  * @brief   MC_Core_EnableChannelsHfPwmsStep5FastDemag
  * @retval None
  */
void  MC_Core_EnableChannelsHfPwmsStep5FastDemag(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity)
{  
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1N, Nch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3N, Nch_polarity);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep1
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep1(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N | LL_TIM_CHANNEL_CH1N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep2
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep2(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH1N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep3
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep3(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep4
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep4(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N | LL_TIM_CHANNEL_CH2N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep5
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep5(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep6
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep6(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH3N);
}
/* QUASI-SYNCHRONOUS RECTIFICATION END 1 */

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep14
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep14(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep25
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep25(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsStep36
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsStep36(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
}

/**
  * @brief  MC_Core_EnableChannelsHfPwmsAll
  * @retval None
  */
void MC_Core_EnableChannelsHfPwmsAll(uint32_t *pHfTimer)
{
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N |\
                                                                      LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N |\
                                                                      LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
}

/**
  * @brief  MC_Core_EnableInputsHfPwmsStep14
  * @note   This function is used for 3PWM interface
  *         For several motors, this function should be modified to control different GPIOs according to the MotorDeviceId
  * @retval None
  */
void MC_Core_EnableInputsHfPwmsStep14(uint8_t MotorDeviceId)
{

}

/**
  * @brief  MC_Core_EnableInputsHfPwmsStep25
  * @note   This function is used for 3PWM interface
  *         For several motors, this function should be modified to control different GPIOs according to the MotorDeviceId
  * @retval None
  */
void MC_Core_EnableInputsHfPwmsStep25(uint8_t MotorDeviceId)
{

}

/**
  * @brief  MC_Core_EnableInputsHfPwmsStep36
  * @note   This function is used for 3PWM interface
  *         For several motors, this function should be modified to control different GPIOs according to the MotorDeviceId
  * @retval None
  */
void MC_Core_EnableInputsHfPwmsStep36(uint8_t MotorDeviceId)
{

}

/**
  * @brief  MC_Core_EnableIrq
  * @retval None
  */
void MC_Core_EnableIrq(void)
{
  __enable_irq();
}

/**
  * @brief  MC_Core_EnableItBreak
  * @param[in] pTimer pointer to the relevant timer
  * @retval None
  */
void MC_Core_EnableItBreak(uint32_t *pTimer)
{
  LL_TIM_ClearFlag_BRK((TIM_TypeDef *) pTimer);
  LL_TIM_EnableIT_BRK((TIM_TypeDef *) pTimer);
}

/* FAST DEMAG BEGIN 2 */
/**
  * @brief  MC_Core_EnableUpdateEvent
  * @param[in] pHfTimer pointer to the HF timer
  * @retval None
  */
void MC_Core_EnableUpdateEvent(uint32_t *pHfTimer)
{
  LL_TIM_EnableUpdateEvent((TIM_TypeDef *) pHfTimer);
}
/* FAST DEMAG END 2 */

/**
  * @brief  MC_Core_GenerateComEvent
  * @param[in] pHfTimer pointer to the HF timer
  * @retval None
  */
void MC_Core_GenerateComEvent(uint32_t *pHfTimer)
{
  LL_TIM_GenerateEvent_COM((TIM_TypeDef *) pHfTimer);
}

/**
  * @brief  MC_Core_GenerateUpdateEvent
  * @param[in] pTimer pointer to the timer
  * @retval None
  */
void MC_Core_GenerateUpdateEvent(uint32_t *pTimer)
{
  LL_TIM_GenerateEvent_UPDATE((TIM_TypeDef *) pTimer);
}

/**
  * @brief  MC_Core_GetAdcResolution
  * @param[in] pMc pointer to the motor control handle
  * @param[in] pAdc pointer to the ADC
  * @retval ADC resolution in number of bits
  */
uint8_t MC_Core_GetAdcResolution(MC_Handle_t *pMc, uint32_t *pAdc)
{
  uint8_t adc_resolution_bits;
  switch (LL_ADC_GetResolution((ADC_TypeDef *)pAdc))
  {
    case LL_ADC_RESOLUTION_12B:
    {
      adc_resolution_bits = 12;
    }
    break;    
    case LL_ADC_RESOLUTION_10B:
    {
      adc_resolution_bits = 10;
    }
    break;    
    case LL_ADC_RESOLUTION_8B:
    {
      adc_resolution_bits = 8;
    }
    break;    
    case LL_ADC_RESOLUTION_6B:
    {
      adc_resolution_bits = 6;
    }
    break;
    default:
      adc_resolution_bits = 12;
  }
  return adc_resolution_bits;
}

/* SENSE COMPARATORS BEGIN 1 */
/**
  * @brief  MC_Core_GetTimerCounterMode
  * @param[in] pTimer pointer to timer
  * @retval 0 if counter mode is up or down
  *         1 if counter mode is center aligned
  */
uint32_t MC_Core_GetTimerCounterMode(uint32_t *pTimer)
{
  uint32_t counter_mode = LL_TIM_GetCounterMode((TIM_TypeDef *) pTimer);
  if ((counter_mode != LL_TIM_COUNTERMODE_UP) && (counter_mode != LL_TIM_COUNTERMODE_DOWN))
  {
    counter_mode = 1;
  }
  else
  {
    counter_mode = 0;
  }
  return counter_mode;
}

/**
  * @brief  MC_Core_SetTimerCounter
  * @param[in] pTimer pointer to timer
  * @param[in] Counter value to set
  * @retval None
  */
void MC_Core_SetTimerCounter(uint32_t *pTimer,  uint32_t Counter)
{
  LL_TIM_SetCounter((TIM_TypeDef *) pTimer, Counter);
}

/**
  * @brief  MC_Core_GetTimerFrequency
  * @param[in] pTimer pointer to timer
  * @retval Gate driving pwm frequency
  */
uint32_t MC_Core_GetTimerFrequency(uint32_t *pTimer)
{
  uint32_t timer_frequency = 0;
  uint32_t counter_mode;
  timer_frequency = (MC_Core_GetSysClockFrequency()) / ((MC_Core_GetTimerPeriod((uint32_t *) pTimer) + 1) * (MC_Core_GetTimerPrescaler((uint32_t *) pTimer) + 1));
  counter_mode = LL_TIM_GetCounterMode((TIM_TypeDef *) pTimer);
  if ((counter_mode != LL_TIM_COUNTERMODE_UP) && (counter_mode != LL_TIM_COUNTERMODE_DOWN))
  {
    timer_frequency >>= 1;
  }
  return timer_frequency;
}
/* SENSE COMPARATORS END 1 */

/**
  * @brief  MC_Core_GetHallStatus
  * @param[in] pMc pointer to the motor control handle
  * @retval None
  */
void MC_Core_GetHallStatus(MC_Handle_t *pMc)
{
    pMc->hall.status = LL_GPIO_IsInputPinSet(pMc->hall.HALL_IOs[0].HALLx_GPIO_Port, pMc->hall.HALL_IOs[0].HALLx_GPIO_Pin) << 2 |\
                       LL_GPIO_IsInputPinSet(pMc->hall.HALL_IOs[1].HALLx_GPIO_Port, pMc->hall.HALL_IOs[1].HALLx_GPIO_Pin) << 1 |\
                       LL_GPIO_IsInputPinSet(pMc->hall.HALL_IOs[2].HALLx_GPIO_Port, pMc->hall.HALL_IOs[2].HALLx_GPIO_Pin);
}

/**
  * @brief  MC_Core_GetSysClockFrequency
  * @retval System clock frequency
  */
uint32_t MC_Core_GetSysClockFrequency(void)
{
  const uint8_t aPLLMULFactorTable[16] = { 2U,  3U,  4U,  5U,  6U,  7U,  8U,  9U,
                                         10U, 11U, 12U, 13U, 14U, 15U, 16U, 16U};
  const uint8_t aPredivFactorTable[16] = { 1U, 2U,  3U,  4U,  5U,  6U,  7U,  8U,
                                           9U,10U, 11U, 12U, 13U, 14U, 15U, 16U};  
  
  uint32_t tmpreg = 0U, prediv = 0U, pllclk = 0U, pllmul = 0U;
  uint32_t sysclockfreq = 0U;
  
  tmpreg = RCC->CFGR;
  
  /* Get SYSCLK source -------------------------------------------------------*/
  switch (tmpreg & RCC_CFGR_SWS)
  {
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSE:  /* HSE used as system clock */
    {
      sysclockfreq = HSE_VALUE;
      break;
    }
    case LL_RCC_SYS_CLKSOURCE_STATUS_PLL:  /* PLL used as system clock */
    {
      pllmul = aPLLMULFactorTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos];
      prediv = aPredivFactorTable[(uint32_t)(RCC->CFGR2 & RCC_CFGR2_PREDIV) >> RCC_CFGR2_PREDIV_Pos];      

      if ((tmpreg & RCC_CFGR_PLLSRC) == LL_RCC_PLLSOURCE_HSE)
      {
        /* HSE used as PLL clock source : PLLCLK = HSE/PREDIV * PLLMUL */
        pllclk = (HSE_VALUE / prediv) * pllmul;
      }
      else
      {
        /* HSI used as PLL clock source : PLLCLK = HSI/2 * PLLMUL */
        pllclk = (uint32_t)((HSI_VALUE >> 1U) * pllmul);
      }
      sysclockfreq = pllclk;
      break;
    }
    case LL_RCC_SYS_CLKSOURCE_STATUS_HSI:  /* HSI used as system clock source */
    default: /* HSI used as system clock */
    {
      sysclockfreq = HSI_VALUE;
      break;
    }
  }
  return sysclockfreq;
}

/**
  * @brief  MC_Core_GetTimerCaptureCompare
  * @param[in] pTimer pointer to relevant timer
  * @retval timer channel capture compare value
  */
uint16_t MC_Core_GetTimerCaptureCompare(uint32_t *pTimer)
{
  return (uint16_t) LL_TIM_OC_GetCompareCH1((TIM_TypeDef *) pTimer);
}

/**
  * @brief  MC_Core_GetTimerCounter
  * @param[in] pTimer pointer to relevant timer
  * @retval timer channel capture compare value
  */
uint16_t MC_Core_GetTimerCounter(uint32_t *pTimer)
{
  return (uint16_t) LL_TIM_GetCounter((TIM_TypeDef *) pTimer);
}

/**
  * @brief  MC_Core_GetTimerPeriod
  * @param[in] pTimer pointer to relevant timer
  * @retval timer period
  */
uint16_t MC_Core_GetTimerPeriod(uint32_t *pTimer)
{
  return (uint16_t) LL_TIM_GetAutoReload((TIM_TypeDef *) pTimer);
}

/**
  * @brief  MC_Core_GetTimerPrescaler
  * @param[in] pTimer pointer to relevant timer
  * @retval timer period
  */
uint16_t MC_Core_GetTimerPrescaler(uint32_t *pTimer)
{
  return (uint16_t) LL_TIM_GetPrescaler((TIM_TypeDef *) pTimer);
}

/**
  * @brief  MC_Core_SetCompareHallTimer
  * @param[in] pHallTimer pointer to the HALL timer
  * @param[in] CommutationDelay delay between a hall sensors status change and
  *            a step commutation
  * @retval None
  */
void MC_Core_SetCompareHallTimer(uint32_t *pHallTimer, uint16_t CommutationDelay)
{
  LL_TIM_OC_SetCompareCH2((TIM_TypeDef *) pHallTimer, CommutationDelay);
}

/**
  * @brief  MC_Core_ResetBemfGpio
  * @param[in] pMc pointer to the motor control handle
  * @retval None
  */
void MC_Core_ResetBemfGpio(MC_Handle_t* pMc)
{
  LL_GPIO_ResetOutputPin(pMc->bemf.GPIO_BEMFon.GPIO_BEMF_Port,pMc->bemf.GPIO_BEMFon.GPIO_BEMF_Pin);
}

/**
  * @brief  MC_Core_SetBemfGpio
  * @param[in] pMc pointer to the motor control handle
  * @retval None
  */
void MC_Core_SetBemfGpio(MC_Handle_t* pMc)
{
  LL_GPIO_SetOutputPin(pMc->bemf.GPIO_BEMFon.GPIO_BEMF_Port,pMc->bemf.GPIO_BEMFon.GPIO_BEMF_Pin);
}

/**
  * @brief  MC_Core_SetDutyCyclePwmForAdcTrig
  * @param[in] pTimer pointer to relevant timer
  * @param[in] Channel channel used to trig the ADC
  * @param[in] PulseValue pulse value of the timer channel used to trig the ADC
  * @retval None
  */
void MC_Core_SetDutyCyclePwmForAdcTrig(uint32_t *pTimer, uint32_t Channel, uint16_t PulseValue)
{
  switch (Channel)
  {
    case LL_TIM_CHANNEL_CH1:
    {
      LL_TIM_OC_SetCompareCH1((TIM_TypeDef *) pTimer, PulseValue);
    }
    break;    
    case LL_TIM_CHANNEL_CH2:
    {
      LL_TIM_OC_SetCompareCH2((TIM_TypeDef *) pTimer, PulseValue);
    }
    break;    
    case LL_TIM_CHANNEL_CH3:
    {
      LL_TIM_OC_SetCompareCH3((TIM_TypeDef *) pTimer, PulseValue);
    }
    break;    
    case LL_TIM_CHANNEL_CH4:
    {
      LL_TIM_OC_SetCompareCH4((TIM_TypeDef *) pTimer, PulseValue);
    }
    break;
  default:
    LL_TIM_OC_SetCompareCH1((TIM_TypeDef *) pTimer, PulseValue);
  }
}

/**
  * @brief  MC_Core_SetDutyCycleHfPwmForStepN
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] PulseValue pulse value of the HF timer channel corresponding to the StepNumber
  * @param[in] StepNumber step number in the 6step sequence
  * @retval None
  */
void MC_Core_SetDutyCycleHfPwmForStepN(uint32_t *pHfTimer, uint16_t PulseValue, uint8_t StepNumber)
{
  uint32_t volatile* ptr = (&((TIM_TypeDef *) pHfTimer)->CCR1);
  ptr += ((--StepNumber) >> 1);
  *ptr = PulseValue;
}

/**
  * @brief  MC_Core_SetDutyCycleHfPwmU
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] PulseValue pulse value of the HF timer channel 1
  * @retval None
  */
void MC_Core_SetDutyCycleHfPwmU(uint32_t *pHfTimer, uint16_t PulseValue)
{
  LL_TIM_OC_SetCompareCH1((TIM_TypeDef *) pHfTimer, PulseValue);
}

/**
  * @brief  MC_Core_SetDutyCycleHfPwmV
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] PulseValue pulse value of the HF timer channel 2
  * @retval None
  */
void MC_Core_SetDutyCycleHfPwmV(uint32_t *pHfTimer, uint16_t PulseValue)
{
  LL_TIM_OC_SetCompareCH2((TIM_TypeDef *) pHfTimer, PulseValue);
}

/**
  * @brief  MC_Core_SetDutyCycleHfPwmW
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] PulseValue pulse value of the HF timer channel 3
  * @retval None
  */
void MC_Core_SetDutyCycleHfPwmW(uint32_t *pHfTimer, uint16_t PulseValue)
{
  LL_TIM_OC_SetCompareCH3((TIM_TypeDef *) pHfTimer, PulseValue);
}

/**
  * @brief  MC_Core_SetDutyCycleRefPwm
  * @param[in] pHfTimer pointer to the REF timer
  * @param[in] PulseValue pulse value of the REF timer channel used to generate the reference voltage
  * @retval None
  */
void MC_Core_SetDutyCycleRefPwm(uint32_t *pRefTimer, uint16_t PulseValue)
{
  LL_TIM_OC_SetCompareCH1((TIM_TypeDef *) pRefTimer, PulseValue);
}

/**
  * @brief  MC_Core_SetPeriodTimer
  * @param[in] PeriodValue period value for the timer
  * @retval None
  */
void MC_Core_SetPeriodTimer(uint32_t *pTimer, uint16_t PeriodValue)
{
  LL_TIM_SetAutoReload((TIM_TypeDef *) pTimer, PeriodValue);
}

/**
  * @brief  MC_Core_ResetPolarityHfPwm
  * @param[in] pHfTimer pointer to the HF timer
  * @retval None
  */
void MC_Core_ResetPolarityHfPwm(uint32_t *pHfTimer, uint32_t ch_polarity, uint32_t Nch_polarity)
{
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH1N, Nch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH2N, Nch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3, ch_polarity);
  LL_TIM_OC_SetPolarity((TIM_TypeDef *) pHfTimer, LL_TIM_CHANNEL_CH3N, Nch_polarity);
}

/**
  * @brief  MC_Core_SetPrescalerLfTimer
  * @param[in] PrescalerValue prescaler value for the LF timer
  * @retval None
  */
void MC_Core_SetPrescalerTimer(uint32_t *pTimer, uint16_t PrescalerValue)
{
  LL_TIM_SetPrescaler((TIM_TypeDef *) pTimer, PrescalerValue);
}

/**
  * @brief  MC_Core_SelectAdcChannel
  * @param[in] pAdcItToBeDisabled pointer to the ADC with the end of conversion interrupt to be disabled
  * @param[in] pAdcItToBeEnabled pointer to the ADC with the end of conversion interrupt to be enabled
  * @param[in] AdcChannel ADC channel to be selected
  * @param[in] SamplingTime ADC sampling time to be selected
  * @retval None
  */
void MC_Core_SelectAdcChannel(uint32_t *pAdcItToBeDisabled, uint32_t *pAdcItToBeEnabled, uint32_t AdcChannel, uint32_t SamplingTime)
{
  if (pAdcItToBeDisabled != NULL)
  {
    LL_ADC_DisableIT_EOC((ADC_TypeDef *)pAdcItToBeDisabled);
  }
  if (pAdcItToBeEnabled != NULL)
  {
    while (LL_ADC_REG_IsConversionOngoing((ADC_TypeDef *)pAdcItToBeEnabled))
    {
      LL_ADC_REG_StopConversion((ADC_TypeDef *)pAdcItToBeEnabled);
      while(LL_ADC_REG_IsStopConversionOngoing((ADC_TypeDef *)pAdcItToBeEnabled));
    }
	/* Sampling time configuration */
	LL_ADC_SetSamplingTimeCommonChannels((ADC_TypeDef *)pAdcItToBeEnabled, SamplingTime);
	/* Regular sequence configuration */
	LL_ADC_REG_SetSequencerChannels((ADC_TypeDef *)pAdcItToBeEnabled, AdcChannel);
	LL_ADC_REG_StartConversion((ADC_TypeDef *)pAdcItToBeEnabled);
	LL_ADC_ClearFlag_EOC((ADC_TypeDef *)pAdcItToBeEnabled);
    LL_ADC_EnableIT_EOC((ADC_TypeDef *)pAdcItToBeEnabled);
  }
}

/**
  * @brief  MC_Core_SelectAdcChannelDuringCallback
  * @note   By design when the ADC channel is selected during the ADC callback
  * there is no more conversion on-going. Indeed there is always only one
  * conversion programmed in the ADC sequence. However the ADC must still be
  * stop start to ensure uncorrupted measurement on next conversion.
  * @param[in] pAdcItToBeDisabled pointer to the ADC with the end of conversion interrupt to be disabled
  * @param[in] pAdcItToBeEnabled pointer to the ADC with the end of conversion interrupt to be enabled
  * @param[in] AdcChannel ADC channel to be selected
  * @param[in] SamplingTime ADC sampling time to be selected
  * @retval None
  */
void MC_Core_SelectAdcChannelDuringCallback(uint32_t *pAdcItToBeDisabled, uint32_t *pAdcItToBeEnabled, uint32_t AdcChannel, uint32_t SamplingTime)
{
  if (pAdcItToBeEnabled != NULL)
  {
	LL_ADC_REG_StopConversion((ADC_TypeDef *)pAdcItToBeEnabled);
	/* Sampling time configuration */
	LL_ADC_SetSamplingTimeCommonChannels((ADC_TypeDef *)pAdcItToBeEnabled, SamplingTime);
	/* Regular sequence configuration */
	LL_ADC_REG_SetSequencerChannels((ADC_TypeDef *)pAdcItToBeEnabled, AdcChannel);
	LL_ADC_REG_StartConversion((ADC_TypeDef *)pAdcItToBeEnabled);
  }
}

/**
  * @brief  MC_Core_SetAdcSamplingTime
  * @param[in] pAdc pointer to the ADC
  * @param[in] AdcChannel ADC channel to be selected
  * @param[in] SamplingTime ADC sampling time to be selected
  * @retval None
  */
void MC_Core_SetAdcSamplingTime(uint32_t *pAdc, uint32_t AdcChannel, uint32_t SamplingTime)
{
  LL_ADC_SetSamplingTimeCommonChannels((ADC_TypeDef *)pAdc, SamplingTime);
}

/**
  * @brief  MC_Core_StartAdcIt
  * @param[in] pAdc pointer to the ADC
  * @retval None
  */
void MC_Core_StartAdcIt(uint32_t *pAdc)
{
  if (LL_ADC_REG_IsConversionOngoing((ADC_TypeDef *)pAdc) == 0)
  {
      /* Set ADC state                                                        */
      /* - Clear state bitfield related to regular group conversion results   */
      /* - Set state bitfield related to regular operation                    */
      LL_ADC_Enable((ADC_TypeDef *)pAdc);
      LL_ADC_ClearFlag_EOC((ADC_TypeDef *)pAdc);
      LL_ADC_ClearFlag_OVR((ADC_TypeDef *)pAdc);
      LL_ADC_EnableIT_EOC((ADC_TypeDef *)pAdc);
      LL_ADC_EnableIT_OVR((ADC_TypeDef *)pAdc);
      ((ADC_TypeDef *)pAdc)->CR |= ADC_CR_ADSTART;
  }
}

/**
  * @brief  MC_Core_StartHfPwms
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] enable enable driver GPIO
  * @retval None
  */
void MC_Core_StartHfPwms(uint32_t *pHfTimer, MC_EN_DRIVER_t enable)
{
  LL_TIM_EnableCounter((TIM_TypeDef *) pHfTimer);
  LL_TIM_EnableAllOutputs((TIM_TypeDef *) pHfTimer);
  if (enable.enDriver_GPIO_Port != NULL) LL_GPIO_SetOutputPin(enable.enDriver_GPIO_Port, enable.enDriver_GPIO_Pin);
}

/* SENSE COMPARATORS BEGIN 2 */
/**
  * @brief  MC_Core_StartLfTimerIt
  * @param[in] pLfTimer pointer to the LF timer
  * @retval None
  */
void MC_Core_StartLfTimerIt(uint32_t *pLfTimer)
{
  LL_TIM_ClearFlag_UPDATE((TIM_TypeDef *) pLfTimer);
  LL_TIM_EnableIT_UPDATE((TIM_TypeDef *) pLfTimer);
  LL_TIM_EnableCounter((TIM_TypeDef *) pLfTimer);
}
/* SENSE COMPARATORS END 2 */

/**
  * @brief  MC_Core_StartRefPwms
  * @param[in] pRefTimer pointer to the REF timer
  * @retval None
  */
void MC_Core_StartRefPwm(uint32_t *pRefTimer)
{
  LL_TIM_EnableCounter((TIM_TypeDef *) pRefTimer);
  LL_TIM_CC_EnableChannel((TIM_TypeDef *) pRefTimer, LL_TIM_CHANNEL_CH1);
}

/**
  * @brief  MC_Core_StopAdcIt
  * @param[in] pAdc pointer to the ADC
  * @retval None
  */
void MC_Core_StopAdcIt(uint32_t *pAdc)
{
  LL_ADC_REG_StopConversion((ADC_TypeDef *)pAdc);
  LL_ADC_DisableIT_EOC((ADC_TypeDef *)pAdc);
  LL_ADC_DisableIT_OVR((ADC_TypeDef *)pAdc);
  const uint32_t tmp_adc_is_disable_on_going = LL_ADC_IsDisableOngoing((ADC_TypeDef *)pAdc);

  /* Verification if ADC is not already disabled:                             */
  /* Note: forbidden to disable ADC (set bit ADC_CR_ADDIS) if ADC is already  */
  /*       disabled.                                                          */
  if ((LL_ADC_IsEnabled((ADC_TypeDef *)pAdc) != 0UL)
      && (tmp_adc_is_disable_on_going == 0UL)
     )
  {
    /* Check if conditions to disable the ADC are fulfilled */
    if ((((ADC_TypeDef *)pAdc)->CR & (ADC_CR_ADSTART | ADC_CR_ADEN)) == ADC_CR_ADEN)
    {
      /* Disable the ADC peripheral */
      LL_ADC_Disable((ADC_TypeDef *)pAdc);
    }
  }
}

/**
  * @brief  MC_Core_StopHfPwms
  * @param[in] pHfTimer pointer to the HF timer
  * @param[in] enable enable driver GPIO
  * @retval None
  */
void MC_Core_StopHfPwms(uint32_t *pHfTimer, MC_EN_DRIVER_t enable)
{
  LL_TIM_DisableAllOutputs((TIM_TypeDef *) pHfTimer);
  LL_TIM_DisableCounter((TIM_TypeDef *) pHfTimer);
  LL_TIM_SetCounter((TIM_TypeDef *) pHfTimer, 0);
  if (enable.enDriver_GPIO_Port != NULL) LL_GPIO_ResetOutputPin(enable.enDriver_GPIO_Port, enable.enDriver_GPIO_Pin);
  LL_TIM_DisableIT_COM((TIM_TypeDef *) pHfTimer);
  LL_TIM_DisableIT_BRK((TIM_TypeDef *) pHfTimer);
  LL_TIM_ClearFlag_COM((TIM_TypeDef *) pHfTimer);
  LL_TIM_ClearFlag_BRK((TIM_TypeDef *) pHfTimer);
}

/**
  * @brief  MC_Core_StopLfTimerIt
  * @param[in] pLfTimer pointer to the LF timer
  * @retval None
  */
void MC_Core_StopLfTimerIt(uint32_t *pLfTimer)
{
  LL_TIM_DisableIT_UPDATE((TIM_TypeDef *) pLfTimer);
  LL_TIM_DisableCounter((TIM_TypeDef *) pLfTimer);
  LL_TIM_ClearFlag_UPDATE((TIM_TypeDef *) pLfTimer); 
  LL_TIM_SetCounter((TIM_TypeDef *) pLfTimer, 0);
}

/**
  * @brief  MC_Core_StopHallTimerIt
  * @param[in] pLfTimer pointer to the LF timer
  * @retval None
  */
void MC_Core_StopHallTimerIt(uint32_t *pLfTimer)
{
  /* Reset the CCxE Bit */
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pLfTimer, LL_TIM_CHANNEL_CH1);
  /* Disable the capture compare Interrupts event */
  LL_TIM_DisableIT_CC1((TIM_TypeDef *) pLfTimer);
  /* Disable the Peripheral */
  LL_TIM_DisableCounter((TIM_TypeDef *) pLfTimer);
}

/**
  * @brief  MC_Core_StopRefPwm
  * @param[in] pRefTimer pointer to the REF timer
  * @retval None
  */
void MC_Core_StopRefPwm(uint32_t *pRefTimer)
{
  /* Disable the Capture compare channel */
  LL_TIM_CC_DisableChannel((TIM_TypeDef *) pRefTimer, LL_TIM_CHANNEL_CH1);
  /* Disable the main output */
  LL_TIM_DisableAllOutputs((TIM_TypeDef *) pRefTimer);
  /* Disable the peripheral */
  LL_TIM_DisableCounter((TIM_TypeDef *) pRefTimer);
  /* Set counter to 0 */
  LL_TIM_SetCounter((TIM_TypeDef *) pRefTimer, 0);  
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

