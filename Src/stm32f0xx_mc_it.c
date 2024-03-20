/**
 ******************************************************************************
 * @file    stm32f0xx_mc_it.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Main Interrupt Service Routines.
 *          This file provides exceptions handler and peripherals interrupt
 *          service routine related to Motor Control for the STM32F0 Family.
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
 * @ingroup STM32F0xx_IRQ_Handlers
 */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mcp_config.h"
#include "mc_tasks.h"

#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "mc_hook_remote_config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
 * @{
 */

/** @addtogroup STM32F0xx_IRQ_Handlers STM32F0xx IRQ Handlers
 * @{
 */

/* USER CODE BEGIN PRIVATE */
extern void MotorUpdatePosition(MC_Handle_t *motor_device);
extern void MotorPositionTargetTest(MC_Handle_t *motor_device);
extern void check_safety(MC_Handle_t *motor_device);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY / 1000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* USER CODE END PRIVATE */
void ADC_ConvCplt_Handler(void);
void TIMx_UP_BRK_M1_Handler(void);
void HALL_COMM_Handler(void);
void USART_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);
void EXTI0_1_IRQHandler(void);

/**
 * @brief     ADC conversion complete interrupt handle
 * @param[in] None
 * @retval None
 */
void ADC_ConvCplt_Handler(void)
{
  /* ========== Check End of Conversion flag for regular group ========== */
  if (LL_ADC_IsActiveFlag_EOC((ADC_TypeDef *)ADC1) && LL_ADC_IsEnabledIT_EOC((ADC_TypeDef *)ADC1))

  {
    MC_Core_ProcessAdcMeasurement((uint32_t *)ADC1, (uint16_t)LL_TIM_GetCounter((TIM_TypeDef *)((&Motor_Device1)->plf_timer)), (uint16_t)LL_ADC_REG_ReadConversionData12((ADC_TypeDef *)ADC1));

    LL_ADC_ClearFlag_EOC((ADC_TypeDef *)ADC1);
  }
  else
    LL_ADC_ClearFlag_EOC((ADC_TypeDef *)ADC1);
  if (LL_ADC_IsActiveFlag_OVR((ADC_TypeDef *)ADC1) && LL_ADC_IsEnabledIT_OVR((ADC_TypeDef *)ADC1))
  {
    LL_ADC_ClearFlag_OVR((ADC_TypeDef *)ADC1);
  }
}

/**
 * @brief     HFtimer interrupt handler
 * @param[in] None
 * @retval None
 */

void TIMx_BRK_M1_Handler(void)
{
  /* TIM Break input event */
  if (LL_TIM_IsActiveFlag_BRK((TIM_TypeDef *)Motor_Device1.phf_timer) && LL_TIM_IsEnabledIT_BRK((TIM_TypeDef *)Motor_Device1.phf_timer))
  {
    LL_TIM_ClearFlag_BRK((TIM_TypeDef *)Motor_Device1.phf_timer);
    Motor_Device1.status = MC_OVERCURRENT;
    hook_setError(ERROR_OVERLOAD);
    MC_Core_Error(&Motor_Device1);
  }
  /* TIM commutation event */
  if (LL_TIM_IsActiveFlag_COM((TIM_TypeDef *)Motor_Device1.phf_timer) && LL_TIM_IsEnabledIT_COM((TIM_TypeDef *)Motor_Device1.phf_timer))
  {
    LL_TIM_ClearFlag_COM((TIM_TypeDef *)Motor_Device1.phf_timer);
  }
}

/**
 * @brief     Hall timer interrupt handler
 * @param[in] None
 * @retval None
 */
void HALL_TIM_M1_IRQHandler(void)
{
  /* Capture compare 1 event */
  if (LL_TIM_IsActiveFlag_CC1((TIM_TypeDef *)Motor_Device1.plf_timer) && LL_TIM_IsEnabledIT_CC1((TIM_TypeDef *)Motor_Device1.plf_timer))
  {
    if (LL_TIM_OC_GetMode((TIM_TypeDef *)Motor_Device1.plf_timer, LL_TIM_CHANNEL_CH1) != LL_TIM_OCMODE_ACTIVE)
    {
      LL_TIM_ClearFlag_CC1((TIM_TypeDef *)Motor_Device1.plf_timer);
      if (Motor_Device1.status != MC_STOP)
      {
        MC_Core_NextStepScheduling(&Motor_Device1);
        MC_Core_HallNextStep(&Motor_Device1, 0);
        MotorUpdatePosition(&Motor_Device1);
        MotorPositionTargetTest(&Motor_Device1);
      }
    }
  }
  /* Capture compare 2 event */
  if (LL_TIM_IsActiveFlag_CC2((TIM_TypeDef *)Motor_Device1.plf_timer) && LL_TIM_IsEnabledIT_CC2((TIM_TypeDef *)Motor_Device1.plf_timer))
  {
    if (LL_TIM_OC_GetMode((TIM_TypeDef *)Motor_Device1.plf_timer, LL_TIM_CHANNEL_CH2) == LL_TIM_OCMODE_PWM2)
    {
      LL_TIM_ClearFlag_CC2((TIM_TypeDef *)Motor_Device1.plf_timer);
      MC_Core_HallPrepareNextStep(&Motor_Device1);
    }
  }
}

// void DMA1_Channel2_3_IRQHandler (void)
//{
//   /* Buffer is ready by the HW layer to be processed */
//   if (LL_DMA_IsActiveFlag_TC (DMA_RX_A, DMACH_RX_A) ){
//     LL_DMA_ClearFlag_TC (DMA_RX_A, DMACH_RX_A);
//     ASPEP_HWDataReceivedIT (&aspepOverUartA);
//   }
// }
// void USARTA_IRQHandler(void)
//{
//   /* USER CODE BEGIN USART1_IRQn 0 */
////  if ( LL_USART_IsActiveFlag_TC (USARTA) )
////  {
////    /* Disable the DMA channel to prepare the next chunck of data*/
////    LL_DMA_DisableChannel( DMA_TX_A, DMACH_TX_A );
////    LL_USART_ClearFlag_TC (USARTA);
////    /* Data Sent by UART*/
////    /* Need to free the buffer, and to check pending transfer*/
////    ASPEP_HWDataTransmittedIT (&aspepOverUartA);
////  }
////  if ( LL_USART_IsActiveFlag_ORE (USARTA) )
////  { /* Stopping the debugger will generate an OverRun error*/
////    LL_USART_ClearFlag_ORE (USARTA);
////    LL_USART_EnableIT_IDLE (USARTA);
////  }
////  if ( LL_USART_IsActiveFlag_IDLE (USARTA) && LL_USART_IsEnabledIT_IDLE (USARTA) )
////  { /* Stopping the debugger will generate an OverRun error*/

////    //LL_USART_ClearFlag_IDLE (USARTA);
////    LL_USART_DisableIT_IDLE (USARTA);
////    /* To be sure we fetch the potential pendig data*/
////    /* We disable the DMA request, Read the dummy data, endable back the DMA request */
////    LL_USART_DisableDMAReq_RX (USARTA);
////    LL_USART_ReceiveData8(USARTA);
////    LL_USART_EnableDMAReq_RX (USARTA);
////    ASPEP_HWDMAReset (&aspepOverUartA);

////  }
//  /* USER CODE END USART1_IRQn 0 */

//  /* USER CODE BEGIN USART1_IRQn 1 */

//  /* USER CODE END USART1_IRQn 1 */
//}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  TSK_HardwareFaultTask();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

void SysTick_Handler(void)
{

#ifdef MC_HAL_IS_USED
  static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  if (SystickDividerCounter == SYSTICK_DIVIDER)
  {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
    SystickDividerCounter = 0;
  }
  SystickDividerCounter++;
#endif /* MC_HAL_IS_USED */

  /* USER CODE BEGIN SysTick_IRQn 1 */
  /* USER CODE END SysTick_IRQn 1 */
  check_safety(&Motor_Device1);
  Motor_Device1.uw_tick_cnt++;
  MC_RunMotorControlTasks();

  /* USER CODE BEGIN SysTick_IRQn 2 */
  /* USER CODE END SysTick_IRQn 2 */
}

/**
 * @brief  This function handles Button IRQ on PIN PF0.
 */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN START_STOP_BTN */
  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_0))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    UI_HandleStartStopButton_cb();
  }

  if (LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_1))
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    UI_HandleStartStopButton1_cb();
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2019 STMicroelectronics *****END OF FILE****/
