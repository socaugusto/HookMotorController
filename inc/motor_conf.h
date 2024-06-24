/**
 ******************************************************************************
 * @file    Motor_Configuration.h
 * @author  IPC Rennes & Motor Control SDK, ST Microelectronics
 * @brief   Header file for the 6step_conf.c file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics International N.V.
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
#ifndef __MOTOR_CONF_H
#define __MOTOR_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{
  */

/** @addtogroup MC_LIB_6STEP
  * @{
  */

/** @defgroup MC_6STEP_CONF
  * @{
  */

/** @defgroup MC_6STEP_CONF_Exported_Defines
  * @{
  */
/*!< Manage the Motor and its related configuration */
/*!< Bull Running (BR2804-1700kv) Motor characteristics */
#define NUMBER_OF_DEVICES       (1)     /*!< Number of devices */

/*!< Motor control alignment parameters */
#define ALIGNMENT_TIME               ((uint16_t) 100)      /*!< Time for alignment (msec) */
#define ALIGNMENT_FORCE_STEP_CHANGE  ((uint16_t) 0)      /*!< If not equal to 0, force a step change when the motor does not start with the value reported by the hall sensors */

/*!< Motor control startup parameters */

#define STARTUP_SPEED_TARGET         ((uint16_t) 100)   /*!< Target speed during startup (open loop) */
#define STARTUP_DIRECTION            ((uint8_t)  0)      /*!< Rotation direction in the motor, 0 for ClowkWise, 1 for CounterClockWise */

#define STARTUP_DUTY_CYCLE           ((uint16_t) 82) /*!< PWM on time in 1/1024 of PWM period - HF timer in VOLTAGE_MODE, REF timer in CURRENT_MODE */

 /*!< Motor control run parameters */
#define RUN_COMMUTATION_ERRORS_MAX   ((uint16_t) 3)      /*!< Maximum number of wrong commutations*/
#define RUN_COMMUTATION_DELAY	     ((uint16_t) 10)      /*!< Delay between Hall cummutation and step change*/
#define RUN_STAY_WHILE_STALL_MS      ((uint32_t) 1000)   /*!< While the motor is stalled, stay in MC_RUN state during this value in ms */

#define RUN_CONTROL_LOOP_TIME        ((uint16_t) 1)      /*!< Periodicity in ms of the loop controlling the HF timer PWMs or the REF timer PWM */
#define RUN_SPEED_TARGET             ((uint16_t) 1200)   /*!< Target speed during startup (open loop) */

/* POTENTIOMETER BEGIN 1 */
#define RUN_SPEED_MIN                ((uint32_t) 1000)   /*!< Minimum speed command in RPM in RUN state */
#define RUN_SPEED_MAX                ((uint32_t) 5000)  /*!< Maximum speed command in RPM in RUN state */
/* POTENTIOMETER END 1 */

/*!< User misceallenous parameters */
#define USER_ADC_TRIG_TIME           ((uint16_t) 512)    /*!< 1/1024 of PWM period elapsed */

/*!< Motor control speed PID regulator parameter */
#define PID_KP                       ((uint16_t) 1000)    /*!< Kp parameter for the PID regulator */
#define PID_KI                       ((uint16_t) 100)      /*!< Ki parameter for the PID regulator */
#define PID_KD                       ((uint16_t) 0)      /*!< Kd parameter for the PID regulator */
#define PID_SCALING_SHIFT            ((uint16_t) 16)     /*!< Kp, Ki, Kd scaling for the PID regulator, from 0 to 15 */
#define PID_OUTPUT_MIN               ((uint16_t) 50)     /*!< Minimum output value of the PID regulator in tenths of percentage of the HF or REF timer period */
#define PID_OUTPUT_MAX               ((uint16_t) 1000)    /*!< Maximum output value of the PID regulator in tenths of percentage of the HF or REF timer period */

/**
  * @} end MC_6STEP_CONF_Exported_Defines
  */

/**
  * @}  end MC_6STEP_CONF
  */

/**
  * @}  end MC_LIB_6STEP
  */

/**
  * @}  end MIDDLEWARES
  */

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
