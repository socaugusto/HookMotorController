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
#ifndef __6STEP_CORE_H
#define __6STEP_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mc_stm_types.h"

  
/** @addtogroup MIDDLEWARES
  * @brief  Middlewares Layer
  * @{ 
  */
  
/** @addtogroup MC_6STEP_LIB
  * @{
  */

/** @addtogroup MC_6STEP_CORE
  * @{
  */ 

/** @defgroup MC_6STEP_CORE_Exported_Defines
  * @{
  */
  #define NUMBER_OF_DEVICES               (1)
/* PWM INTERFACE BEGIN 14 */
#define MC_WAIT_FOR_FALLING_MASK          ((uint8_t)  0x01)
/* PWM INTERFACE END 14 */
#define MC_DUTY_CYCLE_SCALING_SHIFT       ((uint8_t)  10)
#define MC_LF_TIMER_MAX_PERIOD            ((uint16_t) 0xFFFF)
#define MC_SPEED_ARRAY_SIZE               ((uint8_t) (1<<RUN_SPEED_ARRAY_SHIFT))
/* POTENTIOMETER BEGIN 1 */
#define MC_ADC_TO_COMMAND_FILTER_SHIFT    ((uint8_t)  4)
#define MC_ADC_TO_COMMAND_FILTER_COEF     ((uint8_t)  ((1<<MC_ADC_TO_COMMAND_FILTER_SHIFT) - 1))
#define MC_ADC_TO_COMMAND_THRESHOLD       ((uint8_t)  64) 
/* POTENTIOMETER END 1 */
#define NUMBER_OF_USER_ADC_CHANNELS       ((uint8_t)  5)
#define TRUE        ((uint8_t) 1)     /*!< Define TRUE */
#define FALSE       ((uint8_t) 0)     /*!< Define FALSE */
  

///**
//  * @}
//  */
//  
//#define MC_REGULATOR_MIN_MAX_SCALING_DIV  ((uint16_t) 1000)    
//    
//#define MC_SPEED_ARRAY_SIZE               ((uint8_t) (1<<RUN_SPEED_ARRAY_SHIFT))    
    
#define RUN_SPEED_ARRAY_SHIFT        ((uint8_t)  5)      /*!< The size of the speed array is 2^RUN_SPEED_ARRAY_SHIFT */
/** @defgroup MC_6STEP_CORE_Exported_Types
  * @{
  */
  
/** @defgroup MC_6STEP_CORE_Exported_Types
  * @{
  */
typedef enum {
  MC_FUNC_OK   = ((uint8_t) 0),
  MC_FUNC_FAIL,
} MC_FuncStatus_t;

/*!< Motor Control status */
typedef enum {
  MC_IDLE = ((uint8_t) 0),
  MC_STOP,
  MC_ALIGNMENT,
  MC_STARTUP,
  MC_VALIDATION,
  MC_RUN,
  MC_SPEEDFBKERROR,
  MC_OVERCURRENT,
  MC_VALIDATION_FAILURE,
  MC_VALIDATION_BEMF_FAILURE,
  MC_VALIDATION_HALL_FAILURE,
  MC_RUN_WRONG_STEP_FAILURE,
  MC_LF_TIMER_FAILURE,
/* PWM INTERFACE BEGIN 1 */
  MC_ADC_CALLBACK_FAILURE,
  MC_PWM_INTERFACE_FAILURE
/* PWM INTERFACE END 1 */
}MC_Status_t;

/*!< Bemf channels */
typedef enum {
  MC_BEMF_PHASE_1   = ((uint8_t) 0),
  MC_BEMF_PHASE_2   = ((uint8_t) 1),
  MC_BEMF_PHASE_3   = ((uint8_t) 2)
}MC_BemfPhases_t;

/*!< User measurements */
typedef enum {
  MC_USER_MEAS_1    = ((uint8_t) 0),
  MC_USER_MEAS_2    = ((uint8_t) 1),
  MC_USER_MEAS_3    = ((uint8_t) 2),
  MC_USER_MEAS_4    = ((uint8_t) 3),
  MC_USER_MEAS_5    = ((uint8_t) 4)
}MC_UserMeasurements_t;

/* POTENTIOMETER BEGIN 2 */
/*!> Structure for the ADC measurement used to build the speed or the duty cycle command */
typedef struct
{
  uint8_t channel_index;                               /*!< Index of the channel used for speed or duty cycle command */
  uint8_t filter_shift;
  uint16_t filter_coef;                                /*!< Index in the measurements array used for speed or duty cycle command */
  uint16_t change_threshold;  
  uint16_t current;
  uint16_t filtered;
} MC_AdcUserSpeedDutyCycleCommand_t;
/* POTENTIOMETER END 2 */

/* PWM INTERFACE BEGIN 2 */
typedef enum {
  MC_PWM_IF_WAIT_FOR_FALLING_EDGE_OR_PROCESS_RISING = ((uint8_t) 0),  
  MC_PWM_IF_WAIT_FOR_PROCESS_RISING                 = ((uint8_t) 1),
  MC_PWM_IF_WAIT_FOR_FALLING_EDGE                   = ((uint8_t) 2),  
  MC_PWM_IF_WAIT_FOR_PROCESS_FALLING                = ((uint8_t) 3),
  MC_PWM_IF_WAIT_FOR_RISING_EDGE                    = ((uint8_t) 4)
}MC_PwmInterfaceStatus_t;

typedef struct
{
  uint8_t status;
  uint8_t dummy1;
  uint8_t dummy2;
  uint8_t dummy3;
  uint16_t arming_cnt;
  uint16_t arming_valid_ton;
  uint16_t blank_stop_cnt;  
  uint16_t cnt_cycle_hires;                /*!< counter cycle duration in (2^-16) us */  
  uint16_t falling_edge;
  uint16_t rising_edge;
  uint16_t pulse_us_for_command_max;       /*!< pulse duration in us to command max speed */
  uint16_t pulse_us_for_command_min;       /*!< pulse duration in us below which the motor is stopped */
  uint16_t start_cnt;
  uint16_t start_valid_ton;  
  uint16_t stop_cnt;
  uint16_t stop_ms;
  uint16_t stop_periods;
  uint16_t stop_valid_ton;
  uint32_t pulse_hires;                    /*!< captured pwm on duration in (2^-16) us */
  uint32_t pulse_hires_for_command_max;    /*!< pulse duration in (2^-16) us to command max speed or duty cycle */
  uint32_t pulse_hires_for_command_min;    /*!< pulse duration in (2^-16) us to command min speed or duty cycle  */
  uint32_t pulse_hires_wrong_detection;    /*!< pulse duration in (2^-16) us to command min speed or duty cycle  */
  uint32_t pulse_max;                      /*!< maximum pulse value of the timer used for the pwm interface */
  uint32_t pulse_toff_capture;             /*!< captured pwm off value */  
  uint32_t pulse_ton_capture;              /*!< captured pwm on value */   
  uint32_t pulse_us_range;                 /*!< pulse range between min speed and max speed commands in us */  
  uint32_t timer_channel;
  uint32_t* ptimer;
  uint32_t (*get_captured_value)();
} MC_PwmInterface_t;
/* PWM INTERFACE END 2 */

/*!< Structure for USER ADC measurements */
typedef struct
{
  uint8_t channel_index;                               /*!< Index of the array of ADC regular channels for USER measurements */
  uint8_t number_of_channels;
  uint8_t resolution;                                  /*!< Number of adc bits */
  int8_t ts_cal_1_temp_deg_c;
  int8_t ts_cal_2_temp_deg_c;
  uint16_t measurement[NUMBER_OF_USER_ADC_CHANNELS];   /*!< Array of ADC USER measurements: potentiometer, current, vbus, temperature */
  uint16_t trig_time;                                  /*!< Pulse value of the timer channel used to trig the ADC */
  uint16_t trig_timer_period;                          /*!< Period value of the timer used to trig the ADC */
  uint16_t ts_cal_1;
  uint16_t ts_cal_2;
  uint16_t vrefint_cal;
  uint32_t channel[NUMBER_OF_USER_ADC_CHANNELS];       /*!< Array of ADC regular channels used for USER measurements: potentiometer, current, vbus, temperature */  
  uint32_t sampling_time[NUMBER_OF_USER_ADC_CHANNELS]; /*!< Array of ADC USER measurements sampling rates */
  uint32_t trig_timer_channel;                         /*!< Channel of the timer used to trig the ADCs > */
  uint32_t* padc[NUMBER_OF_USER_ADC_CHANNELS];         /*!< Array of pointer to the ADC used for a USER measurement */
  uint32_t* ptrig_timer;                               /*!< Pointer to the timer used to trig the ADCs > */
  /* POTENTIOMETER BEGIN 3 */
  MC_AdcUserSpeedDutyCycleCommand_t spd_dc_command;
  /* POTENTIOMETER END 3 */
} MC_AdcUser_t;

/*!< Structure for USER BUTTON */
typedef struct
{
  uint8_t enabled;
  uint16_t gpio_pin;
  uint16_t debounce_time_ms;
} MC_ButtonUser_t;

/*!< Structure for GPIO BEMF pin */
typedef struct
{ 
  GPIO_TypeDef* GPIO_BEMF_Port;        /*!< GPIO BEMF circuit partition enabling port */
  uint16_t GPIO_BEMF_Pin;              /*!< GPIO BEMF circuit partition enabling pin */
  } MC_GPIO_BEMF_t;

/* SENSE COMPARATORS BEGIN 1 */
/*! < Structure for Sense Comparators Output measurement */
typedef struct
{
  uint16_t full_ramp_start;
  uint16_t guard_after_pwm_edge_zc_cycles;       /*!< Time in ZC timer counter cycles after the HF timer pwm edge to read the comparator output */
  int16_t guard_after_pwm_edge_hf_cycles;        /*!< Time in HF timer counter cycles after the HF timer pwm edge to read the comparator output */
  int16_t guard_before_pwm_edge_hf_cycles;       /*!< Time in HF timer counter cycles before the HF timer pwm edge to read the comparator output */  
  int16_t hf_current_counter_to_pwm_edge_cycles; /*!< Diffence between the HF timer current counter value and the HF timer compare value used to trigger the ZC timer */
  uint16_t hfn;
  uint16_t interrupt_call_number;  
  uint16_t level;
  uint16_t u_high_level;
  uint16_t u_low_level;
  uint16_t u_pin;
  uint16_t v_high_level;
  uint16_t v_low_level;
  uint16_t v_pin;
  uint16_t w_high_level;
  uint16_t w_low_level;
  uint16_t w_pin;  
  uint16_t zc_timer_period;
  uint16_t zcd_time;
  uint16_t zcd_to_comm;
  uint32_t zc_timer_frequency;
  uint32_t* pzc_timer;
  uint32_t* pu_port;
  uint32_t* pv_port;
  uint32_t* pw_port;
} MC_SenseComp_t;
/* SENSE COMPARATORS END 1 */

/*!< Structure for BEMF ADC measurements */
typedef struct
{
  MC_GPIO_BEMF_t GPIO_BEMFon;
  uint16_t const_validation_bemf_events_max;     /*!<VALIDATION_BEMF_EVENTS_MAX*/
  uint16_t const_validation_zero_cross_number;   /*!<VALIDATION_ZERO_CROSS_NUMBER*/
  uint32_t const_validation_steps_max;          /*!<VALIDATION_STEPS_MAX*/
  uint16_t const_validation_demagn_delay;       /*!<VALIDATION_DEMAGN_DELAY*/
  uint16_t const_run_demagn_delay_min;          /*!<RUN_DEMAGN_DELAY_MIN*/
  uint16_t const_run_lf_timer_prescaler;        /*!<RUN_LF_TIMER_PRESCALER*/
  uint16_t const_bemf_demagn_counter;           /*!<MC_BEMF_DEMAGN_COUNTER_INIT_VALUE*/ 
  uint16_t const_adc_threshold_up_on;
  uint16_t const_adc_threshold_up;
  uint16_t const_adc_threshold_down_on; 
  uint16_t const_adc_threshold_down;
  uint8_t over_threshold_events;
  /* PWM ON sensing BEGIN 1 */
  uint8_t pwm_on_sensing_enabled;     /*!< Value where 0 means BEMF is sensed during PWM OFF time and 1 or greater means BEMF is sensed during PWM ON time */  
  /* PWM ON sensing END 1 */
  uint8_t zero_crossing_events;
  uint16_t adc_threshold_down;        /*!< BEMF voltage threshold for zero crossing detection when BEMF is decreasing */
  uint16_t adc_threshold_up;          /*!< BEMF voltage threshold for zero crossing detection when BEMF is increasing */  
  uint16_t demagn_counter;            /*!< Demagnetization counter */
  uint16_t demagn_value;              /*!< Demagnetization value */
  /* PWM ON sensing BEGIN 2 */
  uint16_t pwm_off_sensing_trig_time; /*!< Pulse value of the timer channel used to trig the ADC when sensing occurs during PWM OFF time */
  uint16_t pwm_on_sensing_en_thres;   /*!< Pulse value of HF timer above which the PWM ON sensing is enabled */
  uint16_t pwm_on_sensing_dis_thres;  /*!< Pulse value of HF timer below which the PWM ON sensing is disabled */
  uint16_t pwm_on_sensing_trig_time;  /*!< Pulse value of the timer channel used to trig the ADC when sensing occurs during PWM ON time */
  /* PWM ON sensing END 2 */
  uint16_t trig_time;                 /*!< Current pulse value of the timer channel used to trig the ADC */
  uint16_t trig_timer_period;         /*!< Period value of the timer used to trig the ADC */
  uint16_t zcd_to_comm;               /*!< Zero Crossing detection to commutation delay in 15/128 degrees */
  uint32_t* ptrig_timer;              /*!< Pointer to the timer used to trig the ADC */
  uint32_t adc_channel[3];            /*!< Array of ADC regular channels used for BEMF sensing */
  uint32_t current_adc_channel;       /*!< ADC regular channel to select for BEMF sensing */
  uint32_t demagn_coefficient;        /*!< Proportional parameter to compute the number of HF TIMER periods before checking BEMF threshold */
  uint32_t run_speed_thres_demag;     /*!< Speed threshold above which the RUN_DEMAGN_DELAY_MIN is applied */
  uint32_t sampling_time;             /*!< Sampling time value */
  uint32_t trig_timer_channel;        /*!< Channel of the HF timer used to trig the ADC */  
  uint32_t* padc;                     /*!< Pointer to the ADC */
} MC_Bemf_t;

/*!< Structure for HALL GPIOs */
typedef struct
{ 
  GPIO_TypeDef* HALLx_GPIO_Port;        /*!< Hall sensor GPIO port */
  uint16_t HALLx_GPIO_Pin;              /*!< Hall sensor GPIO pin */
  } MC_Hall_GPIO_t;

/*!< Structure for HALL sensors */
typedef struct
{
  MC_Hall_GPIO_t HALL_IOs[3];           /*!< Hall sensor IO configuration */
  uint8_t alignement_step_change;     /*!< This value is interpreted as a boolean used to force an alignment step change when the motor does not start */
  uint8_t status;                     /*!< Agregate value of the digital hall sensors where one hall output is set as MSB, another one is set as middle bit and the last as LSB */ 
  uint16_t commutation_delay;         /*!< Delay between hall capture and step commutation */
  uint16_t commutation_errors_max;        /*!< Maximum number of wrong commutation */
  uint16_t commutation_errors;        /*!< Maximum number of wrong commutation */
  uint16_t stay_in_run_lf_cnt;        /*!< LF timer max period counter */
  uint16_t stay_in_run_lf_max;        /*!< While the motor is stalled, stay in MC_RUN state during this value in LF timer max period */
} MC_Hall_t;

/*!< Motor Control PID regulator structure */
typedef struct
{
  uint16_t kp;                    /*!< Proportional gain for PID regulator */
  uint16_t ki;                    /*!< Integral gain for PID regulator */
  uint32_t kd;                    /*!< Derivative gain for PID regulator */
  uint16_t scaling_shift;         /*!< Scaling of the gains of the PID regulator */
  int32_t integral_term_sum;      /*!< Integral term sum of the PID regulator */
  uint32_t previous_speed;        /*!< Previous speed fed into the PID regulator */
  int16_t minimum_output;         /*!< Min output value for PID regulator */ 
  uint16_t maximum_output;        /*!< Max output value for PID regulator */
} MC_Pid_t;

/*!< Motor Characteristics structure */
typedef struct
{
  uint8_t pole_pairs;             /*!< Number of motor pole pairs  */
} MC_MotorCharacteristics_t;

/*!< Structure for enablePWM GPIOs */
typedef struct
{ 
  GPIO_TypeDef* enPWMx_GPIO_Port;        /*!< Power bridge enable GPIO port */
  uint16_t enPWMx_GPIO_Pin;              /*!< Power bridge enable GPIO pin */
  } MC_EN_GPIOs_t;


/*!< Structure for enablePWM GPIOs */
typedef struct
{ 
  GPIO_TypeDef* enDriver_GPIO_Port;        /*!< Driver enable GPIO port */
  uint16_t enDriver_GPIO_Pin;              /*!< Driver enable GPIO pin */
  } MC_EN_DRIVER_t;


/*!< Motor Control handle structure */
typedef struct
{
  uint8_t const_startup_direction;              /*!<STARTUP_DIRECTION*/
  uint8_t const_alignment_step;                 /*!<ALIGNMENT_STEP*/
  uint32_t const_startup_acceleration;          /*!<STARTUP_ACCELERATION*/
  uint32_t const_startup_speed_target;          /*!<STARTUP_SPEED_TARGET*/
  uint8_t id;                               /*!< Motor Control device id */
  MC_Status_t status;                       /*!< Motor Control device status */
  uint8_t direction;                        /*!< Motor direction CW:0 CCW:1 */
  uint8_t lf_timer_period_array_completed;  /*!< Completion of LF timer period array Completed: 1 Not completed: 0 */
  uint8_t lf_timer_period_array_index;      /*!< Index of LF timer period array */
  uint8_t reference_to_be_updated;          /*!< Different from 0 if the reference has to be updated */
  uint8_t step_change;                      /*!< SL : Step has been changed and ADC callback has not been called yet when not 0 */
  uint8_t step_pos_next;                    /*!< Next step number for the 6-step algorithm */
  uint8_t step_position;                    /*!< Current step number for the 6-step algorithm */   
  uint8_t step_prepared;                    /*!< Step configuration for the 6-step algorithm Prepared: 1 Not prepared: 0 */
  uint16_t align_index;                     /*!< Index indicating the time elapsed in ms in MC_ALIGNMENT state */
  uint16_t speed_update;                    /*!< Time for speed update in MC_START_UP state */
  uint16_t alignment_time;                  /*!< Time for alignment (msec) */
  uint16_t control_loop_time;               /*!< Periodicity in ms of the loop controlling the HF timer PWMs */
  uint16_t hf_timer_period;                 /*!< Period of the HF timer in clock cycles */
  uint16_t hf_timer_pulse_value_max;        /*!< CM : Maximum HF timer pulse value in clock cycles */
  uint16_t hf_timer_pulse_value_ref;        /*!< PWM on time in 1/1024 of PWM period elapsed */
  uint16_t lf_timer_period;                 /*!< Period of the LF timer in clock cycles */
  uint16_t lf_timer_period_array[MC_SPEED_ARRAY_SIZE]; /*!< LF timer period array */  
  uint16_t lf_timer_prescaler;              /*!< Prescaler of the LF timer in clock cycles */
  uint16_t pulse_command;                   /*!< To be set HF or REF timer pulse value in clock cycles */
  /* PWM INTERFACE BEGIN 3 */
  uint16_t pulse_min_command;               /*!< In clock cycles, minimum pulse command admissible */
  uint16_t pulse_max_command;               /*!< In clock cycles, maximum pulse command admissible */
  /* PWM INTERFACE END 3 */
  uint16_t pulse_value;                     /*!< Current HF or REF timer pulse value in clock cycles */
  uint16_t ref_timer_period;                /*!< Period of the REF timer in clock cycles */
  uint16_t startup_reference;               /*!< Startup value for pulse value or current reference */
  uint16_t tick_cnt;                        /*!< Counter value to be check against the control loop time to control the speed or the duty cycle */
  uint32_t* phf_timer;                      /*!< Pointer to the HF timer > */
  uint32_t* plf_timer;                      /*!< Pointer to the LF timer > */
  uint32_t* pref_timer;                     /*!< Pointer to the REF timer > */
  uint32_t channel_polarity;	  	        /*!< channel PWM driving polarity > */
  uint32_t Nchannel_polarity;	  	        /*!< Nchannel PWM driving polarity > */
  MC_EN_GPIOs_t enPWMx[3];                  /*!< Phase EN GPIOs > */
  MC_EN_DRIVER_t enDRIVER;                  /*!< Master enable GPIO > */
  uint32_t acceleration;                    /*!< Motor acceleration in rpm/s or update rate of the pulse value of HF or REF timer */
  int32_t duty_cycle_ramp_factor;           /*!< Ratio between speed range and duty cycle range at startup */
  int32_t duty_cycle_ramp_residual;         /*!< Parameter storing duty cycle increasing at startup */
  uint32_t gate_driver_frequency;           /*!< Frequency in Hz of the HF timer pwm used to drive the transistor gates on each motor phases */
  uint32_t speed_target_value;              /*!< In rpm, speed target value updated periodically to reach the speed target command */
  uint32_t speed_fdbk_filtered;             /*!< Motor filtered speed variable in rpm */
  uint32_t speed_target_command;            /*!< In rpm, speed target command */
  /* PWM INTERFACE BEGIN 4 */
  uint32_t speed_target_min_command;        /*!< In rpm, minimum speed target command admissible */
  uint32_t speed_target_max_command;        /*!< In rpm, maximum speed target command admissible */
  /* PWM INTERFACE END 4 */
  uint32_t steps;                           /*!< Number of steps since MC_STARTUP until MC_RUN */
  uint32_t uw_tick_cnt;
  MC_MotorCharacteristics_t motor_charac;   /*!< Motor characteristics */
  MC_Bemf_t bemf;                           /*!< SL */
  MC_Hall_t hall;                           /*!< HS */
  /* SENSE COMPARATORS BEGIN 2 */
  MC_SenseComp_t sense_comp;                /*!< SC */
  /* SENSE COMPARATORS END 2 */
  MC_Pid_t pid_parameters;                  /*!< SPDLP */
  MC_AdcUser_t adc_user;
  MC_ButtonUser_t button_user;
  /* PWM INTERFACE BEGIN 5 */
  MC_PwmInterface_t pwm_if;
  /* PWM INTERFACE END 5 */
} MC_Handle_t;
/**
  * @} end MC_6STEP_CORE_Exported_Types
  */


/**
  * @} end MC_6STEP_CORE_Exported_LL_FunctionsPrototype
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