/**
 *@file   : mc_hook_remote_config.h
 *@author : veysel gokdemir,
 *@date   : 22.08.2021
 *@brief  : Prototype general settings, functions
 */

#ifndef _MC_HOOK_REMOTE_CONFIG_H
#define _MC_HOOK_REMOTE_CONFIG_H

/*Includes---------------------------------------------------------------------------------------------------------------------------*/
#include "stdint.h"
#include "mc_config.h"
#include "mc_interface.h"
#include "6step_service_f0xx.h"

/*-----------------------------------------------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C"
{
#endif

/*Definitions, enums, structures-----------------------------------------------------------------------------------------------------*/
// ADC
#define VREFINT_CAL *((uint16_t *)(0x1FFFF7BAU)) // Read VREFINT, internal voltage ref. cal. value stored in the memory.

#define VBUS_SCALE_FACTOR 19.17 // VBUS Feedback onboard voltage divider scale factor.
#define VBUS_ERROR 1.035		// VBUS error coefficient.

#define CURR_SENSE_RSHUNT 0.01 // Current sense resistor R=0.01 Ohm.
#define CURR_SENSE_GAIN 3.13   // Current sense amplification overall Op-amp gain=3.125, onboard.
#define CURR_SENSE_SCALE_FACTOR ((float)(1 / (CURR_SENSE_RSHUNT * CURR_SENSE_GAIN)))

// UART
#define HOOK_MONITOR_REFRESH_TIMESTAMP 250 // Update hook monitoring values per 250ms.

#define REMOTE_BUFF_SIZE 8
#define HOOK_BUFF_SIZE 12

// Hook Position
#define HOOK_MECH_FULL_POS_VAL ((uint32_t)13200)
#define HOOK_MECH_HALF_POS_VAL ((uint32_t)5000) // (HOOK_MECH_FULL_POS_VAL/2))
#define HOOK_MECH_HOME_POS_VAL ((uint32_t)0)

// Hook Speed
#define HOOK_HOMING_SPEED_VAL ((uint32_t)750)

// Headers
#define HDR '$'
#define H_HDR 'H'
#define R_HDR 'R'
#define CR '\r'
#define LF '\n'

	/**@brief enum for motor direction status. */
	typedef enum _motor_direction_stat_t
	{

		CW = 0x00,
		CCW = 0x01

	} motor_direction_stat_t;

#pragma pack(1)
	typedef struct RemoteCommand_t_
	{
		uint8_t operation;
		int16_t Parameter1;
		int16_t Parameter2;
		int16_t Parameter3;

	} RemoteCommand_t;

/**@brief structure for remote command data. */
#pragma pack(1)
	typedef struct _hook_remote_cmd_t
	{

		uint8_t r_data[REMOTE_BUFF_SIZE];
		uint8_t r_header[2];
		uint8_t open_pos;
		uint8_t mid_pos;
		uint8_t close_pos;
		uint8_t cr, lf;
	} hook_remote_cmd_t;

#pragma pack(1)
	typedef struct stdReply_t_
	{
		uint16_t voltage; // mV
		int16_t current;  // mA
		uint16_t position;
		uint8_t error;
		struct
		{
			uint8_t sequenceNumber : 3;
			uint8_t dataType : 1;
			uint8_t dataNumber : 4;
		} command;
		uint8_t dataValues[4];

	} stdReply_t;

#pragma pack(1)
	typedef struct HookReply_t_
	{
		uint8_t header;
		uint8_t type;
		stdReply_t data;
		uint16_t checksum;
	} HookReply_t;

	typedef enum Errors_e_
	{
		ERROR_NONE,
		ERROR_INVALID_PARAMETER,
		ERROR_FAILED_TO_START_MOTOR,
		ERROR_OVERLOAD,
		ERROR_COMMAND_TIMEOUT,
		ERROR_INVALID_SEQUENCE_NUMBER,
		ERROR_ESTOP,

	} Errors_e;

	/*-----------------------------------------------------------------------------------------------------------------------------------*/

	/*Variables--------------------------------------------------------------------------------------------------------------------------*/

	/*-----------------------------------------------------------------------------------------------------------------------------------*/

	/*Private Functions------------------------------------------------------------------------------------------------------------------*/
	/**@brief Function prototype for reading remote data. */
	void hook_command_run(hook_remote_cmd_t *rcmd, MC_Handle_t *motor_device);

	/**@brief Function prototype for motor device start. */
	void MotorStart(MC_Handle_t *motor_device);

	/**@brief Function prototype for motor device stop and reset. */
	void MotorStop(MC_Handle_t *motor_device);

	/**@brief Function prototype for motor device start/stop. */
	void MotorStartStop(MC_Handle_t *motor_device, motor_direction_stat_t motor_direction);

	/**@brief Function prototype for setting motor direction. */
	void MotorSetDirection(MC_Handle_t *motor_device, motor_direction_stat_t motor_direction);

	/**@brief Function prototype for setting motor speed. */
	void MotorSetSpeed(MC_Handle_t *motor_device, uint32_t motor_speed);

	/**@brief Function prototype for handling hook data to remote side as to obtainning and sending batt., load values and led position feedbacks. */
	void hook_monitoring_handle(HookReply_t *hd, MC_Handle_t *motor_device);

	/**@brief Set external errors. */
	void hook_setError(Errors_e number);

	/**@brief Check safety line. */
	void check_safety(MC_Handle_t *motor_device);

	/*-----------------------------------------------------------------------------------------------------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif

/*******************************************************END OF FILE*******************************************************************/
