/**
 *@file   : mc_hook_remote_config.c
 *@author : veysel gokdemir
 *@date   : 22.08.2021
 *@brief  : Prototype general settings, functions
 */

/**
 *@notes:
 * #TIM3 settings:
 * TIM3 is set to take interrupt for every 1ms and triggers ADC1 process per 1ms.
 * Required_freq = TIM3_CLK / (Prescaler + 1) * (ARR + 1)
 * f = 48Mhz / (0 + 1) * (47999 + 1), f = 1000Hz, T = 1ms.
 * TIM3 update event is also used for updating uart tx data for every required timestamp (default:500ms) by using TIM3_TimerTick variable.
 */

/*Includes---------------------------------------------------------------------------------------------------------------------------*/
#include "mc_hook_remote_config.h"
#include "main.h"
#include "stdio.h"
#include <memory.h>

/*Definitions------------------------------------------------------------------------------------------------------------------------*/
typedef enum Errors_e_
{
	ERROR_NONE,
	ERROR_INVALID_PARAMETER,
	ERROR_FAILED_TO_START_MOTOR,

} Errors_e;

typedef enum Command_e_
{
	SPIN_COMMAND_NONE,
	SPIN_COMMAND_MOVE,
	SPIN_COMMAND_STOP,
	SPIN_COMMAND_EACK,
	SPIN_COMMAND_REBOOT,
	SPIN_COMMAND_SET_PARAMETER,
	SPIN_COMMAND_READ_PARAMETER,
} Command_e;

typedef enum Parameters_e_
{
	PARAMETER_NONE,
	PARAMETER_KP,
	PARAMETER_KI,
	PARAMETER_KD,
	PARAMETER_PID_SCALING_SHIFT,
	PARAMETER_PID_OUTPUT_MIN,
	PARAMETER_PID_OUTPUT_MAX,
	PARAMETER_CURRENT_LIMIT_VALUE,
	PARAMETER_CURRENT_LIMIT_TYPE,
	PARAMETER_CURRENT_LIMIT_ADC_FILTER_VALUE,

} Parameters_e;

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*Variables--------------------------------------------------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

uint32_t tim3_TimerTick = 0;
volatile bool TIM3_UpdateFlag_IsActive = false;

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
uint16_t adc1_RawData[3];
volatile bool ADC1_ConvCpltFlag_IsActive = false;

static volatile bool HOOK_OPEN_POS_LED_FLAG = false;
static volatile bool HOOK_MID_POS_LED_FLAG = false;
static volatile bool HOOK_CLOSE_POS_LED_FLAG = false;

static volatile bool HOOK_UNKNOWN_POS_FLAG = false;
static volatile bool HOOK_HOMING_FLAG = false;
static volatile bool HOOK_TESTING_FLAG = false;

static volatile bool HOOK_OPEN_POS_CMD_FLAG = false;
static volatile bool HOOK_MID_POS_CMD_FLAG = false;
static volatile bool HOOK_CLOSE_POS_CMD_FLAG = false;

static volatile bool OpenCMDindex_Flag = false;
static volatile bool MidCMDindex_Flag = false;
static volatile bool CloseCMDindex_Flag = false;

// static uint8_t cmd;
// static uint32_t OpenCMDindex = 0;
// static uint32_t MidCMDindex = 0;
// static uint32_t CloseCMDindex = 0;
// static uint8_t lastHookStatus;
// static hook_process_stat_t hookProcessStat = HOOK_PROCESS_NONE;

static volatile bool HOOK_HOMING_MOTOR_STOP_FLAG = false;

static int32_t hookPosition = UINT16_MAX;
static uint16_t hookTarget = 0;
static Errors_e errorNo = ERROR_NONE;

extern uint16_t pidKp;
extern uint16_t pidKi;
extern uint16_t pidKd;
extern uint16_t pidScaling;
extern uint16_t pidOutMin;
extern uint16_t pidOutMax;
static uint16_t currentLimit = 5;
static uint16_t currentLimitMaxCount = 3;
static uint16_t OvercurrentCounter = 0;

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*Private Functions------------------------------------------------------------------------------------------------------------------*/
/**
 *@brief: Function for hook data default init.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None
 */
void HOOK_DATA_Init(hook_data_t *hd)
{
	// initial data
	hd->h_header[0] = HDR;
	hd->h_header[1] = H_HDR;
	hd->batt_voltage_m = 0;
	hd->load_current_m = 0;
	hd->open_pos_led = 0;
	hd->mid_pos_led = 0;
	hd->close_pos_led = 0;
	hd->fault_led = 0;
	hd->cr = CR;
	hd->lf = LF;
}

/**
 *@brief: Function for motor device start.
 *@param: motor_device: pointer to MC_Handle structure
 *@retval: None
 */
void MotorStart(MC_Handle_t *motor_device)
{
	MC_Status_t status;
	status = MC_Core_GetStatus(motor_device);
	if (status == MC_STOP)
	{
		MC_Core_Start(motor_device);
	}
	else
	{
		errorNo = ERROR_FAILED_TO_START_MOTOR;
	}
}

/**
 *@brief: Function for motor device stop and reset.
 *@param: motor_device: pointer to MC_Handle structure
 *@retval: None
 */
void MotorStop(MC_Handle_t *motor_device)
{
	MC_Status_t status;
	status = MC_Core_GetStatus(motor_device);
	if (status == MC_RUN)
	{
		MC_Core_Stop(motor_device);
		MC_Core_Reset(motor_device);
	}
}

/**
 *@brief: Function for motor device start/stop.
 *@param: motor_device: pointer to MC_Handle structure
 *        motor_direction: variable for setting the direction to CW:0 or CCW:1
 *@retval: None
 */
void MotorStartStop(MC_Handle_t *motor_device, motor_direction_stat_t motor_direction)
{
	MC_Status_t status;
	status = MC_Core_GetStatus(motor_device);
	if (status == MC_RUN)
	{
		MC_Core_Stop(motor_device);
		MC_Core_Reset(motor_device);
	}
	else if (status == MC_STOP)
	{
		MC_Core_SetDirection(motor_device, motor_direction);
		MC_Core_Start(motor_device);
	}
}

/**
 *@brief: Function for setting motor direction.
 *@param: motor_device: pointer to MC_Handle structure
 *        motor_direction: variable for setting the direction to CW:0 or CCW:1
 *@retval: None
 */
void MotorSetDirection(MC_Handle_t *motor_device, motor_direction_stat_t motor_direction)
{
	if (motor_device->status == MC_STOP)
	{ // Change motor direction when motor stops (!!!the direction should not be changed while motor running).
		MC_Core_SetDirection(motor_device, motor_direction);
	}
}

/**
 *@brief: Function for setting motor speed.
 *@param: motor_device: pointer to MC_Handle structure
 *        motor_speed: variable for setting the speed rpm.
 *@retval: None
 */
void MotorSetSpeed(MC_Handle_t *motor_device, uint32_t motor_speed)
{
	MC_Core_SetSpeed(motor_device, motor_speed);
}

/**
 *@brief: To be called when the hall status change.
 *@param: motor_device: pointer to MC_Handle structure
 *@retval: None
 */
void MotorUpdatePosition(MC_Handle_t *motor_device)
{
	if (motor_device->direction)
	{
		hookPosition++;
	}
	else
	{
		hookPosition--;
	}
}

static motor_direction_stat_t MotorGetDirection(int16_t speed)
{
	return ((speed > 0) ? CW : CCW);
}

static void MotorSetTargetPosition(uint16_t target)
{
	hookTarget = target;
}

void hook_command_run(hook_remote_cmd_t *rcmd, MC_Handle_t *motor_device)
{
	if (rcmd->r_data[0] == 0xFE)
	{
		RemoteCommand_t *cmd = (RemoteCommand_t *)&rcmd->r_data[1];

		switch (cmd->operation)
		{
		case SPIN_COMMAND_MOVE:
			MotorSetDirection(motor_device, MotorGetDirection(cmd->Parameter2));
			MotorSetSpeed(motor_device, cmd->Parameter2);
			MotorSetTargetPosition(cmd->Parameter1);
			MotorStart(motor_device);

			break;
		case SPIN_COMMAND_STOP:
			MotorStop(motor_device);

			break;
		case SPIN_COMMAND_EACK:
			errorNo = ERROR_NONE;
			MC_Core_Reset(motor_device);
			motor_device->status = MC_STOP;

			break;
		case SPIN_COMMAND_REBOOT:
			HAL_NVIC_SystemReset();

			break;
		case SPIN_COMMAND_SET_PARAMETER:
			switch (cmd->Parameter1)
			{
			case PARAMETER_KP:
				pidKp = (uint16_t)cmd->Parameter2;
				MC_Core_SpeedRegulatorReset(motor_device);

				break;
			case PARAMETER_KI:
				pidKi = (uint16_t)cmd->Parameter2;
				MC_Core_SpeedRegulatorReset(motor_device);

				break;
			case PARAMETER_KD:
				pidKd = (uint16_t)cmd->Parameter2;
				MC_Core_SpeedRegulatorReset(motor_device);

				break;
			case PARAMETER_PID_SCALING_SHIFT:
				pidScaling = (uint16_t)cmd->Parameter2;
				MC_Core_SpeedRegulatorReset(motor_device);

				break;
			case PARAMETER_PID_OUTPUT_MIN:
				pidOutMin = (uint16_t)cmd->Parameter2;
				MC_Core_SpeedRegulatorReset(motor_device);

				break;
			case PARAMETER_PID_OUTPUT_MAX:
				pidOutMax = (uint16_t)cmd->Parameter2;
				MC_Core_SpeedRegulatorReset(motor_device);

				break;
			case PARAMETER_CURRENT_LIMIT_VALUE:
				currentLimit = (uint16_t)cmd->Parameter2;

				break;
			case PARAMETER_CURRENT_LIMIT_TYPE:
				MX_TIM1_Init((uint16_t)cmd->Parameter2);

				break;
			case PARAMETER_CURRENT_LIMIT_ADC_FILTER_VALUE:
				currentLimitMaxCount = (uint16_t)cmd->Parameter2;

				break;
			case PARAMETER_NONE:
			default:
				errorNo = ERROR_INVALID_PARAMETER;

				break;
			}

			break;
		case SPIN_COMMAND_READ_PARAMETER:

			break;
		}
	}
}

/**
 *@brief: Function for ADC process handling as to the obtainning and the calculation of Vref Int, VBus and Current Sense values.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
void ADC_ProcessHandle(hook_data_t *hd)
{
	float Vref = 0;
	float CurrentSense = 0;
	float VBus = 0;

	if (ADC1_ConvCpltFlag_IsActive)
	{

		// VrefInt formatting
		uint16_t VrefInt_adcVal = adc1_RawData[2]; // Read adc raw data, ADC_CHANNEL_VREFINT.

		Vref = (float)((3.3 * VREFINT_CAL) / VrefInt_adcVal); // Internal ref. voltage conversion by reading vrefint calibration values from registers.

		// Current sense formatting
		uint16_t CurrentSense_adcVal = adc1_RawData[0]; // Read adc raw data, PA4.

		CurrentSense = (float)((Vref * CurrentSense_adcVal) / 4095); // Current conversion, Vadc by MCU.
		CurrentSense *= CURR_SENSE_SCALE_FACTOR;					 // Current calc., Vshunt=Ishunt*Rshunt, Vshunt*GAIN=Vadc, Is=Vadc/(Rs*GAIN).

		if (CurrentSense > currentLimit && ++OvercurrentCounter > currentLimitMaxCount)
		{
			Motor_Device1.status = MC_OVERCURRENT;
			MC_Core_Error(&Motor_Device1);
			OvercurrentCounter = 0;
		}
		else if (CurrentSense < 1)
		{
			OvercurrentCounter = 0;
		}

		hd->load_current_m = CurrentSense;

		// VBus formatting
		uint16_t VBus_adcVal = adc1_RawData[1]; // Read adc raw data, PA5.

		VBus = (float)((Vref * VBus_adcVal) / 4095); // Voltage conversion, Vadc by MCU.
		VBus *= VBUS_SCALE_FACTOR;					 // Voltage calc., VBUS voltage equals: VBus*VBUS_SCALE_FACTOR.

		hd->batt_voltage_m = VBus;

		// Clear flag
		ADC1_ConvCpltFlag_IsActive = false;

		// Reset&Restart ADC, clear garbage old data in DMA buff..
		HAL_ADC_Stop_DMA(&hadc);
		HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc1_RawData, 3);
	}
}

/**
 *@brief: Function for UART tx transmit handling as to sending hook data to remote side.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
void UART_TransmitHandle(hook_data_t *hd)
{

	if (tim3_TimerTick >= HOOK_MONITOR_REFRESH_TIMESTAMP)
	{

		// Voltage formatting
		uint8_t v1 = (uint8_t)hd->batt_voltage_m;
		uint8_t v2 = (uint8_t)((uint16_t)(hd->batt_voltage_m * 100) - (uint16_t)(v1 * 100));

		// Current formatting
		uint8_t c1 = (uint8_t)hd->load_current_m;
		uint8_t c2 = (uint8_t)((uint16_t)(hd->load_current_m * 100) - (uint16_t)(c1 * 100));

		// UART buffer formatting
		hd->h_data[0] = hd->h_header[0];
		hd->h_data[1] = hd->h_header[1];
		hd->h_data[2] = v1 + '0';
		hd->h_data[3] = v2 + '0';
		hd->h_data[4] = c1 + '0';
		hd->h_data[5] = c2 + '0';
		hd->h_data[6] = hd->open_pos_led + '0';
		hd->h_data[7] = hd->mid_pos_led + '0';
		hd->h_data[8] = hd->close_pos_led + '0';
		hd->h_data[9] = hd->fault_led + '0';
		hd->h_data[10] = hd->cr;
		hd->h_data[11] = hd->lf;

		// UART transmit
		HAL_UART_Transmit_DMA(&huart1, hd->h_data, sizeof(hd->h_data));

		// Clear timer tick
		tim3_TimerTick = 0;
	}
}

/**
 *@brief: Function for handling hook data to remote side as to obtainning and sending batt., load values and led position feedbacks.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
void hook_monitoring_handle(hook_data_t *hd)
{

	if (TIM3_UpdateFlag_IsActive)
	{

		// ADC handle
		ADC_ProcessHandle(hd);

		// UART transmit process handle
		UART_TransmitHandle(hd);

		// Clear flag
		TIM3_UpdateFlag_IsActive = false;
	}
}

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*******************************************************END OF FILE*******************************************************************/
