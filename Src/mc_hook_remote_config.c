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
#include "stdio.h"

/*Definitions------------------------------------------------------------------------------------------------------------------------*/

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

static uint8_t cmd;

static volatile bool HOOK_OPEN_POS_CMD_FLAG = false;
static volatile bool HOOK_MID_POS_CMD_FLAG = false;
static volatile bool HOOK_CLOSE_POS_CMD_FLAG = false;

static uint32_t OpenCMDindex = 0;
static uint32_t MidCMDindex = 0;
static uint32_t CloseCMDindex = 0;
static volatile bool OpenCMDindex_Flag = false;
static volatile bool MidCMDindex_Flag = false;
static volatile bool CloseCMDindex_Flag = false;

static uint8_t lastHookStatus;
static int32_t hook_position = 0xff;

static hook_process_stat_t hookProcessStat = HOOK_PROCESS_NONE;

static volatile bool HOOK_HOMING_MOTOR_STOP_FLAG = false;

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
 *@brief: Function for reading remote data.
 *@param: rcmd: pointer for remote cmd structure to store data
 *@retval: None
 *Remote Data Protocol: |$R|OpenPosBtnCmd|MidPosBtnCmd|ClosePosBtnCmd|<CR>|<LF>|
 *d0:$, d1:R, d2:open pos cmd, d3:mid pos cmd, d4:<CR>, 5:<LF>
 */
void hook_remote_data_get(hook_remote_cmd_t *rcmd)
{
	if (!(rcmd->r_data[0] == HDR && rcmd->r_data[1] == R_HDR))
	{
		return;
	}

	if (rcmd->r_data[5] == CR && rcmd->r_data[6] == LF)
	{
		// Remote buttons cmd formating
		rcmd->open_pos = rcmd->r_data[2] - 48;	// Convert data to int
		rcmd->mid_pos = rcmd->r_data[3] - 48;	// Convert data to int
		rcmd->close_pos = rcmd->r_data[4] - 48; // Convert data to int
	}
}

/**
 *@brief: Function for clearing remote buffer.
 *@param: rcmd: pointer for remote cmd structure to store data
 *@retval: None
 */
void ClearRemoteBuffer(hook_remote_cmd_t *rcmd)
{
	for (int i = 0; i < REMOTE_BUFF_SIZE; i++)
	{
		rcmd->r_data[i] = 0x00;
	}

	rcmd->open_pos = 0xff;
	rcmd->mid_pos = 0xff;
	rcmd->close_pos = 0xff;
}

/**
 *@brief: Function for clearing hook data buffer.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None
 */
void ClearHookBuffer(hook_data_t *hd)
{
	// HOOK buffer clearing
	//	hd->h_data[0] = 0;
	//	hd->h_data[1] = 0;
	hd->h_data[2] = 0;
	hd->h_data[3] = 0;
	hd->h_data[4] = 0;
	hd->h_data[5] = 0;
	hd->h_data[6] = 0;
	hd->h_data[7] = 0;
	hd->h_data[8] = 0;
	hd->h_data[9] = 0;
	//	hd->h_data[10] = 0;
	//	hd->h_data[11] = 0;
	//	hd->open_pos_led = 0xff;
	//	hd->mid_pos_led = 0xff;
	//	hd->close_pos_led = 0xff;
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
 *@brief: Function for the checking and the tackling of hook position.
 *@param: motor_device: pointer to MC_Handle structure
 *        command: variable for getting remote command case
 *        hook_pos_count: variable for getting hook position value
 *@retval: None
 */
void HOOK_PositionHandle(MC_Handle_t *motor_device, hook_remote_cmd_t *rcmd, uint8_t command, int32_t hook_pos_count, STM_Handle_t *stmh)
{
	(void)stmh;
	// Perform the cmd cases
	switch (command)
	{
	case OPEN_POS_CMD:
	{
		if (HOOK_OPEN_POS_CMD_FLAG)
		{
			if (lastHookStatus == MID_POS || lastHookStatus == CLOSE_POS)
			{

				MotorSetDirection(motor_device, CCW);
				MotorStart(motor_device);

				// Handle the position
				if (hook_pos_count >= (int32_t)HOOK_MECH_FULL_POS_VAL)
				{
					MotorStop(motor_device);
					lastHookStatus = OPEN_POS;

					hook_position = HOOK_MECH_FULL_POS_VAL;

					// Position feedback flags
					HOOK_OPEN_POS_LED_FLAG = true;
					HOOK_MID_POS_LED_FLAG = false;
					HOOK_CLOSE_POS_LED_FLAG = false;

					// Clear hook remote command flag
					HOOK_OPEN_POS_CMD_FLAG = false;

					cmd = DEFAULT_CMD;

					OpenCMDindex = 0;
					MidCMDindex = 0;
					CloseCMDindex = 0;

					ClearRemoteBuffer(rcmd);
				}
			}
		}
	}
	break;

	case MID_POS_CMD:
	{
		if (HOOK_MID_POS_CMD_FLAG)
		{
			if (lastHookStatus == CLOSE_POS || lastHookStatus == OPEN_POS)
			{

				// Check last hook position for MID_POS and determine the direction to be set.
				if (lastHookStatus == CLOSE_POS)
				{
					MotorSetDirection(motor_device, CCW);
				}
				else if (lastHookStatus == OPEN_POS)
				{
					MotorSetDirection(motor_device, CW);
				}
				else
				{
					// Do nothing
				}

				MotorStart(motor_device);

				// Handle the position
				if ((lastHookStatus == CLOSE_POS && (hook_pos_count >= (int32_t)HOOK_MECH_HALF_POS_VAL)) ||
					(lastHookStatus == OPEN_POS && (hook_pos_count <= (int32_t)HOOK_MECH_HALF_POS_VAL)))
				{
					MotorStop(motor_device);
					lastHookStatus = MID_POS;

					// Position feedback flags
					HOOK_OPEN_POS_LED_FLAG = false;
					HOOK_MID_POS_LED_FLAG = true;
					HOOK_CLOSE_POS_LED_FLAG = false;

					// Clear hook remote command flag
					HOOK_MID_POS_CMD_FLAG = false;

					cmd = DEFAULT_CMD;

					OpenCMDindex = 0;
					MidCMDindex = 0;
					CloseCMDindex = 0;

					ClearRemoteBuffer(rcmd);
				}
			}
			else
				MotorStop(motor_device);
		}
	}
	break;

	case CLOSE_POS_CMD:
	{
		if (HOOK_CLOSE_POS_CMD_FLAG)
		{
			if (lastHookStatus == MID_POS || lastHookStatus == OPEN_POS)
			{

				hookProcessStat = HOOK_PROCESS_HOMING;
			}
		}
	}
	break;

	default:
		break;
	}
}

/**
 *@brief: Function for handling hook motor control and motor positioning by hall step changes against remote cmd.
 *@param: motor_device: pointer to MC_Handle structure
 *        rcmd: pointer for remote cmd structure to store data
 *        stmh: pointer for state machine motor control handle
 *@retval: None
 *Remote Data Protocol: |$R|OpenPosBtnCmd|MidPosBtnCmd|ClosePosBtnCmd|<CR>|<LF>|
 *d0:$, d1:R, d2:open pos cmd, d3:mid pos cmd, d4:close pos cmd, d5:<CR>, d6:<LF>
 */
void hook_motor_control_handle(MC_Handle_t *motor_device, hook_remote_cmd_t *rcmd, STM_Handle_t *stmh)
{
	static uint8_t lastHallStatus = 0xff;

	// Check remote cmd
	if ((rcmd->open_pos == 2 && rcmd->mid_pos == 2 && rcmd->close_pos == 0) && (hook_position != HOOK_MECH_HOME_POS_VAL))
	{
		hookProcessStat = HOOK_PROCESS_HOMING;
	}
	else if (rcmd->open_pos == 3 && rcmd->mid_pos == 0 && rcmd->close_pos == 3)
	{
		hookProcessStat = HOOK_PROCESS_SYSRESET;
	}
	else if ((rcmd->open_pos == 0 && rcmd->mid_pos == 4 && rcmd->close_pos == 4) && ((lastHookStatus == CLOSE_POS) && HOOK_CLOSE_POS_LED_FLAG))
	{
		hookProcessStat = HOOK_PROCESS_TESTING;
	}
	else
	{
		// Do nothing
	}

	if ((rcmd->open_pos == 1 || rcmd->mid_pos == 1 || rcmd->close_pos == 1) && HOOK_HOMING_FLAG)
	{
		HOOK_HOMING_MOTOR_STOP_FLAG = true;
	}

	// Perform processes
	switch (hookProcessStat)
	{

	case HOOK_PROCESS_NONE:
	{
		lastHookStatus = UNKNOWN_POS;
		HOOK_UNKNOWN_POS_FLAG = true;
		HOOK_HOMING_FLAG = false;
	}
	break;

	case HOOK_PROCESS_HOMING:
	{
		HOOK_UNKNOWN_POS_FLAG = false;
		HOOK_HOMING_FLAG = true;

		MotorSetSpeed(motor_device, HOOK_HOMING_SPEED_VAL);

		MotorSetDirection(motor_device, CW);
		MotorStart(motor_device);

		// Get motor fault state upon self motor stop due to fault condition
		uint32_t MotorFaultState = MC_Core_GetFaultState(motor_device);
		if ((motor_device->direction == CW) && (MotorFaultState == 128 || MotorFaultState == MC_OVERCURRENT))
		{

			// Clear faults
			// MC_Core_Init(motor_device);
			MC_Core_Stop(motor_device);
			STM_Init(stmh);
			STM_FaultAcknowledged(stmh);

			// Set home position to close pos
			lastHookStatus = CLOSE_POS;
			HOOK_OPEN_POS_LED_FLAG = false;
			HOOK_MID_POS_LED_FLAG = false;
			HOOK_CLOSE_POS_LED_FLAG = true;

			hook_position = HOOK_MECH_HOME_POS_VAL;

			// Clear hook remote command flag
			HOOK_OPEN_POS_CMD_FLAG = false;
			HOOK_MID_POS_CMD_FLAG = false;
			HOOK_CLOSE_POS_CMD_FLAG = false;

			cmd = DEFAULT_CMD;
			OpenCMDindex = 0;
			MidCMDindex = 0;
			CloseCMDindex = 0;

			ClearRemoteBuffer(rcmd);

			// Set the process to idle mode.
			hookProcessStat = HOOK_PROCESS_IDLE;

			HOOK_HOMING_FLAG = false;
		}

		// Stop the motor in homing mode if needed.
		if (HOOK_HOMING_MOTOR_STOP_FLAG)
		{
			MotorStop(motor_device);

			HOOK_HOMING_FLAG = false;
			HOOK_HOMING_MOTOR_STOP_FLAG = false;

			HOOK_OPEN_POS_CMD_FLAG = false;
			HOOK_MID_POS_CMD_FLAG = false;
			HOOK_CLOSE_POS_CMD_FLAG = false;

			HOOK_OPEN_POS_LED_FLAG = false;
			HOOK_MID_POS_LED_FLAG = false;
			HOOK_CLOSE_POS_LED_FLAG = false;

			OpenCMDindex = 0;
			MidCMDindex = 0;
			CloseCMDindex = 0;
			hookProcessStat = HOOK_PROCESS_NONE;
			ClearRemoteBuffer(rcmd);
		}
	}
	break;

	case HOOK_PROCESS_SYSRESET:
	{
		HAL_NVIC_SystemReset(); // Reset system if needed.
	}
	break;

	case HOOK_PROCESS_TESTING:
	{
		HOOK_TESTING_FLAG = true;
		hookProcessStat = HOOK_PROCESS_IDLE;
	}
	break;

	case HOOK_PROCESS_IDLE:
	{
		// Check remote cmd in idle mode
		if (rcmd->open_pos == 1 && rcmd->mid_pos == 0 && rcmd->close_pos == 0)
		{
			OpenCMDindex_Flag = true;
		}
		else if (rcmd->mid_pos == 1 && rcmd->open_pos == 0 && rcmd->close_pos == 0)
		{
			MidCMDindex_Flag = true;
		}
		else if (rcmd->close_pos == 1 && rcmd->open_pos == 0 && rcmd->mid_pos == 0)
		{
			CloseCMDindex_Flag = true;
		}
		else
		{
			// Do nothing
		}

		// Check command index values Determine remote cmd status
		if (OpenCMDindex_Flag && rcmd->open_pos == 0)
		{
			OpenCMDindex++;
			OpenCMDindex_Flag = false;
		}
		if (MidCMDindex_Flag && rcmd->mid_pos == 0)
		{
			MidCMDindex++;
			MidCMDindex_Flag = false;
		}
		if (CloseCMDindex_Flag && rcmd->close_pos == 0)
		{
			CloseCMDindex++;
			CloseCMDindex_Flag = false;
		}

		// Determine remote cmd status
		if (OpenCMDindex > 0)
		{
			cmd = OPEN_POS_CMD;
			HOOK_OPEN_POS_CMD_FLAG = true;
		}
		if (MidCMDindex > 0)
		{
			cmd = MID_POS_CMD;
			HOOK_MID_POS_CMD_FLAG = true;
		}
		if (CloseCMDindex > 0)
		{
			cmd = CLOSE_POS_CMD;
			HOOK_CLOSE_POS_CMD_FLAG = true;
		}

		// Watch the commands and stop the motor to avoid and eliminate unexpected multiple commands while previous command taking action.
		// Stop the motor in idle mode if needed.
		if (HOOK_OPEN_POS_CMD_FLAG)
		{

			if ((OpenCMDindex > 1) || (MidCMDindex > 0) || (CloseCMDindex > 0))
			{
				MotorStop(motor_device);

				HOOK_OPEN_POS_CMD_FLAG = false;
				HOOK_MID_POS_CMD_FLAG = false;
				HOOK_CLOSE_POS_CMD_FLAG = false;

				// HOOK_OPEN_POS_LED_FLAG = false;
				HOOK_MID_POS_LED_FLAG = false;
				HOOK_CLOSE_POS_LED_FLAG = false;

				HOOK_TESTING_FLAG = false;

				OpenCMDindex = 0;
				MidCMDindex = 0;
				CloseCMDindex = 0;

				hookProcessStat = HOOK_PROCESS_NONE;

				ClearRemoteBuffer(rcmd);
			}
		}

		if (HOOK_MID_POS_CMD_FLAG)
		{

			if ((OpenCMDindex > 0) || (MidCMDindex > 1) || (CloseCMDindex > 0))
			{
				MotorStop(motor_device);

				HOOK_OPEN_POS_CMD_FLAG = false;
				HOOK_MID_POS_CMD_FLAG = false;
				HOOK_CLOSE_POS_CMD_FLAG = false;

				HOOK_OPEN_POS_LED_FLAG = false;
				// HOOK_MID_POS_LED_FLAG = false;
				HOOK_CLOSE_POS_LED_FLAG = false;

				HOOK_TESTING_FLAG = false;

				OpenCMDindex = 0;
				MidCMDindex = 0;
				CloseCMDindex = 0;

				hookProcessStat = HOOK_PROCESS_NONE;

				ClearRemoteBuffer(rcmd);
			}
		}

		if (HOOK_CLOSE_POS_CMD_FLAG)
		{

			if ((OpenCMDindex > 0) || (MidCMDindex > 0) || (CloseCMDindex > 1))
			{
				MotorStop(motor_device);

				HOOK_OPEN_POS_CMD_FLAG = false;
				HOOK_MID_POS_CMD_FLAG = false;
				HOOK_CLOSE_POS_CMD_FLAG = false;

				HOOK_OPEN_POS_LED_FLAG = false;
				HOOK_MID_POS_LED_FLAG = false;
				// HOOK_CLOSE_POS_LED_FLAG = false;

				HOOK_TESTING_FLAG = false;

				OpenCMDindex = 0;
				MidCMDindex = 0;
				CloseCMDindex = 0;

				hookProcessStat = HOOK_PROCESS_NONE;

				ClearRemoteBuffer(rcmd);
			}
		}

		// Handle hook position
		HOOK_PositionHandle(motor_device, rcmd, cmd, hook_position, stmh);

		// Get the position by reading hall sensors' step-changes
		if (motor_device->status == MC_RUN)
		{

			MC_Core_GetHallStatus(motor_device);

			uint8_t currentHallStatus = motor_device->hall.status;

			if (currentHallStatus != lastHallStatus)
			{
				if (motor_device->direction)
				{
					hook_position++;
				}
				else
				{
					hook_position--;
				}

				lastHallStatus = currentHallStatus;
			}
		}
	}
	break;

	default:
		break;
	}
}

/**
 *@brief: Function for ADC process handling as to the obtainning and the calculation of Vref Int, VBus and Current Sense values.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
static uint8_t OvercurrentCounter = 0;
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

		if (CurrentSense > 5 && ++OvercurrentCounter > 3)
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
 *@brief: Function for handling hook position feedback to remote side indicator leds.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
void Position_FeedbackHandle(hook_data_t *hd, MC_Handle_t *motor_device)
{
	// Open pos. LED feedback formating
	if (HOOK_OPEN_POS_LED_FLAG == true && HOOK_MID_POS_LED_FLAG == false && HOOK_CLOSE_POS_LED_FLAG == false)
	{
		hd->open_pos_led = 1;
		hd->mid_pos_led = 0;
		hd->close_pos_led = 0;
	}
	// Mid pos. LED feedback formating
	else if (HOOK_OPEN_POS_LED_FLAG == false && HOOK_MID_POS_LED_FLAG == true && HOOK_CLOSE_POS_LED_FLAG == false)
	{
		hd->open_pos_led = 0;
		hd->mid_pos_led = 1;
		hd->close_pos_led = 0;
	}
	// Close pos. LED feedback formating
	else if (HOOK_OPEN_POS_LED_FLAG == false && HOOK_MID_POS_LED_FLAG == false && HOOK_CLOSE_POS_LED_FLAG == true)
	{
		hd->open_pos_led = 0;
		hd->mid_pos_led = 0;
		hd->close_pos_led = 1;
	}
	else
	{
		// Do nothing
		hd->open_pos_led = 0xff;
		hd->mid_pos_led = 0xff;
		hd->close_pos_led = 0xff;
	}

	// Toggle the remote leds while the commands taking action
	if (HOOK_OPEN_POS_CMD_FLAG)
	{
		hd->open_pos_led = 2; // 1:led on, 2:led toggle
		if (motor_device->status == MC_STOP)
			hd->open_pos_led = 1; // 1:led on, 2:led toggle
	}
	if (HOOK_MID_POS_CMD_FLAG)
	{
		hd->mid_pos_led = 2; // 1:led on, 2:led toggle
		if (motor_device->status == MC_STOP)
			hd->mid_pos_led = 1; // 1:led on, 2:led toggle
	}
	if (HOOK_CLOSE_POS_CMD_FLAG)
	{
		hd->close_pos_led = 2; // 1:led on, 2:led toggle
		if (motor_device->status == MC_STOP)
			hd->close_pos_led = 1; // 1:led on, 2:led toggle
	}

	// Define fault led condition against unknown position, homing and testing process.
	if (HOOK_UNKNOWN_POS_FLAG)
	{
		hd->fault_led = 1;
	}
	else if (HOOK_HOMING_FLAG)
	{
		hd->fault_led = 2;
	}
	else if (HOOK_TESTING_FLAG)
	{
		hd->fault_led = 3;
	}
	else
	{
		hd->fault_led = 0;
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
void hook_monitoring_handle(hook_data_t *hd, MC_Handle_t *motor_device)
{

	if (TIM3_UpdateFlag_IsActive)
	{

		// ADC handle
		ADC_ProcessHandle(hd);

		// Positon feedback handle
		Position_FeedbackHandle(hd, motor_device);

		// UART transmit process handle
		UART_TransmitHandle(hd);

		// Clear flag
		TIM3_UpdateFlag_IsActive = false;
	}
}

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*******************************************************END OF FILE*******************************************************************/
