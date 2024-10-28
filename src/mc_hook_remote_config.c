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
 * TIM3 update event is also used for updating uart tx data for every required timestamp (default:500ms) by using
 *TIM3_TimerTick variable.
 */

/*Includes---------------------------------------------------------------------------------------------------------------------------*/
#include "mc_hook_remote_config.h"
#include "encoding_checksum.h"
#include "main.h"
#include "stdio.h"
#include <memory.h>

/*Definitions------------------------------------------------------------------------------------------------------------------------*/
typedef enum Command_e_
{
    SPIN_COMMAND_NONE,
    SPIN_COMMAND_MOVE,
    // Reserved from 1 to 9
    SPIN_COMMAND_STOP = 10,
    SPIN_COMMAND_EACK,
    SPIN_COMMAND_REBOOT,
    SPIN_COMMAND_SET_POSITION,
    SPIN_COMMAND_SET_PARAMETER,
    SPIN_COMMAND_READ_PARAMETER,
    SPIN_COMMAND_READY_FOR_LOADING,
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
    PARAMETER_CURRENT_PIN_CONFIG,
    PARAMETER_IGNORE_SENSOR,

} Parameters_e;

typedef struct Measurements_t_
{
    float voltage;
    float current;
} Measurements_t;

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*Variables--------------------------------------------------------------------------------------------------------------------------*/
extern UART_HandleTypeDef huart1;

uint32_t tim17_TimerTick = 0;
volatile bool TIM17_UpdateFlag_IsActive = false;

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;
uint16_t adc1_RawData[ADC_CONV_LENGTH];
#define CURRENT_FILTER_LENGTH 150
static uint16_t currentFilter[CURRENT_FILTER_LENGTH];
static uint32_t currentFilteridx = 0;

volatile bool ADC1_ConvCpltFlag_IsActive = false;

static uint8_t ignoreSensor = 0;
static uint8_t endOfStrokeSensor = 0;
static int32_t hookPosition = UINT16_MAX;
static uint16_t hookTarget = 0;
static Errors_e errorNo = ERROR_NONE;
static uint8_t initialized = 0;

extern uint16_t pidKp;
extern uint16_t pidKi;
extern uint16_t pidKd;
extern uint16_t pidScaling;
extern uint16_t pidOutMin;
extern uint16_t pidOutMax;
static uint16_t currentLimit = 10000;
static uint16_t currentLimitMaxCount = 3;
static uint16_t OvercurrentCounter = 0;
static Measurements_t measurement;
static uint32_t readyForLoading = 0;
static uint32_t loadingData = 0;
static bool currentLimitType = true;
static Parameters_e readParameter;
static int32_t valueParameter;

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*Private
 * Functions------------------------------------------------------------------------------------------------------------------*/
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
    MC_Core_Stop(motor_device);
    MC_Core_Reset(motor_device);
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

void MotorPositionTargetTest(MC_Handle_t *motor_device)
{
    if (hookPosition == hookTarget)
    {
        MotorStop(motor_device);
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

void hook_setError(Errors_e number)
{
    errorNo = number;
}

void hook_command_run(RemoteCommand_t *cmd, MC_Handle_t *motor_device)
{
    switch (cmd->operation)
    {
    case SPIN_COMMAND_MOVE:
        MotorSetDirection(motor_device, MotorGetDirection(cmd->Parameter2));
        MotorSetSpeed(motor_device, (cmd->Parameter2 > 0) ? cmd->Parameter2 : -cmd->Parameter2);
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
    case SPIN_COMMAND_SET_POSITION:
        hookPosition = cmd->Parameter1;

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
            if (currentLimitType != (uint16_t)cmd->Parameter2)
            {
                currentLimitType = (uint16_t)cmd->Parameter2;
                MX_TIM1_Init(currentLimitType);
            }

            break;
        case PARAMETER_CURRENT_LIMIT_ADC_FILTER_VALUE:
            currentLimitMaxCount = (uint16_t)cmd->Parameter2;

            break;
        case PARAMETER_CURRENT_PIN_CONFIG:
            if (cmd->Parameter2 & 0x01)
            {
                HAL_GPIO_WritePin(GPIOF, OCTH_STBY1_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOF, OCTH_STBY1_Pin, GPIO_PIN_RESET);
            }

            if (cmd->Parameter2 & 0x02)
            {
                HAL_GPIO_WritePin(GPIOF, OCTHSTBY2_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOF, OCTHSTBY2_Pin, GPIO_PIN_RESET);
            }

            break;
        case PARAMETER_IGNORE_SENSOR:
            ignoreSensor = cmd->Parameter2;

            break;
        case PARAMETER_NONE:
        default:
            errorNo = ERROR_INVALID_PARAMETER;

            break;
        }

        break;
    case SPIN_COMMAND_READ_PARAMETER:
        switch (cmd->Parameter1)
        {
        case PARAMETER_KP:
            readParameter = PARAMETER_KP;
            valueParameter = pidKp;

            break;
        case PARAMETER_KI:
            readParameter = PARAMETER_KI;
            valueParameter = pidKi;

            break;
        case PARAMETER_KD:
            readParameter = PARAMETER_KD;
            valueParameter = pidKd;

            break;
        case PARAMETER_PID_SCALING_SHIFT:
            readParameter = PARAMETER_PID_SCALING_SHIFT;
            valueParameter = pidScaling;

            break;
        case PARAMETER_PID_OUTPUT_MIN:
            readParameter = PARAMETER_PID_OUTPUT_MIN;
            valueParameter = pidOutMin;

            break;
        case PARAMETER_PID_OUTPUT_MAX:
            readParameter = PARAMETER_PID_OUTPUT_MAX;
            valueParameter = pidOutMax;

            break;
        case PARAMETER_CURRENT_LIMIT_VALUE:
            readParameter = PARAMETER_CURRENT_LIMIT_VALUE;
            valueParameter = currentLimit;

            break;
        case PARAMETER_CURRENT_LIMIT_TYPE:
            readParameter = PARAMETER_CURRENT_LIMIT_TYPE;
            valueParameter = currentLimitType;

            break;
        case PARAMETER_CURRENT_LIMIT_ADC_FILTER_VALUE:
            readParameter = PARAMETER_CURRENT_LIMIT_ADC_FILTER_VALUE;
            valueParameter = currentLimitMaxCount;

            break;
        case PARAMETER_CURRENT_PIN_CONFIG:
            readParameter = PARAMETER_CURRENT_PIN_CONFIG;
            if (HAL_GPIO_ReadPin(GPIOF, OCTH_STBY1_Pin))
            {
                valueParameter = 0x01;
            }
            else
            {
                valueParameter = 0x00;
            }

            if (HAL_GPIO_ReadPin(GPIOF, OCTHSTBY2_Pin))
            {
                valueParameter |= 0x02;
            }

            break;
        case PARAMETER_IGNORE_SENSOR:
            readParameter = PARAMETER_IGNORE_SENSOR;
            valueParameter = ignoreSensor;

            break;
        case PARAMETER_NONE:
        default:
            errorNo = ERROR_INVALID_PARAMETER;

            break;
        }

        break;
    case SPIN_COMMAND_READY_FOR_LOADING:
        readyForLoading = (uint16_t)cmd->Parameter1;
        loadingData = (uint16_t)cmd->Parameter2 << 16;
        loadingData |= (uint16_t)cmd->Parameter3;

        break;
    default:

        break;
    }
}

/**
 *@brief: Function for ADC process handling as to the obtainning and the calculation of Vref Int, VBus and Current Sense
 *values.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
Measurements_t getMeasurements(void)
{
    float Vref = 0;
    float CurrentSense = 0;

    if (ADC1_ConvCpltFlag_IsActive)
    {

        // VrefInt formatting
        uint32_t temp = adc1_RawData[2];
        for (uint32_t i = (2 + 3); i < ADC_CONV_LENGTH; i += 3)
        {
            temp += adc1_RawData[i];
            temp /= 2;
        }
        uint16_t VrefInt_adcVal = temp; // Read adc raw data, ADC_CHANNEL_VREFINT.

        Vref = (float)((3.3 * VREFINT_CAL) / VrefInt_adcVal); // Internal ref. voltage conversion by reading vrefint
                                                              // calibration values from registers.

        // Current sense formatting
        temp = adc1_RawData[0];
        for (uint32_t i = (0 + 3); i < ADC_CONV_LENGTH; i += 3)
        {
            temp += adc1_RawData[i];
            temp /= 2;
        }
        currentFilter[currentFilteridx] = temp;
        currentFilteridx = ((currentFilteridx + 1) % CURRENT_FILTER_LENGTH);

        for (uint32_t i = 0; i < CURRENT_FILTER_LENGTH; ++i)
        {
            temp += currentFilter[i];
            temp /= 2;
        }
        uint16_t CurrentSense_adcVal = temp; // Read adc raw data, PA4.

        CurrentSense = (float)((Vref * CurrentSense_adcVal) / 4095); // Current conversion, Vadc by MCU.
        CurrentSense *= (CURR_SENSE_SCALE_FACTOR *
                         1000); // Current calc., Vshunt=Ishunt*Rshunt, Vshunt*GAIN=Vadc, Is=Vadc/(Rs*GAIN).

        if (!currentLimitType && CurrentSense > currentLimit && ++OvercurrentCounter > currentLimitMaxCount)
        {
            Motor_Device1.status = MC_OVERCURRENT;
            hook_setError(ERROR_OVERLOAD);
            MC_Core_Error(&Motor_Device1);
            OvercurrentCounter = 0;

            memset(currentFilter, 0x00, sizeof(currentFilter));
        }
        else if (CurrentSense < 1)
        {
            OvercurrentCounter = 0;
        }

        measurement.current = CurrentSense;

        // Clear flag
        ADC1_ConvCpltFlag_IsActive = false;

        // Reset&Restart ADC, clear garbage old data in DMA buff..
        HAL_ADC_Stop_DMA(&hadc);
        HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc1_RawData, ADC_CONV_LENGTH);
    }

    return measurement;
}

/**
 *@brief: Function for UART tx transmit handling as to sending hook data to remote side.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
void updateReply(Measurements_t measurements)
{
    (void)(measurements);

    if (tim17_TimerTick >= HOOK_MONITOR_REFRESH_TIMESTAMP)
    {
        enum Position
        {
            POSITION_UNKNOWN,
            POSITION_CLOSED,
            POSITION_MID,
            POSITION_OPEN,
            POSITION_MOVING,
        } currentPosition = POSITION_MOVING;

        if (hookPosition > HOOK_MECH_OPEN_POS_VAL)
        {
            currentPosition = POSITION_UNKNOWN;
        }
        else if (hookPosition <= HOOK_MECH_CLOSE_POS_VAL)
        {
            currentPosition = POSITION_CLOSED;
        }
        else if (hookPosition == HOOK_MECH_MID_POS_VAL)
        {
            currentPosition = POSITION_MID;
        }
        else if (hookPosition == HOOK_MECH_OPEN_POS_VAL)
        {
            currentPosition = POSITION_OPEN;
        }

        switch (currentPosition)
        {
        case POSITION_UNKNOWN:
            HAL_GPIO_WritePin(CLOSE_OUT_GPIO_Port, CLOSE_OUT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MID_OUT_GPIO_Port, MID_OUT_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_SET);

            break;
        case POSITION_CLOSED:
            HAL_GPIO_WritePin(MID_OUT_GPIO_Port, MID_OUT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(CLOSE_OUT_GPIO_Port, CLOSE_OUT_Pin, GPIO_PIN_SET);

            break;
        case POSITION_MID:
            HAL_GPIO_WritePin(CLOSE_OUT_GPIO_Port, CLOSE_OUT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(MID_OUT_GPIO_Port, MID_OUT_Pin, GPIO_PIN_SET);

            break;
        case POSITION_OPEN:
            HAL_GPIO_WritePin(CLOSE_OUT_GPIO_Port, CLOSE_OUT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MID_OUT_GPIO_Port, MID_OUT_Pin, GPIO_PIN_RESET);

            HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_SET);

            break;
        case POSITION_MOVING:
            HAL_GPIO_WritePin(CLOSE_OUT_GPIO_Port, CLOSE_OUT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MID_OUT_GPIO_Port, MID_OUT_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(OPEN_OUT_GPIO_Port, OPEN_OUT_Pin, GPIO_PIN_RESET);

            break;
        }

        if (errorNo != ERROR_NONE)
        {
            HAL_GPIO_WritePin(FAULT_OUT_GPIO_Port, FAULT_OUT_Pin, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(FAULT_OUT_GPIO_Port, FAULT_OUT_Pin, GPIO_PIN_RESET);
        }

        // Clear timer tick
        tim17_TimerTick = 0;
    }
}

/**
 *@brief: Function for handling hook data to remote side as to obtainning and sending batt., load values and led
 *position feedbacks.
 *@param: hd: pointer for hook data structure to store data
 *@retval: None.
 */
void hook_monitoring_handle(MC_Handle_t *motor_device)
{
    if (HAL_GPIO_ReadPin(END_STROKE_SENSOR_Port, END_STROKE_SENSOR_Pin) == GPIO_PIN_RESET)
    {
        if (!endOfStrokeSensor)
        {
            MotorStop(motor_device);

            if (initialized == 0)
            {
                hookPosition = 0;
                initialized = 1;
            }
            else if (initialized && hookPosition > 1000)
            {
                hook_setError(ERROR_PROTECTION_ACTIVATED);
            }
        }
        endOfStrokeSensor = 1;
    }
    else
    {
        endOfStrokeSensor = 0;
    }

    if (TIM17_UpdateFlag_IsActive)
    {
        // ADC handle
        Measurements_t values = getMeasurements();

        // UART transmit process handle
        updateReply(values);

        // Clear flag
        TIM17_UpdateFlag_IsActive = false;
    }
}

static RemoteCommand_t cmdBuffer[8];
static int8_t idxCount = 0;
RemoteCommand_t *hook_get_commands(MC_Handle_t *motor_device)
{
    RemoteCommand_t *result = NULL;
    MC_Status_t status = MC_Core_GetStatus(motor_device);
    if (idxCount == 0 && status == MC_STOP)
    {
        if (initialized == 0) // uninitialized
        {
            if (HAL_GPIO_ReadPin(CLOSE_CMD_GPIO_Port, CLOSE_CMD_Pin) == 0 ||
                HAL_GPIO_ReadPin(MID_CMD_GPIO_Port, MID_CMD_Pin) == 0 ||
                HAL_GPIO_ReadPin(OPEN_CMD_GPIO_Port, OPEN_CMD_Pin) == 0)
            {
                idxCount = 3; // Number of commands
                uint8_t idx = idxCount;

                cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_TYPE;
                cmdBuffer[idx].Parameter2 = 1;
                cmdBuffer[idx].Parameter3 = 0;
                --idx;

                uint8_t op = SPIN_COMMAND_MOVE;
                cmdBuffer[idx].operation = op;
                cmdBuffer[idx].Parameter1 = HOOK_MECH_HOME_POS_VAL;
                cmdBuffer[idx].Parameter2 = MOTOR_CLOSE_DIRECTION * HOOK_HOMING_SPEED_VAL;
                cmdBuffer[idx].Parameter3 = 0;
                --idx;

                cmdBuffer[idx].operation = SPIN_COMMAND_EACK;
                cmdBuffer[idx].Parameter1 = 0;
                cmdBuffer[idx].Parameter2 = 0;
                cmdBuffer[idx].Parameter3 = 0;
            }
        }
        else // Initialized
        {
            if (HAL_GPIO_ReadPin(CLOSE_CMD_GPIO_Port, CLOSE_CMD_Pin) == 0 && hookPosition > HOOK_MECH_CLOSE_POS_VAL)
            {
                idxCount = 2; // Number of commands
                uint8_t idx = idxCount;

                cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_TYPE;
                cmdBuffer[idx].Parameter2 = 1;
                cmdBuffer[idx].Parameter3 = 0;
                --idx;

                uint8_t op = SPIN_COMMAND_MOVE;
                cmdBuffer[idx].operation = op;
                cmdBuffer[idx].Parameter1 = HOOK_MECH_CLOSE_POS_VAL;
                cmdBuffer[idx].Parameter2 = MOTOR_CLOSE_DIRECTION * HOOK_CLOSING_SPEED_VAL;
                cmdBuffer[idx].Parameter3 = 0;
            }
            else if (HAL_GPIO_ReadPin(MID_CMD_GPIO_Port, MID_CMD_Pin) == 0 && hookPosition != HOOK_MECH_MID_POS_VAL)
            {
                if (hookPosition > HOOK_MECH_MID_POS_VAL)
                {
                    idxCount = 2; // Number of commands
                    uint8_t idx = idxCount;

                    cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                    cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_TYPE;
                    cmdBuffer[idx].Parameter2 = 1;
                    cmdBuffer[idx].Parameter3 = 0;
                    --idx;

                    uint8_t op = SPIN_COMMAND_MOVE;
                    cmdBuffer[idx].operation = op;
                    cmdBuffer[idx].Parameter1 = HOOK_MECH_MID_POS_VAL;
                    cmdBuffer[idx].Parameter2 = MOTOR_CLOSE_DIRECTION * HOOK_CLOSING_SPEED_VAL;
                    cmdBuffer[idx].Parameter3 = 0;
                }
                else
                {
                    idxCount = 3; // Number of commands
                    uint8_t idx = idxCount;

                    cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                    cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_VALUE;
                    cmdBuffer[idx].Parameter2 = HOOK_CURRENT_LIMIT_OPEN;
                    cmdBuffer[idx].Parameter3 = 0;
                    --idx;

                    cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                    cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_TYPE;
                    cmdBuffer[idx].Parameter2 = 0;
                    cmdBuffer[idx].Parameter3 = 0;
                    --idx;

                    uint8_t op = SPIN_COMMAND_MOVE;
                    cmdBuffer[idx].operation = op;
                    cmdBuffer[idx].Parameter1 = HOOK_MECH_MID_POS_VAL;
                    cmdBuffer[idx].Parameter2 = MOTOR_OPEN_DIRECTION * HOOK_OPENING_SPEED_VAL;
                    cmdBuffer[idx].Parameter3 = 0;
                }
            }
            else if (HAL_GPIO_ReadPin(OPEN_CMD_GPIO_Port, OPEN_CMD_Pin) == 0 && hookPosition != HOOK_MECH_OPEN_POS_VAL)
            {

                idxCount = 3; // Number of commands
                uint8_t idx = idxCount;

                cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_VALUE;
                cmdBuffer[idx].Parameter2 = HOOK_CURRENT_LIMIT_OPEN;
                cmdBuffer[idx].Parameter3 = 0;
                --idx;

                cmdBuffer[idx].operation = SPIN_COMMAND_SET_PARAMETER;
                cmdBuffer[idx].Parameter1 = PARAMETER_CURRENT_LIMIT_TYPE;
                cmdBuffer[idx].Parameter2 = 0;
                cmdBuffer[idx].Parameter3 = 0;
                --idx;

                uint8_t op = SPIN_COMMAND_MOVE;
                cmdBuffer[idx].operation = op;
                cmdBuffer[idx].Parameter1 = HOOK_MECH_OPEN_POS_VAL;
                cmdBuffer[idx].Parameter2 = MOTOR_OPEN_DIRECTION * HOOK_OPENING_SPEED_VAL;
                cmdBuffer[idx].Parameter3 = 0;
            }
        }
    }

    if (errorNo != ERROR_NONE)
    {
        idxCount = 0;
    }
    else if (idxCount && status == MC_STOP)
    {
        result = &cmdBuffer[idxCount];
        --idxCount;
    }

    return result;
}

/*-----------------------------------------------------------------------------------------------------------------------------------*/

/*******************************************************END OF
 * FILE*******************************************************************/
