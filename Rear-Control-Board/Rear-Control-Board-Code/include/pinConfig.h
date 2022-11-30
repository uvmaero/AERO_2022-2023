/**
 * @file pinConfig.h
 * @author dominic gasperini 
 * @brief this file holds the pin layout for the board I/O
 * @version 0.1
 * @date 2022-10-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// Sensors
#define WHEEL_SPEED_BR_SENSOR               1
#define WHEEL_SPEED_BL_SENSOR               2

#define WHEEL_HEIGHT_BR_SENSOR              5
#define WHEEL_HEIGHT_BL_SENSOR              6

#define WATER_TMP_IN_PIN                    7
#define WATER_TMP_OUT_PIN                   8


// Inputs
#define CAN_MESSAGE_INTERRUPT_PIN           15             
#define VICORE_PIN                          25


// Outputs
#define PUMP_ENABLE_PIN                     23
#define BRAKE_LIGHT_PIN                     19
#define FAN_ENABLE_PIN                      22
#define IMD_FAULT_PIN                       20
#define BMS_FAULT_PIN                       21