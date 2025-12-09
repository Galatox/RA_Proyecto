/*

RAdef.h*
Created on: Nov 11, 2025
Author: Erwan Bernard and Joaquin del Valle Zamudio*/

#ifndef INC_RADEFH
#define INC_RADEFH


#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

// STATUS
#define RA_STATUS0 0b00
#define RA_STATUS1 0b01
#define RA_STATUS2 0b10
#define RA_STATUS3 0b11

// STATEMENTS
#define RA_DEFAULT -1
#define RA_FALSE 0
#define RA_TRUE 1
#define RA_SPEED 2
#define RA_POSITION 3
#define RA_MAX_SPEED_CRR 511
#define RA_STEP 4
#define RA_STOP 5
// HARDWARE PINS

// INPUTS
#define RA_D0 GPIO_PIN_15
#define RA_D1 GPIO_PIN_14

// OUTPUTS
#define RA_AIN1 GPIO_PIN_8
#define RA_AIN2 GPIO_PIN_7
#define RA_STDBY GPIO_PIN_9

// LOGIC FUNCTIONS
void RA_stdbyToggle(void);
void RA_DriverTTB(void);

// HARDWARE FUNCTIONS
void RA_Motor_Neutral(void);
void RA_Motor_Reverse(void);
void RA_Motor_Forward(void);
void RA_Motor_Break(void);

// HARDWARE CONTROL
float RA_SpeedController(int16_t speedStorage[3]);
float RA_PositionController(int16_t posStorage[2]);
void RA_controlModeToggle(int8_t controlMode);

// UART
int RA_inputModeControl(char buffer[32]);
void RA_inputValues(char buffer[32]);
void RA_UART_Interface(void);
void RA_Print_Speed(void);
void RA_Print_Position(void);
void RA_Print_Step(void);
void RA_UART(void);

#endif /* INC_RADEFH */
