/*
 * RAdef.c
 *
 *  Created on: Nov 19, 2025
 *      Author: Erwan Bernard and Joaquin del Valle Zamudio
 */

#include "RAdef.h"

// LOGIC FUNCTIONS

void RA_stdbyToggle(){
	//@brief: Does a toggle of the PC7 which correspond to the STDBY pin in the RAdef.h.

	HAL_GPIO_TogglePin(GPIOC,RA_STDBY);
}

void RA_DriverTTB(void){
	//@brief: Driver truth table, manual way of controlling the motor, doesn't include speed limits.
	//		  Default mode will put the speed of the motor to MAX_SPEED_CCR.
	//@param pinsValues: global variable (for debugging purpose) that will store the read value.

	extern int pinsValues;
	pinsValues &= ~(0b11<<0);
	pinsValues |= (HAL_GPIO_ReadPin(GPIOB,RA_D0) << 0);
	pinsValues |= (HAL_GPIO_ReadPin(GPIOB,RA_D1) << 1);

	switch (pinsValues){

	case RA_STATUS0: // OFF
		HAL_GPIO_WritePin(GPIOC,RA_AIN1, 0);
		HAL_GPIO_WritePin(GPIOC,RA_AIN2, 0);
		break;

	case RA_STATUS1: // Reverse
		HAL_GPIO_WritePin(GPIOC,RA_AIN1, 0);
		HAL_GPIO_WritePin(GPIOC,RA_AIN2, 1);
		break;

	case RA_STATUS2: // Forward
		HAL_GPIO_WritePin(GPIOC,RA_AIN1, 1);
		HAL_GPIO_WritePin(GPIOC,RA_AIN2, 0);
		break;

	case RA_STATUS3: //Break
		HAL_GPIO_WritePin(GPIOC,RA_AIN1, 1);
		HAL_GPIO_WritePin(GPIOC,RA_AIN2, 1);
		break;
	}
}

//HARDWARE FUNCTIONS
void RA_Motor_Neutral(void){
	//@brief: puts the motor in default mode

	HAL_GPIO_WritePin(GPIOC,RA_AIN1, 0);
	HAL_GPIO_WritePin(GPIOC,RA_AIN2, 0);
}

void RA_Motor_Reverse(void){
	//@brief: turns the motor on in reverse mode.

	HAL_GPIO_WritePin(GPIOC,RA_AIN1, 1);
	HAL_GPIO_WritePin(GPIOC,RA_AIN2, 0);
}
void RA_Motor_Forward(void){
	//@brief: turns the motor on in forward mode.

	HAL_GPIO_WritePin(GPIOC,RA_AIN1, 0);
	HAL_GPIO_WritePin(GPIOC,RA_AIN2, 1);
}

void RA_Motor_Break(void){
	//@brief: breaking mode of the motor.
	HAL_GPIO_WritePin(GPIOC,RA_AIN1, 1);
	HAL_GPIO_WritePin(GPIOC,RA_AIN2, 1);
}

float RA_SpeedController(float speedStorage[3]){
	//@brief: the fcn will regulate the output of the motor in order to obtain
	//		  the willing value of speed that enters in the system.
	//@param speedStorage[3]:
	//speedStorage[0]: actual speed input.
	//speedStorage[1]: previous speed input.
	//speedStorage[2]: previous speed output.
	//@param Y1: output of the controller.


	float zero = 0.8474;
	float k = 1.9624;

	float Y1 = k*(speedStorage[RA_ACTUAL_SPEED]-zero*speedStorage[RA_PREVIOUS_SPEED]) + speedStorage[RA_PREVIOUS_SPEED_OUTPUT];

	if(Y1 > 0){
		Y1 = ((Y1 >= 511)? 511 : Y1);
		RA_Motor_Forward();

	}
	else if (Y1 < 0){

		Y1 = ((Y1 <= -511)? -511 : Y1);
		RA_Motor_Reverse();
	}

	if(Y1 == 0){
		RA_Motor_Break();
		RA_Motor_Neutral();
	}
	return Y1;
}

float RA_PositionController(float posStorage[2]){
	//@brief: the fcn will regulate the output of the motor in order to obtain the willing value of position
	//        that enters in the system.
	//@param posStorage[2]:
	//posStorage[0]: actual pos. input including the error because of the feedback loop.
	//posStorage[1]: previous pos. input.
	//@param Y1: output of the controller in RPMs.
	extern int controlMode;

	float zero = ((controlMode == RA_POSITION_REV) ? 0 : 0.9616);
	float k = ((controlMode == RA_POSITION_REV)? 3.1497 * 100  : 52.48);
	//k = (controlMode == RA_POSITION_REV? k : (k / 360));

	float Y1 = k*(posStorage[RA_ACTUAL_POS]-zero*posStorage[RA_PREVIOUS_POS]);

	//Y1 = Y1 >= 40? 40:Y1;
	return Y1;
}

void controlModeToggle(int8_t controlMode){
	//@brief: function that will enable the selection between speed mode control or position mode
	//@param controlMode: must a be a global variable, will be use to make the toggles for the CCR.

	if(controlMode == RA_DEFAULT){
		controlMode = RA_SPEED;
		return;
	}

	if(controlMode == RA_SPEED){
		controlMode = RA_POSITION_REV;
		return;
	}

	if(controlMode == RA_POSITION_REV){
		controlMode = RA_POSITION_DEG;
		return;
	}
	if(controlMode == RA_POSITION_DEG){
			controlMode = RA_SPEED;
			return;
		}
}

int RA_inputModeControl(char buffer[32]){
	//@brief: fcn will choose the mode that the user wants through the UART.
	//@param buffer[32]: buffer that you want to check. If needed change the size for larger text inputs

	extern int controlMode;
	extern UART_HandleTypeDef huart2;
	extern int16_t Ref;
	extern float pos; // Actual position of the motor
	extern int32_t count;
	extern int16_t speedStorage;
	extern int16_t posStorage;

	const char mode1[11] = "mode 1";
	const char mode2[11] = "mode 2";
	const char mode3[11] = "mode 3";
	const char stop[11] = "stop";
	const char step[11] = "step";

	const char TX_Mode[16] = "Chosen mode:\r\n";
	const char TX_Speed[16] = "Speed\r\n";
	const char TX_Pos_Rev[32] = "Position in revolutions\r\n";
	const char TX_Pos_Deg[32] = "Position in degrees \r\n";
	const char TX_Stop[16] = "Stop \r\n";
	const char TX_Step[16] = "Step \r\n";

	if((!strcmp(buffer,mode1))){
		HAL_GPIO_WritePin(GPIOC, RA_STDBY, GPIO_PIN_SET);
		controlMode = RA_SPEED;

		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Mode, strlen(TX_Mode), 200);
		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Speed, strlen(TX_Speed), 200);
		RA_Motor_Break();
		speedStorage = 0;

		Ref = 0;
		return 1;
	}

	if((!strcmp(buffer,mode2))){
		HAL_GPIO_WritePin(GPIOC, RA_STDBY, GPIO_PIN_SET);
		controlMode = RA_POSITION_REV;
		//Makes the controller starts on the relative position and not the absolute one.
		count = 0;
		pos = 0;
		Ref = 0;
		posStorage = 0;
		speedStorage = 0;

		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Mode, strlen(TX_Mode), 200);
		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Pos_Rev,strlen(TX_Pos_Rev), 200);
		RA_Motor_Break();
		return 1;
	}


	if((!strcmp(buffer,mode3))){
		HAL_GPIO_WritePin(GPIOC, RA_STDBY, GPIO_PIN_SET);
		controlMode = RA_POSITION_DEG;
		//Makes the controller starts on the relative position and not the absolute one.
		count = 0;
		pos = 0;
		Ref = 0;
		posStorage = 0;
		speedStorage = 0;

		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Mode, strlen(TX_Mode), 200);
		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Pos_Deg,strlen(TX_Pos_Deg), 200);
		RA_Motor_Break();
		return 1;
	}


	if((!strcmp(buffer,stop))){

		RA_Motor_Break();
		RA_Motor_Neutral();
		HAL_GPIO_WritePin(GPIOC, RA_STDBY, GPIO_PIN_RESET);

		Ref = 0;
		controlMode = RA_STOP;

		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Mode, strlen(TX_Mode), 200);
		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Stop,strlen(TX_Stop), 200);
		RA_UART_Interface();
		return 0;
	}
	if((!strcmp(buffer,step))){
		HAL_GPIO_WritePin(GPIOC, RA_STDBY, GPIO_PIN_SET);

		controlMode = RA_STEP;
		Ref = 0;

		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Mode, strlen(TX_Mode), 200);
		HAL_UART_Transmit(&huart2, (uint8_t*) TX_Step,strlen(TX_Step), 200);
		return 1;
	}
	return 0;
}

void RA_inputValues(char buffer[32]){
	//@brief: Does the cheking of the input value and do the conversion to int16_t

	extern int16_t Ref;
	extern int controlMode;

	int16_t ref2 = 0;
	ref2 = atof(buffer);

	//STEP control
	if(controlMode == RA_STEP){

		if((ref2<-204)){
			Ref =  -204;
		}
		else if((ref2>204)){
			Ref =  204;
		}

		else{
			Ref =  ref2;
		}
		return;
	}

	if((ref2 >= -999) || (ref2 <= 999)){
		Ref =  ref2;
		return;
	}
}

void RA_UART_Interface(void){
	//@brief: Make an UI through the UART for a simpler understanding for the use of the program

	extern UART_HandleTypeDef huart2;

	const char menu[] = "Choose between the following mode:\r\n"
			"-Speed regulation (Enter: mode 1)\r\n"
			"-Position regulation in revolutions (Enter: mode 2)\r\n"
			"-Position regulation in degrees (Enter: mode 3)\r\n"
			"-Step (Enter: step)\r\n"
			"-Stop (Enter: stop)\r\n";

	HAL_UART_Transmit(&huart2, (uint8_t*) menu, strlen(menu), 100);
}

void RA_Print_Speed(void){
	//@brief: Print the current speed of the motor in RPM

	extern UART_HandleTypeDef huart2;
	extern int16_t Ref;
	extern int16_t speedRPM;

	char speed[32] = "";
	float cspeed  = speedRPM;

	if(Ref < 0){
		sprintf(speed,"Current speed: -%.1f rpm\r\n", cspeed);

	}
	else if(Ref >= 0){
		sprintf(speed,"Current speed: %.1f rpm\r\n", cspeed);
	}

	HAL_UART_Transmit(&huart2, (uint8_t*) speed, strlen(speed), 100);
}

void RA_Print_Position(void){
	//@brief: Print the current position of the motor in revolution

	extern UART_HandleTypeDef huart2;
	extern float pos;
	extern int controlMode;

	char position_TX[32] = "";
	sprintf(position_TX,"Current position: %.1f %s\r\n",pos,controlMode == RA_POSITION_REV? "Rev" :"degrees");

	HAL_UART_Transmit(&huart2, (uint8_t*) position_TX, strlen(position_TX), 100);
}

void RA_Print_Step(void){
	//@brief: Print the current ref of the motor in RPM

	extern UART_HandleTypeDef huart2;
	extern int16_t Ref;

	char ref[32] = "";
	sprintf(ref,"Current step ref: %d rpm\r\n",Ref);

	HAL_UART_Transmit(&huart2, (uint8_t*) ref, strlen(ref), 100);
}

void RA_UART(void){
	//@brief: UART's logic
	//@param uartIndex: allow to drive through the values of the buffer

	extern char uartByte[1];
	extern char uartBuffer[32];
	extern int controlMode;
	extern UART_HandleTypeDef huart2;

	static int uartIndex = 0;
	int y = 0;

	if((!strcmp(uartByte,"\r")) || (!strcmp(uartByte,"\n"))){

		uartIndex = 0;
		//Checking of the buffer
		y = RA_inputModeControl(uartBuffer);
		//Avoid possible corruption of Ref
		if(y){
			memset(uartBuffer,0,strlen(uartBuffer));
			memset(uartByte,0,1);
			HAL_UART_Receive_IT(&huart2, (uint8_t*)uartByte, sizeof(uartByte));
			return;
		}

		if((controlMode != RA_STOP) || (controlMode != RA_DEFAULT))
			RA_inputValues(uartBuffer);

		//Cleaning the buffers to avoid memory leaks
		memset(uartBuffer,0,strlen(uartBuffer));
		memset(uartByte,0,1);

		//Start the reception again
		HAL_UART_Receive_IT(&huart2, (uint8_t*)uartByte, sizeof(uartByte));
		return;
	}

	else{
		uartBuffer[uartIndex] = uartByte[0];
		uartIndex++;
		memset(uartByte,0,1);
		HAL_UART_Receive_IT(&huart2, (uint8_t*)uartByte, sizeof(uartByte));
	}
}
