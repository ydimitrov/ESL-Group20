
/*Yordan*/



#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "fsmReceive.h"
#include "in4073.h"

#define READBYTE 0
#define PREAMBLEBYTE 1 
#define LENGTHBYTE 2 
#define PACKETTYPEBYTE 3 
#define MESSAGE 4 

#define INITIALSTATE 0 
#define CRCCHECK 1 
#define STOREVALUES 2

#define SAFE 0x00 // Packet length for a MODE command
#define PANIC 0x01 // Packet length for a MODE command
#define MANUAL 0x02 // Packet length for a MOVE command
#define CALIBRATION 0x03 // Packet length for a MOVE command
#define YAW 0x04 // Packet length for a MOVE command
#define FULL 0x05 // Packet length for a MOVE command
#define RAW 0x06 // Packet length for a MOVE command
#define HEIGHT 0x07 // Packet length for a MOVE command
#define WIRELESS 0x08 // Packet length for a MOVE command
#define P_INC 9
#define P_DEC 10
#define P1_INC 11
#define P1_DEC 12
#define P2_INC 13
#define P2_DEC 14

#define PACKETLEN 8 // will be changed

// int8_t temp;
void (* statesFunc)(void) = initialState;
uint8_t buffer[PACKETLEN];
uint8_t arrIndex, stateIndex, temp, packetLen, crc;



void (* packetStatesArr[])(void) = {readByte, 
								    preambleByte, 
								    lengthByte, 
								    packetTypeByte, 
								    message}; 
							  

void (* fsmStatesArr[])(void) = {initialState,
							     crcCheck,
							     storeValues};


void printCurrentState(uint8_t state){
	printf("S%d\n", state);
}


/*
* Function: initialState
* Author: Yordan Dimitrov
* ----------------------------
*   Resets buffer container that will contain packet, resets crc value, and index values for state and buffer
*   Used to restart state machine responsible for receiving and parsing packets
*
*   inputs: none
*   returns: none 
*   
*/

void initialState(void){
	// printCurrentState(0);
	arrIndex = 0;
	stateIndex = 1;
	crc = 0;
	statesFunc = packetStatesArr[READBYTE];
	buffer[0] = 0x00;
	packetLen = 0x08;
}

/*
* Function: readByte
* Author: Yordan Dimitrov
* ----------------------------
*   Reads next element from rx queue and sets function pointer to next corresponding state. 
*   If queue is empty exits function
*
*   inputs: none
*   returns: none 
*   
*/

void readByte(void){
	// printCurrentState(1);
	// printf("RXQUEUE SIZE: %d \n", rx_queue.count);	
	if(rx_queue.count > 0) {
		buffer[arrIndex] = dequeue(&rx_queue);
		statesFunc = packetStatesArr[stateIndex];
		// nrf_delay_ms(1);	
  	} else {
  		return;
  	}
}

/*
* Function: preambleByte
* Author: Yordan Dimitrov
* ----------------------------
*   Checks if last read byte is corresponds to the header of the package. 
*   If it does, it increases the array index and state index variable which control the buffer array and state machine states
*	and redirects to readByte state
*	If the byte is not correct it redirects back to the initial state
*
*   inputs: none
*   returns: none 
*   
*/

void preambleByte(void){
	// stateIndex = 1
	// printCurrentState(2);
	temp = buffer[arrIndex];
	if(temp == 0xAA){
		arrIndex++;
    	stateIndex++;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		printf("temp is not equal to 0xAA\n");
		statesFunc = fsmStatesArr[INITIALSTATE];
	}	
}

/*
* Function: lengthByte
* Author: Yordan Dimitrov
* ----------------------------
*   Checks if last read byte is corresponds to the cirrect length of the package. 
*   If it does, it increases the array index and state index variable which control the buffer array and state machine states
*	and redirects to readByte state
*	If the byte is not correct it redirects back to the initial state
*
*   inputs: none
*   returns: none 
*   
*/

void lengthByte(void){
	// stateIndex = 2
	// printCurrentState(3);
	temp = buffer[arrIndex];
	if(temp == PACKETLEN){
		arrIndex++;
    	stateIndex++;
		packetLen = temp;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		printf("temp is not equal to PACKETLEN\n");
		statesFunc = fsmStatesArr[INITIALSTATE];
	}
}

/*
* Function: packetTypeByte
* Author: Yordan Dimitrov
* ----------------------------
*   Checks if last read byte is corresponds to a correct drone mode or P value. 
*   If it does, it increases the array index and state index variable which control 
*	the buffer array and state machine states and redirects to readByte state
*	If the byte is not correct it redirects back to the initial state
*
*   inputs: none
*   returns: none 
*   
*/

void packetTypeByte(void){
	// stateIndex = 3
	// printCurrentState(4);
	temp = buffer[arrIndex];
	if(checkModeByte(temp)){
		arrIndex++;
    	stateIndex++;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		printf("temp is not equal to MODE\n");
		statesFunc = fsmStatesArr[INITIALSTATE];
	}
}

/*
* Function: message
* Author: Yordan Dimitrov
* ----------------------------
*   Responsible for control over package payload. It essentially redirects back to the readByte state
*   the number of times required to store all payload information in the buffer array.
*	This action is repeated until the required number of bytes has been received.
*	When the required number of bytes has been received it transitions to the crc check state.
*
*   inputs: none
*   returns: none 
*   
*/

void message(void){
	// stateIndex = 4
	// printCurrentState(5);
	if(arrIndex == packetLen - 1){
    	stateIndex++;
		statesFunc = fsmStatesArr[CRCCHECK];
	} else {
	    arrIndex++;
	    statesFunc = packetStatesArr[READBYTE];
 	}
}

/*
* Function: crcCheck
* Author: Yordan Dimitrov
* ----------------------------
*   Computes the crc values for the entire package and compares it to the crc value provided by the PC.
*   If the values are correct it transitions to storing the values to be used by the drone.
*	If the values do not match it redirects back to the initial state.
*	
*   inputs: none
*   returns: none 
*   
*/
void crcCheck(void){
	// printCurrentState(6);
	
	for (int i = 0; i < packetLen - 1; i++){
		crc ^= buffer[i];
	}

	if(crc == buffer[packetLen - 1]){
		// printf("Packet OK!\n");
    	stateIndex++;
		statesFunc = fsmStatesArr[STOREVALUES];
	} else {
		printf("rx_queue.count = %d\n", rx_queue.count);
		printf("Packet ERROR!\n");
		statesFunc = fsmStatesArr[INITIALSTATE];
	}
}


/*
* Function: storeValues
* Author: Yordan Dimitrov
* ----------------------------
*	Stores roll, pitch, yaw and lift values, which will be used by the drone.
*	Redirects back to initial state
*
*   inputs: none
*   returns: none 
*   
*/

void storeValues(void){
	// printCurrentState(7);
	//int8_t testpitch = buffer[4];
	// printf("buffer[pitch] = %d\n", buffer[4]);


	modeStore(&buffer[2]);
	flightParameters.roll  = (int8_t)   buffer[3];
	flightParameters.pitch = (int8_t)   buffer[4];
	flightParameters.yaw   = (int8_t)	buffer[5];
	flightParameters.lift  = (uint32_t)	buffer[6];

	// printf("buffer_after[pitch] = %d  flightParameters.pitch = %ld \n ", buffer[4], flightParameters.pitch);

	statesFunc = fsmStatesArr[INITIALSTATE];
}


/*
* Function: fsmReceive
* Author: Yordan Dimitrov
* ----------------------------
*	
*	Executes latest function pointed to by the pointer.
*
*   inputs: none
*   returns: none 
*   
*/
void fsmReceive(){

	// (statesFunc)();

	for (int i = 0; i < 17; i++){  // 17 [0-16] states are required for one packet
		(statesFunc)();
	}
}

/*
* Function: checkModeByte
* Author: Yordan Dimitrov
* ----------------------------
*	Checks if mode byte corresponds to a correct mode.
*	
*   inputs: none
*   returns: none 
*   
*/

uint8_t checkModeByte(uint8_t byte){

	switch(byte){
		case SAFE:
		case PANIC:
		case MANUAL:
		case CALIBRATION:
		case YAW:
		case FULL:
		case RAW:
		case HEIGHT:
		case WIRELESS:
		case P_INC:
		case P_DEC:
		case P1_INC:
		case P1_DEC:
		case P2_INC:
		case P2_DEC:
			return 1;
		default:
			return 0;
	}		
}


/*
* Function: modeStore
* Author: Yordan Dimitrov
* ----------------------------
*	Stores candidate mode in buffer and checks P, P1, P2 values for corresponding mode.
*
*   inputs: none
*   returns: none 
*   
*/

void modeStore(uint8_t *p){
	
	if (*p >= 0 && *p <= 8){
		candidate_mode = *p;
	} else if ( mode == 4 && *p == 9) {
		P += 1;
	} else if ( mode == 4 && *p == 10) {
		P = (P > 0 ? P - 1 : 0); 
	} else if ( mode == 5 && *p == 9) {
		P += 1;
	} else if ( mode == 5 && *p == 10) {
		P = (P > 0 ? P - 1 : 0);
	} else if ( mode == 5 && *p == 11) {
		P1 += 1;
	} else if ( mode == 5 && *p == 12) {
		P1 = (P1 > 0 ? P1 - 1 : 0);
	} else if ( mode == 5 && *p == 13) {
		P2 += 1;
	} else if ( mode == 5 && *p == 14) {
		P2 = (P2 > 0 ? P2 - 1 : 0);
	} else if ( mode == 6 && *p == 9) {
		P += 1;	
	} else if (mode == 6 && *p == 10) {
		P = (P > 0 ? P - 1 : 0);
	} else if (mode == 6 && *p == 11) {
		P1 += 1;
	} else if (mode == 6 && *p == 12) {
		P1 = (P1 > 0 ? P1 - 1 : 0);
	} else if (mode == 6 && *p == 13) {
		P2 += 1;
	} else if (mode == 6 && *p == 14) {
		P2 = (P2 > 0 ? P2 - 1 : 0);
	} else if (mode == 7 && *p == 9) {
		P += 1;
	} else if (mode == 7 && *p == 10) {
		P = (P > 0 ? P - 1 : 0); 
	}
}