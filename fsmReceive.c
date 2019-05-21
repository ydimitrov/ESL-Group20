#include <stdio.h>
#include <inttypes.h>
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


// void printCurrentState(uint8_t state)
// {
// 	printf("S%d\n", state);
// }


void initialState(){
	arrIndex = 0;
	stateIndex = 1;
	crc = 0;
	statesFunc = packetStatesArr[READBYTE];
	packetLen = 0x08;
}

void readByte(void){
	if(rx_queue.count > 0) {	
		buffer[arrIndex] = dequeue(&rx_queue);
    	statesFunc = packetStatesArr[stateIndex];
  	} else {
  		// printf("No byte in rx_queue\n");
  	}
}

void preambleByte(void){
	temp = buffer[arrIndex];
	// printf("temp in preambleByte is = %x\n", temp);
	if(temp == 0xAA){
		arrIndex++;
    	stateIndex++;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		// printf("temp is not equal to 0xAA\n");
		statesFunc = fsmStatesArr[INITIALSTATE];
	}	
}

void lengthByte(void){
	temp = buffer[arrIndex];
	// if(temp >= MODELEN && temp <= MOVELEN ){
	if(temp == PACKETLEN){
		arrIndex++;
    	stateIndex++;
		packetLen = temp;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		statesFunc = fsmStatesArr[INITIALSTATE];
	}

}

void packetTypeByte(void){
	temp = buffer[arrIndex];
	if(checkModeByte(temp)){
		arrIndex++;
    	stateIndex++;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		statesFunc = fsmStatesArr[INITIALSTATE];
	}
}

void message(void){
	if(arrIndex == packetLen - 1){
    	stateIndex++;
		statesFunc = fsmStatesArr[CRCCHECK];
	} else {
	    arrIndex++;
	    statesFunc = packetStatesArr[READBYTE];
 	}
}

void crcCheck(void){
	
	for (int i = 0; i < packetLen - 1; i++){
		crc ^= buffer[i];
	}

	if(crc == buffer[packetLen - 1]){
		printf("Packet OK!\n");
    	stateIndex++;
		statesFunc = fsmStatesArr[STOREVALUES];
	} else {
		printf("Packet ERROR!\n");
		statesFunc = fsmStatesArr[INITIALSTATE];
	}
}

void storeValues(void){
	// printf("buffer[3] = %.2x, buffer[4] = %.2x, buffer[5] = %.2x, buffer[6] = %.2x\n", buffer[3], buffer[4], buffer[5], buffer[6]);
	printf("mode = %d\n", mode);
	
	if (buffer[2] >= 0 && buffer[2] <= 8){
		mode = buffer[2];
	} else if ( mode == 4 && buffer[2] == 9) {
		P += 1;
	} else if ( mode == 4 && buffer[2] == 10) {
		P = (P > 0 ? P - 1 : 0); 
	} else if ( mode == 5 && buffer[2] == 9) {
		P += 1;
	} else if ( mode == 5 && buffer[2] == 10) {
		P = (P > 0 ? P - 1 : 0);
	} else if ( mode == 5 && buffer[2] == 11) {
		P1 += 1;
	} else if ( mode == 5 && buffer[2] == 12) {
		P1 = (P1 > 0 ? P1 - 1 : 0);
	} else if ( mode == 5 && buffer[2] == 13) {
		P2 += 1;
	} else if ( mode == 5 && buffer[2] == 14) {
		P2 = (P2 > 0 ? P2 - 1 : 0);
	}

	flightParameters.roll  = (int8_t)  	buffer[3]; 
	flightParameters.pitch = (int8_t)	buffer[4];
	flightParameters.yaw   = (int8_t)	buffer[5];
	flightParameters.lift  = (int8_t)	buffer[6];


	statesFunc = fsmStatesArr[INITIALSTATE];
}

void fsmReceive(){
	
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();

	for (int i = 0; i < 6; i++){
		(statesFunc)();
	}
}


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
