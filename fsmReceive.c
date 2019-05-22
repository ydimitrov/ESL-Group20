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


void printCurrentState(uint8_t state)
{
	printf("S%d\n", state);
}


void initialState(){
	// printCurrentState(0);
	arrIndex = 0;
	stateIndex = 1;
	crc = 0;
	statesFunc = packetStatesArr[READBYTE];
	buffer[0] = 0x00;
	packetLen = 0x08;
}

void readByte(void){
	// printCurrentState(1);
		// printf("RXQUEUE SIZE: %d \n", rx_queue.count);	
	if(rx_queue.count > 0) {
		// printf("RXQUEUE SIZE: %d \n", rx_queue.count);	
		buffer[arrIndex] = dequeue(&rx_queue);
		statesFunc = packetStatesArr[stateIndex];
  	} else {
		statesFunc = fsmStatesArr[INITIALSTATE];
  		return;
	 	// nrf_delay_ms(1);
	 	// printf("No byte in rx_queue\n");
  	}
}

void preambleByte(void){
	// stateIndex = 1
	// printCurrentState(2);
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
	// stateIndex = 2
	// printCurrentState(3);
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
	// stateIndex = 3
	// printCurrentState(4);
	temp = buffer[arrIndex];
	if(temp == SAFE || temp == PANIC || temp == MANUAL || temp == CALIBRATION || temp == YAW || temp == FULL || temp == RAW || temp == HEIGHT || temp == WIRELESS){
		arrIndex++;
    	stateIndex++;
		statesFunc = packetStatesArr[READBYTE];
	} else {
		statesFunc = fsmStatesArr[INITIALSTATE];
	}
}

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

void crcCheck(void){
	// printCurrentState(6);
	
	for (int i = 0; i < packetLen - 1; i++){
		crc ^= buffer[i];
	}

	// printf("FSMcrc = %d\n", crc);

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
	// printCurrentState(7);
	// printf("buffer[3] = %.2x, buffer[4] = %.2x, buffer[5] = %.2x, buffer[6] = %.2x\n", buffer[3], buffer[4], buffer[5], buffer[6]);
	mode		 	  = buffer[2]; 
	flightParameters.roll  = buffer[3]; 
	flightParameters.pitch = buffer[4];
	flightParameters.yaw   = buffer[5];
	flightParameters.lift  = buffer[6];
	printf("mode = %d\n", mode);

	statesFunc = fsmStatesArr[INITIALSTATE];
}

void fsmReceive(){
	
	(statesFunc)();
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();
	// (statesFunc)();

	// for (int i = 0; i < 1; i++){
	// 	(statesFunc)();
	// }
}