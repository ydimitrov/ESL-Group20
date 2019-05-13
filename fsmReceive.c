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

#define MODELEN 0x05 // Packet length for a MODE command
#define MOVELEN 0x08 // Packet length for a MOVE command

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
  		printf("No byte in rx_queue\n");
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
	if(temp >= MODELEN && temp <= MOVELEN ){
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
	if(temp == MODELEN || temp == MOVELEN){
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
	printf("buffer[3] = %.2x, buffer[4] = %.2x, buffer[5] = %.2x, buffer[6] = %.2x\n", buffer[3], buffer[4], buffer[5], buffer[6]);

	// pilotValue1 = buffer[3]; 
	// pilotValue2 = buffer[4];
	// pilotValue3 = buffer[5];
	// pilotValue4 = buffer[6];

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