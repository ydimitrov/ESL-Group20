#include "dr_t20.h"

/* 
 *  Magic number definitions
 */

#define MODELEN 0x04 // Packet length for a MODE command
#define MOVELEN 0x07 // Packet length for a MOVE command
#define TELELEN 0x30 // Packet length for a TELE command

/*
 *  Receive function of the T20 protocol. It receives bytes from the serial port
 *  and translates them into configuration for the drone like rotor speeds,
 *  mode, etc.
 */

/*Thomas*/
void dr_t20_packet_tx(Packet* p) {

	// Transmit packet byte-by-byte

	uint8_t *packetPtr = (uint8_t *) p;
	uint8_t *byteToSend;
	uint8_t numberOfBytes = p->length;

	if(numberOfBytes){

		for(byteToSend=packetPtr; numberOfBytes--; ++byteToSend)	
		{	
			uart_put(*byteToSend);
			nrf_delay_ms(1); // derp implement timer interrupt DUDES!!!
			// Wait for transmission to complete
		}
	}
}

/*
* Function: dr_packet_init
* Author: Yordan Dimitrov
* ----------------------------
*   Constructs a telemetry packet that will be sent from drone to the PC
*   
*
*   inputs: 
*   uint8_t startByte
*   uint8_t length
*   uint8_t functionCode
*   uint8_t system_time
*   uint8_t roll
*   uint8_t pitch
*   uint8_t yaw
*   uint8_t elevation
*   uint8_t phi
*   uint8_t theta
*   uint8_t psi
*   uint8_t sp
*   uint8_t sq
*   uint8_t sr
*   uint8_t temp
*   uint8_t volt
*   uint8_t press
*
*   returns: structure of type Packet 
*   
*/
Packet dr_packet_init(uint8_t startByte,uint8_t length,uint8_t functionCode,uint8_t system_time,uint8_t roll,uint8_t pitch,uint8_t yaw,uint8_t elevation,uint8_t phi,uint8_t theta,uint8_t psi,uint8_t sp,uint8_t sq,uint8_t sr,uint8_t temp,uint8_t volt,uint8_t press)
{
	Packet x;
	x.startByte =  startByte;
	x.length =  length;
	x.functionCode =  functionCode;
	x.system_time =  system_time;
	x.roll =  roll;
	x.pitch =  pitch;
	x.yaw =  yaw;
	x.elevation =  elevation;
	x.phi =  phi;
	x.theta =  theta;
	x.psi =  psi;
	x.sp =  sp;
	x.sq =  sq;
	x.sr =  sr;
	x.temp =  temp;
	x.volt =  volt;
	x.press =  press;

	return x;
}

