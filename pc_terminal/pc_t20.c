#include "pc_t20.h"
#include "termIO.h"
#include "rs232.h"
#include <unistd.h>
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
void pc_t20_packet_tx(Packet* p) {

	// Transmit packet byte-by-byte
	// printf("In packet sending\n");
	
	uint8_t *packetPtr = (uint8_t *) p;
	const uint8_t *byteToSend;
	int numberOfBytes = p->length;

	for(byteToSend=packetPtr; numberOfBytes--; ++byteToSend)	
	{	
		// Wait for transmission to complete
		rs232_putchar(*byteToSend);
		// usleep(1);
	}
}

/*
* Function: pc_packet_init
* Author: Yordan Dimitrov
* ----------------------------
*   Constructs a control packet that will be sent from PC to the drone
*   
*
*   inputs:
*	uint8_t startByte
*	uint8_t length
*	uint8_t mode
*	int8_t roll
*	int8_t pitch
*	int8_t yaw
*	uint8_t elevation
*
*   returns: structure of type Packet 
*   
*/
Packet pc_packet_init(uint8_t startByte, uint8_t length, uint8_t mode,
					  int8_t roll, 	 int8_t pitch,  int8_t yaw,
					  uint8_t elevation)
{
	Packet x;
	x.startByte = startByte;
	x.length = length;
	x.mode = mode;
	x.roll = roll;
	x.pitch = pitch;
	x.yaw = yaw;
	x.elevation = elevation;
	// printf("sending to crcCalc\n");
	crcCalc(&x);
	
	return x;
}

/*
* Function: crcCalc
* Author: Yordan Dimitrov
* ----------------------------
*   Computes the crc values for the entire package and stores in the last field of the packet.
*	
*   inputs: packet of type Packet
*   returns: none 
*   
*/

void crcCalc (Packet *p) {
	uint8_t crc = 0;
	crc ^= p->startByte;
	crc ^= p->length;
	crc ^= p->mode;
	crc ^= p->roll;
	crc ^= p->pitch;
	crc ^= p->yaw;
	crc ^= p->elevation;
	p->crc = crc;
	// printf("crc = %d\n", crc);
}



uint8_t startByte
uint8_t length
uint8_t mode
int8_t roll
int8_t pitch
int8_t yaw
uint8_t elevation