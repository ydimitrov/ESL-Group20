#include "t20.h"

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

void t20_packet_rx(packet* p) {
	
	// Check preamble
	if(p->startByte == 0xAA){
		// Good, check packet length
		switch (p->length) {
			case MODELEN:
				// TODO: CRC check
				// Read mode
				// setMode(p->mode);
			break;

			case MOVELEN:
				// TODO: CRC check
				// setRotors(p->roll, p->pitch, p->yaw, p->elevation);
			break;

			case TELELEN:
				// TODO: CRC check
			break;

			default:
				// No valid command received. Drop packet.
			break;
		}
	}
}

void t20_packet_tx(packet* p) {

	// Transmit packet byte-by-byte

	unsigned char *packetPtr = (unsigned char *) p;
	const unsigned char *byteToSend;
	int numberOfBytes = p->length;

	for(byteToSend=packetPtr; numberOfBytes--; ++byteToSend)	
	{	
		uart_put(*byteToSend);
		// Wait for transmission to complete
	}

}

