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

void t20_packet_rx() {

	printf("Receiving...");
	while(&rx_queue.count){
		// Check preamble
		if(dequeue(&rx_queue) == 0xAA){
			uint8_t length = dequeue(&rx_queue);
			switch (length) {
				case MODELEN:
				;
					// TODO: CRC check
					// Read mode
					// setMode(p->mode);
					//printf("MODE PACKET RECEIVED");
				    uint8_t foo = 159;
					uart_put(foo);
				break;

				case MOVELEN:
					// TODO: CRC check
					// setRotors(p->roll, p->pitch, p->yaw, p->elevation);
					printf("MOVELEN PACKET RECEIVED");

				break;

				case TELELEN:
					// TODO: CRC check
					printf("TELELEN PACKET RECEIVED");
				break;

				default:
					// No valid command received. Drop packet.
					printf("DEFAULT");
				break;
			}
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

