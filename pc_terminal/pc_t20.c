#include "pc_t20.h"
#include "consoleIO.h"
#include "rs232.h"
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

void pc_t20_packet_rx() {

	printf("Receiving...");
// 	while(&rx_queue.count){
// 		// Check preamble
// 		if(dequeue(&rx_queue) == 0xAA){
// 			uint8_t length = dequeue(&rx_queue);
// 			switch (length) {
// 				case MODELEN:
// 				;
// 					// TODO: CRC check
// 					// Read mode
// 					// setMode(p->mode);
// 					//printf("MODE PACKET RECEIVED");
// 				    // uint8_t foo = 159;
// 				    packet test = pc_packet_init(0xAA,0x30,0xAB,0xAC,0xAD,0xAE,0xAF,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xCA,0xCB,0xCC,0xCD);
// 				    pc_t20_packet_tx(&test);
// 					// uart_put(foo);

// 				break;

// 				case MOVELEN:
// 				;
// 					// TODO: CRC check
// 					// setRotors(p->roll, p->pitch, p->yaw, p->elevation);
// 					// printf("MOVELEN PACKET RECEIVED");
// 				    uint8_t boo = 2;
// 					uart_put(boo);

// 				break;

// 				case TELELEN:
// 					// TODO: CRC check
// 					printf("TELELEN PACKET RECEIVED");
// 				break;

// 				default:
// 					// No valid command received. Drop packet.
// 					printf("DEFAULT");
// 				break;
// 			}
// 		}
// 	}
}

void pc_t20_packet_tx(packet* p) {

	// Transmit packet byte-by-byte

	unsigned char *packetPtr = (unsigned char *) p;
	const unsigned char *byteToSend;
	int numberOfBytes = p->length;

	for(byteToSend=packetPtr; numberOfBytes--; ++byteToSend)	
	{	
		rs232_putchar(*byteToSend);
		// Wait for transmission to complete
	}

}

packet pc_packet_init(uint8_t startByte, uint8_t length,  uint8_t functionCode,
				   	  uint8_t roll, 	 uint8_t pitch,	  uint8_t yaw,
  				      uint8_t elevation, uint8_t ae[4],   uint8_t phi,
				      uint8_t theta,     uint8_t psi,     uint8_t sp,
				      uint8_t sq,		 uint8_t sr,      uint8_t temp,
				      uint8_t volt,      uint8_t press,   uint8_t mode)
{
	packet x;
	x.startByte = startByte;
	x.length = length;
	x.functionCode = functionCode;
	x.roll = roll;
	x.pitch = pitch;
	x.yaw = yaw;
	x.elevation = elevation;
	for (int i = 0; i < 4; i++)
		x.ae[i] = ae[i];
	x.phi = phi;
	x.theta = theta;
	x.psi = psi;
	x.sp = sp;
	x.sq = sq;
	x.sr = sr;
	x.temp = temp;
	x.volt = volt;
	x.press = press;
	x.mode = mode;

	return x;
}