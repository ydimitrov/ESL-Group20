#include "dr_t20.h"

/* 
 *  Magic number definitions
 */

#define PREAMBLE 0xAA // Start byte
#define LENGTH 0x26   // Length of packet in hex
#define MODE 0x01     // Mode for the telemetry packet



/**
 * @author Thomas Makryniotis
 * 
 * Transmit function for the T20 protocol
 *
 * @return
 * @retval None
 */

void dr_t20_packet_tx(Packet* p) {

	// Transmit packet byte-by-byte

	uint8_t *packetPtr = (uint8_t *) p;
	uint8_t *byteToSend;
	uint8_t numberOfBytes = p->length;

	if(numberOfBytes){

		for(byteToSend=packetPtr; numberOfBytes--; byteToSend++) {	
			uart_put(*byteToSend); // Wait for transmission to complete
		}
	}
}

Packet dr_t20_packet_init(uint16_t ae1, uint16_t ae2, 
					  	  uint16_t ae3, uint16_t ae4,uint16_t phi, 
					      uint16_t theta, uint16_t psi, uint16_t sp, 
					      uint16_t sq, uint16_t sr, uint32_t temp, 
					      uint16_t volt, uint32_t press)
{
	Packet tx;
	tx.startByte 	=	PREAMBLE;
	tx.length 		=	LENGTH;
	tx.functionCode =	MODE;
	tx.system_time 	=  	get_time_us();
	
	tx.ae1  =  ae1;
	tx.ae2  =  ae2;
	tx.ae3 	=  ae3;
	tx.ae4  =  ae4;
	
	tx.phi 	 =  phi;
	tx.theta =  theta;
	tx.psi 	 =  psi;
	
	tx.sp = sp;
	tx.sq = sq;
	tx.sr = sr;
	
	tx.temp  =  temp;
	tx.volt  =  volt;
	tx.press =  press;

	return tx;
}
