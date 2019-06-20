#include <stdio.h>
#include "in4073.h"

// Define packet structure

typedef struct {
	uint8_t startByte;
	uint8_t length;
	uint8_t functionCode;
    uint32_t system_time;

	uint16_t ae1;
	uint16_t ae2;
	uint16_t ae3;
	uint16_t ae4;

	uint16_t phi;
	uint16_t theta;
	uint16_t psi;

	uint16_t sp;
	uint16_t sq;
	uint16_t sr;
	
	uint32_t temp;
	uint16_t volt;
	uint32_t press;

	uint8_t crc;
} __attribute__((packed)) Packet;

void dr_t20_packet_rx(void);
void dr_t20_packet_tx(Packet* p);

Packet dr_t20_packet_init(uint16_t ae1, uint16_t ae2,
					      uint16_t ae3, uint16_t ae4, uint16_t phi,
					      uint16_t theta, uint16_t psi, uint16_t sp,
					      uint16_t sq, uint16_t sr, uint32_t temp,
					      uint16_t volt, uint32_t press);