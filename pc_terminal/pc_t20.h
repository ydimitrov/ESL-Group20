#include <stdio.h>
#include <inttypes.h>

// Define packet structure

//#define QUEUE_SIZE 256

typedef struct {
	uint8_t startByte;
	uint8_t length;
	uint8_t mode;
	uint8_t roll;	 
	uint8_t pitch; 
	uint8_t yaw;
	uint8_t elevation;
	uint8_t crc;
} Packet;

void pc_t20_packet_rx(void);
void pc_t20_packet_tx(Packet* p);

Packet pc_packet_init(uint8_t startByte, uint8_t length, uint8_t mode,
					  uint8_t roll, 	 uint8_t pitch,  uint8_t yaw,
					  uint8_t elevation);


void crcCalc (Packet *p);

// uint8_t startByte = 0xAA
// uint8_t length = 0x30
// uint8_t functionCode = 0xAB
// uint8_t roll = 0xAC
// uint8_t pitch = 0xAD
// uint8_t yaw = 0xAE
// uint8_t elevation = 0xAF 
// uint8_t phi = 0xBA
// uint8_t theta = 0xBB
// uint8_t psi = 0xBC
// uint8_t sp = 0xBD
// uint8_t sq = 0xBE
// uint8_t sr = 0xBF
// uint64_t temp = 0xCA
// uint8_t volt = 0xCB
// uint8_t press = 0xCC
// uint8_t mode = 0xCD
// 0xAA,0x30,0xAB,0xAC,0xAD,0xAE,0xAF ,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xCA,0xCB,0xCC,0xCD

//SENDING FROM PC TO DRONE
