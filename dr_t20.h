#include <stdio.h>
#include "in4073.h"

// Define packet structure

//#define QUEUE_SIZE 256
typedef struct {
	uint8_t startByte;
	uint8_t length;
	uint8_t functionCode;
	uint8_t roll;
	uint8_t pitch;
	uint8_t yaw;
	uint8_t elevation;
	int16_t ae[4];
	int16_t phi;
	int16_t theta;
	int16_t psi;
	int16_t sp;
	int16_t sq;
	int16_t sr;
	uint64_t temp;
	int16_t volt;
	int16_t press;
	int16_t mode;
} packet;

void dr_t20_packet_rx(void);
void dr_t20_packet_tx(packet* p);

packet dr_packet_init(uint8_t startByte, uint8_t length,  uint8_t functionCode,
                      uint8_t roll,      uint8_t pitch,   uint8_t yaw,
                      uint8_t elevation, int16_t ae[4],   int16_t phi,
                      int16_t theta,     int16_t psi,     int16_t sp,
                      int16_t sq,        int16_t sr,      uint64_t temp,
              		  int16_t volt,      int16_t press,   int16_t mode);

// uint8_t startByte = 0xAA
// uint8_t length = 0x30
// uint8_t functionCode = 0xAB
// uint8_t roll = 0xAC
// uint8_t pitch = 0xAD
// uint8_t yaw = 0xAE
// uint8_t elevation = 0xAF 
// int16_t phi = 0xBA
// int16_t theta = 0xBB
// int16_t psi = 0xBC
// int16_t sp = 0xBD
// int16_t sq = 0xBE
// int16_t sr = 0xBF
// uint64_t temp = 0xCA
// int16_t volt = 0xCB
// int16_t press = 0xCC
// int16_t mode = 0xCD
// 0xAA,0x30,0xAB,0xAC,0xAD,0xAE,0xAF ,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xCA,0xCB,0xCC,0xCD

//SENDING FROM DRONE TO PC