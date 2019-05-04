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

void t20_packet_rx(void);
void t20_packet_tx(packet* p);