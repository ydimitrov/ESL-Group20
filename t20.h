#include <stdio.h>

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
} packet;

//void init_queue(queue *q);
//void enqueue(queue *q, char x);
//char dequeue(queue *q);

void t20_packet_rx(packet p);
void t20_packet_tx(packet p);