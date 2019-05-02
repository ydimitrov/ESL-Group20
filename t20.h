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
  	uint8_t roll;
} packet;

//void init_queue(queue *q);
//void enqueue(queue *q, char x);
//char dequeue(queue *q);

void t20_packet_rx(packet p);
void t20_packet_tx(packet p);