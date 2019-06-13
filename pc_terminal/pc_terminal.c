/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include <ctype.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include "pc_t20.h"
#include "rs232.h"
#include "joystick.h"
#include "termIO.h"

#define JS_DEV	"/dev/input/js1"
#define max(x, y) (((x) > (y)) ? (x) : (y))
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define SAFE_MODE 0
#define PANIC_MODE 1
#define MANUAL_MODE 2
#define CALIBRATION_MODE 3
#define YAW_MODE 4
#define FULL_MODE 5
#define RAW_MODE 6
#define HEIGHT_MODE 7
#define WIRELESS_MODE 8
#define P_INCREMENT 9
#define P_DECREMENT 10
#define P1_INCREMENT 11
#define P1_DECREMENT 12
#define P2_INCREMENT 13
#define P2_DECREMENT 14
#define STARTBYTE 0xAA
#define PACKETLEN 0X08

/* current axis and button readings
 */

int	axis[6] = {0,0,0,0,0,0};
/* values stored in axis array, needs to be verified
0: yaw
1: roll
2: pitch
3: lift
4: X
5: X
*/
int	button[12]= {0,0,0,0,0,0,0,0,0,0,0,0};
/*
0: safe mode
1: panic mode
2: manual mode
3: calibration mode
4: yaw control mode
5: full control mode
6: raw mode
7: height control mode
8: wireless mode
*/

struct input_status
{
	int8_t yaw;
	int8_t roll;
	int8_t pitch;
	int8_t lift;
	uint8_t mode;
	int8_t yaw_offset;
	int8_t roll_offset;
	int8_t pitch_offset;
	int8_t lift_offset;
	uint8_t P;
	uint8_t P1;
	uint8_t P2;	
}input;

//timer
unsigned int    mon_time_ms(void)
{
        unsigned int    ms;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        ms = 1000 * (tv.tv_sec % 65); // 65 sec wrap around
        ms = ms + tv.tv_usec / 1000;
        return ms;
}

int8_t checkByteOverflow(int8_t value, int8_t offset) {
	
	// Make sure overflows won't happen.

	int8_t txValue;

	if((int16_t)(value + offset) > 127) {
		txValue = 127;
	} else if ((int16_t)(value + offset) < -128) {
		txValue = -128;	
	} else {
		txValue = value + offset; 
	}
				
	return txValue;
} 

uint8_t checkByteOverflowLift(int8_t value, int8_t offset) {
	
	// Make sure overflows won't happen.

	uint8_t txValue;

	if((int16_t)(value + offset +127) > 255) {
		txValue = 255;
	} else if ((int16_t)(value + offset + 127) < 0) {
		txValue = 0;	
	} else {
		txValue = value + offset + 127; 
	}
				
	return txValue;
} 

void keyboardfunction()
{
	int c; //keyboard input
	
	if((c = term_getchar_nb()) != -1)
	{
		switch(c)
		{
			// mode change
			case '0': input.mode = SAFE_MODE;
				break;
			case '1': input.mode = PANIC_MODE;
				break;
			case '2': input.mode = MANUAL_MODE;
				break;
			case '3': input.mode = CALIBRATION_MODE;
				break;
			case '4': input.mode = YAW_MODE;
				break;
			case '5': input.mode = FULL_MODE;
				break;
			case '6': input.mode = RAW_MODE;
				break;
			case '7': input.mode = HEIGHT_MODE;
				break;
			case '8': input.mode = WIRELESS_MODE;
				break;
		}
		if (input.mode != SAFE_MODE)
		{
			switch(c)
			{
				// movement change
				case 'a': //set lift up
					input.lift_offset = checkByteOverflow(input.lift_offset, 1);
					break;
				case 'z': //set lift down
					input.lift_offset = checkByteOverflow(input.lift_offset, -1);
					break;
				case 'w': //yaw rate up
					input.yaw_offset = checkByteOverflow(input.yaw_offset, 1);
					break;
				case 'q': //yaw rate down
					input.yaw_offset = checkByteOverflow(input.yaw_offset, -1);
					break;
				case 'u': //set P of yaw up
					input.P = 2;
					break;
				case 'j': //set P of yaw down
				 	input.P = 1;
					break;
				case 'i': //set roll/pitch P1 up
				 	input.P1 = 2;
					break;
				case 'k': //set roll/pitch P1 down
				 	input.P1 = 1;
					break;
				case 'o': //set roll/pitch P2 up
				 	input.P2 = 2;
					break;
				case 'l': //set roll/pitch P2 down
				 	input.P2 = 1;
					break;

				//arrow keys
				case 65: //pitch down (up arrow)
					input.pitch_offset = checkByteOverflow(input.pitch_offset, 1);
					break;
				case 66: //pitch up (down arrow)
					input.pitch_offset = checkByteOverflow(input.pitch_offset, -1);
					break;
				case 67: //roll up (right arrow)
				 	input.roll_offset = checkByteOverflow(input.roll_offset, 1);
				 	break;
				case 68: //roll down (left arrow)
				 	input.roll_offset = checkByteOverflow(input.roll_offset, -1);
				 	break;
				default: break; //other key was pressed
			}
		}
	}
}
Packet txPacket;

// void *thread_period_send()
// {
//  	uint64_t period = 20000; 
//  	for(;;)
//  	{
//  		// pthread_mutex_lock(&send_mutex);
//  		pc_t20_packet_tx(&txPacket);
//  		// pthread_mutex_unlock(&send_mutex);
//  		usleep(period);
//  	}
// }


void *thread_receive(){
	int c;
	while (1){
		if((c = rs232_getchar_nb()) != -1)
			term_putchar(c); 
	}	
}

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{

	// initialize

	int 			fd;
	struct 			js_event js;
	unsigned int 	time;


	int8_t yawTx;
	int8_t rollTx;
	int8_t pitchTx;
	uint8_t liftTx;
	uint8_t modeTx;


	input.roll = 0;
	input.yaw = 0;
	input.pitch = 0;
	input.lift = -127;

	input.roll_offset = 0;
	input.yaw_offset = 0;
	input.pitch_offset = 0;
	input.lift_offset = 0;

	input.mode = 0;		   // set together
	uint8_t oldmode = 0;   // set together

	input.P = 0;
	input.P1 = 0;
	input.P2 = 0;

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		//perror("jstest");
		printf("Failed to initiate communication with joystick!\n");
		exit(1);
	}

	fcntl(fd, F_SETFL, O_NONBLOCK);


	term_initio();
	rs232_open();

	pthread_t thread1;
	pthread_create(&thread1, NULL, thread_receive, NULL);
	
	time = mon_time_ms();
	
	for (;;)
	{
		if (mon_time_ms() - time >= 20){

			while(read(fd, &js, sizeof(struct js_event)) == sizeof(struct js_event))   {
				switch(js.type & ~JS_EVENT_INIT) {
					case JS_EVENT_BUTTON:
						button[js.number] = js.value;
						if (js.value == 1)
						{
							if (js.number == 0)
							{
								input.mode = PANIC_MODE;
							}
							else if (js.number == 1)
							{
								input.mode = SAFE_MODE;
							}
							else
							{
								input.mode = js.number;
							}
						}
						break;
					case JS_EVENT_AXIS:
						axis[js.number] = js.value;
						if (js.number == 0)
						{
							input.roll = (int) js.value/256;
						}
						else if (js.number == 1)
						{
							input.pitch = -(int) js.value/256;
						}
						else if (js.number == 2)
						{	
							input.yaw = (int) js.value/256;
						}
						else if (js.number == 3)
						{
							input.lift = (int) -js.value/256;
						}
						break;
					default: break;
				}
			}

			keyboardfunction();

			pitchTx = checkByteOverflow(input.pitch, input.pitch_offset);
			rollTx = checkByteOverflow(input.roll, input.roll_offset);
			yawTx = checkByteOverflow(input.yaw, input.yaw_offset);
			liftTx = checkByteOverflowLift(input.lift, input.lift_offset);

			if (oldmode != input.mode) {
				modeTx = input.mode;
			} else {
				if(input.P != 0) {
					if(input.P == 1) {
						modeTx = P_DECREMENT;
						input.P = 0;
					} else {
						modeTx = P_INCREMENT;
						input.P = 0;
					}
				} else if (input.P1 != 0) {
					if(input.P1 == 1) {
						modeTx = P1_DECREMENT;
						input.P1 = 0;
					} else {
						modeTx = P1_INCREMENT;
						input.P1 = 0;
					}
				} else if (input.P2 != 0) {
					if(input.P2 == 1) {
						modeTx = P2_DECREMENT;
						input.P2 = 0;
					} else {
						modeTx = P2_INCREMENT;
						input.P2 = 0;
					}
				} else {
					modeTx = input.mode;
				}
			}

			txPacket = pc_packet_init(STARTBYTE, PACKETLEN, modeTx, rollTx, pitchTx, yawTx, liftTx);
			pc_t20_packet_tx(&txPacket);
			
			oldmode = input.mode;			
			time = mon_time_ms();
		}
	}
	
	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}
