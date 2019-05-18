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
#include "pc_t20.h"
#include "rs232.h"
#include "joystick.h"
#include "termIO.h"

#define JS_DEV	"/dev/input/js0"
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
	char yaw;
	char roll;
	char pitch;
	char lift;
	char mode;
	char yaw_offset;
	char roll_offset;
	char pitch_offset;
	char lift_offset;
	char P;
	char P1;
	char P2;	
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

void keyboardfunction()
{
	int c; //keyboard input
	
	if((c = term_getchar_nb()) != -1)
	{
		if (input.mode != PANIC_MODE)
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
						 input.lift_offset = input.lift_offset;
						 break;
					case 'z': //set lift down
						 input.lift_offset = input.lift_offset;
						 break;
					case 'w': //yaw rate up
						 input.yaw_offset = input.yaw_offset;
						 break;
					case 'q': //yaw rate down
						 input.yaw_offset = input.yaw_offset;
					         break;
					case 'u': //set P of yaw up
						 input.P = min(input.P + 1,127);
						 break;
					case 'j': //set P of yaw down
						 input.P = max(input.P - 1,0);
						 break;
					case 'i': //set roll/pitch P1 up
						 input.P1 = min(input.P1 + 1,127);
						 break;
					case 'k': //set roll/pitch P1 down
						 input.P1 = max(input.P1 - 1,0);
						 break;
					case 'o': //set roll/pitch P2 up
						 input.P2 = min(input.P2 + 1,127);
						 break;
					case 'l': //set roll/pitch P2 down
						 input.P2 = max(input.P2 - 1,0);
						 break;

					//arrow keys
					case 65: //pitch down (up arrow)
						 input.pitch_offset = input.pitch_offset;
						 break;
					case 66: //pitch up (down arrow)
						 input.pitch_offset = input.pitch_offset;
						 break;
					case 67: //roll up (right arrow)
						 input.roll_offset = input.roll_offset;
						 break;
					case 68: //roll down (left arrow)
						 input.roll_offset = input.roll_offset;
						 break;
					default: break; //other key was pressed
				}
			}
		}
	}
}








/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{

	// initialize

	int 			period = 100; // defines transmission frequency
	int 			fd;
	struct 			js_event js;
	unsigned int	t = mon_time_ms();

	// Initialize input values

	input.roll = 0;
	input.yaw = 0;
	input.pitch = 0;
	input.lift = 0;

	input.roll_offset = 0;
	input.yaw_offset = 0;
	input.pitch_offset = 0;
	input.lift_offset = 0;

	input.mode = 2;

	input.P = 0;
	input.P1 = 0;
	input.P2 = 0;


	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		//perror("jstest");
		printf("Failed to initiate communication with joystick!\n");
		exit(1);
	}

	term_initio();
	rs232_open();
	//term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	//term_puts("Type ^C to exit\n");

	term_puts("blabla");
	
	for (;;)
	{

		// get joystick values
		if(read(fd, &js, sizeof(struct js_event)) == 
		       			sizeof(struct js_event))   {
		
			//printf("%5d   ",t);
			/* register data
			 */
			// fprintf(stderr,".");
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
						input.yaw = (int) js.value/256 + input.yaw_offset;
					}
					else if (js.number == 1)
					{
						input.pitch = (int) js.value/256 + input.pitch_offset;
					}
					else if (js.number == 2)
					{	
						input.roll = (int) js.value/256 + input.roll_offset;
					}
					else if (js.number == 3)
					{
						input.lift = (int) js.value/256 + input.lift_offset;
					}
					break;
				default: break;
			}
		}
		
		// get keyboard values and update mode and setpoint if needed
		keyboardfunction();

		// printf("%5d  %5d  ",t ,mon_time_ms());
		// printf("Axis: %d %d %d %d %d %d \n Butons: %d %d %d %d %d %d %d %d %d %d %d %d \n\n", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7], button[8], button[9], button[10], button[11]); 
		

		// Communicate periodically
		

		if (t < period)
		{
			unsigned int t_now = mon_time_ms();
			if (t < t_now && t_now < 64900)
			{
			
				printf("Send packet.\n");
				Packet txPacket = pc_packet_init(0xAA, 0x07, input.mode, input.pitch, input.roll, input.yaw, input.lift);
				
				pc_t20_packet_tx(&txPacket);

				printf("%5d  %5d  ",t ,mon_time_ms());
				//printf("Axis: %d %d %d %d %d %d \n Butons: %d %d %d %d %d %d %d %d %d %d %d %d \n\n", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7], button[8], button[9], button[10], button[11]); 
				printf("Mode is: %d\t", input.mode);
				t = (t + period) % 65000; //set next transmission time
			} 
		}
		else
		{
			if (t < mon_time_ms())
			{

				printf("Send packet.\n");
				Packet txPacket = pc_packet_init(0xAA, 0x07, input.mode, input.pitch, input.roll, input.yaw, input.lift);

				// TODO: Check the order of fields and the function code for a move command
				pc_t20_packet_tx(&txPacket);
			
				printf("%5d  %5d ",t,mon_time_ms());
				//printf("Axis: %d %d %d %d %d %d \n Butons: %d %d %d %d %d %d %d %d %d %d %d %d \n\n", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7], button[8], button[9], button[10], button[11]); 
				printf("Mode is: %d\n", input.mode);
				t = (t + period) % 65000; //set next transmission time
			}
		}
				
	}		

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}
