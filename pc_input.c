#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "dr_t20.h"
#include "joystick.h"

#define JS_DEV	"/dev/input/js2"
#define SAFE_MODE = 0;
#define PANIC_MODE = 1;
#define MANUAL_MODE = 2;
#define CALIBRATION_MODE = 3;
#define YAW_MODE = 4;
#define FULL_MODE = 5;
#define RAW_MODE = 6;
#define HEIGHT_MODE = 7;
#define WIRELESS = 8;

/* current axis and button readings
 */

int	axis[6];
/* values stored in axis array, needs to be verified
0: yaw
1: roll
2: pitch
3: lift
4: X
5: X
*/
int	button[12];
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
	int yaw;
	int roll;
	int pitch;
	int lift;
	int mode;
	int P;
	int P1;
	int P2;	
}    

input_status input;

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
			switch(c)
			{
				// mode change
				case 0: input.mode = SAFE_MODE;
					break;
				case 1: input.mode = PANIC_MODE;
					break;
				case 2: input.mode = MANUAL_MODE;
					break;
				case 3: input.mode = CALIBRATION_MODE;
					break;
				case 4: input.mode = YAW_MODE;
					break;
				case 5: input.mode = FULL_MODE;
					break;
				case 6: input.mode = RAW_MODE;
					break;
				case 7: input.mode = HEIGHT_MODE;
					break;
				case 8: input.mode = WIRELESS_MODE;
					break;
			}
			if (input.mode != SAFE_MODE)
				switch(c)
				{
					// movement change
					case 'a': //set lift up
						 input.lift = min(input.lift + 1,127);
						 break;
					case 'z': //set lift down
						 input.lift = max(input.lift - 1,0);
						 break;
					case 'w': //yaw rate up
						 input.yaw = min(input.yaw + 1,127);
						 break;
					case 'q': //yaw rate down
						 input.yaw = max(input.yaw - 1,0);
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
						 input.pitch = min(input.pitch + 1,127);
						 break;
					case 66: //pitch up (down arrow)
						 input.pitch = max(input.pitch - 1,0);
						 break;
					case 67: //roll up (right arrow)
						 input.roll = max(input.roll - 1,0);
						 break;
					case 68: //roll down (left arrow)
						 input.roll = min(input.roll + 1,127);
						 break;
					default: break; //other key was pressed
				}
			}
	}
}

int main (int argc, char **argv)
{
	// initialize
	int 		freq = 100; // defines transmission frequency
	int 		fd;
	struct js_event js;
	unsigned int	t = 0;

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}

	while(1)
	{
		// get joystick values
		if (read(fd, &js, sizeof(struct js_event)) == 
		       			sizeof(struct js_event))  {
		
			printf("%5d   ",t);
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
						input.yaw = (uint8_t) js.value/256;
					}
					else if (js.number == 1)
					{
						input.pitch = (uint8_t) js.value/256;
					}
					else if (js.number == 2)
					{	
						input.roll = (uint8_t) js.value/256;
					}
					else if (js.number == 3)
					{
						input.lift = (uint8_t) js.value/256;
					}
					break;
			}
		}

		// get keyboard values and update mode and setpoint if needed
		keyboardfunction();
		// create packet
		
		// send packet periodically
		if (t < freq)
		{
			unsigned int t_now = mon_time_ms();
			if (t < t_now && t_now < 64900)
			{
				//SEND PACKET
				t = (t + freq) % 65000; //set next transmission time
			} 
		}
		else
		{
			if (t < mon_time_ms())
			{
				//SEND PACKET
				t = (t + freq) % 65000; //set next transmission time
			}
		}
				
	}
}
