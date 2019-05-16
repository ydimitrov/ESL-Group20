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
#include <termios.h>
#include "joystick.h"
#include "pc_terminal.h"

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



/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
struct termios 	savetty;

void	term_initio()
{
	struct termios tty;

	tcgetattr(0, &savetty);
	tcgetattr(0, &tty);

	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
	tty.c_cc[VTIME] = 0;
	tty.c_cc[VMIN] = 0;

	tcsetattr(0, TCSADRAIN, &tty);
}

void	term_exitio()
{
	tcsetattr(0, TCSADRAIN, &savetty);
}

void	term_puts(char *s)
{
	fprintf(stderr,"%s",s);
}

void	term_putchar(char c)
{
	putc(c,stderr);
}

int	term_getchar_nb()
{
        static unsigned char 	line [2];

        if (read(0,line,1)) // note: destructive read
        		return (int) line[0];

        return -1;
}

int	term_getchar()
{
        int    c;

        while ((c = term_getchar_nb()) == -1)
                ;
        return c;
}


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
	int yaw;
	int roll;
	int pitch;
	int lift;
	int mode;
	int yaw_offset;
	int roll_offset;
	int pitch_offset;
	int lift_offset;
	int P;
	int P1;
	int P2;	
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

int main (int argc, char **argv)
{
	// initialize
	int 		freq = 100; // defines transmission frequency
	int 		fd;
	struct js_event js;
	unsigned int	t = mon_time_ms();

	term_initio();

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}

	while(1)
	{
		// get joystick values
		printf("Outside if\n");
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
						input.yaw = (int) js.value/256 + yaw_offset;
					}
					else if (js.number == 1)
					{
						input.pitch = (int) js.value/256 + pitch_offset;
					}
					else if (js.number == 2)
					{	
						input.roll = (int) js.value/256 + roll_offset;
					}
					else if (js.number == 3)
					{
						input.lift = (int) js.value/256 + lift_offset;
					}
					break;
				default: break;
			}
		}
		
		// get keyboard values and update mode and setpoint if needed
		keyboardfunction();
		// create packet
		printf("%5d  %5d  ",t ,mon_time_ms());
			printf("Axis: %d %d %d %d %d %d \n Butons: %d %d %d %d %d %d %d %d %d %d %d %d \n\n", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7], button[8], button[9], button[10], button[11]); 
		// send packet periodically
		if (t < freq)
		{
			unsigned int t_now = mon_time_ms();
			if (t < t_now && t_now < 64900)
			{
				//SEND PACKET
			printf("%5d  %5d  ",t ,mon_time_ms());
			printf("Axis: %d %d %d %d %d %d \n Butons: %d %d %d %d %d %d %d %d %d %d %d %d \n\n", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7], button[8], button[9], button[10], button[11]); 
				t = (t + freq) % 65000; //set next transmission time
			} 
		}
		else
		{
			if (t < mon_time_ms())
			{
				//SEND PACKET
			printf("%5d  %5d ",t,mon_time_ms());
			printf("Axis: %d %d %d %d %d %d \n Butons: %d %d %d %d %d %d %d %d %d %d %d %d \n\n", axis[0], axis[1], axis[2], axis[3], axis[4], axis[5], button[0], button[1], button[2], button[3], button[4], button[5], button[6], button[7], button[8], button[9], button[10], button[11]); 
				t = (t + freq) % 65000; //set next transmission time
			}
		}
				
	}
}
