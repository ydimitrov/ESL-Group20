
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#include "joystick.h"


/* current axis and button readings
 */
int	axis[6];
int	button[12];


/* time
 */
#include <time.h>
#include <assert.h>


#define JS_DEV	"/dev/input/js1"

void joystick_read (int argc, char **argv)
{
	int 		fd;
	struct js_event js;
	unsigned int	t, i;

	if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
		perror("jstest");
		exit(1);
	}

	/* non-blocking mode
	 */
	fcntl(fd, F_SETFL, O_NONBLOCK);

	/* check up on JS
	 */
	while (read(fd, &js, sizeof(struct js_event)) == 
	       			sizeof(struct js_event))  {
	
		/* register data
		 */
		// fprintf(stderr,".");
		switch(js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_BUTTON:
				button[js.number] = js.value;
				break;
			case JS_EVENT_AXIS:
				axis[js.number] = js.value;
				break;
		}
	}
	if (errno != EAGAIN) {
		perror("\njs: error reading (EAGAIN)");
		exit (1);
	}

	printf("\n");
	printf("%5d   ",t);
	for (i = 0; i < 6; i++) {
		printf("%6d ",axis[i]);
	}
	printf(" |  ");
	for (i = 0; i < 12; i++) {
		printf("%d ",button[i]);
	}
	if (button[0])
		break;

}
