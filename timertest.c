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

unsigned int    mon_time_further_ms(void)
{
        unsigned int    ms;
        struct timeval  tv;
        struct timezone tz;

        gettimeofday(&tv, &tz);
        ms = (1000 * tv.tv_sec + 100) % 65000; // 65 sec wrap around
        ms = ms + tv.tv_usec / 1000;
        return ms;
}

int main()
{
	unsigned int t = 0;
	while(1)
	{
		if (t < 100)
		{
			unsigned int t_now = mon_time_ms();
			if (t < t_now && t_now < 64900)
			{
			//SEND PACKET
			printf("Time = %d\n", t);
			t = (t + 100) % 65000;
			// takes current time and adds 100 ms, might need to be changed later 
			// for more consistent and well-defined frequency
			} 
		}
		else
		{
			if (t < mon_time_ms())
			{
				//SEND PACKET
				printf("Time = %d\n", t);
				t = (t + 100) % 65000;
			}
		}
	}

}
