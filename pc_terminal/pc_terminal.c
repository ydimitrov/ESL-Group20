/*------------------------------------------------------------
 * Simple pc terminal in C
 *
 * Arjan J.C. van Gemund (+ mods by Ioannis Protonotarios)
 *
 * read more: http://mirror.datenwolf.net/serial/
 *------------------------------------------------------------
 */

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <inttypes.h>
#include "pc_t20.h"
#include "consoleIO.h"
#include "rs232.h"
#include <ctype.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>

/*------------------------------------------------------------
 * console I/O
 *------------------------------------------------------------
 */
// struct termios 	savetty;

// void	term_initio()
// {
// 	struct termios tty;

// 	tcgetattr(0, &savetty);
// 	tcgetattr(0, &tty);

// 	tty.c_lflag &= ~(ECHO|ECHONL|ICANON|IEXTEN);
// 	tty.c_cc[VTIME] = 0;
// 	tty.c_cc[VMIN] = 0;

// 	tcsetattr(0, TCSADRAIN, &tty);
// }

// void	term_exitio()
// {
// 	tcsetattr(0, TCSADRAIN, &savetty);
// }

// void	term_puts(char *s)
// {
// 	fprintf(stderr,"%s",s);
// }

// void	term_putchar(char c)
// {
// 	putc(c,stderr);
// }

// int	term_getchar_nb()
// {
//         static unsigned char 	line [2];

//         if (read(0,line,1)) // note: destructive read
//         		return (int) line[0];

//         return -1;
// }

// int	term_getchar()
// {
//         int    c;

//         while ((c = term_getchar_nb()) == -1)
//                 ;
//         return c;
// }

/*------------------------------------------------------------
 * Serial I/O
 * 8 bits, 1 stopbit, no parity,
 * 115,200 baud
 *------------------------------------------------------------
 */


// int serial_device = 0;
// int fd_RS232;

// void rs232_open(void)
// {
//   	char 		*name;
//   	int 		result;
//   	struct termios	tty;

//        	fd_RS232 = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);  // Hardcode your serial port here, or request it as an argument at runtime

// 	assert(fd_RS232>=0);

//   	result = isatty(fd_RS232);
//   	assert(result == 1);

//   	name = ttyname(fd_RS232);
//   	assert(name != 0);

//   	result = tcgetattr(fd_RS232, &tty);
// 	assert(result == 0);

// 	tty.c_iflag = IGNBRK; /* ignore break condition */
// 	tty.c_oflag = 0;
// 	tty.c_lflag = 0;

// 	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* 8 bits-per-character */
// 	tty.c_cflag |= CLOCAL | CREAD; /* Ignore model status + read input */

// 	cfsetospeed(&tty, B115200);
// 	cfsetispeed(&tty, B115200);

// 	tty.c_cc[VMIN]  = 0;
// 	tty.c_cc[VTIME] = 1; // added timeout

// 	tty.c_iflag &= ~(IXON|IXOFF|IXANY);

// 	result = tcsetattr (fd_RS232, TCSANOW, &tty); /* non-canonical */

// 	tcflush(fd_RS232, TCIOFLUSH); /* flush I/O buffer */
// }


// void 	rs232_close(void)
// {
//   	int 	result;

//   	result = close(fd_RS232);
//   	assert (result==0);
// }


// int	rs232_getchar_nb()
// {
// 	int 		result;
// 	unsigned char 	c;
// 	result = read(fd_RS232, &c, 1);

// 	if (result == 0)
// 		return -1;

// 	else
// 	{
// 		assert(result == 1);
// 		return (int) c;
// 	}
// }

// int	rs232_getchar_stream()
// {
// 	int 		result;
// 	unsigned char 	c[48];

// 	result = read(fd_RS232, &c, 48); // !!! WARNING, consider byte number (last argument)

// 	if (result == 0)
// 		return -1;

// 	else
// 	{
// 		assert(result == 48);
// 		return c;
// 	}
// }


// void pc_t20_packet_rx() {

// 	printf("Receiving on pc...");
	
// 	if (rs232_getchar()){

// 	}

// 	while(&rx_queue.count){
// 		// Check preamble
// 		if(dequeue(&rx_queue) == 0xAA){
// 			uint8_t length = dequeue(&rx_queue);
// 			switch (length) {
// 				case MODELEN:
// 				;
// 					// TODO: CRC check
// 					// Read mode
// 					// setMode(p->mode);
// 					//printf("MODE PACKET RECEIVED");
// 				    uint8_t foo = 9;
// 					uart_put(foo);
// 				break;

// 				case MOVELEN:
// 					// TODO: CRC check
// 					// setRotors(p->roll, p->pitch, p->yaw, p->elevation);
// 					printf("MOVELEN PACKET RECEIVED");

// 				break;

// 				case TELELEN:
// 					// TODO: CRC check
// 					printf("TELELEN PACKET RECEIVED");
// 				break;

// 				default:
// 					// No valid command received. Drop packet.
// 					printf("DEFAULT");
// 				break;
// 			}
// 		}
// 	}
// }


// uint8_t* 	rs232_getchar()
// {
// 	int c, i = 0;

// 	uint8_t buffer[48];

// 	while ((c = rs232_getchar_stream()) == -1){
// 		buffer[i] = c;
// 		i++;
// 		printf("%c",c);
// 	}

// 	return &buffer;
// }

// uint8_t 	rs232_getchar()
// {
// 	int c;
	
// 	while ((c = rs232_getchar_nb()) == -1){
// 		term_putchar(c);
// 	}

// 	return c;
// }


// int 	rs232_putchar(char c)
// {
// 	int result;

// 	do {
// 		result = (int) write(fd_RS232, &c, 1);
// 	} while (result == 0);

// 	assert(result == 1);
// 	return result;
// }

// void t20_packet_tx(packet* p) {

// 	// Transmit packet byte-by-byte

// 	unsigned char *packetPtr = (unsigned char *) p;
// 	const unsigned char *byteToSend;
// 	int numberOfBytes = p->length;

// 	for(byteToSend=packetPtr; numberOfBytes--; ++byteToSend)	
// 	{	
// 		rs232_putchar(*byteToSend);
// 		// Wait for transmission to complete
// 	}

// }

/*----------------------------------------------------------------
 * main -- execute terminal
 *----------------------------------------------------------------
 */
int main(int argc, char **argv)
{
	int	c;

	term_puts("\nTerminal program - Embedded Real-Time Systems\n");

	term_initio();
	rs232_open();

	term_puts("Type ^C to exit\n");

	/* discard any incoming text
	 */
	while ((c = rs232_getchar_nb()) != -1)
		fputc(c,stderr);

	/* send & receive
	 */
	for (;;)
	{
		// if ((c = term_getchar_nb()) != -1)
		// 	rs232_putchar(c);

		// if ((c = rs232_getchar_nb()) != -1)
		// 	term_putchar(c);

		packet foobar;
		foobar.startByte = 0xAA;
		foobar.length = 0x04;
		foobar.mode = 0xED;

		pc_t20_packet_tx(&foobar);
		
		if ((c = rs232_getchar_nb()) != -1)
			printf(" %d",c);
		// rs232_getchar();

		printf(" %d", rs232_getchar_nb());
	}
	

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}
