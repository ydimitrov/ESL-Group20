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
#include <ctype.h>
#include <fcntl.h>
#include <assert.h>
#include <time.h>
#include "pc_t20.h"
#include "rs232.h"
#include "consoleIO.h"

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
			// term_putchar(c);
			printf(" %d",c);
		// rs232_getchar();

		// term_putchar(rs232_getchar_nb());
	}
	

	term_exitio();
	rs232_close();
	term_puts("\n<exit>\n");

	return 0;
}
