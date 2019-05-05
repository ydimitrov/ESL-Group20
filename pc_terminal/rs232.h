#ifndef RS232_H_
#define RS232_H_

void	rs232_open(void);
void 	rs232_close(void);
int		rs232_getchar_nb();
uint8_t rs232_getchar();
int 	rs232_putchar(char c);


#endif