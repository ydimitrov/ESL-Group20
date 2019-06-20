/*------------------------------------------------------------------
 *  in4073.c -- test QR engines and sensors
 *
 *  reads ae[0-3] uart rx queue
 *  (q,w,e,r increment, a,s,d,f decrement)
 *
 *  prints timestamp, ae[0-3], sensors to uart tx queue
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  June 2016
 *------------------------------------------------------------------
 */

#include "in4073.h"
#include "dr_t20.h"
#include "fsmReceive.h"
#include "control.h"
#include "data_logging.h"


/*------------------------------------------------------------------
 * main -- everything you need is here :)
 *------------------------------------------------------------------
 */

int main(void)
{
	uart_init();
	gpio_init();
	timers_init();
	adc_init();
	twi_init();
	imu_init(true, 100);
	baro_init();
	spi_flash_init();
	flash_chip_erase(); // Erase flash chip
	// ble_init();
	
	mode = SAFE;
	P = P1 = P2 = 0;
	alive = 1;
	uint32_t counter = 0;

	Packet teleTx;

	while (alive)
	{
		if (check_timer_flag()) 
		{
			if (counter++%20 == 0) 
			{
				nrf_gpio_pin_toggle(BLUE);
			}

			adc_request_sample();
			read_baro();

			//printf("%10ld | ", get_time_us());
			//printf("%3d %3d %3d %3d | ", ae[0], ae[1], ae[2], ae[3]);
			//printf("%d | ", mode);
			//printf("%6d %6d %6d | ", phi, theta, psi);
			//printf("%6d %6d %6d | ", sp, sq, sr);
			//printf("%4d | %4ld | %4ld |", bat_volt, temperature, pressure);
			//printf("P = %3d | P1 = %3d | P2 = %3d |\n", P, P1, P2);
			
			clear_timer_flag();
		}

		fsmReceive();

		/**
		 * Data logging
		 * @author Thomas Makryniotis
		 */
		// Write telemetry data to SPI Flash

		uint32_t sys_time = get_time_us(); // Get current system time

		// Create array to be logged
		uint8_t data[20];
		data[0] = (uint8_t)((sys_time & 0xFF000000) >> 24);
		data[1] = (uint8_t)((sys_time & 0x00FF0000) >> 16);
		data[2] = (uint8_t)((sys_time & 0x0000FF00) >> 8);
		data[3] = (uint8_t)((sys_time & 0x000000FF));
		data[4] = ae[0];
		data[5] = ae[1];
		data[6] = ae[2];
		data[7] = ae[3];
		data[8] = phi;
		data[9] = theta;
		data[10] = psi;
		data[11] = sp;
		data[12] = sq;
		data[13] = sr;
		data[14] = temperature;
		data[15] = pressure;
		data[16] = bat_volt;
		data[17] = sax;
		data[18] = say;
		data[19] = saz;

		// Do the actual write

		if(counter%10 == 0) {
		 	writeLog(&data[0], 20);
		}
		
		/**
		 * Trasmit telemetry data
		 * @author Thomas Makryniotis
		 */

		if(counter%40 == 0) {	
			teleTx = dr_t20_packet_init((uint16_t)ae[0], (uint16_t)ae[1], 
						    (uint16_t)ae[2], (uint16_t)ae[3], 
						    phi, theta, psi, 
						    sp, sq, sr, 
						    temperature, bat_volt, pressure);
			teleTx.crc = 0xBB; // Used as an end byte rather than a CRC
			dr_t20_packet_tx(&teleTx);
		}

		if (check_sensor_int_flag()) {
			
			get_dmp_data();
			run_filters_and_control();
			commStatus();
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
