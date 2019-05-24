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
	// ble_init();

	uint32_t counter = 0;
	demo_done = false;

	while (!demo_done)
	{
		// if (rx_queue.count) {
		// 	process_key( dequeue(&rx_queue) );
		// }

		if (check_timer_flag()) 
		{
			if (counter++%20 == 0) 
				{
					nrf_gpio_pin_toggle(BLUE);
				}

			adc_request_sample();
			// read_baro();

			printf("%10ld | ", get_time_us());
			printf("%3d %3d %3d %3d | ", ae[0], ae[1], ae[2], ae[3]);
			printf("%6d %6d %6d | ", phi, theta, psi);
			printf("%6d %6d %6d | ", sp, sq, sr);
			printf("%4d | %4ld | %6ld \n", bat_volt, temperature, pressure);

			clear_timer_flag();

		}

		fsmReceive();
		


		if (check_sensor_int_flag()) 
		{
			get_dmp_data();
			run_filters_and_control();
		}
	}	

	printf("\n\t Goodbye \n\n");
	nrf_delay_ms(100);

	NVIC_SystemReset();
}
