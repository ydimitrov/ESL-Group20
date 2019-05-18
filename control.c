/*------------------------------------------------------------------
 *  control.c -- here you can implement your control algorithm
 *		 and any motor clipping or whatever else
 *		 remember! motor input =  0-1000 : 125-250 us (OneShot125)
 *
 *  I. Protonotarios
 *  Embedded Software Lab
 *
 *  July 2016
 *------------------------------------------------------------------
 */

#include <inttypes.h>
#include "in4073.h"
#include "control.h"

uint8_t roll;
uint8_t pitch;
uint8_t yaw;
uint8_t lift;
//consider removing these values if there is delay
void initialize_flight_Parameters()
{
	roll 	= flightParameters.roll;
	pitch 	= flightParameters.pitch;
	yaw 	= flightParameters.yaw;
	lift 	= flightParameters.lift;
}

void update_motors(void)
{	
    for(int i=0; i<4; i++)
	{
		if(ae[i]>MAX_SPEED)
		{
			motor[i] = MAX_SPEED;
		}
		else
		{
			motor[i] = ae[i];
		}		
	}	
}

void reset_motors()
{
	ae[0] = 0;
	ae[1] = 0;
	ae[2] = 0;
	ae[3] = 0;
	update_motors();
}

void run_filters_and_control()
{

	switch(mode)
	{
		case PANIC_MODE : 
			panic();
			break;
		case SAFE_MODE :
			break;
		case MANUAL_MODE : 
			manual();
			break;
		default:
			panic();
			break;
	}
	update_motors();
}

void panic()
{
	uint16_t average = (ae[0] + ae[1] + ae[2] + ae[3])/4;
	average = average / 2;
	while(i < 1000)
	{
		ae[0] = average;
		ae[1] = average;
		ae[2] = average;
		ae[3] = average;
		i++;	
		update_motors();
	}
	while(ae[0] != 0)
	{
		ae[0] -= 10;
		ae[1] -= 10;
		ae[2] -= 10;
		ae[3] -= 10;
		update_motors(); 
	}

	mode = SAFE_MODE;	
}

void manual()
{
	int lift_status; 

	initialize_flight_Parameters();

	//set lift to uint16_t

	lift == 0 ? lift_status = 0 : lift_status = 1;

	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) + (yaw * MOTOR_RELATION)) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) - (yaw * MOTOR_RELATION)) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) + (yaw * MOTOR_RELATION)) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) - (yaw * MOTOR_RELATION)) * lift_status;
	
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = MIN_SPEED;
	if(ae[1] < MIN_SPEED) ae[1] = MIN_SPEED;
	if(ae[2] < MIN_SPEED) ae[2] = MIN_SPEED;
	if(ae[3] < MIN_SPEED) ae[3] = MIN_SPEED;

	update_motors();
}

void safe()
{
	reset_motors();
}


void calibration()
{
	panic();
	get_dmp_data();
	zp = sp;
	zq = sq;
	zr = sr;
	zax = sax;
	zay = say;
	zaz = saz;
}

void yaw_control()
{
	int lift_status; 
	int error;

	initialize_flightParameters();

	//set lift to uint16_t

	
	lift == 0 ? lift_status = 0 : lift_status = 1;
	
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) + (yaw * MOTOR_RELATION)) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) - (yaw * MOTOR_RELATION)) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) + (yaw * MOTOR_RELATION)) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) - (yaw * MOTOR_RELATION)) * lift_status;
	
	error = yaw - (sr - zr);

	ae[0] = ae[0] - (P * error);
	ae[1] = ae[1] + (P * error);
	ae[2] = ae[2] - (P * error);
	ae[3] = ae[3] + (P * error);

	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = MIN_SPEED;
	if(ae[1] < MIN_SPEED) ae[1] = MIN_SPEED;
	if(ae[2] < MIN_SPEED) ae[2] = MIN_SPEED;
	if(ae[3] < MIN_SPEED) ae[3] = MIN_SPEED;

	update_motors();
}	
