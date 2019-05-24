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

int8_t roll;
int8_t pitch;
int8_t yaw;
uint8_t lift;
int int_error = 0; //integral of error, needed for yaw control

void initialize_flight_Parameters()
{
	roll 	= flightParameters.roll;
	pitch 	= flightParameters.pitch;
	yaw 	= flightParameters.yaw;
	lift 	= flightParameters.lift;
	//consider removing these values if there is delay
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
		case CALIBRATION_MODE:
			calibration();
			break;
		case YAW_MODE:
			yaw_control();
			break;
		case FULL_MODE:
			full_control();
			break;
		case RAW_MODE:
			raw_control();
			break;
		case HEIGHT_MODE:
			height_control();
			break;
		case WIRELESS_MODE:
			wireless_control();
			break;
		default:
			panic();
			break;
	}
	update_motors();
}

void panic()
{
	uint16_t i, average = (ae[0] + ae[1] + ae[2] + ae[3]);
	average = average >> 3;
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
	int8_t lift_status; 

	initialize_flight_Parameters();

	//set lift to uint16_t

	lift_status = (lift == 0 ? 0 : 1);

	ae[0] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION)) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION)) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION)) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION)) * lift_status;
	
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
	// readLog();
}


void calibration()
{
	panic();
	zp = zq = zr = zax = zay = zaz = 0;
	//get_dmp_data();
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

	initialize_flight_Parameters();
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	error = yaw - (sr - zr); //calculate yaw rate error
	int_error = int_error + error; //integrate error

	//update motors based on pitch,roll,lift and control for yaw
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - (P * int_error)) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + (P * int_error)) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - (P * int_error)) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + (P * int_error)) * lift_status;

	//ensure motor speeds are within acceptable bounds
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


void full_control(){

}

void raw_control(){

}

void height_control(){

}

void wireless_control(){

}