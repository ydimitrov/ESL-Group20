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

#define SHIFTMASK1 0xFFFFC000
#define SHIFTMASK2 0x3FFF
#define SHIFT 14

#define round(num) num > 0 ? ((num % 128) > 64 ? (num >> 7) + 1 : (num >> 7)) : (((num % 128) > -64) ? (num >> 7) + 1  : (num >> 7))
#define cap_base(num) num < 0 ? 0 : num
#define cap_lift(num,elevation) num > elevation ? elevation : num

//butterworth definitions
#define a0 16384	//1
#define a1 32768	//2
#define a2 16384	//1
#define b1 -6763	//-0.41280
#define b2 18727	//1.14298
#define gain 242890

//kalman definitions
#define P2PHI 377
#define Q2THETA 377
#define C1 131072 //2097152 // divide by 16 probably: 131072
#define C2 16384

int32_t roll;
int32_t pitch;
int32_t yaw;
uint32_t lift;
uint16_t commCounter = 0;
int8_t flag;

//butterworth variables
int32_t _x0 = 0;
int32_t _x1 = 0;
int32_t _x2 = 0;
int32_t _y0 = 0;
int32_t _y1 = 0;
int32_t _y2 = 0;

int32_t x_roll[3] = {0, 0, 0};
int32_t x_pitch[3] = {0, 0, 0};
int32_t x_yaw[3] = {0, 0, 0};
int32_t x_lift[3] = {0, 0, 0};
int32_t x_roll_a[3] = {0, 0, 0};
int32_t x_pitch_a[3] = {0, 0, 0};

int32_t y_roll[3] = {0, 0, 0};
int32_t y_pitch[3] = {0, 0, 0};
int32_t y_yaw[3] = {0, 0, 0};
int32_t y_lift[3] = {0, 0, 0};
int32_t y_roll_a[3] = {0, 0, 0};
int32_t y_pitch_a[3] = {0, 0, 0};

//kalman variables
int32_t p;
int32_t q;
int32_t kalman_phi;
int32_t kalman_theta;
int32_t p_b;
int32_t q_b;

//height
uint32_t prev_lift;

/*Yordan*/
void commStatus(){
    if(rx_queue.count > 0){
        commCounter = 0;
    } else {
        commCounter++;
        // printf("rx_queue.count = %d\n", rx_queue.count);
        // printf("commCounter--\n");
        // printf("commCounter = %d\n", commCounter);
    }
    
    if(commCounter == 1500 && mode != SAFE){
        commCounter = 0;
        panic_mode();
        alive = 0;
        // mode = PANIC;
        // candidate_mode = SAFE;
    }
}

/* array and enum flightmode(in4073.h) must be in sync! */

/*Yordan*/
void (* state[])(void) = {safe_mode, panic_mode, manual_mode, calibration_mode, yaw_control_mode, full_control_mode, raw_control_mode, height_control_mode, wireless_control_mode};
void (* state_fun)(void) = safe_mode; // global

enum ret_codes {fail, ok};

enum ret_codes state_transitions[9][9] = {
{fail, fail, ok,   ok,   ok,   ok,   ok,   ok,   ok},
{ok,   fail, fail, fail, fail, fail, fail, fail, fail},
{ok,   ok,   ok,   fail, fail, fail, fail, fail, fail},
{ok,   ok,   fail, fail, fail, fail, fail, fail, fail},
{ok,   ok,   fail, fail, ok,   fail, fail, fail, fail},
{ok,   ok,   fail, fail, fail, ok,   fail, fail, fail},
{ok,   ok,   fail, fail, fail, fail, ok,   fail, fail},
{ok,   ok,   fail, fail, fail, fail, fail, ok,   fail},
{ok,   ok,   fail, fail, fail, fail, fail, fail, ok}};


/*Yordan*/
enum ret_codes lookup_transitions(enum flightmode mode, enum flightmode candidate) {

    // rows correspond to current state
	// columns correspond to candidate state
    return state_transitions[mode][candidate];
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

/*
 * Function: gradual_lift
 * Author: Srinidhi Srinivasan
 * ----------------------------
 *   Ensures motor values rise gradually until lift value
 */
void gradual_lift()
{
	int32_t lift_status; 

	uint32_t MOTOR_RELATION_VALUE;

	initialize_flight_Parameters();

	if(mode == 2)
	{
		MOTOR_RELATION_VALUE = MOTOR_RELATION >>1;
	}
	else
	{
		MOTOR_RELATION_VALUE = MOTOR_RELATION;
	}

	lift_status = (lift == 0 ? 0 : 1);

	reset_motors();
	while((ae[0] + ae[1] + ae[2] + ae[3]) < ((4 * ((lift * MOTOR_RELATION_VALUE) + MIN_SPEED)) * lift_status))
	{
		ae[0] += 5;
		ae[1] += 5;
		ae[2] += 5;
		ae[3] += 5;
		ae[0] = cap_lift(ae[0], (((lift * MOTOR_RELATION_VALUE) + MIN_SPEED) * lift_status));
		ae[1] = cap_lift(ae[1], (((lift * MOTOR_RELATION_VALUE) + MIN_SPEED) * lift_status));
		ae[2] = cap_lift(ae[2], (((lift * MOTOR_RELATION_VALUE) + MIN_SPEED) * lift_status));
		ae[3] = cap_lift(ae[3], (((lift * MOTOR_RELATION_VALUE) + MIN_SPEED) * lift_status));

		update_motors();
		for (int j = 0; j < 100000; ++j)
		{
			if(j == 0)
			{
				printf("%10ld | ", get_time_us());
				printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
				printf("%d | ", mode);
				printf("%6d %6d %6d | ", phi, theta, psi);
				printf("%6d %6d %6d | ", sp, sq, sr);
				printf("%4d | %4ld | %4ld | inside\n", bat_volt, temperature, pressure);
			}
		}
	}

}
/*Yordan*/
void droneState(enum flightmode candidate) {

    enum ret_codes rc;

    rc = lookup_transitions(mode, candidate);
    
    if (rc) {
    	mode = candidate;
    	state_fun = state[mode];
    	state_fun();
    }
}

/*Yordan*/
void initialize_flight_Parameters()
{
	roll 	= flightParameters.roll;
	pitch 	= flightParameters.pitch;
	yaw 	= flightParameters.yaw;
	lift 	= flightParameters.lift;
	//consider removing these values if there is delay
}

/*
 * Function: intToFix
 * Author: Thies de Boer (4513614)
 * ----------------------------
 *   Converts an integer values to a fixed point value
 */
int32_t intToFix(int32_t a){
	return a * 16384;
}

/*Yordan*/
int32_t fixToInt(int32_t a){
	return a / 16384;
}

void run_filters_and_control(){
	
	droneState(candidate_mode);
}

/*
 * Function: panic_mode
 * Author: Srinidhi Srinivasan
 * ----------------------------
 *   Reduces motor values and goes to safe_mode
 */
void panic_mode()
{
	uint16_t average;
	average = (ae[0] + ae[1] + ae[2] + ae[3]);
	// average = average >> 3;
	average = average >> 2;
	for (int i = 0; i < 100; ++i)
	{
		ae[0] = average;
		ae[1] = average;
		ae[2] = average;
		ae[3] = average;
		i++;	
		update_motors();
		for (int j = 0; j < 100000; ++j)
		{
			if(j == 0)
			{
				printf("%10ld | ", get_time_us());
				printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
				printf("%d | ", mode);
				printf("%6d %6d %6d | ", phi, theta, psi);
				printf("%6d %6d %6d | ", sp, sq, sr);
				printf("%4d | %4ld | %4ld | inside\n", bat_volt, temperature, pressure);
			}
		}
	}
	while((ae[0]+ae[1]+ae[2]+ae[3]) != 0)
	{
		ae[0] -= 5;
		ae[1] -= 5;
		ae[2] -= 5;
		ae[3] -= 5;
		ae[0] = cap_base(ae[0]);
		ae[1] = cap_base(ae[1]);
		ae[2] = cap_base(ae[2]);
		ae[3] = cap_base(ae[3]);
		update_motors();
		for (int j = 0; j < 100000; ++j)
		{
			if(j == 0)
			{
				printf("%10ld | ", get_time_us());
				printf("%3d %3d %3d %3d | ",ae[0],ae[1],ae[2],ae[3]);
				printf("%d | ", mode);
				printf("%6d %6d %6d | ", phi, theta, psi);
				printf("%6d %6d %6d | ", sp, sq, sr);
				printf("%4d | %4ld | %4ld | inside\n", bat_volt, temperature, pressure);
			}
		}
	}
	printf("I'm done panicking bro!\n");
	// mode = SAFE;
	safe_mode();
}

/*
 * Function: manual_mode
 * Author: Srinidhi Srinivasan
 * ----------------------------
 *   Enables control of motors with plain joystick values
 */
void manual_mode()
{

	if(!flag){
		gradual_lift();
		flag = 1;
	}

	int8_t lift_status; 

	// uint8_t MOTOR_RELATION_M = MOTOR_RELATION >> 1;

	initialize_flight_Parameters();

	//set lift to uint16_t

	lift_status = (lift == 0 ? 0 : 1);

	ae[0] = (((lift * MOTOR_RELATION_M) - (pitch * MOTOR_RELATION_M) - (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	ae[1] = (((lift * MOTOR_RELATION_M) - (roll * MOTOR_RELATION_M) + (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	ae[2] = (((lift * MOTOR_RELATION_M) + (pitch * MOTOR_RELATION_M) - (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	ae[3] = (((lift * MOTOR_RELATION_M) + (roll * MOTOR_RELATION_M) + (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	
	//printf("%d %d %d %d", ae[0],ae[1],ae[2],ae[3]);
	//printf("Lift: %ld Roll: %ld Pitch: %ld Yaw: %ld\n", lift, roll, pitch, yaw);

	if(ae[0] > MAX_SPEED_M) ae[0] = MAX_SPEED_M;
	if(ae[1] > MAX_SPEED_M) ae[1] = MAX_SPEED_M;
	if(ae[2] > MAX_SPEED_M) ae[2] = MAX_SPEED_M;
	if(ae[3] > MAX_SPEED_M) ae[3] = MAX_SPEED_M;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);

	update_motors();
}

/*Yordan*/
void safe_mode()
{	
	flag = 0;
	reset_motors();
	mode = SAFE;
}

/*Nidhi*/
void calibration_mode()
{
	if(!flag){
		imu_init(true, 100);
		flag = 1;
	}

	zp = zq = zr = zax = zay = zaz = zpressure = 0;
	for (int i = 0; i < 128; i++){
		
		get_dmp_data();
		zp += sp;
		zq += sq;
		zr += sr;
		zax += sax;
		zay += say;
		zaz += saz;
		zpressure += pressure;
	}

	zp = round(zp);
	zq = round(zq);
	zr = round(zr);
	zax = round(zax);
	zay = round(zay);
	zaz = round(zaz);
	zpressure = round(zpressure);

	printf("Calibration completed!\n");
}

/*
 * Function: yaw_control_mode
 * Author: Thies de Boer (4513614)
 * ----------------------------
 *   Controls yaw rate
 */
void yaw_control_mode()
{
	//initialize sensor readout at first time in yaw mode after mode change
	if(!flag){
		imu_init(true, 100);
		gradual_lift();
		flag = 1;
	}

	int lift_status; 
	int error;

	initialize_flight_Parameters();
	get_dmp_data();
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	error = yaw - (sr - zr); //calculate yaw rate error

	//update motors based on pitch,roll,lift and control for yaw
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) + ((P * error)>>6) - (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) - ((P * error)>>6) + (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) + ((P * error)>>6) - (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) - ((P * error)>>6) + (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	
	//ensure motor speeds are within acceptable bounds
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);
	
	update_motors();
}	

/*
 * Function: full_control_mode
 * Author: Thies de Boer (4513614)
 * ----------------------------
 *   Controls pitch, roll, yaw rate using the get_dmp_data
 */
void full_control_mode()
{
	//initialize sensor readout at first time in full mode after mode change
	if(!flag){
		imu_init(true, 100);
		gradual_lift();
		flag = 1;
	}

	int lift_status; 
	int error_yawrate;
	int error_roll;
	int error_pitch;

	int K_r; //roll action
	int K_p; //pitch action

	initialize_flight_Parameters();
	get_dmp_data();	
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	//yaw rate control
	error_yawrate = yaw - (sr - zr); //calculate yaw rate error
	
	//roll control
	error_roll = roll - (phi >> 8); //calculate roll error
	K_r = (4 * P1) * error_roll - (P2 * (sp - zp)>>2); //terms based on roll angle and rollrate error added

	//pitch control
	error_pitch = -(pitch - (theta >> 8)); //calculate pitch error
	K_p = (4 * P1) * error_pitch - (P2 * (sq - zq)>>2); //terms based on pitch angle and pitchrate error added

	//update motors based on lift and control for pitch, roll and yaw rate
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) - (K_p>>6) + ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) - (K_r>>6) - ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) + (K_p>>6) + ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) + (K_r>>6) - ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;

	//ensure motor speeds are within acceptable bounds
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);
	
	update_motors();
}

/*
 * Function: raw_control_mode
 * Author: Thies de Boer (4513614)
 * ----------------------------
 *   Controls pitch, roll, yaw rate using the raw sensor data
 */
void raw_control_mode(){
	//initialize sensor readout at first time in raw mode after mode change
	if(!flag){
		gradual_lift();
		imu_init(false,50);
		flag = 1;
	}
		
	int32_t lift_status; 
	int32_t error_yawrate;
	int32_t error_roll;
	int32_t error_pitch;
	
	int32_t K_r; //roll action
	int32_t K_p; //pitch action

	initialize_flight_Parameters();
	
	//get data from sensors and filter them using Butterworth and Kalman filtering
	get_raw_sensor_data();
	butterworth(x_yaw, y_yaw, sr);
	//butterworth(x_pitch, y_pitch, sq);
	y_roll[2] = intToFix(sp);
	y_pitch[2] = intToFix(sq);
	//butterworth(x_roll, y_roll, sp);
	butterworth(x_pitch_a, y_pitch_a, sax);
	butterworth(x_roll_a, y_roll_a, say);
	kalman();
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	//yaw rate control
	error_yawrate = intToFix(yaw) - y_yaw[2]; //calculate yaw rate error

	//roll control
	error_roll = roll - (phi >> 2); //calculate roll error
	K_r = (4 * P1) * error_roll - ((P2) * fixToInt(p)); //terms based on roll angle and rollrate error added

	//pitch control
	error_pitch = -(pitch - (theta >> 2)); //calculate pitch error
	K_p = (4 * P1) * error_pitch - ((P2) * fixToInt(q)); //terms based on pitch angle and pitchrate error added

	//update motors based on lift and control for pitch,roll,yaw rate
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) - (K_p>>6) + fixToInt(((fixed_mul_14(intToFix(P), error_yawrate)))) + MIN_SPEED) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) - (K_r>>6) - fixToInt(((fixed_mul_14(intToFix(P), error_yawrate)))) + MIN_SPEED) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) + (K_p>>6) + fixToInt(((fixed_mul_14(intToFix(P), error_yawrate)))) + MIN_SPEED) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) + (K_r>>6) - fixToInt(((fixed_mul_14(intToFix(P), error_yawrate)))) + MIN_SPEED) * lift_status;

	//ensure motor speeds are within acceptable bounds
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);

	update_motors();
}

/*Yordan*/
int32_t fixed_div_14(int32_t x, int32_t y){
    return ((int64_t)x * (1 << SHIFT)) / y;
}

/*Yordan*/
int32_t fixed_mul_14(int32_t x, int32_t y){
    return ((int64_t)x * (int64_t)y) / (1 << SHIFT);
}

/*Nidhi*/
void butterworth(int32_t *x, int32_t *y, int32_t sensor){

	// sensor = sensor >> 6;
	
	
		sensor >>= 6;
		x[0] = x[1];
		x[1] = x[2];
		x[2] = fixed_div_14(intToFix(sensor), gain);


		y[0] = y[1]; 
		y[1] = y[2]; 

		_x0 = x[0];
		_x1 = x[1];
		_x2 = x[2];

		_y1 = y[1];
		_y0 = y[0];



		_y2 = fixed_mul_14(a0,_x0) + fixed_mul_14(a1,_x1) + fixed_mul_14(a2,_x2) + fixed_mul_14(b1,_y0) + fixed_mul_14(b2,_y1);
		y[2] = _y2;

}

/*
 * Function: kalman
 * Author: Thies de Boer (4513614)
 * ----------------------------
 *   uses gyrometer and accelerometer sensors to get more accurate values for angles and rates
 */
void kalman()
{
	//Kalman filter for theta (ax) and rate of theta (q)
	q = y_pitch[2] - q_b;
	kalman_theta = kalman_theta + fixed_mul_14(q,Q2THETA);
	kalman_theta = kalman_theta - fixed_div_14((kalman_theta - y_pitch_a[2]), (C1));
	q_b = q_b + fixed_div_14(fixed_div_14((kalman_theta - y_pitch_a[2]),(intToFix(1000))),intToFix(1000));

	theta = fixToInt(kalman_theta);

	//Kalman filter for phi (ay) and rate of phi (p)
	p = y_roll[2] - p_b;
	kalman_phi = kalman_phi + fixed_mul_14(p,P2PHI);
	kalman_phi = kalman_phi - fixed_div_14((kalman_phi - y_roll_a[2]), (C1));
	p_b = p_b + fixed_div_14(fixed_div_14((kalman_phi - y_roll_a[2]),(intToFix(1000))),intToFix(1000));

	phi = fixToInt(kalman_phi);

}

/*
 * Function: gradual_lift
 * Author: Srinidhi Srinivasan
 * ----------------------------
 *   Ensures stability in height by controlling the pressure sensor through the butterworth filter
 */
void height_control_mode(){
	
	if(!flag){
		imu_init(false, 50);
		gradual_lift();
		flag = 1;
	}

	initialize_flight_Parameters();
	get_raw_sensor_data();
	int8_t lift_status; 
	int32_t error_lift;
	
	error_lift = pressure - zpressure;
	butterworth(x_lift,y_lift,(error_lift<<5));	


	lift_status = (lift == 0 ? 0 : 1);

	ae[0] = (((lift * MOTOR_RELATION_M) + P * fixToInt(y_lift[2]) - (pitch * MOTOR_RELATION_M) - (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	ae[1] = (((lift * MOTOR_RELATION_M) + P * fixToInt(y_lift[2]) - (roll * MOTOR_RELATION_M) + (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	ae[2] = (((lift * MOTOR_RELATION_M) + P * fixToInt(y_lift[2]) + (pitch * MOTOR_RELATION_M) - (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);
	ae[3] = (((lift * MOTOR_RELATION_M) + P * fixToInt(y_lift[2]) + (roll * MOTOR_RELATION_M) + (yaw * MOTOR_RELATION_M) + MIN_SPEED) * lift_status);


	if(ae[0] > MAX_SPEED_M) ae[0] = MAX_SPEED_M;
	if(ae[1] > MAX_SPEED_M) ae[1] = MAX_SPEED_M;
	if(ae[2] > MAX_SPEED_M) ae[2] = MAX_SPEED_M;
	if(ae[3] > MAX_SPEED_M) ae[3] = MAX_SPEED_M;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);

	update_motors();
	//printf("pres: %ld error_f = %ld P = %d error=%ld motors: %d %d %d %d\n",pressure,fixToInt(y_lift[2]),P,error_lift,ae[0],ae[1],ae[2],ae[3]);  
}	

/**/
void wireless_control_mode(){

}
