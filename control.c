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

#define a0 16384
#define a1 32768
#define a2 16384
#define b1 -6763
#define b2 18727

int32_t roll;
int32_t pitch;
int32_t yaw;
uint32_t lift;
uint16_t commCounter = 0;
int8_t flag;

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

int32_t y_roll[3] = {0, 0, 0};
int32_t y_pitch[3] = {0, 0, 0};
int32_t y_yaw[3] = {0, 0, 0};
int32_t y_lift[3] = {0, 0, 0};



void commStatus(){
    if(rx_queue.count > 0){
        commCounter = 0;
    } else {
        commCounter++;
        printf("rx_queue.count = %d\n", rx_queue.count);
        printf("commCounter--\n");
        printf("commCounter = %d\n", commCounter);
    }
    
    if(commCounter == 500 && mode != SAFE){
        commCounter = 0;
        panic_mode();
        alive = 0;
        // mode = PANIC;
        // candidate_mode = SAFE;
    }
}

/* array and enum flightmode(in4073.h) must be in sync! */
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

void droneState(enum flightmode candidate) {

    enum ret_codes rc;

    rc = lookup_transitions(mode, candidate);
    
    if (rc) {
    	mode = candidate;
    	state_fun = state[mode];
    	state_fun();
    }
}


void initialize_flight_Parameters()
{
	roll 	= flightParameters.roll;
	pitch 	= flightParameters.pitch;
	yaw 	= flightParameters.yaw;
	lift 	= flightParameters.lift;
	//consider removing these values if there is delay
}

void initialize_flight_Parameters_RAW()
{
	roll 	= intToFix(flightParameters.roll);
	pitch 	= intToFix(flightParameters.pitch);
	yaw 	= intToFix(flightParameters.yaw);
	lift 	= intToFix(flightParameters.lift);
	//consider removing these values if there is delay
}

int32_t intToFix(int32_t a){
	return a * 16384;
}

int32_t fixToInt(int32_t a){
	return a / 16384;
}

void run_filters_and_control(){
	
	droneState(candidate_mode);
}

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
	mode = SAFE;
}

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
	
	printf("%d %d %d %d", ae[0],ae[1],ae[2],ae[3]);
	printf("Lift: %ld Roll: %ld Pitch: %ld Yaw: %ld\n", lift, roll, pitch, yaw);

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

void safe_mode()
{	
	flag = 0;
	reset_motors();
}


void calibration_mode()
{
	if(!flag){
		imu_init(true, 100);
		flag = 1;
	}

	zp = zq = zr = zax = zay = zaz = 0;
	for (int i = 0; i < 128; i++){
		
		get_dmp_data();
		zp += sp;
		zq += sq;
		zr += sr;
		zax += sax;
		zay += say;
		zaz += saz;
	}

	zp = round(zp);
	zq = round(zq);
	zr = round(zr);
	zax = round(zax);
	zay = round(zay);
	zaz = round(zaz);

	printf("Calibration completed!\n");
}

void yaw_control_mode()
{

	if(!flag){
		imu_init(true, 100);
		gradual_lift();
		flag = 1;
	}

	int lift_status; 
	int error;

	initialize_flight_Parameters();
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	error = yaw - (sr - zr); //calculate yaw rate error

	//update motors based on pitch,roll,lift and control for yaw
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - ((P * error)>>8) - (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + ((P * error)>>8) + (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - ((P * error)>>8) - (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + ((P * error)>>8) + (yaw * MOTOR_RELATION) + MIN_SPEED) * lift_status;

	// printf("Error = %d Yaw = %d P = %d\n",error,yaw,P);
	
	//ensure motor speeds are within acceptable bounds
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);

	// printf("Motor speeds: %3d %3d %3d %3d\n", ae[0], ae[1], ae[2], ae[3]);
	
	update_motors();
}	


void full_control_mode()
{
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
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	//yaw rate control
	error_yawrate = yaw - (sr - zr); //calculate yaw rate error
	
	//roll control
	error_roll = roll - (phi >> 8); //calculate roll error
	K_r = (4 * P1) * error_roll - (P2 * (sp - zp)>>2); //integrate terms based on roll and rollrate error added

	//pitch control
	error_pitch = -(pitch - (theta >> 8)); //calculate pitch error
	K_p = (4 * P1) * error_pitch - (P2 * (sq - zq)>>2); //integrate terms based on pitch and pitchrate error added

	//update motors based on lift and control for pitch,roll,yaw rate
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) - (K_p>>6) - ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) - (K_r>>6) + ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) + (K_p>>6) - ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) + (K_r>>6) + ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;

	//ensure motor speeds are within acceptable bounds
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);

	// printf("pitch = %d, error_p = %d, K_p = %d, roll = %d, error_r = %d, K_r = %d, P = %d, P1 = %d, P2 = %d, sp = %d, sq = %d\n",pitch,error_pitch,K_p,roll,error_roll,K_r,P, P1,P2,sp,sq);
	// printf("pitch = %d, roll = %d,  P = %d, P1 = %d, P2 = %d\n", pitch, roll, P, P1, P2);
	// printf("%3d %3d %3d %3d \n", ae[0], ae[1], ae[2], ae[3]);	
	
	update_motors();
}

void raw_control_mode(){


	if(!flag){
		imu_init(false, 1000);
		gradual_lift();
		flag = 1;
	}
		
	int32_t lift_status; 
	int32_t error_yawrate;
	int32_t error_roll;
	int32_t error_pitch;
	
	int32_t K_r; //roll action
	int32_t K_p; //pitch action

	initialize_flight_Parameters_RAW();

	butterworth(x_yaw, y_yaw, sr);
	// kalmanFilter();
	
	lift_status = (lift == 0 ? 0 : 1);	
	
	//yaw rate control
	error_yawrate = yaw - y_yaw[0]; //calculate yaw rate error
	
	//roll control with kalman
	// p = sp - p_b;
	// phi = phi + p * P2PHI;
	// phi = phi - (phi - say) / C1;
	// p_b = p_b + (phi - say) / C2;

	error_roll = roll - (phi >> 8); //calculate roll error
	K_r = (4 * P1) * error_roll - (P2 * (sp - zp)>>2); //integrate terms based on roll and rollrate error added

	//pitch control with kalman
	// q = sq - q_b;
	// theta = theta + q * P2THETA;
	// theta = theta - (theta - sax) / C1;
	// q_b = q_b + (theta - sax) / C2;

	error_pitch = -(pitch - (theta >> 8)); //calculate pitch error
	K_p = (4 * P1) * error_pitch - (P2 * (sq - zq)>>2); //integrate terms based on pitch and pitchrate error added

	//update motors based on lift and control for pitch,roll,yaw rate
	ae[0] = ((lift * MOTOR_RELATION) - (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) - (K_p>>6) - ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[1] = ((lift * MOTOR_RELATION) - (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) - (K_r>>6) + ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[2] = ((lift * MOTOR_RELATION) + (pitch * MOTOR_RELATION) - (yaw * MOTOR_RELATION) + (K_p>>6) - ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;
	ae[3] = ((lift * MOTOR_RELATION) + (roll * MOTOR_RELATION) + (yaw * MOTOR_RELATION) + (K_r>>6) + ((P * error_yawrate)>>6) + MIN_SPEED) * lift_status;

	//ensure motor speeds are within acceptable bounds
	if(ae[0] > MAX_SPEED) ae[0] = MAX_SPEED;
	if(ae[1] > MAX_SPEED) ae[1] = MAX_SPEED;
	if(ae[2] > MAX_SPEED) ae[2] = MAX_SPEED;
	if(ae[3] > MAX_SPEED) ae[3] = MAX_SPEED;

	if(ae[0] < MIN_SPEED) ae[0] = (MIN_SPEED * lift_status);
	if(ae[1] < MIN_SPEED) ae[1] = (MIN_SPEED * lift_status);
	if(ae[2] < MIN_SPEED) ae[2] = (MIN_SPEED * lift_status);
	if(ae[3] < MIN_SPEED) ae[3] = (MIN_SPEED * lift_status);

	// printf("pitch = %d, error_p = %d, K_p = %d, roll = %d, error_r = %d, K_r = %d, P = %d, P1 = %d, P2 = %d, sp = %d, sq = %d\n",pitch,error_pitch,K_p,roll,error_roll,K_r,P, P1,P2,sp,sq);
	// printf("pitch = %d, roll = %d,  P = %d, P1 = %d, P2 = %d\n", pitch, roll, P, P1, P2);
	// printf("%3d %3d %3d %3d \n", ae[0], ae[1], ae[2], ae[3]);	
	
	update_motors();
}

int32_t fpMult (int32_t a, int32_t b){

    // printf("a                  = "); printBits(sizeof(a), &a);
    // printf("b                  = "); printBits(sizeof(b), &b);
    int32_t wPartA, wPartB;
    int32_t dPartA, dPartB;
    int32_t sum1, sum2, sum3, sum4;

    wPartA = a & SHIFTMASK1;
    wPartB = b & SHIFTMASK1;
    dPartA = a & SHIFTMASK2;
    dPartB = b & SHIFTMASK2;

    sum1 = (wPartA >> SHIFT) * (wPartB >> SHIFT) << SHIFT;
    // printf("sum1 = "); printBits(sizeof(sum1), &sum1);
    sum2 = (wPartA * dPartB) >> SHIFT;
    // printf("sum2 = "); printBits(sizeof(sum2), &sum2);
    sum3 = (dPartA * wPartB) >> SHIFT;
    // printf("sum3 = "); printBits(sizeof(sum3), &sum3);
    sum4 = (dPartA * dPartB) >> SHIFT;
    // printf("sum4 = "); printBits(sizeof(sum4), &sum4);

    return sum1 + sum2 + sum3 + sum4;
} 

int32_t fpDiv(int32_t a, int32_t b)
{
    int64_t temp;
    temp = (int64_t) a << SHIFT;
    return (int32_t) (temp / b);
}


void butterworth(int32_t *x, int32_t *y, int16_t sensor){

	x[2] = x[1];
	x[1] = x[0];
	x[0] = intToFix(sensor);

	y[2] = y[1]; 
	y[1] = y[0]; 

	_x0 = x[0];
	_x1 = x[1];
	_x2 = x[2];

	_y1 = y[1];
	_y2 = y[2];

	_y0 = fpMult(a0,_x0) + fpMult(a1,_x1)+fpMult(a2,_x2) - fpMult(b1,_y1)-fpMult(b2,_y2);
	y[0] = _y0;
}


void height_control_mode(){

}

void wireless_control_mode(){

}