#ifndef CONTROL_H_
#define CONTROL_H_

void initialize_flight_Parameters(void);
void reset_motors(void);
void panic_mode(void);
void manual_mode(void);
void safe_mode(void);
void yaw_control_mode(void);
void calibration_mode(void);
void full_control_mode(void);
void raw_control_mode(void);
void height_control_mode(void);
void wireless_control_mode(void);

#endif