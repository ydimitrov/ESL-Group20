#ifndef CONTROL_H_
#define CONTROL_H_

void reset_motors(void);
void panic(void);
void manual(void);
void safe(void);
void initialize_flight_Parameters(void);
void full_control(void);
void raw_control(void);
void height_control(void);
void wireless_control(void);

#endif