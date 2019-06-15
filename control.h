#ifndef CONTROL_H_
#define CONTROL_H_

void initialize_flight_Parameters(void);
void initialize_flight_Parameters_RAW(void);
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
void gradual_lift(void);
void commStatus(void);
int32_t intToFix(int32_t a);
int32_t fixToInt(int32_t a);
int32_t fpMult (int32_t a, int32_t b);
int32_t fpDiv(int32_t a, int32_t b);
void butterworth(int32_t *x, int32_t *y, int16_t sensor);

#endif