#ifndef MOTOR_DRIVE
#define MOTOR_DRIVE
extern int test_pin;

extern int test_value;


void init_motors();

void command_motors();

double map2(double input, double x1, double x2, double y1, double y2);
#endif