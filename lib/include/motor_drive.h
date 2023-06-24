#ifndef MOTOR_DRIVE
#define MOTOR_DRIVE
#include "stdint.h"

class MotorDriver{

    static uint8_t test_pin;
    static uint16_t test_value;

    void init_motors();

    /**
     * Grants manual control over motor positions.
     * If test pin in range 0-12 update the specified pin under test with the pwm value indicated.
     * 
    */
    void test_position_update();

    /**
     * Updates the motor positons by calculating the pwm for a desired angle and setting each pin the the correct pwm.
    */
    void command_motors();

    void test_pwm_cb(int test_pwm);


    /**
     * Takes an input angle and maps it to an output pwm in within the specified range
     * @param input The input angle in radians
     * @param x1 The minimum angle allowed by the motor
     * @param x2 The maximum angle allowed by the motor
     * @param y1 The minumum pwm allowed by the motor
     * @param y2 The maximum pwm allowed by the motor
     * @return The mapped pwm within range of y1 and y2
     * 
    */
    double map2(double input, double x1, double x2, double y1, double y2);
};
#endif