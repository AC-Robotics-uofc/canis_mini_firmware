#include "motor_drive.h"
#include "robot_constant.h"
#include "pin_map.h"
#include "../include/ros_connect.h"

#define SERVOMIN  60 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  520 // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

uint8_t servonum = 0;



// See this lib: https://learn.adafruit.com/16-channel-pwm-servo-driver?view=all
//pulselength = map(degrees, 0, 180, SERVOMIN, SERVOMAX);  for ease of use

void init_motors() {

    pwm.begin();
    //Serial.println("Initting");
    pwm.setOscillatorFrequency(27000000); //set w/ ossicloscope?
    pwm.setPWMFreq(SERVO_FREQ);

}

void command_motors() {

    superior_right_shoulder_abductor_pwm = map2(superior_right_shoulder_abductor_pos, -1.104, 1.2736, 160, 440) + superior_right_shoulder_abductor_offset;
    superior_left_shoulder_abductor_pwm = map2(superior_left_shoulder_abductor_pos, -1.104, 1.2736, 420, 140) + superior_left_shoulder_abductor_offset;
    inferior_right_shoulder_abductor_pwm = map2(inferior_right_shoulder_abductor_pos, -1.104, 1.2736, 420, 140) + inferior_right_shoulder_abductor_offset;
    inferior_left_shoulder_abductor_pwm = map2(inferior_left_shoulder_abductor_pos, -1.104, 1.2736, 160, 440) + inferior_left_shoulder_abductor_offset;

    superior_right_arm_extensor_pwm = map2(superior_right_arm_extensor_pos, -1.7831, 1.6133, 500, 100) + superior_right_arm_extensor_offset;
    superior_left_arm_extensor_pwm = map2(superior_left_arm_extensor_pos, -1.7831, 1.6133, 80, 480) + superior_left_arm_extensor_offset;
    inferior_right_arm_extensor_pwm = map2(inferior_right_arm_extensor_pos, -1.4434, 1.6982, 460, 90) + inferior_right_arm_extensor_offset;
    inferior_left_arm_extensor_pwm = map2(inferior_left_arm_extensor_pos, -1.4434, 1.6982, 120, 490) + inferior_left_arm_extensor_offset;

    superior_right_forearm_extensor_pwm = map2(superior_right_forearm_extensor_pos, -0.933987, 2.7171, 180, 610) + superior_right_forearm_extensor_offset;
    superior_left_forearm_extensor_pwm = map2(superior_left_forearm_extensor_pos, 0, 2.80196, 290, -40) + superior_left_forearm_extensor_offset;
    inferior_right_forearm_extensor_pwm = map2(inferior_right_forearm_extensor_pos, -0.2547, 2.8869, 260, 630) + inferior_right_forearm_extensor_offset;
    inferior_left_forearm_extensor_pwm = map2(inferior_left_forearm_extensor_pos, -0.33963, 2.46233, 330, 0) + inferior_left_forearm_extensor_offset;


    pwm.setPWM(0, 0, superior_right_shoulder_abductor_pwm);
    pwm.setPWM(1, 0, superior_left_shoulder_abductor_pwm);
    pwm.setPWM(2, 0, inferior_right_shoulder_abductor_pwm);
    pwm.setPWM(3, 0, inferior_left_shoulder_abductor_pwm);

    pwm.setPWM(4, 0, superior_right_arm_extensor_pwm);
    pwm.setPWM(5, 0, superior_left_arm_extensor_pwm);
    pwm.setPWM(6, 0, inferior_right_arm_extensor_pwm);
    pwm.setPWM(7, 0, inferior_left_arm_extensor_pwm);

    pwm.setPWM(8, 0, superior_right_forearm_extensor_pwm);
    pwm.setPWM(9, 0, superior_left_forearm_extensor_pwm);
    pwm.setPWM(10, 0, inferior_right_forearm_extensor_pwm);
    pwm.setPWM(11, 0, inferior_left_forearm_extensor_pwm);

}

void superior_right_shoulder_abductor_cb(const std_msgs::Float64 &superior_right_shoulder_pos_msg) {
    superior_right_shoulder_abductor_pwm = map2(superior_right_shoulder_pos_msg.data, -1.104, 1.2736, 160, 440) + superior_right_shoulder_abductor_offset;
    pwm.setPWM(0, 0, superior_right_shoulder_abductor_pwm);
}
void superior_left_shoulder_abductor_cb(const std_msgs::Float64 &superior_left_shoulder_pos_msg) {
    superior_left_shoulder_abductor_pwm = map2(superior_left_shoulder_pos_msg.data, -1.104, 1.2736, 420, 140) + superior_left_shoulder_abductor_offset;
    pwm.setPWM(1, 0, superior_left_shoulder_abductor_pwm);
}
void inferior_right_shoulder_abductor_cb(const std_msgs::Float64 &inferior_right_shoulder_pos_msg) {
    inferior_right_shoulder_abductor_pwm = map2(inferior_right_shoulder_pos_msg.data, -1.104, 1.2736, 420, 140) + inferior_right_shoulder_abductor_offset;
    pwm.setPWM(2, 0, inferior_right_shoulder_abductor_pwm);
}
void inferior_left_shoulder_abductor_cb(const std_msgs::Float64 &inferior_left_shoulder_pos_msg) {
    inferior_left_shoulder_abductor_pwm = map2(inferior_left_shoulder_pos_msg.data, -1.104, 1.2736, 160, 440) + inferior_left_shoulder_abductor_offset;
    pwm.setPWM(3, 0, inferior_left_shoulder_abductor_pwm);
}

void superior_right_arm_extensor_cb(const std_msgs::Float64 &superior_right_arm_pos_msg) {
    superior_right_arm_extensor_pwm = map2(superior_right_arm_pos_msg.data, -1.7831, 1.6133, 500, 100) + superior_right_arm_extensor_offset;
    pwm.setPWM(4, 0, superior_right_arm_extensor_pwm);
}
void superior_left_arm_extensor_cb(const std_msgs::Float64 &superior_left_arm_pos_msg) {
    superior_left_arm_extensor_pwm = map2(superior_left_arm_pos_msg.data, -1.7831, 1.6133, 80, 480) + superior_left_arm_extensor_offset;
    pwm.setPWM(5, 0, superior_left_arm_extensor_pwm);
}
void inferior_right_arm_extensor_cb(const std_msgs::Float64 &inferior_right_arm_pos_msg) {
    inferior_right_arm_extensor_pwm = map2(inferior_right_arm_pos_msg.data, -1.4434, 1.6982, 460, 90) + inferior_right_arm_extensor_offset;
    pwm.setPWM(6, 0, inferior_right_arm_extensor_pwm);
}
void inferior_left_arm_extensor_cb(const std_msgs::Float64 &inferior_left_arm_pos_msg) {
    inferior_left_arm_extensor_pwm = map2(inferior_left_arm_pos_msg.data, -1.4434, 1.6982, 120, 490) + inferior_left_arm_extensor_offset;
    pwm.setPWM(7, 0, inferior_left_arm_extensor_pwm);
}

void superior_right_forearm_extensor_cb(const std_msgs::Float64 &superior_right_forearm_pos_msg) {
    superior_right_forearm_extensor_pwm = map2(superior_right_forearm_pos_msg.data, -0.933987, 2.7171, 180, 610) + superior_right_forearm_extensor_offset;
    pwm.setPWM(8, 0, superior_right_forearm_extensor_pwm);
}
void superior_left_forearm_extensor_cb(const std_msgs::Float64 &superior_left_forearm_pos_msg) {
    superior_left_forearm_extensor_pwm = map2(superior_left_forearm_pos_msg.data, 0, 2.80196, 290, -40) + superior_left_forearm_extensor_offset;
    pwm.setPWM(9, 0, superior_left_forearm_extensor_pwm);
}
void inferior_right_forearm_extensor_cb(const std_msgs::Float64 &inferior_right_forearm_pos_msg) {
    inferior_right_forearm_extensor_pwm = map2(inferior_right_forearm_pos_msg.data, -0.2547, 2.8869, 260, 630) + inferior_right_forearm_extensor_offset;
    pwm.setPWM(10, 0, inferior_right_forearm_extensor_pwm);
}
void inferior_left_forearm_extensor_cb(const std_msgs::Float64 &inferior_left_forearm_pos_msg) {
    inferior_left_forearm_extensor_pwm = map2(inferior_left_forearm_pos_msg.data, -0.33963, 2.46233, 330, 0) + inferior_left_forearm_extensor_offset;
    pwm.setPWM(11, 0, inferior_left_forearm_extensor_pwm);
}

double map2(double input, double x1, double x2, double y1, double y2) {
    if (input <= x1) return y1;
    if (input >= x2) return y2;
    double slope = (y2 - y1) / (x2 - x1);
    return (slope * (input - x1) + y1);
}