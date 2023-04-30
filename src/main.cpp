#include <Arduino.h>
#include <iostream>
#include <sstream>
#include <regex>

#include "pin_map.h"
#include "robot_constant.h"
#include "kinematics.h"
#include "motor_drive.h"
#include "ros_connect.h"
#include "multithreading.h"
#include "access_point.h"
//#include "imu.h"

#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


/* This is the main file for the CANIS_mini arduino controller
 *  
 * ####### Notes #######
 * Inverse Kinematics:
 * In regards to inverse kinematics, a research paper is linked containing the formulas used for IK, see documents/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf
 * 1. shoulder_abductor position is angle from the plane of base_link (robot body frame)
 * 2. arm_extensor position is angle forwards from the z unit vector relative to the xy plane through base_link
 * 3. forearm_extensor position is angle backwards from line through the arm joint
 * 
 */ 

//Threads
//TimedAction wifiThread = TimedAction(1000,wifiCheck);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MPU6050 mpu = Adafruit_MPU6050();

sensors_event_t a, g, temp;

double superior_right_shoulder_abductor_pos = 0;
double superior_left_shoulder_abductor_pos = 0;
double inferior_right_shoulder_abductor_pos = 0;
double inferior_left_shoulder_abductor_pos = 0;

double superior_right_arm_extensor_pos = 0;
double superior_left_arm_extensor_pos = 0;
double inferior_right_arm_extensor_pos = 0;
double inferior_left_arm_extensor_pos = 0;//1.45;

double superior_right_forearm_extensor_pos = 0; //2.32;
double superior_left_forearm_extensor_pos = 0;
double inferior_right_forearm_extensor_pos = 0;
double inferior_left_forearm_extensor_pos = 0;

double superior_right_shoulder_abductor_offset = 70; // Stores servo position offset in degrees from 0 to 180
double superior_left_shoulder_abductor_offset = -140;  // Stores servo position offset in degrees from 0 to 180
double inferior_right_shoulder_abductor_offset = -150;  // Stores servo position offset in degrees from 0 to 180
double inferior_left_shoulder_abductor_offset = -80;  // Stores servo position offset in degrees from 0 to 180

double superior_right_arm_extensor_offset = 10;  // Stores servo position offset in degrees from 0 to 180
double superior_left_arm_extensor_offset = -10;  // Stores servo position offset in degrees from 0 to 180
double inferior_right_arm_extensor_offset = 10;  // Stores servo position offset in degrees from 0 to 180
double inferior_left_arm_extensor_offset = -60;  // Stores servo position offset in degrees from 0 to 180

double superior_right_forearm_extensor_offset = -110;  // Stores servo position offset in degrees from 0 to 180
double superior_left_forearm_extensor_offset = 150;  // Stores servo position offset in degrees from 0 to 180
double inferior_right_forearm_extensor_offset = -90;  // Stores servo position offset in degrees from 0 to 180
double inferior_left_forearm_extensor_offset = 180;  // Stores servo position offset in degrees from 0 to 180

int superior_right_shoulder_abductor_pwm = 0;
int superior_left_shoulder_abductor_pwm = 0;
int inferior_right_shoulder_abductor_pwm = 0;
int inferior_left_shoulder_abductor_pwm = 0;

int superior_right_arm_extensor_pwm = 0;
int superior_left_arm_extensor_pwm = 0;
int inferior_right_arm_extensor_pwm = 0;
int inferior_left_arm_extensor_pwm = 0;

int superior_right_forearm_extensor_pwm = 0;
int superior_left_forearm_extensor_pwm = 0;
int inferior_right_forearm_extensor_pwm = 0;
int inferior_left_forearm_extensor_pwm = 0;

double superior_right_x = 0;
double superior_right_y = 0.055;
double superior_right_z = -0.15;

double superior_left_x = 0; 
double superior_left_y = 0.055; 
double superior_left_z = -0.15; 

double inferior_right_x = -0.05;
double inferior_right_y = 0.055;
double inferior_right_z = -0.15;

double inferior_left_x = -0.05; 
double inferior_left_y = 0.055; 
double inferior_left_z = -0.15; 

double shoulder_length = 0.060;
double arm_length = 0.107;
double forearm_length = 0.135;

int test_pin = -1;

int test_value = -1;

/**
 * Note a pwm message must be sent using command: rostopic pub /test_pwm std_msgs/String "'X_X'"
 * @param test_pwm_msg A message in the form X_X where x are integers correlating to the desired pin and pwm value to be sent.
 * 
 * 
*/
void test_pwm_cb(const std_msgs::String &test_pwm_msg){
  std::string data = test_pwm_msg.data;
  std::regex rgx("^([\\d]+)_([\\d]+)$");
  std::smatch base_match;
  if(!std::regex_match(data, base_match, rgx)){
    std::string bitch = "Failed to match String: "+data;
    debug_msg.data = bitch.c_str();
    debug_pub.publish(&debug_msg);
    return;
  }
  std::string message = "Writing pwm value ";
  message += base_match[2];
  message += "to pin ";
  message += base_match[1];
  debug_msg.data = message.c_str();
  debug_pub.publish(&debug_msg);
  test_pin = std::stoi(base_match[1].str());
  test_value = std::stoi(base_match[2].str());
}

void getOffset(double offset){
  inferior_left_forearm_extensor_offset = offset;
} 

void setup() {
  //Serial.println("Initting");
  init_motors();
  initROS();
  initWifi();
  //debug_msg.data = "Test";
}

void loop() {

  std::ostringstream os;
  os << superior_right_forearm_extensor_pwm;

  // debug_msg.data = "bitch\n";
  // debug_pub.publish(&debug_msg);

  //nh.spinOnce();
  //pub_imu_raw();

  //ik();
  command_motors();
  nh.spinOnce();

}


