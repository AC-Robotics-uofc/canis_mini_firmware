    
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

typedef ros::NodeHandle_<ArduinoHardware, 25, 25, 2048, 2048>
    NodeHandle; // default 25, 25, 512, 512


// #### Robot Params ####
    nh_.param<double>("/shoulder_length", shoulder_length, 0.055);
    nh_.param<double>("/arm_length", arm_length, 0.105);
    nh_.param<double>("/forearm_length", forearm_length, 0.136);
    nh_.param<double>("/body_width", body_width, 0.038);
    nh_.param<double>("/center_to_front", center_to_front, 0.1);
    nh_.param<double>("/center_to_back", center_to_back, 0.1);

    nh_.param<double>("/frequency", operating_freq, 30);
    nh_.param<double>("/walking_height", walking_z, 0.15);
    nh_.param<double>("/step_height", step_height, 0.05);

            /*
         * Robot Params (Can be passed as params)
         */

        double shoulder_length;
        double arm_length;
        double forearm_length;

        double max_extend;
        double max_tangential;

        double body_width;
        double center_to_front;
        double center_to_back;


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

extern Adafruit_PWMServoDriver pwm;
extern Adafruit_MPU6050 mpu;

extern sensors_event_t a, g, temp;

extern double superior_right_shoulder_abductor_pos;
extern double superior_left_shoulder_abductor_pos;
extern double inferior_right_shoulder_abductor_pos;
extern double inferior_left_shoulder_abductor_pos;

extern double superior_right_arm_extensor_pos;
extern double superior_left_arm_extensor_pos;
extern double inferior_right_arm_extensor_pos;
extern double inferior_left_arm_extensor_pos;

extern double superior_right_forearm_extensor_pos;
extern double superior_left_forearm_extensor_pos;
extern double inferior_right_forearm_extensor_pos;
extern double inferior_left_forearm_extensor_pos;

extern double superior_right_shoulder_abductor_offset;
extern double superior_left_shoulder_abductor_offset;
extern double inferior_right_shoulder_abductor_offset;
extern double inferior_left_shoulder_abductor_offset;

extern double superior_right_arm_extensor_offset;
extern double superior_left_arm_extensor_offset;
extern double inferior_right_arm_extensor_offset;
extern double inferior_left_arm_extensor_offset;

extern double superior_right_forearm_extensor_offset;
extern double superior_left_forearm_extensor_offset;
extern double inferior_right_forearm_extensor_offset;
extern double inferior_left_forearm_extensor_offset; 

extern int superior_right_shoulder_abductor_pwm;
extern int superior_left_shoulder_abductor_pwm;
extern int inferior_right_shoulder_abductor_pwm;
extern int inferior_left_shoulder_abductor_pwm;

extern int superior_right_arm_extensor_pwm;
extern int superior_left_arm_extensor_pwm;
extern int inferior_right_arm_extensor_pwm;
extern int inferior_left_arm_extensor_pwm;

extern int superior_right_forearm_extensor_pwm;
extern int superior_left_forearm_extensor_pwm;
extern int inferior_right_forearm_extensor_pwm;
extern int inferior_left_forearm_extensor_pwm;

extern double superior_right_x;
extern double superior_right_y;
extern double superior_right_z;

extern double superior_left_x; 
extern double superior_left_y; 
extern double superior_left_z; 

extern double inferior_right_x;
extern double inferior_right_y;
extern double inferior_right_z;

extern double inferior_left_x; 
extern double inferior_left_y; 
extern double inferior_left_z; 


extern double canis_length;
extern double canis_width;

extern double shoulder_length;
extern double arm_length;
extern double forearm_length;

shoulder_length: 0.055
arm_length: 0.105
forearm_length: 0.136
body_width: 0.038
center_to_front: 0.1
center_to_back: 0.1
frequency: 30.0
walking_height: 0.14
step_height: 0.04
leg_x_offset: -0.045
leg_x_separation: 0.07
leg_y_separation: 0.00

    private:
        // #### Gait Variables ####
        double walking_z;
        double step_height;
        double percent_step;
        double x_vel;
        double theta_vel;
        double delta_percent;