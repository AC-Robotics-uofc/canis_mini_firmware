
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

