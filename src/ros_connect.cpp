#include "../include/ros_connect.h"

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/tf.h>

NodeHandle nh;

std_msgs::Float64 superior_right_shoulder_pos_msg;
std_msgs::Float64 superior_left_shoulder_pos_msg;
std_msgs::Float64 inferior_right_shoulder_pos_msg;
std_msgs::Float64 inferior_left_shoulder_pos_msg;

std_msgs::Float64 superior_right_arm_pos_msg;
std_msgs::Float64 superior_left_arm_pos_msg;
std_msgs::Float64 inferior_right_arm_pos_msg;
std_msgs::Float64 inferior_left_arm_pos_msg;

std_msgs::Float64 superior_right_forearm_pos_msg;
std_msgs::Float64 superior_left_forearm_pos_msg;
std_msgs::Float64 inferior_right_forearm_pos_msg;
std_msgs::Float64 inferior_left_forearm_pos_msg;

std_msgs::String debug_msg;
sensor_msgs::Imu imu_msg;

ros::Publisher debug_pub("/debug", &debug_msg);
ros::Publisher imu_pub("/imu/data_raw", &imu_msg);

ros::Subscriber<std_msgs::Float64> superior_right_shoulder_abductor_sub("/actuation/leg/shoulder/superior/right", &superior_right_shoulder_abductor_cb);
ros::Subscriber<std_msgs::Float64> superior_left_shoulder_abductor_sub("/actuation/leg/shoulder/superior/left", &superior_left_shoulder_abductor_cb);
ros::Subscriber<std_msgs::Float64> inferior_right_shoulder_abductor_sub("/actuation/leg/shoulder/inferior/right", &inferior_right_shoulder_abductor_cb);
ros::Subscriber<std_msgs::Float64> inferior_left_shoulder_abductor_sub("/actuation/leg/shoulder/inferior/left", &inferior_left_shoulder_abductor_cb);

ros::Subscriber<std_msgs::Float64> superior_right_arm_extensor_sub("/actuation/leg/arm/superior/right", &superior_right_arm_extensor_cb);
ros::Subscriber<std_msgs::Float64> superior_left_arm_extensor_sub("/actuation/leg/arm/superior/left", &superior_left_arm_extensor_cb);
ros::Subscriber<std_msgs::Float64> inferior_right_arm_extensor_sub("/actuation/leg/arm/inferior/right", &inferior_right_arm_extensor_cb);
ros::Subscriber<std_msgs::Float64> inferior_left_arm_extensor_sub("/actuation/leg/arm/inferior/left", &inferior_left_arm_extensor_cb);

ros::Subscriber<std_msgs::Float64> superior_right_forearm_extensor_sub("/actuation/leg/forearm/superior/right", &superior_right_forearm_extensor_cb);
ros::Subscriber<std_msgs::Float64> superior_left_forearm_extensor_sub("/actuation/leg/forearm/superior/left", &superior_left_forearm_extensor_cb);
ros::Subscriber<std_msgs::Float64> inferior_right_forearm_extensor_sub("/actuation/leg/forearm/inferior/right", &inferior_right_forearm_extensor_cb);
ros::Subscriber<std_msgs::Float64> inferior_left_forearm_extensor_sub("/actuation/leg/forearm/inferior/left", &inferior_left_forearm_extensor_cb);
    

void superior_right_shoulder_abductor_cb(const std_msgs::Float64 &superior_right_shoulder_pos_msg);
void superior_left_shoulder_abductor_cb(const std_msgs::Float64 &superior_left_shoulder_pos_msg);
void inferior_right_shoulder_abductor_cb(const std_msgs::Float64 &inferior_right_shoulder_pos_msg);
void inferior_left_shoulder_abductor_cb(const std_msgs::Float64 &inferior_left_shoulder_pos_msg);

void superior_right_arm_extensor_cb(const std_msgs::Float64 &superior_right_arm_pos_msg);
void superior_left_arm_extensor_cb(const std_msgs::Float64 &superior_left_arm_pos_msg);
void inferior_right_arm_extensor_cb(const std_msgs::Float64 &inferior_right_arm_pos_msg);
void inferior_left_arm_extensor_cb(const std_msgs::Float64 &inferior_left_arm_pos_msg);

void superior_right_forearm_extensor_cb(const std_msgs::Float64 &superior_right_forearm_pos_msg);
void superior_left_forearm_extensor_cb(const std_msgs::Float64 &superior_left_forearm_pos_msg);
void inferior_right_forearm_extensor_cb(const std_msgs::Float64 &inferior_right_forearm_pos_msg);
void inferior_left_forearm_extensor_cb(const std_msgs::Float64 &inferior_left_forearm_pos_msg);


void initROS() {
  nh.initNode();
  //sub
  nh.subscribe(superior_right_shoulder_abductor_sub);
  nh.subscribe(superior_left_shoulder_abductor_sub);
  nh.subscribe(inferior_right_shoulder_abductor_sub);
  nh.subscribe(inferior_left_shoulder_abductor_sub);

  nh.subscribe(superior_right_arm_extensor_sub);
  nh.subscribe(superior_left_arm_extensor_sub);
  nh.subscribe(inferior_right_arm_extensor_sub);
  nh.subscribe(inferior_left_arm_extensor_sub);

  nh.subscribe(superior_right_forearm_extensor_sub);
  nh.subscribe(superior_left_forearm_extensor_sub);
  nh.subscribe(inferior_right_forearm_extensor_sub);
  nh.subscribe(inferior_left_forearm_extensor_sub);
  
  // pub
  nh.advertise(debug_pub);
  nh.advertise(imu_pub);
  // give serial_node.py a chance to get to know the topics
  nh.negotiateTopics();

}