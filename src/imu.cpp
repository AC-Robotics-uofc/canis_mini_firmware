#include <Wire.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include "robot_constant.h"
#include "ros_connect.h"

void pub_imu_raw() {
    mpu.getEvent(&a, &g, &temp);

    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu";

    imu_msg.linear_acceleration.x = a.acceleration.x;
    imu_msg.linear_acceleration.y = a.acceleration.y;
    imu_msg.linear_acceleration.z = a.acceleration.z;

    imu_msg.angular_velocity.x = g.gyro.x;
    imu_msg.angular_velocity.y = g.gyro.y;
    imu_msg.angular_velocity.z = g.gyro.z;

    imu_pub.publish(&imu_msg);
}