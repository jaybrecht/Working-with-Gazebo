#include <cstdlib>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double front = msg->ranges[0];
    if (front > msg->range_min && front < msg->range_max){
        ROS_INFO_STREAM("The closest obstacle in front is " << front << " meters away");
    } else {
        ROS_INFO_STREAM("No obstacle detected");
    }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidarCallback);

  geometry_msgs::Twist vel;

  ros::Rate loop_rate(1);

  while (ros::ok()){
    // int linear_vel = rand() % 2;
    // vel.linear.x = linear_vel;

    // int angular_vel = rand() % 2;
    // vel.angular.z = angular_vel;

    // ROS_INFO_STREAM("Linear Velocity: " << linear_vel <<
    //                 " | Angular Velocity: " << angular_vel);

    // vel_pub.publish(vel);

    ros::spinOnce();
  }
}