#include <cstdlib>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

bool near_obstacle = false;
double threshold = 0.2;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double front = msg->ranges[0];
    if (front > msg->range_min && front < msg->range_max){
        if (front < threshold){
            near_obstacle = true;
        }
    } 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);
  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, lidarCallback);

  geometry_msgs::Twist vel;

  ros::Rate loop_rate(1);

  vel.linear.x = 0.1;
  vel_pub.publish(vel);

  while (ros::ok()){

    if (near_obstacle){
        ROS_WARN_STREAM("Got too close to an obstacle. Stopping");
        vel.linear.x = 0;
        vel_pub.publish(vel);
        break;
    }

    ros::spinOnce();
  }
}