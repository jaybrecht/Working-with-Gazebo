#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "../include/TurtlebotWalker.h"

TurtlebotWalker::TurtlebotWalker() {
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TurtlebotWalker::lidarCallback, this);
    near_obstacle = false;
    threshold = 0.2;
}

void TurtlebotWalker::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    double front = msg->ranges[0];
    if (front > msg->range_min && front < msg->range_max){
        if (front < threshold){
            near_obstacle = true;
        }
    } 
}

void TurtlebotWalker::drive(double speed) {
    geometry_msgs::Twist vel;
    vel.linear.x = speed;

    vel_pub.publish(vel);
}

void TurtlebotWalker::stop() {
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.z = 0;

    vel_pub.publish(vel);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");

  TurtlebotWalker walker;

  ros::Rate loop_rate(1);

  walker.drive(.1);

  while (ros::ok()){

    if (walker.near_obstacle){
        ROS_WARN_STREAM("Got too close to an obstacle. Stopping");
        walker.stop();
        break;
    }

    ros::spinOnce();
  }
}