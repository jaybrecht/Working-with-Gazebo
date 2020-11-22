#include <vector>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include "../include/TurtlebotWalker.h"

TurtlebotWalker::TurtlebotWalker(double ds, double ts) {
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
    scan_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, &TurtlebotWalker::lidarCallback, this);
    near_obstacle = false;
    front_clear = true;
    obs_threshold = 0.4;
    drive_speed = ds;
    turn_speed = ts;
}

void TurtlebotWalker::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    int cone_angle = 45;
    std::vector<double> front_cone;
    std::vector<double> b;

    b = std::vector<double>(msg->ranges.begin(),msg->ranges.begin()+cone_angle);
    front_cone = std::vector<double>(msg->ranges.end()-cone_angle,msg->ranges.end());

    front_cone.insert(front_cone.end(),b.begin(),b.end());

    front_clear = true;

    for (auto& range : front_cone) {
        if (range < obs_threshold) {
            near_obstacle = true;
            front_clear = false;
        }
    }
}

void TurtlebotWalker::drive() {
    geometry_msgs::Twist vel;
    vel.linear.x = drive_speed;

    vel_pub.publish(vel);
}


void TurtlebotWalker::turn() {
    geometry_msgs::Twist vel;
    vel.angular.z = turn_speed;

    vel_pub.publish(vel);
}

void TurtlebotWalker::stop() {

    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.angular.z = 0;

    vel_pub.publish(vel);
    ros::Duration(0.5).sleep();
}

void TurtlebotWalker::rotateUntilClear(){
    turn();
    ros::Rate rate(10);
    while (!front_clear){
        rate.sleep();
        ros::spinOnce();
    }
    stop();
    near_obstacle = false;
}

void TurtlebotWalker::navigate() {
    while (ros::ok()){
        drive();

        if (near_obstacle){
            stop();
            rotateUntilClear();
        }

        ros::spinOnce();
    }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");

  TurtlebotWalker walker(0.2,0.2);

  walker.navigate();
}
