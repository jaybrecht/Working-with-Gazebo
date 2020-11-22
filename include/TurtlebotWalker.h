// Copyright [2020] Justin Albrecht

#pragma once

/**
 * @brief      This class describes a turtlebot walker.
 */
class TurtlebotWalker {
 public:
    /**
     * @brief      Constructs a new instance.
     *
     * @param[in]  ds    { drive speed }
     * @param[in]  ts    { turn speed }
     */
    TurtlebotWalker(double ds, double ts);
    /**
     * @brief      { directs the robot to drive forward at drive speed }
     */
    void drive();
    /**
     * @brief      { directs the robot to stop moving }
     */
    void stop();
    /**
     * @brief      { directs the robot to turn at turn speed }
     */
    void turn();
    /**
     * @brief      { rotates until the front of the robot is clear of obstacles }
     */
    void rotateUntilClear();
    /**
     * @brief      { executes basic obstacle avoidance }
     */
    void navigate();
    bool near_obstacle;
    bool front_clear;

 private:
    /**
     * @brief      { Callback that analyzes the data from the Lidar sensor }
     *
     * @param[in]  msg   Laser Scan message
     */
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber scan_sub;
    geometry_msgs::Twist vel;

    double obs_threshold;
    double drive_speed;
    double turn_speed;
};
