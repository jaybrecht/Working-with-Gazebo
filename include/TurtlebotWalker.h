class TurtlebotWalker {
  public:
    TurtlebotWalker(double ds, double ts);
    void drive();
    void stop();
    void turn();
    void rotateUntilClear();
    void navigate();
    bool near_obstacle;
    bool front_clear;

  private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber scan_sub;
    geometry_msgs::Twist vel;

    double obs_threshold;
    double drive_speed;
    double turn_speed;
};