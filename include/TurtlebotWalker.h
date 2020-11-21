class TurtlebotWalker {
  public:
    TurtlebotWalker();
    void drive(double speed);
    void stop();
    bool near_obstacle;

  private:
    void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    ros::NodeHandle nh;
    ros::Publisher vel_pub;
    ros::Subscriber scan_sub;
    geometry_msgs::Twist vel;

    double threshold;
};