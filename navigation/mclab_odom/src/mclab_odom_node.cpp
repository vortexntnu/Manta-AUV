#include "mclab_odom.h"

int main(int argc, char** argv)
{
  const bool DEBUG_MODE = false;  // debug logs are printed to console when true

  ros::init(argc, argv, "simple_odometry");
  ros::NodeHandle nh;

  if (DEBUG_MODE)
  {
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();
  }

  // ros::Rate rate(50);
  SimpleOdom mclab_odom(nh);
  mclab_odom.spin();
}