#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher orientation_pub;
ros::Publisher position_pub;
ros::Publisher pose_pub;

void odomCallback(const nav_msgs::Odometry::ConstPtr odometry_data) {
    orientation_pub.publish(odometry_data->pose.pose.orientation);
    position_pub.publish(odometry_data->pose.pose.position);

    geometry_msgs::PoseStamped pose_out;
    pose_out.pose = odometry_data->pose.pose;
    pose_out.header.frame_id = "/base_link";
    pose_out.pose.position.x = 0.0;
    pose_out.pose.position.y = 0.0;
    pose_out.pose.position.z = 0.0;
    pose_pub.publish(pose_out);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "odometry_decomposer");

  ros::NodeHandle n;

  orientation_pub = n.advertise<geometry_msgs::Quaternion>("/odometry/orientation", 1);
  position_pub = n.advertise<geometry_msgs::Point>("/odometry/position", 1);
  pose_pub = n.advertise<geometry_msgs::PoseStamped>("/odometry/pose", 1);

  ros::Subscriber sub = n.subscribe("/odometry/filtered", 1000, odomCallback);
  ros::spin();

  return 0;
}
