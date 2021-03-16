/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
 
int main(int argc, char **argv)
{
  /*
  if(argc < 2)
  {
    std::cout << "usage: set_location door_number";
    return 1;
  }
  */
  //define 2d estimate pose  
/*    
  double pos_x = 7.30;//0.00;
  double pos_y = 0.27;//0.00;
  double ori_z = -0.67;
  double ori_w = 0.75;
*/
  double pos_x = 16.32;
  double pos_y = -7.74;
  double ori_z = -0.14;
  double ori_w = 0.99;
    
  ros::init(argc, argv, "set_location");
  ros::NodeHandle nh;
  ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  ros::Rate loop_rate(1);
  int i = 5;
   
 
  while (i--)
  {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
 
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.pose.position.x = pos_x;
    pose_msg.pose.pose.position.y = pos_y;
    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[6 * 1 + 1] = 0.25;
    pose_msg.pose.covariance[6 * 5 + 5] = 0.06853891945200942;
    pose_msg.pose.pose.orientation.z = ori_z;
    pose_msg.pose.pose.orientation.w = ori_w;
 
    initial_pose_pub.publish(pose_msg);
    ROS_INFO("Setting position to :(%f, %f)  Setting orientation to :(%f, %f)",pos_x, pos_y, ori_z, ori_w);
    ros::spinOnce();
 
    loop_rate.sleep();
  }
 
  return 0;
}
