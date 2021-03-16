/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <get_location/get_location.h>

GetLocalPosition::GetLocalPosition()
{
}

void GetLocalPosition::getPos()
{
    try
    {        
        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);        
    }
    catch(tf::TransformException& e)
    {
        ROS_ERROR("Received an exception trying to transform: %s", e.what());
    }
    origin.x = transform.getOrigin().getX();
    origin.y = transform.getOrigin().getY();
    origin.z = transform.getOrigin().getZ();
    
    quaternion.x = transform.getRotation().getX();
    quaternion.y = transform.getRotation().getY();
    quaternion.z = transform.getRotation().getZ();
    quaternion.w = transform.getRotation().getW();
}

geometry_msgs::Point GetLocalPosition::getOrigin(){ return origin; }
geometry_msgs::Quaternion GetLocalPosition::getQuaternion(){ return quaternion; }
 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_location");
  ros::NodeHandle nh;
  ros::Publisher initial_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 10);
  ros::Rate loop_rate(1);
  GetLocalPosition locater;
  locater.getPos();
  //define 2d estimate pose  
  double pos_x = locater.getOrigin().x;
  double pos_y = locater.getOrigin().y;
  double ori_z = locater.getQuaternion().z;
  double ori_w = locater.getQuaternion().w;
 
  while (ros::ok())
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