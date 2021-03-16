/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int16.h>
#include <boost/thread.hpp>

void spinThread(){
  ros::spin();
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "get_to_drug_shelf");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int16>("find_target", 1);
  //boost::thread spin_thread = boost::thread(boost::bind(&spinThread));
  MoveBaseClient ac("move_base");
  //give some time for connections to register
  sleep(2.0);
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 24.63; 
  goal.target_pose.pose.position.y = -12.27;
  goal.target_pose.pose.orientation.z = 0.99;
  goal.target_pose.pose.orientation.w = 0.15;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Arrived");    
    std_msgs::Int16 msg;
    msg.data = 1;
    pub.publish(msg);
    ros::shutdown();
  } 
  else
    ROS_INFO("Failed");
  
  ros::spin();
  return 0;
}
