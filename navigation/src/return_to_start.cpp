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
void callBack(const std_msgs::Int16 &msg)
{
  if(msg.data == 1)
  {
    //boost::thread spin_thread = boost::thread(boost::bind(&spinThread));
    MoveBaseClient ac("move_base");
    //give some time for connections to register
    sleep(2.0);
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = 16.23;
    goal.target_pose.pose.position.y = -7.74;
    goal.target_pose.pose.orientation.z = -0.14;
    goal.target_pose.pose.orientation.w = 1.00;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Arrived");
	ros::shutdown();
    } 
    else
        ROS_INFO("Failed");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "return");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("return", 1, callBack);    

    ros::spin();
    return 0;

}
