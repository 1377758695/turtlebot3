/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <std_msgs/Float32.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop");
    ros::NodeHandle nh;
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("vel_ctrl",1);
    float velocity;
    int duration;

    while(true)
    {
        std::cin >> velocity;
    	std_msgs::Float32 msg;
    	msg.data = velocity;
    	vel_pub.publish(msg);
	    ROS_INFO("publish velocity = %.2f", velocity);	            
    }
    

    return 0;
}