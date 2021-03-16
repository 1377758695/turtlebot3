/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <opencv2/highgui/highgui.hpp>

float linear_x = 0.2;

class SubscribeAndPublish
{
public:
    SubscribeAndPublish()
    {
        pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        sub = nh.subscribe("vel_ctrl", 1, &SubscribeAndPublish::callBack, this);
    }

    void callBack(const std_msgs::Float32 &msg)
    {
        ROS_INFO("msg: %.2f", msg.data);
        linear_x = msg.data;

        geometry_msgs::Twist move_cmd = geometry_msgs::Twist();        
        ros::Rate r(50);
        int duration = 10;
        while(duration--)
        {        
            move_cmd.linear.x = linear_x;
            pub.publish(move_cmd);
            ROS_INFO("Publish linear x = %.2f", move_cmd.linear.x);
        }
    }
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
};


int main(int argc, char** argv)
{   
    ros::init(argc, argv, "velocity_control");    
    SubscribeAndPublish obj;
    ros::spin();
    return 0;
}
