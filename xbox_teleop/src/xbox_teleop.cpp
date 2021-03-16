/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<sensor_msgs/Joy.h>
#include<iostream>
using namespace std;

class XboxTeleop
{
public:
    XboxTeleop();

private:
    /*data*/
    void CallBack(const sensor_msgs::Joy::ConstPtr& Joy);
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    double v_linear, v_angular;
    int axis_linear, axis_angular, button;
};

XboxTeleop::XboxTeleop()
{
    n.param<int>("axis_linear", axis_linear, 1);
    n.param<int>("axis_angular", axis_angular, 0);
    n.param<double>("vel_linear", v_linear, 0.2);
    n.param<double>("vel_angular", v_angular, 0.5);
    n.param<int>("button", button, 5);

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    sub = n.subscribe<sensor_msgs::Joy>("joy",10,&XboxTeleop::CallBack, this);

}

void XboxTeleop::CallBack(const sensor_msgs::Joy::ConstPtr& Joy)
{
    geometry_msgs::Twist v;

    if(Joy->buttons[button])
    {
        v.linear.x = (Joy->axes[axis_linear]) * v_linear;
        v.angular.z = -(Joy->axes[axis_angular]) * v_angular;
        ROS_INFO("linear:%.3lf   angular:%.3lf", v.linear.x, v.angular.z);
        pub.publish(v);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy");
    XboxTeleop telelog;
    ros::spin();
    return 0;
}