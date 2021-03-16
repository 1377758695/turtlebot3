/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_location");    
    ros::NodeHandle nh;
    ros::Rate r(100);

    GetLocalPosition locater;
    while(nh.ok())
    {
        locater.getPos();
        ROS_INFO("origin: (%.2f, %.2f, %.2f)  quaternion:(%.2f, %.2f, %.2f, %.2f)",
                locater.getOrigin().x, locater.getOrigin().y, locater.getOrigin().z,
                locater.getQuaternion().x, locater.getQuaternion().y, locater.getQuaternion().z, locater.getQuaternion().w);
        r.sleep();
    }
}