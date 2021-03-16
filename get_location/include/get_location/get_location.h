#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>

class GetLocalPosition
{
public:
    GetLocalPosition();
    virtual ~GetLocalPosition(){}
    void getPos();
    geometry_msgs::Point getOrigin();
    geometry_msgs::Quaternion getQuaternion();
private:
    tf::TransformListener listener;
    tf::StampedTransform transform;
    geometry_msgs::Point origin;
    geometry_msgs::Quaternion quaternion; 
};