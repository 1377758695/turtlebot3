#ifndef IMAGEVIEWER_H_
#define IMAGEVIEWER_H_

#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

namespace myturtlebot3
{

class ImageViewer
{
public:
    ImageViewer();
    virtual ~ImageViewer();

    void rawImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void compressedImageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
    cv::Mat objectImg;
    cv::Mat obj_rgb;
    cv::Mat sceneImg;
    cv::Mat scene_rgb;
    int AREA_MIN;
    int AREA_MAX;
private:
    ros::NodeHandle n;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;    
    std::vector<cv::Point2f> obj_corner;
    std::vector<cv::Point2f> scene_corner;
    std::vector<cv::Point2f> rect_center;
    std::vector<int> areas;
    ros::Publisher vel_pub;
    ros::Publisher dest_pub;
    int targetFindedTimes;
    int targetNumber;
    float velocity;
    bool stop;    
    void matchTarget();    
};

}

#endif