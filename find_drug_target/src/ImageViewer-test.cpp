/*****author: 潘薇鸿*****/
/*****update: 2020/08/18*****/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "myturtlebot3/ImageViewer.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <signal.h>
#include <stdio.h>
#include <string>
// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>

#ifdef HAVE_OPENCV_NONFREE
  #if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >=4
  #include <opencv2/nonfree/gpu.hpp>
  #include <opencv2/nonfree/features2d.hpp>
  #endif
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif
#define VEL 0.025
#define DIST 50
#define AREA_MIN 70000
#define AREA_MAX 170000
#define WID_LEN_MIN 0.90
#define WID_LEN_MAX 1.00
using namespace std;
using namespace cv;

namespace myturtlebot3
{

// ImageViewer::ImageViewer(int img_index)
ImageViewer::ImageViewer():it(n)
{
    img_sub = it.subscribe("/usb_cam/image_raw", 1, &ImageViewer::rawImageCallback, this, image_transport::TransportHints("compressed"));
	vel_pub = n.advertise<std_msgs::Float32>("vel_ctrl", 1);
    //string img_path = "../ImageData/"+to_string(img_index)+".jpg";
    string img_path = "/home/pwh/objects/3.png";
    obj_rgb = imread(img_path);
    objectImg = imread(img_path, IMREAD_GRAYSCALE);
    cv::imshow("object", objectImg);
    cv::waitKey();
	targetFindedTimes = 0;
	targetNumber = 0;
	velocity = 0;
	stop = false;
	dest_pub = n.advertise<std_msgs::Int16>("destination", 1);
	/*
	ros::Rate r(50);
	geometry_msgs::Twist move_cmd = geometry_msgs::Twist();
	ros::Publisher publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	move_cmd.linear.x = VEL;
	int goal_distance = 2;
	double linear_duration = goal_distance / move_cmd.linear.x;
	
	int ticks = int(linear_duration * 50);
	for(int t = 0; t < ticks; t++)
	{
		ROS_INFO("(%d/%d)", t, ticks);
		publisher.publish(move_cmd);
		r.sleep();
	}
	*/

}

ImageViewer::~ImageViewer()
{
    cv::destroyAllWindows();
}
/******** CallBackWithRawImage **************/
void ImageViewer::rawImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {        
        scene_rgb = cv_bridge::toCvCopy(msg, "bgr8")->image;        
        std::printf("receive image\n");
        cv::imshow("view", scene_rgb);
        cv::waitKey(1);
        cv::cvtColor(scene_rgb, sceneImg, CV_BGR2GRAY);
        cv::imshow("gray",sceneImg);
        cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from %s to 'bgr8'.", msg->encoding.c_str() );
    }
    matchTarget();
}

void ImageViewer::matchTarget()
{
	ROS_INFO("stop = %d", stop);
    if(!objectImg.empty() && !sceneImg.empty())
	{
		//printf("Loading images: %d ms\n", time.restart());
		std::vector<cv::KeyPoint> objectKeypoints;
		std::vector<cv::KeyPoint> sceneKeypoints;
		cv::Mat objectDescriptors;
		cv::Mat sceneDescriptors;

		////////////////////////////
		// EXTRACT KEYPOINTS
		////////////////////////////
		cv::Ptr<cv::FeatureDetector> detector;
		// The detector can be any of (see OpenCV features2d.hpp):
#if CV_MAJOR_VERSION == 2
		// detector = cv::Ptr(new cv::DenseFeatureDetector());
		// detector = cv::Ptr(new cv::FastFeatureDetector());
		// detector = cv::Ptr(new cv::GFTTDetector());
		// detector = cv::Ptr(new cv::MSER());
		// detector = cv::Ptr(new cv::ORB());
		detector = cv::Ptr<cv::FeatureDetector>(new cv::SIFT());
		// detector = cv::Ptr(new cv::StarFeatureDetector());
		// detector = cv::Ptr(new cv::SURF(600.0));
		// detector = cv::Ptr(new cv::BRISK());
#elif CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 3)
		detector = cv::xfeatures2d::SIFT::create();
#else // >= 4.3.0
        detector = cv::SIFT::create();
#endif
		detector->detect(objectImg, objectKeypoints);

		detector->detect(sceneImg, sceneKeypoints);


		////////////////////////////
		// EXTRACT DESCRIPTORS
		////////////////////////////
		cv::Ptr<cv::DescriptorExtractor> extractor;
#if CV_MAJOR_VERSION == 2
		// The extractor can be any of (see OpenCV features2d.hpp):
		// extractor = cv::Ptr(new cv::BriefDescriptorExtractor());
		// extractor = cv::Ptr(new cv::ORB());
		extractor = cv::Ptr<cv::DescriptorExtractor>(new cv::SIFT());
		// extractor = cv::Ptr(new cv::SURF(600.0));
		// extractor = cv::Ptr(new cv::BRISK());
		// extractor = cv::Ptr(new cv::FREAK());
#elif CV_MAJOR_VERSION < 4 || (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION < 3)
		extractor = cv::xfeatures2d::SIFT::create();
#else // >= 4.3.0
        extractor = cv::SIFT::create();
#endif
		extractor->compute(objectImg, objectKeypoints, objectDescriptors);
		extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);

		////////////////////////////
		// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
		////////////////////////////
		cv::Mat results;
		cv::Mat dists;
		std::vector<std::vector<cv::DMatch> > matches;
		int k=2; // find the 2 nearest neighbors
		bool useBFMatcher = false; // SET TO TRUE TO USE BRUTE FORCE MATCHER
		if(objectDescriptors.type()==CV_8U)
		{
			// Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
			printf("Binary descriptors detected...\n");
			if(useBFMatcher)
			{
				cv::BFMatcher matcher(cv::NORM_HAMMING); // use cv::NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see ORB constructor)
				matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
			}
			else
			{
				// Create Flann LSH index
				cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

				// search (nearest neighbor)
				flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
			}
		}
		else
		{
			// assume it is CV_32F
			//printf("Float descriptors detected...\n");
			if(useBFMatcher)
			{
				cv::BFMatcher matcher(cv::NORM_L2);
				matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
			}
			else
			{
				// Create Flann KDTree index
				cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
				// search (nearest neighbor)
				flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
			}
		}

		// Conversion to CV_32F if needed
		if(dists.type() == CV_32S)
		{
			cv::Mat temp;
			dists.convertTo(temp, CV_32F);
			dists = temp;
		}


		////////////////////////////
		// PROCESS NEAREST NEIGHBOR RESULTS
		////////////////////////////


		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		float nndrRatio = 0.8f;
		std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
		std::vector<int> indexes_1, indexes_2; // Used for homography
		std::vector<uchar> outlier_mask;  // Used for homography
		// Check if this descriptor matches with those of the objects
		if(!useBFMatcher)
		{
			for(int i=0; i<objectDescriptors.rows; ++i)
			{
				// Apply NNDR
				//printf("q=%d dist1=%f dist2=%f\n", i, dists.at<float>(i,0), dists.at<float>(i,1));
				if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
				   dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
				{
					mpts_1.push_back(objectKeypoints.at(i).pt);
					indexes_1.push_back(i);

					mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
					indexes_2.push_back(results.at<int>(i,0));
				}
			}
		}
		else
		{
			for(unsigned int i=0; i<matches.size(); ++i)
			{
				// Apply NNDR
				//printf("q=%d dist1=%f dist2=%f\n", matches.at(i).at(0).queryIdx, matches.at(i).at(0).distance, matches.at(i).at(1).distance);
				if(matches.at(i).size() == 2 &&
				   matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
				{
					mpts_1.push_back(objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
					indexes_1.push_back(matches.at(i).at(0).queryIdx);

					mpts_2.push_back(sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
					indexes_2.push_back(matches.at(i).at(0).trainIdx);
				}
			}
		}

		// FIND HOMOGRAPHY
		unsigned int minInliers = 8;
		if(mpts_1.size() >= minInliers)
		{
			//time.start();
			cv::Mat H = findHomography(mpts_1,
					mpts_2,
					cv::RANSAC,
					1.0,
					outlier_mask);
			int inliers=0, outliers=0;
			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlier_mask.at(k))
				{
					++inliers;
				}
				else
				{
					++outliers;
				}
			}
			//printf("Inliers=%d Outliers=%d\n", inliers, outliers);
			//draw points
			for(unsigned int k=0; k < mpts_1.size();k++)
			{
				if(outlier_mask.at(k))
				{
					cv::circle(obj_rgb, mpts_1.at(k), 3, cv::Scalar(0,255,0), -1);
					cv::circle(scene_rgb, mpts_2.at(k), 3, cv::Scalar(0,255,0), -1);
				}
				else
				{
					cv::circle(obj_rgb, mpts_1.at(k), 3, cv::Scalar(0,0,255), -1);
					cv::circle(scene_rgb, mpts_2.at(k), 3, cv::Scalar(0,0,255), -1);
				}				
			}
			
			
			obj_corner.push_back(cv::Point2f(0,0));
			obj_corner.push_back(cv::Point2f(0, obj_rgb.rows));
			obj_corner.push_back(cv::Point2f(obj_rgb.cols, 0));
			obj_corner.push_back(cv::Point2f(obj_rgb.cols, obj_rgb.rows));
			cv::Rect obj_rect = cv::boundingRect(obj_corner);
			cv::rectangle(obj_rgb, obj_rect, cv::Scalar(0,255,0), 1, cv::LINE_8, 0);
			
			if(!H.empty())
				cv::perspectiveTransform(obj_corner, scene_corner, H);			
			cv::Rect scene_rect = cv::boundingRect(scene_corner);
			cv::rectangle(scene_rgb, scene_rect, cv::Scalar(0,255,0), 1, cv::LINE_8, 0);

			cv::imshow("obj", obj_rgb);
			cv::waitKey(1);
			cv::imshow("scen", scene_rgb);
			cv::waitKey(1);
			
			double center_x = (scene_corner[1].x + scene_corner[2].x) / 2;
			double distance = center_x - scene_rgb.cols/2;
			distance = distance > 0 ? distance : -distance;			
			ROS_INFO("scene_corner[0]: (%.2f, %.2f)", scene_corner[0].x, scene_corner[0].y);
			ROS_INFO("scene_corner[1]: (%.2f, %.2f)", scene_corner[1].x, scene_corner[1].y);
			ROS_INFO("scene_corner[2]: (%.2f, %.2f)", scene_corner[2].x, scene_corner[2].y);
			ROS_INFO("scene_corner[3]: (%.2f, %.2f)", scene_corner[3].x, scene_corner[3].y);
			ROS_INFO("area: %d", scene_rect.width*scene_rect.height);
			ROS_INFO("width/length = %.2f", (double)scene_rect.width/(double)scene_rect.height);
			//ROS_INFO("distance: %.2f", distance);
			
			if(scene_corner[0].x > 0 && 
			   scene_corner[1].y < scene_rgb.rows && 
			   scene_corner[2].x < scene_rgb.cols &&
			   (double)scene_rect.width/(double)scene_rect.height > WID_LEN_MIN &&
			   (double)scene_rect.width/(double)scene_rect.height < WID_LEN_MAX
			   //scene_rect.width*scene_rect.height > AREA_MIN &&
			   //scene_rect.width*scene_rect.height < AREA_MAX
			   )
			{
				targetFindedTimes++;
				ROS_INFO("target find times: %d", targetFindedTimes);
				if ( targetFindedTimes >=2 || stop == true)
				{
					ROS_INFO("target detected");					
					targetNumber++;															
					targetFindedTimes = 0;
					
					//if()
					velocity = 0;					
					ROS_INFO("publish velocity = %.2f", velocity);
					if(stop)
					{						
						std_msgs::Int16 msg;
						msg.data = 1;
						ROS_INFO("go to destination = %d", msg.data);
						int duration = 5;
						//while(duration--)
						dest_pub.publish(msg);
						ros::shutdown();
					}
					stop = true;
					
					//cv::waitKey();
				}
				else velocity = VEL;
			}
			else
			{
				//if (stop == true) cv::waitKey();
				velocity = VEL;
			}
			
		}
		else
		{
			printf("Not enough matches (%d) for homography...\n", (int)mpts_1.size());
			targetFindedTimes = 0;
			velocity = 0;
		}		
	}
	else
	{
		printf("Images are not valid!\n");
		targetFindedTimes = 0;
		velocity = 0;
	}

	std_msgs::Float32 msg;
	msg.data = velocity;
	//int duration = 2;
	//while(duration--)
	vel_pub.publish(msg);
	ROS_INFO("publish velocity = %.2f", velocity);
	if(velocity == 0)
	{
		stop = true;
	}			
}

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_viewer");
    myturtlebot3::ImageViewer viewer; 
    ros::spin();
    
    return 0;
}
