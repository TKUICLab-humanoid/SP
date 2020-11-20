#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "strategy/aruco_matrix.h"
#include "tku_libs/RosCommunication.h"
/****************************************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"

#include <highgui.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/flann.hpp>
#include "opencv2/highgui.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d.hpp"

#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <opencv2/aruco.hpp>
#include "opencv2/aruco/dictionary.hpp"
using namespace std;
using namespace cv;

class arucoimage
{
public:
	arucoimage(ros::NodeHandle &nh)
	{
		image_transport::ImageTransport it(nh);
		arucoimage_publish = it.advertise("final_aruco_image",1);
		arucomatrix_publish = nh.advertise<strategy::aruco_matrix>("/strategy/ArucoMatrix_Topic",1000);
		Imagesource_subscriber = nh.subscribe("/usb_cam/image_raw",10,&arucoimage::Getimagefunction,this);
	
				///////////////
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();	
	};

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	~arucoimage(){};
	void arucomain();
	void Getimagefunction(const sensor_msgs::ImageConstPtr& msg);
/****************************************************************************************************/
	image_transport::Publisher arucoimage_publish;		//image_publish
	ros::Subscriber Imagesource_subscriber;				//image_publish
	sensor_msgs::ImagePtr msg_arucoimage;				//convert image format to publish
	ros::Publisher arucomatrix_publish;					//publish the matrix
	strategy::aruco_matrix aruco_pub;					//save the parameter of the tvecs to publish

	cv::VideoCapture inputimage;
	cv::Mat orign = cv::Mat::zeros(cv::Size(240, 320), CV_8U);
	cv::Mat imageCopy = cv::Mat::zeros(cv::Size(240, 320), CV_8U);; //image
	
	//cv::Mat markerImage; //show marker number
	cv::Mat cameraMatrix, distCoeffs; //camera parermeter
	std::vector<std::vector<cv::Point3f> > objectsPoints;
	std::vector<std::vector<cv::Point2f> > imagePoints;
	std::vector<int> markerIds;
	cv::Size imageSize ; 
	std::string cameraname;
	
	std::ostringstream vector_to_marker;
/****************************************************************************************************/
	bool get_image = true;
};
