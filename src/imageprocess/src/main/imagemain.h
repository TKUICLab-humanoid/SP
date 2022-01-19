#ifndef IMAGEMAIN_H
#define IMAGEMAIN_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <highgui.h>
#include <sys/time.h>
#include <stdio.h>

#include "tku_msgs/ButtonColorForm.h"
#include "tku_msgs/HSVInfo.h"
#include "tku_msgs/HSVValue.h"
#include "tku_msgs/BuildModel.h"
#include "tku_msgs/SaveHSV.h"
#include "sensor_msgs/Image.h"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include "tku_msgs/ColorData.h"
#include "tku_msgs/ColorArray.h"
#include "tku_msgs/ObjectList.h"
#include "tku_msgs/LabelModelObjectList.h"
#include "tku_msgs/DrawImage.h"

#include "../ModelBase/tmodelunit.hpp"
#include "../ImageBase/imagebase.hpp"

#include "../DrawImage/DrawImage.hpp"
//#include <vector>
using namespace std;
using namespace cv;

enum DrawMode
{
    etDrawLine,
    etDrawObject
};

ros::Publisher LabelModel_Publish;
ros::Publisher Object_Publish;
ros::Publisher Start_Publish;

ros::Subscriber ModelingButton_subscriber;
ros::Subscriber HSVValue_subscriber;
ros::Subscriber WebStart_subscriber;
ros::Subscriber DrawImage_subscriber;
ros::Subscriber CameraId_subscriber;
ros::Subscriber Imagesource_subscriber;
ros::Subscriber Gazeboimage_subscriber;

ros::ServiceServer HSV_service;
ros::ServiceServer Build_service;
ros::ServiceServer Save_service;

image_transport::Publisher pub_orign;
image_transport::Publisher pub_colormodel;
image_transport::Publisher pub_compressimage;

VideoCapture capwebcam;

tku_msgs::ColorData colordata;
tku_msgs::ColorArray colorarray;
tku_msgs::ObjectList objectlist;

int FPScnt;
int drawImagecnt;
int cameraid = 0;
int thickness = 1;
int linetype = 8;
bool checkcameraid = false;
bool checkImageSource = false;
bool checkRealImage = false;
vector<drawImageInfo> drawImageArray;


sensor_msgs::ImagePtr msg_orign;
sensor_msgs::ImagePtr msg_colormodel;
sensor_msgs::ImagePtr msg_compressimage;

cv::Mat buffer;
cv::Mat orign;
cv::Mat colormodel;
cv::Mat compress_image;

drawimage dra;

#endif // IMAGEMAIN_H
