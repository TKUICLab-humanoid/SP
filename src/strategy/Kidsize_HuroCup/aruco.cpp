#include"strategy/aruco.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "arucoimage");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	arucoimage arucoimage(nh);//class name & name you want

	ros::Rate loop_rate(30);
	while (nh.ok()) 
	{
		arucoimage.arucomain();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
void arucoimage::arucomain()
{
	if(strategy_info->getStrategyStart())
	{
    	ros::Rate loop_rate(150);
    	//image_transport::ImageTransport it(nh);
    	//inputimage.open(1);

    	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    	cameraMatrix = (Mat_<double>(3, 3) << 929.087, 0, 509.582, 0, 911.194, 196.78, 0, 0, 1);
    	distCoeffs = (Mat_<double>(14, 1) << 3.09144, -62.9916, -0.0113575, 0.106171, 173.466,2.65127,-59.7773,165.194,0,0,0,0,0,0);    
    	//inputimage.grab();
    	//inputimage.retrieve(orign);
    	imageSize = orign.size();
    	orign.copyTo(imageCopy);
    	std::vector<std::vector<cv::Point2f> > markerCorners;
    	cv::aruco::detectMarkers(orign, dictionary, markerCorners, markerIds);
    	if (markerIds.size() > 0) 
    	{
			get_image = true;
			aruco_pub.get_arucoimage = get_image;
    	    cv::aruco::drawDetectedMarkers(imageCopy, markerCorners, markerIds);
    	    vector<Vec3d> rvecs, tvecs;
    	    cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.05, cameraMatrix, distCoeffs, rvecs, tvecs);
    	    //cout<<"R :"<<rvecs[0]<<endl;
			cout<<"T :"<<tvecs[0]<<endl;
    	    for(int i=0; i<markerIds.size(); i++)
    	    {
				cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4) 
    	                         << "x: " << std::setw(8)<<  tvecs[0](0);
    	        cv::putText(imageCopy, vector_to_marker.str(), 
    	              		cvPoint(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, 
    	                    cvScalar(0, 252, 124), 1, CV_AA);
	
				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4) 
    	                             << "y: " << std::setw(8) << tvecs[0](1); 
				cv::putText(imageCopy, vector_to_marker.str(), 
							cvPoint(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
							cvScalar(0, 252, 124), 1, CV_AA);
	
				vector_to_marker.str(std::string());
				vector_to_marker << std::setprecision(4)
									<< "z: " << std::setw(8) << tvecs[0](2);
				cv::putText(imageCopy, vector_to_marker.str(), 
							cvPoint(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
							cvScalar(0, 252, 124), 1, CV_AA);
    	    }

			for(int i = 0; i<3; i++)
			{
				aruco_pub.tvecs.push_back(tvecs[0](i));
			} 
			arucomatrix_publish.publish(aruco_pub);
			for(int i = 0; i<3; i++)											//enpty the array
			{
				aruco_pub.tvecs.pop_back();
			} 
    	}
		else
		{
			cout<<"no image"<<endl;
			get_image = false;
			aruco_pub.get_arucoimage = get_image;
			for(int i = 0; i<3; i++)
			{
				aruco_pub.tvecs.push_back(0);
			}
			arucomatrix_publish.publish(aruco_pub);
			for(int i = 0; i<3; i++)											//enpty the array
			{
				aruco_pub.tvecs.pop_back();
			}
		}


    	//cv::imshow("image",imageCopy);
		msg_arucoimage = cv_bridge::CvImage(std_msgs::Header(),"bgr8",imageCopy).toImageMsg();				//imageCopy是你要传的影像
		arucoimage_publish.publish(msg_arucoimage);
    	cv::waitKey(10);
	}
}
void arucoimage::Getimagefunction(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		orign = cv_ptr->image;
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
}
