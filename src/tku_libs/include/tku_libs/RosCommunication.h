#ifndef ROSCOMMUNICATION_H
#define ROSCOMMUNICATION_H

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include "tku_msgs/Interface.h"
#include "tku_msgs/HeadPackage.h"
#include "tku_msgs/SandHandSpeed.h"
#include "tku_msgs/SensorSet.h"
#include "tku_msgs/DrawImage.h"
#include "tku_msgs/SingleMotorData.h"
#include "tku_msgs/ObjectList.h"
#include "tku_msgs/LabelModelObjectList.h"
#include "tku_msgs/RobotPos.h"
#include "tku_msgs/SensorSet.h"
#include "tku_msgs/SensorPackage.h"
#include "tku_msgs/SetGoalPoint.h"
#include "tku_msgs/SoccerData.h"
#include "tku_msgs/SoccerDataList.h"
#include "tku_libs/strategy_info.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>

class RosCommunication
{
	public:
		RosCommunication();
        ~RosCommunication();
        void initialize();
        void closeSubscriber(std::string sum_name);
        virtual bool isInitialize();

		void sendBodyAuto(int x, int y, int z, int theta, WalkingMode mode, SensorMode sensor);
	    void sendContinuousValue(int x, int y, int z, int theta, SensorMode sensor);
		void drawImageFunction(int cnt, DrawMode mode, int xmin, int xmax, int ymin, int ymax, int r, int g, int b);
		void sendHeadMotor(HeadMotorID ID, int Position, int Speed);
		void sendBodySector(int Sector);
		void sendHandSpeed(int Sector, int Speed);
		void sendSingleMotor(int ID, int Position, int Speed);
		void sendSensorSet(float P, float I, float D, int modeset);
		void sendSensorReset();

        void getObject(const tku_msgs::ObjectList &msg);
		void getLabelModel(const tku_msgs::LabelModelObjectList &msg);
		void catchImage(const sensor_msgs::ImageConstPtr& img);
		void startFunction(const std_msgs::Bool& msg);
		void DIOackFunction(const std_msgs::Int16 &msg);
		void getRobotPos(const tku_msgs::RobotPos &msg);
		void getGoalPoint(const tku_msgs::SetGoalPoint &msg);
		void getSoccerData(const tku_msgs::SoccerDataList &msg);
		void sensorPackageFunction(const tku_msgs::SensorPackage &msg);
        
	private:
		ros::NodeHandle nh;
		ros::Publisher walkingGait_pub;
		ros::Publisher head_motor_pub;
		ros::Publisher sector_pub;
		ros::Publisher hand_speed_pub;
		ros::Publisher draw_image_pub;
		ros::Publisher continuous_value_pub;
		ros::Publisher single_motor_data_pub;
		ros::Publisher sensor_pub;

        ros::Subscriber start_sub;
		ros::Subscriber DIO_ack_sub;
		ros::Subscriber label_model_sub;
		ros::Subscriber object_list_sub;
		ros::Subscriber compress_image_sub;
		ros::Subscriber robot_position_sub;
		ros::Subscriber goal_point_sub;
		ros::Subscriber soccer_data_sub;
		ros::Subscriber sensor_sub;

	protected:
		StrategyInfoInstance *strategy_info;
        bool is_initialize;
};

class RosCommunicationInstance : public RosCommunication
{
    public:
        RosCommunicationInstance() : RosCommunication(){}
        ~RosCommunicationInstance(){};
        static RosCommunicationInstance* getInstance();
        static void deleteInstance();
		bool isInitialize();

	private:
        static RosCommunicationInstance *m_pInstance;
};

#endif
