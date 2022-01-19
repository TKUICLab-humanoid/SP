#ifndef ROS2MULTICOMMUNICATION_H
#define ROS2MULTICOMMUNICATION_H
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include "tku_libs/RobotCupInfo.h"
#include "tku_msgs/Ros2MultiCommunication.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <map>
#include <vector>


class Ros2MultiCommunication
{
    public:
        Ros2MultiCommunication();
        ~Ros2MultiCommunication();

        virtual void closeSubscriber(std::string sum_name);
        virtual bool isInitialize();
        void sendRobotCupInfo(RobotCupInfo *robotcup_info);
        void setTimePass(double check_time_ms, bool init_flag = true);
        // void testShowTimer();
        // std::string getPRS(); //Parner Robot State;

    protected:
        void initialize();
        void callBackRobot1(const tku_msgs::Ros2MultiCommunication&);
        void callBackRobot2(const tku_msgs::Ros2MultiCommunication&);
        void callBackRobot3(const tku_msgs::Ros2MultiCommunication&);
        void callBackRobot4(const tku_msgs::Ros2MultiCommunication&);
        void saveCallBackData(const tku_msgs::Ros2MultiCommunication&);

    private:
        ros::NodeHandle nh;
        std::map<std::string, ros::Publisher> robots_pub;
        std::map<std::string, ros::Subscriber> robots_sub;
        // std::map<std::string, TimeClass> callback_timer;

	protected:
        bool is_initialize;
        RobotCupInfoInstance *robotcup_info;
};

class Ros2MultiCommunicationInstance : public Ros2MultiCommunication
{
    public:
        Ros2MultiCommunicationInstance() : Ros2MultiCommunication(){}
        ~Ros2MultiCommunicationInstance();
        //using get rosparam to set topic name
        static Ros2MultiCommunicationInstance* getInstance();
        static void deleteInstance();
		bool isInitialize();

	private:
        static Ros2MultiCommunicationInstance *m_pInstance;
};

#endif