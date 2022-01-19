#include <ros/ros.h>
#include <sys/time.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <map>
#include <math.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/JointRequest.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelProperties.h>
#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetJointProperties.h>

typedef struct structmodelinfo
{
	std::string Name;
	geometry_msgs::Point Pose;
	geometry_msgs::Point Angular;
	geometry_msgs::Twist Velocity;
	float Sim_Time;  
	
}StructModelInfo, *pStructModelInfo;

typedef struct transformpose
{
	geometry_msgs::Point pose;
	geometry_msgs::Point angular;
}StructTFPose;

typedef struct jointname
{
	std::vector<std::string> joint_name;
	//std::map<std::string, float> joint_position;
}StructJointName;

typedef std::vector<std::string> vstring;

class gazebo_tool
{
public:
	gazebo_tool();
	~gazebo_tool();
	void set_Client(ros::NodeHandle &nh);
	void pause_world(void);
	void unpause_world(void);
	void reset_modelpose(void);
	void get_simtime(double *simtime);
	void simDelay(double delay_time);
	void show_simtime(void);
	void show_allmodels(void);
	vstring getAllRobotsName(void);
	bool check_model(std::string model_name);
	void get_modelinfo(std::string model_name);
	void set_modelinfo(std::string model_name, StructTFPose TFPose);
	void get_modelproperties(std::string model_name);
	float get_jointproperties(std::string joint_name);
	float get_linkposeZ(std::string link_name);
	StructTFPose TFPoseFunction(float x, float y, float z, float R, float P, float Y);

public:
	std::map<std::string, StructJointName> mapJointName;
	std::map<std::string, StructModelInfo> mapModelInfo;
private:
	ros::NodeHandle *nh;
	ros::ServiceClient GetModelState_Client;
	ros::ServiceClient SetModelState_Client;
	ros::ServiceClient PausePhysics_Client;
	ros::ServiceClient GetLinkState_Client;
	ros::ServiceClient UnPausePhysics_Client;
	ros::ServiceClient ResetSimulation_Client;
	ros::ServiceClient GetWorldProperties_Client;
	ros::ServiceClient GetModelProperties_Client;
	ros::ServiceClient GetJointProperties_Client;
};

// extern gazebo_tool *tool_gz;