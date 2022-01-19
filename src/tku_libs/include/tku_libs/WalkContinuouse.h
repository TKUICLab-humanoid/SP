#ifndef WALK_CONTINUOUSE
#define WALK_CONTINUOUSE
#include "tku_libs/RosCommunication.h"
#include "tku_libs/strategy_info.h"
#include <ros/node_handle.h>
#include "tku_libs/TKU_tool.h"

class WalkValue
{
public:
	WalkValue(){x = 0; y = 0; z = 0; theta = 0;}
	~WalkValue(){}
	float x;
	float y;
	float z;
	float theta;
	void setValue(float x, float y, float z, float theta)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->theta = theta;
	}
};

class WalkContinuouse
{
public:
	WalkContinuouse()
	{
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
		walking_mode = WalkingMode::ContinuousStep;
		sensor_mode = SensorMode::None;
		this->continuous_flag = false;
		this->setWalkParameterInit_flag = false;
		this->setWalkParameterMax_flag = false;
		this->setWalkParameterMin_flag = false;
		this->setWalkParameterExp_flag = false;
		this->setWalkParameterOneAddValueAndPeriod_flag = false;
		this->delay_time_ms = 100; //msec
		this->timer = nh.createTimer(ros::Duration(this->delay_time_ms/1000.0), &WalkContinuouse::changeWalkParameterCallback, this, false, false);
	}
	~WalkContinuouse(){}
	// ---
	bool setWalkParameterInit(float x_init, float y_init, float z_init, float theta_init);
	bool setWalkParameterMax(float x_max, float y_max, float z_max, float theta_max);
	bool setWalkParameterMin(float x_min, float y_min, float z_min, float theta_min);
	bool setWalkParameterExp(float x_exp, float y_exp, float z_exp, float theta_exp);
	bool setWalkParameterOneAddValueAndPeriod(float x_one, float y_one, float z_one, float theta_one, int t);
	bool startContinuous(WalkingMode walking_mode, SensorMode sensor_mode);
	bool stopContinuous();
	bool checkAllFlag();
	bool isStartContinuous();
	void changeWalkParameterCallback(const ros::TimerEvent &event);
	void showAllDatavalue();
	void showRealTimeWalkValue();
	int getX();
	int getY();
	int getZ();
	int getTheta();
private:
	bool checkDataValue();
	void changeValue(float &value, float &value_exp, float &value_one);
	// ---
	ros::Timer timer;
	ros::NodeHandle nh;
	// ---
	RosCommunicationInstance *ros_com;
	WalkValue walk_init;
	WalkValue walk_max;
	WalkValue walk_min;
	WalkValue walk_exp;
	WalkValue walk_one;
	WalkValue walk;
	ToolInstance *tool;
	// ---
	WalkingMode walking_mode;
	SensorMode sensor_mode;
	// ---
	bool continuous_flag;
	bool setWalkParameterInit_flag;
	bool setWalkParameterMin_flag;
	bool setWalkParameterMax_flag;
	bool setWalkParameterExp_flag;
	bool setWalkParameterOneAddValueAndPeriod_flag;
	// ---
	float delay_time_ms; //sec
};

class WalkContinuouseInstance : public WalkContinuouse
{
public:
	WalkContinuouseInstance() : WalkContinuouse(){}
	~WalkContinuouseInstance(){}
	static WalkContinuouseInstance* getInstance();
	static void deleteInstance();

private:
	static WalkContinuouseInstance *m_pInstance;

};
#endif