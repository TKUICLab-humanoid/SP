#include <iostream>
#include "tku_libs/WalkContinuouse.h"

bool WalkContinuouse::setWalkParameterInit(float x_init, float y_init, float z_init, float theta_init)
{
	if(!this->continuous_flag)
	{
		this->walk_init.x = x_init;
		this->walk_init.y = y_init;
		this->walk_init.z = z_init;
		this->walk_init.theta = theta_init;
		this->walk.x = this->walk_init.x;
		this->walk.y = this->walk_init.y;
		this->walk.z = this->walk_init.z;
		this->walk.theta = this->walk_init.theta;
		this->setWalkParameterInit_flag = true;
		return true;
	}
	else
	{
		std::printf("\033[0;33mFailed to set walk_init, please stop walking continuous to setting.\033[0m\n");	
		return false;
	}
}
bool WalkContinuouse::setWalkParameterMax(float x_max, float y_max, float z_max, float theta_max)
{
	if(!this->continuous_flag)
	{
		this->walk_max.x = x_max;
		this->walk_max.y = y_max;
		this->walk_max.z = z_max;
		this->walk_max.theta = theta_max;
		this->setWalkParameterMax_flag = true;
		return true;
	}
	else
	{
		std::printf("\033[0;33mFailed to set walk_max, please stop walking continuous to setting.\033[0m\n");
		return false;
	}
}
bool WalkContinuouse::setWalkParameterMin(float x_min, float y_min, float z_min, float theta_min)
{
	if(!this->continuous_flag)
	{
		this->walk_min.x = x_min;
		this->walk_min.y = y_min;
		this->walk_min.z = z_min;
		this->walk_min.theta = theta_min;
		this->setWalkParameterMin_flag = true;
		return true;
	}
	else
	{
		std::printf("\033[0;33mFailed to set walk_min, please stop walking continuous to setting.\033[0m\n");
		return false;
	}
}
bool WalkContinuouse::setWalkParameterExp(float x_exp, float y_exp, float z_exp, float theta_exp)
{
	if(this->walk_exp.x == x_exp && this->walk_exp.y == y_exp && this->walk_exp.z == z_exp && this->walk_exp.theta == theta_exp && this->setWalkParameterExp_flag)return true;
	this->walk_exp.x = x_exp;
	this->walk_exp.y = y_exp;
	this->walk_exp.z = z_exp;
	this->walk_exp.theta = theta_exp;
	this->setWalkParameterExp_flag = true;
	return checkDataValue();
}
bool WalkContinuouse::startContinuous(WalkingMode walking_mode, SensorMode sensor_mode)
{
	this->sensor_mode = sensor_mode;
	this->walking_mode = walking_mode;
	if(walking_mode != WalkingMode::ContinuousStep && walking_mode != WalkingMode::ContinuousStep_second && walking_mode != WalkingMode::ContinuousStep_third)
	{
		std::printf("\033[0;31mthe function is used by continuous mode.\nif you want to start walking continuous please input continuous mode.\033[0m\n");
		return false;
	}
	if(this->continuous_flag)
	{
		// std::printf("\033[0;33mwalking continuous is already starting.\033[0m\n");
		return false;
	}
	else
	{
		if(checkDataValue() && checkAllFlag())
		{
			ros_com->sendBodyAuto(this->walk_init.x, this->walk_init.y, this->walk_init.z, this->walk_init.theta, walking_mode, sensor_mode);
			this->continuous_flag = true;
			this->timer.start();// start the timer to start control value
			return true;
		}
		else
		{
			std::printf("\033[0;31mhave some value error.\033[0m\n");
			return false;
		}
	}
}
bool WalkContinuouse::stopContinuous()
{
	if(walking_mode != WalkingMode::ContinuousStep && walking_mode != WalkingMode::ContinuousStep_second && walking_mode != WalkingMode::ContinuousStep_third)
	{
		std::printf("\033[0;31mthe function is used by continuous mode.\nif you want to stop walking continuous please input continuous mode.\033[0m\n");
		return false;
	}
	if(this->continuous_flag)
	{
		tool->Delay(10);
		ros_com->sendBodyAuto(walk.x, walk.y, walk.z, walk.theta, walking_mode, sensor_mode);
		this->continuous_flag = false;
		this->timer.stop();// stop the timer to stop control value
		setWalkParameterInit(this->walk_init.x, this->walk_init.y, this->walk_init.z, this->walk_init.theta);
		return true;
	}
	else
	{
		// std::printf("\033[0;33mwalking continuous is already stoped.\033[0m\n");
		return false;
	}
}
void WalkContinuouse::changeWalkParameterCallback(const ros::TimerEvent &event) // use timer to control this function
{
	changeValue(this->walk.x, this->walk_exp.x, this->walk_one.x);
	changeValue(this->walk.y, this->walk_exp.y, this->walk_one.y);
	changeValue(this->walk.z, this->walk_exp.z, this->walk_one.z);
	changeValue(this->walk.theta, this->walk_exp.theta, this->walk_one.theta);
	ros_com->sendContinuousValue(this->walk.x, this->walk.y, this->walk.z, this->walk.theta, sensor_mode);
}
bool WalkContinuouse::checkDataValue()
{
	bool test = false;
	if(this->walk_max.x < this->walk_min.x)
	{
		std::printf("\033[0;31myour x_max=%d < x_min=%d, please check your value.\033[0m\n", (int)this->walk_max.x, (int)this->walk_min.x);
		test = true;
	}
	if(this->walk_max.y < this->walk_min.y)
	{
		std::printf("\033[0;31myour y_max=%d < y_min=%d, please check your value.\033[0m\n", (int)this->walk_max.y, (int)this->walk_min.y);
		test = true;
	}
	if(this->walk_max.z < this->walk_min.z)
	{
		std::printf("\033[0;31myour z_max=%d < z_min=%d, please check your value.\033[0m\n", (int)this->walk_max.z, (int)this->walk_min.z);
		test = true;
	}
	if(this->walk_max.theta < this->walk_min.theta)
	{
		std::printf("\033[0;31myour theta_max=%d < theta_min=%d, please check your value.\033[0m\n", (int)this->walk_max.theta, (int)this->walk_min.theta);
		test = true;
	}
	if(this->walk_init.x < this->walk_min.x || this->walk_init.x > walk_max.x)
	{
		std::printf("\033[0;31myour x_init=%d maybe less than x_min=%d or more than x_max=%d, please check your value.\033[0m\n", (int)this->walk_init.x, (int)this->walk_min.x, (int)this->walk_max.x);
		test = true;
	}
	if(this->walk_init.y < this->walk_min.y || this->walk_init.y > walk_max.y)
	{
		std::printf("\033[0;31myour y_init=%d maybe less than y_min=%d or more than y_max=%d, please check your value.\033[0m\n", (int)this->walk_init.y, (int)this->walk_min.y, (int)this->walk_max.y);
		test = true;
	}
	if(this->walk_init.z < this->walk_min.z || this->walk_init.z > walk_max.z)
	{
		std::printf("\033[0;31myour z_init=%d maybe less than z_min=%d or more than z_max=%d, please check your value.\033[0m\n", (int)this->walk_init.z, (int)this->walk_min.z, (int)this->walk_max.z);
		test = true;
	}
	if(this->walk_init.theta < this->walk_min.theta || this->walk_init.theta > walk_max.theta)
	{
		std::printf("\033[0;31myour theta_init=%d maybe less than theta_min=%d or more than theta_max=%d, please check your value.\033[0m\n", (int)this->walk_init.theta, (int)this->walk_min.theta, (int)this->walk_max.theta);
		test = true;
	}
	if(this->walk_exp.x < this->walk_min.x)
	{
		std::printf("\033[0;33myour x_exp=%d less than x_min=%d, now auto change x_exp value equal x_min value.\033[0m\n", (int)this->walk_exp.x, (int)this->walk_min.x);
		this->walk_exp.x = this->walk_min.x;
	}
	else if(this->walk_exp.x > this->walk_max.x)
	{
		std::printf("\033[0;33myour x_exp=%d more than x_max=%d, now auto change x_exp value equal x_max value.\033[0m\n", (int)this->walk_exp.x, (int)this->walk_max.x);
		this->walk_exp.x = this->walk_max.x;
	}
	if(this->walk_exp.y < this->walk_min.y)
	{
		std::printf("\033[0;33myour y_exp=%d less than y_min=%d, now auto change y_exp value equal y_min value.\033[0m\n", (int)this->walk_exp.y, (int)this->walk_min.y);
		this->walk_exp.y = this->walk_min.y;
	}
	else if(this->walk_exp.y > this->walk_max.y)
	{
		std::printf("\033[0;33myour y_exp=%d more than y_max=%d, now auto change y_exp value equal y_max value.\033[0m\n", (int)this->walk_exp.y, (int)this->walk_max.y);
		this->walk_exp.y = this->walk_max.y;
	}
	if(this->walk_exp.z < this->walk_min.z)
	{
		std::printf("\033[0;33myour z_exp=%d less than z_min=%d, now auto change z_exp value equal z_min value.\033[0m\n", (int)this->walk_exp.z, (int)this->walk_min.z);
		this->walk_exp.z = this->walk_min.z;
	}
	else if(this->walk_exp.z > this->walk_max.z)
	{
		std::printf("\033[0;33myour z_exp=%d more than z_max=%d, now auto change z_exp value equal z_max value.\033[0m\n", (int)this->walk_exp.z, (int)this->walk_max.z);
		this->walk_exp.z = this->walk_max.z;
	}
	if(this->walk_exp.theta < this->walk_min.theta)
	{
		std::printf("\033[0;33myour theta_exp=%d less than theta_min=%d, now auto change theta_exp value equal theta_min value.\033[0m\n", (int)this->walk_exp.theta, (int)this->walk_min.theta);
		this->walk_exp.theta = this->walk_min.theta;
	}
	else if(this->walk_exp.theta > this->walk_max.theta)
	{
		std::printf("\033[0;33myour theta_exp=%d more than theta_max=%d, now auto change theta_exp value equal theta_max value.\033[0m\n", (int)this->walk_exp.theta, (int)this->walk_max.theta);
		this->walk_exp.theta = this->walk_max.theta;
	}
	if(test)
	{
		return false;
	}
	else
	{
		return true;	
	}
}
bool WalkContinuouse::checkAllFlag()
{
	this->setWalkParameterInit_flag ? std::printf("setWalkParameterInit_flag = True\n") : std::printf("\033[0;31msetWalkParameterInit_flag = False\033[0m\n");
	this->setWalkParameterMax_flag ? std::printf("setWalkParameterMax_flag = True\n") : std::printf("\033[0;31msetWalkParameterMax_flag = False\033[0m\n");
	this->setWalkParameterMin_flag ? std::printf("setWalkParameterMin_flag = True\n") : std::printf("\033[0;31msetWalkParameterMin_flag = False\033[0m\n");
	this->setWalkParameterExp_flag ? std::printf("setWalkParameterExp_flag = True\n") : std::printf("\033[0;31msetWalkParameterExp_flag = False\033[0m\n");
	this->setWalkParameterOneAddValueAndPeriod_flag ? std::printf("setWalkParameterOneAddValueAndPeriod_flag = True\n") : std::printf("\033[0;31msetWalkParameterOneAddValueAndPeriod_flag = False\033[0m\n");
	this->continuous_flag ? std::printf("continuous_flag = True\n") : std::printf("\033[0;31mcontinuous_flag = False\033[0m\n");
	if(this->setWalkParameterInit_flag && this->setWalkParameterMax_flag
		&& this->setWalkParameterMin_flag && this->setWalkParameterExp_flag && this->setWalkParameterOneAddValueAndPeriod_flag)
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool WalkContinuouse::setWalkParameterOneAddValueAndPeriod(float x_one, float y_one, float z_one, float theta_one, int t)
{
	this->walk_one.x = x_one;
	this->walk_one.y = y_one;
	this->walk_one.z = z_one;
	this->walk_one.theta = theta_one;
	this->delay_time_ms = t;
	this->timer.setPeriod(ros::Duration(this->delay_time_ms/1000.0), false);
	// this->wall_timer.start();
	this->setWalkParameterOneAddValueAndPeriod_flag = true;
	return true;
}
void WalkContinuouse::changeValue(float &value, float &value_exp, float &value_one)
{
	if(value < value_exp - value_one)
	{
		value += value_one;
	}
	else if(value > value_exp + value_one)
	{
		value -= value_one;
	}
	else
	{
		value = value_exp;
	}
}
void WalkContinuouse::showAllDatavalue()
{
	std::printf("x_init = %-5d, y_init = %-5d, z_init = %-5d, theta_init = %-5d\n", (int)walk_init.x, (int)walk_init.y, (int)walk_init.z, (int)walk_init.theta);
	std::printf("x_max = %-5d, y_max = %-5d, z_max = %-5d, theta_max = %-5d\n", (int)walk_max.x, (int)walk_max.y, (int)walk_max.z, (int)walk_max.theta);
	std::printf("x_min = %-5d, y_min = %-5d, z_min = %-5d, theta_min = %-5d\n", (int)walk_min.x, (int)walk_min.y, (int)walk_min.z, (int)walk_min.theta);
	std::printf("x_exp = %-5d, y_exp = %-5d, z_exp = %-5d, theta_exp = %-5d\n", (int)walk_exp.x, (int)walk_exp.y, (int)walk_exp.z, (int)walk_exp.theta);
	std::printf("x_one = %-5d, y_one = %-5d, z_one = %-5d, theta_one = %-5d\n", (int)walk_one.x, (int)walk_one.y, (int)walk_one.z, (int)walk_one.theta);
	std::printf("x = %-5d, y = %-5d, z = %-5d, theta = %-5d\n", (int)walk.x, (int)walk.y, (int)walk.z, (int)walk.theta);	
}
void WalkContinuouse::showRealTimeWalkValue()
{
	std::printf("x = %-5d, y = %-5d, z = %-5d, theta = %-5d\n", (int)walk.x, (int)walk.y, (int)walk.z, (int)walk.theta);	
}
bool WalkContinuouse::isStartContinuous()
{
	return this->continuous_flag;
}
int WalkContinuouse::getX()
{
	return (int)walk.x;
}
int WalkContinuouse::getY()
{
	return (int)walk.y;
}
int WalkContinuouse::getZ()
{
	return (int)walk.z;
}
int WalkContinuouse::getTheta()
{
	return (int)walk.theta;
}

WalkContinuouseInstance* WalkContinuouseInstance::m_pInstance;

WalkContinuouseInstance* WalkContinuouseInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new WalkContinuouseInstance();
    return m_pInstance;
}

void WalkContinuouseInstance::deleteInstance()
{
    if (m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = nullptr;
    }
}