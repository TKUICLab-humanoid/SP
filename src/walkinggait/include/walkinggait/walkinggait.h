/**
 * @file /qlistener/listener.hpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef LISTENER_NODE_HPP_
#define LISTENER_NODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <stdio.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <std_msgs/String.h>

#include "tku_libs/TKU_tool.h"
#include "tku_msgs/Parameter_message.h"
#include "tku_msgs/Walking_message.h"
#include "tku_msgs/Interface.h"
#include "tku_msgs/parameter.h"
#include "tku_msgs/WalkingGaitParameter.h"
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "walkinggait/parameterinfo.h"

#include <string>

#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/mman.h>
#include <stdbool.h>
#include <ros/ros.h>

#define SIZE 100
using namespace std;

ros::Subscriber ContinousMode_subscriber;
ros::Subscriber ChangeContinuousValue_subscriber;
ros::Subscriber chatter_subscriber;
ros::Subscriber save_parameter_subscriber;
ros::Subscriber walkack_subscriber;
ros::Subscriber continuousback_subscriber;
ros::ServiceServer LoadWalkinggaitParameter_service;

ros::Publisher paradata_Pub;
ros::Publisher walkdata_Pub;

#define PI 3.1415926535897932384626433832795	//pi

ToolInstance *tool = ToolInstance::getInstance();

tku_msgs::Parameter_message paradata;
tku_msgs::Walking_message walkdata;

int ContMode = 1;
bool continuousback_flag = false;
int pre_x = 0.0;
int pre_theta = 0;
int IsWalking = false;

#endif /* LISTENER_NODE_HPP_ */
