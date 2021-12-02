#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "strategy/aruco_matrix.h"
#include "tku_libs/RosCommunication.h"
/****************************************************************************************************/
#include "TSprintinfo.h"

#include "strategy/quickchange.h" 				//网页
#include "strategy/Loadquickchange.h" 			//网页

using namespace std;
ros::Subscriber SaveParameter_subscribe;		//网页
ros::ServiceServer Loadquickchange_service;		//网页
ros::Subscriber Load_arucomatrix;				//aruco
class KidsizeStrategy 
{

public:
	KidsizeStrategy(ros::NodeHandle &nh) 
	{
		Loadquickchange_service = nh.advertiseService("/web/Loadquickchange",&KidsizeStrategy::load_strategy_parameter,this);	//网页
   		SaveParameter_subscribe = nh.subscribe("/web/SaveParameter",1,&KidsizeStrategy::SaveParameter,this);					//网页
		Load_arucomatrix = nh.subscribe("/strategy/ArucoMatrix_Topic",1,&KidsizeStrategy::GetArucoMatrix,this);					//aruco
			///////////////
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();	
	};

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	~KidsizeStrategy(){};
	void strategymain();
/****************************************************************************************************/
	double timeuse;
	struct timeval tstart, tend;
/****************************************************************************************************/
	void determine_object(void);

	void classify_strategy(void);
	bool head_strategy(void);
	void initial_strategy(void);
	
	void GetArucoMatrix(const strategy::aruco_matrix &msg);
	void initparameterpath(void);
	void SaveParameter(const strategy::quickchange &msg);
	bool load_strategy_parameter(strategy::Loadquickchange::Request &Parameter_req, strategy::Loadquickchange::Response &Parameter_res);
	void load_strategy_parameter_ini(void);
	float readvalue(fstream &fin, string title, bool mode);

	void do_sprint_forward_part(int left_pixel, int right_pixel, int left_turn_value, int right_turn_value);
	void do_forward(void);

	void do_sprint_backward_part_yb(double left_dis, double right_dis, int left_turn_value, int right_turn_value);//change_mode = 0
	void do_backward_without_watch_ball(int left_turn_value, int right_turn_value);//change_mode = 1
	void do_backward_ybrat(void);
	float IMU_Value_Yaw(void);
/****************************************aruco*******************************************************/
	void aruco_head_strategy(void);
	void aruco_move_strategy(void);
	void detect_aruco_direction(int left_information, int right_information, int left_turn_value, int right_turn_value);
	void aruco_do_forward(void);
	void aruco_do_backward(void);
/****************************************************************************************************/
	#define PI 3.14159265358979323846
	#define ImageLength 240
	#define ImageWidth 320

	#define Forward_ThetaMax 3
	#define Forward_ThetaMin -3

	#define Backward_ThetaMax 3
	#define Backward_ThetaMin -3

	#define CHECKBACK_CNT 1
	#define Initial_Y 0

	#define HEADERROR 120//old:30
	#define forward_left_pixel 145
	#define forward_right_pixel 175

	#define REDSIZE_50 300//220
	#define REDSIZE_100 440//350
	#define REDSIZE_150 720//600
	#define REDSIZE_200 5000//1200
	#define REDSIZE_250 3300//3100
	#define REDSIZE_300 5000//9100	//without 10000

	#define REDSIZE_losttarget 2500
	#define losttarget_ybrate 1
	#define losttarget_theta 6
/****************************************************************************************************/
	int FRAMCOUNT;//if in change_mode = 0 FARMCOUNT usually be 3!!!!
	int change_mode;
	int total_size;
	int losttmp_theta;

	int backward_initial_x;
	int backward_theta_left_one;
	int backward_theta_right_one;
	int backward_theta_left_two;
	int backward_theta_right_two;
	int backward_theta_left_three;
	int backward_theta_right_three;
	int backward_x_add;
	int backward_initial_theta;
	int backward_x_max;
	int back_total_size;

	int slowdown_250;
	int slowdown_300;
	int slowdown_initial_x; //for do_sprint_backward_part_without_watch_ball 
				//if do_sprint_backward_part_yb you need to change to forward_initial_x
	int forward_initial_x;
	int forward_theta_right_one;
	int forward_theta_left_one;
	int forward_theta_right_two;
	int forward_theta_left_two;
	int forward_theta_right_three;
	int forward_theta_left_three;
	int forward_x_add;
	int forward_initial_theta;
	int forward_x_max;

	int delay_ybtime;
	int without_left;
	int without_right;
	int right_timeuse;
	int initial_timeuse;
	int left_timeuse;
	float IMU_angle_range;
/****************************************************************************************************/
	int Old_Toltal_Size;
	double YBrat;
	bool time_flag = true;
	bool get_timeuse_flag = true;
	bool first_in;	
	bool once_flag=true;
	bool check_headerror = true;
/**********************************************aruco************************************************/
	bool get_arucoimage = true;
	float aruco_direction;
	float aruco_head;
	float aruco_distence;
	int aruco_headmove;
	int aruco_headangle;
	int aruco_headangle_limit;
	int aruco_distence_check;

	#define aruco_back_distence 695
	#define ARUCODISTENCE_50 3115
	#define ARUCODISTENCE_100 2250
	#define ARUCODISTENCE_150 1700
	#define ARUCODISTENCE_200 1200
	#define ARUCODISTENCE_250 900
	#define ARUCODISTENCE_300 800

	#define ARUCO_MIDLE_LINE_0 130 //no use
	#define ARUCO_MIDLE_LINE_50 220
	#define ARUCO_MIDLE_LINE_100 180
	#define ARUCO_MIDLE_LINE_150 140
	#define ARUCO_MIDLE_LINE_200 100
	#define ARUCO_MIDLE_LINE_250 65
	#define ARUCO_MIDLE_LINE_300 32


	string parameter_path = "N";
};
