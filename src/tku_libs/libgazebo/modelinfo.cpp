#include "tku_libs/modelinfo.h"

// gazebo_tool *tool_gz = new gazebo_tool;

gazebo_tool::gazebo_tool()
{
	// printf("remember to call \"tool_gz->set_Client(nh)\"\n");
	// printf("pause_world(void)\n");
	// printf("unpause_world(void)\n");
	// printf("reset_modelpose(void)\n");
	// printf("get_simtime(void)  Current_Sim_Time_sec\n");
	// printf("show_simtime(void)\n");
	// printf("show_allmodels(void)\n");
	// printf("check_model(std::string model_name)\n");
	// printf("set_modelinfo(std::string model_name, StructTFPose TFPose)\n");
	// printf("tool_gz->TFPoseFunction(float x, float y, float z, float R, float P, float Y)\n");
	// printf("get_modelinfo(std::string model_name) data save in mapModelInfo\n");
	// printf("std::map<std::string, StructModelInfo> mapModelInfo\n");
	// printf("ModelInfo.Name\nModelInfo.Pose.x ModelInfo.Pose.y ModelInfo.Pose.z\nModelInfo.Angular.x ModelInfo.Angular.y ModelInfo.Angular.z\nmapModelInfo[ModelInfo.Name] = ModelInfo\n");
}

gazebo_tool::~gazebo_tool()
{

} 

void gazebo_tool::set_Client(ros::NodeHandle &nh)
{
	this->nh = &nh;
	GetModelState_Client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
	SetModelState_Client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	PausePhysics_Client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	UnPausePhysics_Client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	ResetSimulation_Client = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
	GetWorldProperties_Client = nh.serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	GetModelProperties_Client = nh.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
	GetJointProperties_Client = nh.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
	GetLinkState_Client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
}

void gazebo_tool::pause_world(void)
{
	std_srvs::Empty srv;
	PausePhysics_Client.call(srv);
}

void gazebo_tool::unpause_world(void)
{
	std_srvs::Empty srv;
	UnPausePhysics_Client.call(srv);
}

void gazebo_tool::reset_modelpose(void)
{
	std_srvs::Empty srv;
	ResetSimulation_Client.call(srv);
}

void gazebo_tool::get_simtime(double *sim_time)
{
	gazebo_msgs::GetWorldProperties srv;
	if(GetWorldProperties_Client.call(srv))
	{
		*sim_time = srv.response.sim_time;
	}
	else
	{
		ROS_ERROR("Failed to get Sim_Time");
	}
}

void gazebo_tool::show_simtime(void)
{
	gazebo_msgs::GetWorldProperties srv;
	if(GetWorldProperties_Client.call(srv))
	{
		printf("sim_time = %f\n", srv.response.sim_time);
	}
	else
	{
		ROS_ERROR("Failed to get Sim_Time");
	}
}

void gazebo_tool::simDelay(const double delaytime)
{
	gazebo_msgs::GetWorldProperties srv;
	double tstart, tend;
	if(GetWorldProperties_Client.call(srv))
	{
		tstart = srv.response.sim_time;
		tend = tstart;
		while(delaytime > (tend-tstart)*1000 && nh->ok())
		{
			if(GetWorldProperties_Client.call(srv))
			{
				tend = srv.response.sim_time;
			}
			else
			{
				ROS_ERROR("Failed to get Sim_Time, in delayfunction 2");
			}
		}
	}
	else
	{
		ROS_ERROR("Failed to get Sim_Time, in delayfunction 1");
	}
}

void gazebo_tool::show_allmodels(void)
{
	gazebo_msgs::GetWorldProperties srv;
	if(GetWorldProperties_Client.call(srv))
	{
		printf("models = {");
		for(int i=0; i < srv.response.model_names.size(); i++)
		{
			if(i == 0)
			{
				printf("%s", srv.response.model_names[i].c_str());
			}
			else if(i == (srv.response.model_names.size()-1) )
			{
				printf(", %s}\n", srv.response.model_names[i].c_str());
			}
			else
			{
				printf(", %s", srv.response.model_names[i].c_str());
			}
		}
	}
	else
	{
		ROS_ERROR("Failed to get WorldAllModels");
	}
}

vstring gazebo_tool::getAllRobotsName(void)
{
	gazebo_msgs::GetWorldProperties srv;
	if(GetWorldProperties_Client.call(srv))
	{
		return srv.response.model_names;
	}
	else
	{
		ROS_ERROR("Failed to get WorldAllModels");
	}
	
}

bool gazebo_tool::check_model(std::string model_name)
{
	gazebo_msgs::GetWorldProperties srv;
	if(GetWorldProperties_Client.call(srv))
	{
		for(int i=0; i < srv.response.model_names.size(); i++)
		{
			if(model_name == srv.response.model_names[i].c_str())return true;
		}
	}
	else
	{
		std::string tmp = "Failed to check model exist, model name is "+model_name;
		ROS_ERROR(tmp.c_str());
	}
	return false;
}


void gazebo_tool::get_modelinfo(std::string model_name)
{
	if(check_model(model_name))
	{
		double x,y,z,w;
		StructModelInfo ModelInfo;
		gazebo_msgs::GetModelState srv;
		srv.request.model_name = model_name;
		if(GetModelState_Client.call(srv))
		{
			ModelInfo.Name = model_name;
			ModelInfo.Pose.x = srv.response.pose.position.x;
			ModelInfo.Pose.y = srv.response.pose.position.y;
			ModelInfo.Pose.z = srv.response.pose.position.z;
			x = srv.response.pose.orientation.x;
			y = srv.response.pose.orientation.y;
			z = srv.response.pose.orientation.z;
			w = srv.response.pose.orientation.w;
			ModelInfo.Angular.x = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y));
			ModelInfo.Angular.y = asin(2*(w*y - z*x));
			ModelInfo.Angular.z = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
			ModelInfo.Velocity.linear.x = srv.response.twist.linear.x;
			ModelInfo.Velocity.linear.y = srv.response.twist.linear.y;
			ModelInfo.Velocity.linear.z = srv.response.twist.linear.z;
			ModelInfo.Velocity.angular.x = srv.response.twist.angular.x;
			ModelInfo.Velocity.angular.y = srv.response.twist.angular.y;
			ModelInfo.Velocity.angular.z = srv.response.twist.angular.z;
			mapModelInfo[ModelInfo.Name] = ModelInfo;
		}
		else
		{
			std::string tmp = "Failed to get modelinfo, model name is "+model_name;
			ROS_ERROR(tmp.c_str());
		}
	}
	else
	{
		ROS_ERROR("check_model Failed in get_modelinfo Function");
	}
}

void gazebo_tool::set_modelinfo(std::string model_name, StructTFPose TFPose)
{
	if(check_model(model_name))
	{
		gazebo_msgs::SetModelState srv;
		srv.request.model_state.model_name = model_name;
		srv.request.model_state.pose.position.x = TFPose.pose.x;
		srv.request.model_state.pose.position.y = TFPose.pose.y;
		srv.request.model_state.pose.position.z = TFPose.pose.z;
		srv.request.model_state.pose.orientation.x = sin(TFPose.angular.x)*cos(TFPose.angular.y)*cos(TFPose.angular.z) 
		- cos(TFPose.angular.x)*sin(TFPose.angular.y)*sin(TFPose.angular.z);
		srv.request.model_state.pose.orientation.y = cos(TFPose.angular.x)*sin(TFPose.angular.y)*cos(TFPose.angular.z) 
		+ sin(TFPose.angular.x)*cos(TFPose.angular.y)*sin(TFPose.angular.z);
		srv.request.model_state.pose.orientation.z = cos(TFPose.angular.x)*cos(TFPose.angular.y)*sin(TFPose.angular.z) 
		- sin(TFPose.angular.x)*sin(TFPose.angular.y)*cos(TFPose.angular.z);
		srv.request.model_state.pose.orientation.w = cos(TFPose.angular.x)*cos(TFPose.angular.y)*cos(TFPose.angular.z) 
		- sin(TFPose.angular.x)*sin(TFPose.angular.y)*sin(TFPose.angular.z);
		srv.request.model_state.twist.linear.x = 0;
		srv.request.model_state.twist.linear.y = 0;
		srv.request.model_state.twist.linear.z = 0;
		srv.request.model_state.twist.angular.x = 0;
		srv.request.model_state.twist.angular.y = 0;
		srv.request.model_state.twist.angular.z = 0;
		if(!SetModelState_Client.call(srv))
		{
			std::string tmp = "Failed to set modelinfo, model name is "+model_name;
			ROS_ERROR(tmp.c_str());
		}
	}
	else
	{
		ROS_ERROR("check_model Failed in set_modelinfo Function");
	}
}

void gazebo_tool::get_modelproperties(std::string model_name)
{
	gazebo_msgs::GetModelProperties srv;
	srv.request.model_name = model_name;
	if(GetModelProperties_Client.call(srv))
	{
		mapJointName[model_name].joint_name.clear();
		for(int i=0; i<srv.response.joint_names.size(); i++)
		{
			mapJointName[model_name].joint_name.push_back(srv.response.joint_names[i]);
			//ROS_INFO("%s",mapJointName[model_name].joint_name[i].c_str());
		}
		printf("model_name = %s\n", model_name.c_str());
		printf("joints = {");
		for(int i=0; i < mapJointName[model_name].joint_name.size(); i++)
		{
			if(i == 0)
			{
				printf("%s", mapJointName[model_name].joint_name[i].c_str());
			}
			else if(i == (mapJointName[model_name].joint_name.size()-1) )
			{
				printf(", %s}\n", mapJointName[model_name].joint_name[i].c_str());
			}
			else
			{
				printf(", %s", mapJointName[model_name].joint_name[i].c_str());
			}
		}
	}
	else
	{
		std::string tmp = "Failed to get modelproperties, model name is "+model_name;
		ROS_ERROR(tmp.c_str());
	}
}

float gazebo_tool::get_jointproperties(std::string joint_name)
{
	gazebo_msgs::GetJointProperties srv;
	srv.request.joint_name = joint_name;
	if(GetJointProperties_Client.call(srv))
	{
		if(!srv.response.position.empty())
		{
			return srv.response.position[0];
		}
		else
		{
			std::string tmp = "joint position is empty, joint name is "+joint_name;
			ROS_ERROR(tmp.c_str());
			return NULL;
		}
	}
	else
	{
		std::string tmp = "Failed to get jointproperties, joint name is "+joint_name;
		ROS_ERROR(tmp.c_str());
	}
}

float gazebo_tool::get_linkposeZ(std::string link_name)
{
	gazebo_msgs::GetLinkState srv;
	srv.request.link_name = link_name;
	if(GetLinkState_Client.call(srv))
	{
		return srv.response.link_state.pose.position.z;
	}
	else
	{
		std::string tmp = "Failed to get get_linkpose, link name is "+link_name;
		ROS_ERROR(tmp.c_str());
	}
}

StructTFPose gazebo_tool::TFPoseFunction(float x, float y, float z, float R, float P, float Y)
{
	StructTFPose TFPose;
	TFPose.pose.x = x;
	TFPose.pose.y = y;
	TFPose.pose.z = z;
	TFPose.angular.x = R;
	TFPose.angular.y = P;
	TFPose.angular.z = Y;
	return TFPose;
}
