#include "tku_libs/RosCommunication.h"

RosCommunicationInstance* RosCommunicationInstance::m_pInstance;

RosCommunicationInstance* RosCommunicationInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new RosCommunicationInstance();
    return m_pInstance;
}

void RosCommunicationInstance::deleteInstance()
{
    if (m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = nullptr;
    }
}

bool RosCommunicationInstance::isInitialize()
{
    if(!is_initialize)ROS_ERROR("Did you forget to initialize for RosCommunicatio?");
    return is_initialize;
}

RosCommunication::RosCommunication()
{
    initialize();
}

RosCommunication::~RosCommunication()
{

}

void RosCommunication::initialize()
{
    walkingGait_pub = nh.advertise<tku_msgs::Interface>("SendBodyAuto_Topic", 100);
    head_motor_pub = nh.advertise<tku_msgs::HeadPackage>("/package/HeadMotor", 10);
    sector_pub = nh.advertise<std_msgs::Int16>("/package/Sector", 100);
    hand_speed_pub = nh.advertise<tku_msgs::SandHandSpeed>("/package/motorspeed", 100);
    draw_image_pub = nh.advertise<tku_msgs::DrawImage>("/strategy/drawimage", 100);
    continuous_value_pub = nh.advertise<tku_msgs::Interface>("/ChangeContinuousValue_Topic", 100);
    single_motor_data_pub = nh.advertise<tku_msgs::SingleMotorData>("/package/SingleMotorData", 100);
    sensor_pub = nh.advertise<tku_msgs::SensorSet>("sensorset", 100);

    object_list_sub = nh.subscribe("/Object/List", 10, &RosCommunication::getObject,this);
    label_model_sub = nh.subscribe("/LabelModel/List", 10, &RosCommunication::getLabelModel,this);
    compress_image_sub = nh.subscribe("compress_image", 10, &RosCommunication::catchImage,this);
    start_sub = nh.subscribe("/web/start", 10, &RosCommunication::startFunction,this);
    DIO_ack_sub = nh.subscribe("/package/FPGAack", 10, &RosCommunication::DIOackFunction,this);
    robot_position_sub = nh.subscribe("/localization/robotpos", 10, &RosCommunication::getRobotPos,this);
    goal_point_sub = nh.subscribe("/web/setgoalpoint", 10, &RosCommunication::getGoalPoint,this);
    soccer_data_sub = nh.subscribe("/vision/soccer_topic", 10, &RosCommunication::getSoccerData,this);
    sensor_sub = nh.subscribe("/package/sensorpackage", 10, &RosCommunication::sensorPackageFunction,this);

    strategy_info = StrategyInfoInstance::getInstance();
    is_initialize = true;
}

bool RosCommunication::isInitialize()
{
    if(!is_initialize)ROS_ERROR("Did you forget to initialize for RosCommunication?");
    return is_initialize;
}

void RosCommunication::closeSubscriber(std::string sum_name)
{
    if(sum_name == "object_list_sub")
    {
        object_list_sub.shutdown();
    }
    else if(sum_name == "label_model_sub")
    {
        label_model_sub.shutdown();
    }
    else if(sum_name == "compress_image_sub")
    {
        compress_image_sub.shutdown();
    }
    else if(sum_name == "DIO_ack_sub")
    {
        DIO_ack_sub.shutdown();
    }
    else if(sum_name == "start_sub")
    {
        start_sub.shutdown();
    }
    else
    {
        ROS_ERROR("no name [%s] subscriber", sum_name.c_str());
        ROS_ERROR("[object_list_sub, label_model_sub, compress_image_sub, DIO_ack_sub, start_sub]");
    }
    
}

void RosCommunication::sendBodyAuto(int x, int y, int z, int theta, WalkingMode mode, SensorMode sensor)
{
    tku_msgs::Interface walkdata;
    walkdata.x = x;
    walkdata.y = y;
    walkdata.z = z;
    walkdata.theta = theta;
    walkdata.walking_mode = (int)mode;
    walkdata.sensor_mode = (int)sensor;
    walkingGait_pub.publish(walkdata);
}

void RosCommunication::sendContinuousValue(int x, int y,  int z, int theta, SensorMode sensor)
{
    tku_msgs::Interface walkdata;
    walkdata.x = x;
    walkdata.y = y;
    walkdata.z = z;
    walkdata.theta = theta;
    walkdata.sensor_mode = (int)sensor;
    continuous_value_pub.publish(walkdata);
}

void RosCommunication::drawImageFunction(int cnt, DrawMode mode, int xmin, int xmax, int ymin, int ymax, int r, int g, int b)
{
    tku_msgs::DrawImage ImageData;
    ImageData.cnt = cnt;
    ImageData.XMax = xmax;
    ImageData.XMin = xmin;
    ImageData.YMax = ymax;
    ImageData.YMin = ymin;
    ImageData.rValue = r;
    ImageData.gValue = g;
    ImageData.bValue = b;
    ImageData.Mode = (int)mode;
    draw_image_pub.publish(ImageData);
}

void RosCommunication::sendHeadMotor(HeadMotorID ID, int Position, int Speed)
{
    tku_msgs::HeadPackage HeadData;
    HeadData.ID = (int)ID;
    HeadData.Position = Position;
    HeadData.Speed = Speed;
    head_motor_pub.publish(HeadData);
}

void RosCommunication::sendBodySector(int Sector)
{
    std_msgs::Int16 SectorData;
    SectorData.data = Sector;
    sector_pub.publish(SectorData);
}

void RosCommunication::sendHandSpeed(int Sector, int Speed)
{
    tku_msgs::SandHandSpeed HandSpeedData;
    HandSpeedData.sector = Sector;
    HandSpeedData.speed = Speed;
    hand_speed_pub.publish(HandSpeedData);
}

void RosCommunication::sendSingleMotor(int ID, int Position, int Speed)
{
    tku_msgs::SingleMotorData MotorData;
    MotorData.ID = ID;
    MotorData.Position = Position;
    MotorData.Speed = Speed;
    single_motor_data_pub.publish(MotorData);
}

void RosCommunication::sendSensorSet(float P, float I, float D, int modeset)
{
    tku_msgs::SensorSet msg;
    msg.sensor_P = P * 1000;
    msg.sensor_I = I * 1000;
    msg.sensor_D = D * 1000;
    msg.sensor_modeset = modeset;
    sensor_pub.publish(msg);
}

void RosCommunication::sendSensorReset()
{
    tku_msgs::SensorSet msg;
    msg.sensor_modeset = 0x02;
    sensor_pub.publish(msg);
}

void RosCommunication::getObject(const tku_msgs::ObjectList &msg)
{
    if(strategy_info->get_image_flag)
    {
        for(int i = 0; i < 8; i++)
        {
            strategy_info->color_mask_subject_cnts[i] = msg.Objectlist[i].cnt;
            for(int j = 0; j < strategy_info->color_mask_subject_cnts[i]; j++)
            {
                strategy_info->color_mask_subject[i][j].X = msg.Objectlist[i].Colorarray[j].X;
                strategy_info->color_mask_subject[i][j].Y = msg.Objectlist[i].Colorarray[j].Y;
                strategy_info->color_mask_subject[i][j].XMin = msg.Objectlist[i].Colorarray[j].XMin;
                strategy_info->color_mask_subject[i][j].YMin = msg.Objectlist[i].Colorarray[j].YMin;
                strategy_info->color_mask_subject[i][j].XMax = msg.Objectlist[i].Colorarray[j].XMax;
                strategy_info->color_mask_subject[i][j].YMax = msg.Objectlist[i].Colorarray[j].YMax;
                strategy_info->color_mask_subject[i][j].Width = msg.Objectlist[i].Colorarray[j].Width;
                strategy_info->color_mask_subject[i][j].Height = msg.Objectlist[i].Colorarray[j].Height;
                strategy_info->color_mask_subject[i][j].size = msg.Objectlist[i].Colorarray[j].size;
            }
        }
    }
    strategy_info->get_image_flag = false;
}

void RosCommunication::getLabelModel(const tku_msgs::LabelModelObjectList &msg)
{
    for(int i = 0; i < 320*240; i++)
    {
        strategy_info->label_model[i] = msg.LabelModel[i];
    }
    strategy_info->get_label_model_flag = false;
}

void RosCommunication::catchImage(const sensor_msgs::ImageConstPtr& img)
{
    strategy_info->cvimg = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
}

void RosCommunication::startFunction(const std_msgs::Bool &msg)
{
    if(msg.data)
    {
        ROS_INFO("start");
        strategy_info->is_start = true;
    }
    else
    {
        ROS_INFO("stop");
        strategy_info->is_start = false;
    }
}

void RosCommunication::DIOackFunction(const std_msgs::Int16 &msg)
{
    if(msg.data & 0x10)
        strategy_info->is_start = true;
    else
        strategy_info->is_start = false;
    strategy_info->DIOValue.DInput = msg.data;
}

void RosCommunication::getRobotPos(const tku_msgs::RobotPos &msg)
{
    strategy_info->robot_position.x = msg.x;
    strategy_info->robot_position.y = msg.y;
    strategy_info->robot_position.dir = msg.dir;
}

void RosCommunication::getGoalPoint(const tku_msgs::SetGoalPoint &msg)
{
    if(msg.number == 0)
    {
        strategy_info->goal_point.x = msg.x;
        strategy_info->goal_point.y = msg.y;
    }
}

void RosCommunication::getSoccerData(const tku_msgs::SoccerDataList &msg)
{
    for(int i = 0; i < msg.object_cnt; i++)
    {
        ObjectInfo tmp;
        switch(msg.ObjectList[i].object_mode)
        {
            case 0:
                tmp.object_mode = ObjectMode::SOCCER;
                break;
            case 1:
                tmp.object_mode = ObjectMode::GOAL;
                break;
            case 2:
                tmp.object_mode = ObjectMode::NOTHING;
                break;
        }
        tmp.x = msg.ObjectList[i].x;
        tmp.y = msg.ObjectList[i].y;
        tmp.height = msg.ObjectList[i].height;
        tmp.width = msg.ObjectList[i].width;
        tmp.x_distance = msg.ObjectList[i].distance.x_dis;
        tmp.y_distance = msg.ObjectList[i].distance.y_dis;
        tmp.distance = msg.ObjectList[i].distance.dis;
        strategy_info->soccer_info.push_back(tmp);
    }
}


void RosCommunication::sensorPackageFunction(const tku_msgs::SensorPackage &msg)
{
    if(msg.IMUData.empty())return;
    if(msg.IMUData[0] == 0 && msg.IMUData[1] == 0 && msg.IMUData[2] == 0)return;
    strategy_info->imu_value.Roll  = msg.IMUData[0];
    strategy_info->imu_value.Pitch = msg.IMUData[1];
    strategy_info->imu_value.Yaw   = msg.IMUData[2];
}
