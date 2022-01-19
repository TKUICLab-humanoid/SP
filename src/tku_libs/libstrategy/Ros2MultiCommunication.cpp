#include "tku_libs/Ros2MultiCommunication.h"

Ros2MultiCommunicationInstance* Ros2MultiCommunicationInstance::m_pInstance;

Ros2MultiCommunication::Ros2MultiCommunication()
{
    robotcup_info = RobotCupInfoInstance::getInstance();
    // TimeClass timeclass_temp;
    // timeclass_temp.setTimePass(3000, false);
    // for(int i = 0; i < StrE::robot_size; i++)
    // {
    //     callback_timer[StrE::robot[i]] = timeclass_temp;
    // }
    if(robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot == StrE::robot[(int)ERobot::robot1])
    {
        robots_pub[StrE::robot[(int)ERobot::robot1]] = nh.advertise<tku_msgs::Ros2MultiCommunication>(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot1]], 10);
        robots_sub[StrE::robot[(int)ERobot::robot2]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot2]], 10, &Ros2MultiCommunication::callBackRobot2, this);
        robots_sub[StrE::robot[(int)ERobot::robot3]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot3]], 10, &Ros2MultiCommunication::callBackRobot3, this);
        robots_sub[StrE::robot[(int)ERobot::robot4]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot4]], 10, &Ros2MultiCommunication::callBackRobot4, this);
    }
    else if(robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot == StrE::robot[(int)ERobot::robot2])
    {
        robots_pub[StrE::robot[(int)ERobot::robot2]] = nh.advertise<tku_msgs::Ros2MultiCommunication>(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot2]], 10);
        robots_sub[StrE::robot[(int)ERobot::robot1]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot1]], 10, &Ros2MultiCommunication::callBackRobot1, this);
        robots_sub[StrE::robot[(int)ERobot::robot3]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot3]], 10, &Ros2MultiCommunication::callBackRobot3, this);
        robots_sub[StrE::robot[(int)ERobot::robot4]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot4]], 10, &Ros2MultiCommunication::callBackRobot4, this);
    }
    else if(robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot == StrE::robot[(int)ERobot::robot3])
    {
        robots_pub[StrE::robot[(int)ERobot::robot3]] = nh.advertise<tku_msgs::Ros2MultiCommunication>(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot3]], 10);
        robots_sub[StrE::robot[(int)ERobot::robot1]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot1]], 10, &Ros2MultiCommunication::callBackRobot1, this);
        robots_sub[StrE::robot[(int)ERobot::robot2]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot2]], 10, &Ros2MultiCommunication::callBackRobot2, this);
        robots_sub[StrE::robot[(int)ERobot::robot4]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot4]], 10, &Ros2MultiCommunication::callBackRobot4, this);
    }
    else if(robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot == StrE::robot[(int)ERobot::robot4])
    {
        robots_pub[StrE::robot[(int)ERobot::robot4]] = nh.advertise<tku_msgs::Ros2MultiCommunication>(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot4]], 10);
        robots_sub[StrE::robot[(int)ERobot::robot1]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot1]], 10, &Ros2MultiCommunication::callBackRobot1, this);
        robots_sub[StrE::robot[(int)ERobot::robot2]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot2]], 10, &Ros2MultiCommunication::callBackRobot2, this);
        robots_sub[StrE::robot[(int)ERobot::robot3]] = nh.subscribe(robotcup_info->topic_names[StrE::robot[(int)ERobot::robot3]], 10, &Ros2MultiCommunication::callBackRobot3, this);
    }
    else
    {
        ROS_ERROR("whitch_robot name = %s, robot1 or robot2 or robot3 or robot4", robotcup_info->topic_names["whitch_robot"].c_str());
        ROS_INFO("Please ctrl c to exit");
        while(ros::ok());
    }
    is_initialize = true;
    // initialize();
}

Ros2MultiCommunication::~Ros2MultiCommunication()
{

}

void Ros2MultiCommunication::closeSubscriber(std::string sum_name)
{

}

// std::string Ros2MultiCommunication::getPRS()
// {
//     std::string temp = "R";
//     for(std::map<std::string, TimeClass>::iterator it = callback_timer.begin(); it != callback_timer.end(); it++)
//     {
//         if(it->first != robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot)
//         {
//             if(it->second.checkTimePass())
//             {
//                 robotcup_info->character_info->who[it->first]->initialize();
//             }
//             else
//             {
//                 temp += it->first[it->first.length()-1];
//             }
//         }
//     }
//     return temp;
// }

// void Ros2MultiCommunication::setTimePass(double check_time_ms, bool init_flag)
// {
//     for(std::map<std::string, TimeClass>::iterator it = callback_timer.begin(); it != callback_timer.end(); it++)
//     {
//         if(it->first != robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot)
//         {
//             it->second.setTimePass(check_time_ms, init_flag);
//         }
//     }
// }

void Ros2MultiCommunication::initialize()
{
    
}

bool Ros2MultiCommunication::isInitialize()
{
    if(!is_initialize)ROS_ERROR("Did you forget to initialize for Ros2MultiCommunication?");
    return is_initialize;
}

void Ros2MultiCommunication::sendRobotCupInfo(RobotCupInfo *robotcup_info)
{
    tku_msgs::ObjectInfo enemy_temp;
    tku_msgs::ObjectInfo object_temp;
    tku_msgs::CharacterInfo character_temp;
    tku_msgs::Ros2MultiCommunication myself;

    character_temp.which_robot = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot;
    character_temp.name = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->name;
    character_temp.x = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->x;
    character_temp.y = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->y;
    character_temp.exist_flag = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->exist_flag;
    character_temp.theta.local = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->theta.local;
    character_temp.theta.global = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->theta.global;
    character_temp.dist.local = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->dist.local;
    character_temp.dist.global = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->dist.global;

    for(int i = 0; i < StrE::object_size; i++)
    {
        object_temp.name = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].name;
        object_temp.x = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].x;
        object_temp.y = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].y;
        object_temp.exist_flag = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].exist_flag;
        object_temp.theta.local = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].theta.local;
        object_temp.theta.global = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].theta.global;
        object_temp.dist.local = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].dist.local;
        object_temp.dist.global = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->object[StrE::object[i]].dist.global;
        character_temp.object.push_back(object_temp);
    }
    
    for(int i = 0; i < StrE::enemy_size; i++)
    {
        enemy_temp.name = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].name;
        enemy_temp.x = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].x;
        enemy_temp.y = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].y;
        enemy_temp.exist_flag = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].exist_flag;
        enemy_temp.theta.local = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].theta.local;
        enemy_temp.theta.global = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].theta.global;
        enemy_temp.dist.local = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].dist.local;
        enemy_temp.dist.global = robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->enemy[StrE::enemy[i]].dist.global;
        character_temp.enemy.push_back(enemy_temp);
    }

    myself.who.push_back(character_temp);
    robots_pub[robotcup_info->character_info->who[StrE::character[(int)ECharacter::myself]]->which_robot].publish(myself);
}
void Ros2MultiCommunication::saveCallBackData(const tku_msgs::Ros2MultiCommunication& unknow_robot)
{
    for(int i = 0; i < unknow_robot.who.size(); i++)
    {
        if(unknow_robot.who[i].name != StrE::character[(int)ECharacter::attacker] && unknow_robot.who[i].name != StrE::character[(int)ECharacter::suporter1] && unknow_robot.who[i].name != StrE::character[(int)ECharacter::suporter2]
         && unknow_robot.who[i].name != StrE::character[(int)ECharacter::defender] && unknow_robot.who[i].name != StrE::character[(int)ECharacter::free] && unknow_robot.who[i].name != StrE::character[(int)ECharacter::null])break;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->which_robot = unknow_robot.who[i].which_robot;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->name = unknow_robot.who[i].name;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->x = unknow_robot.who[i].x;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->y = unknow_robot.who[i].y;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->exist_flag = unknow_robot.who[i].exist_flag;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->theta.local = unknow_robot.who[i].theta.local;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->theta.global = unknow_robot.who[i].theta.global;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->dist.local = unknow_robot.who[i].dist.local;
        robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->dist.global = unknow_robot.who[i].dist.global;
        for(int j = 0; j < unknow_robot.who[i].enemy.size(); j++)
        {
            // if(unknow_robot.who[i].enemy[j].name != StrE::enemy[(int)EEnemy::enemy1] && unknow_robot.who[i].enemy[j].name != StrE::enemy[(int)EEnemy::enemy2]
            //  && unknow_robot.who[i].enemy[j].name != StrE::enemy[(int)EEnemy::enemy3] && unknow_robot.who[i].enemy[j].name != StrE::enemy[(int)EEnemy::enemy4])break;
            if(unknow_robot.who[i].enemy[j].name == "")break;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].name = unknow_robot.who[i].enemy[j].name;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].x = unknow_robot.who[i].enemy[j].x;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].y = unknow_robot.who[i].enemy[j].y;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].exist_flag = unknow_robot.who[i].enemy[j].exist_flag;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].theta.local = unknow_robot.who[i].enemy[j].theta.local;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].theta.global = unknow_robot.who[i].enemy[j].theta.global;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].dist.local = unknow_robot.who[i].enemy[j].dist.local;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->enemy[unknow_robot.who[i].enemy[j].name].dist.global = unknow_robot.who[i].enemy[j].dist.global;
        }
        for(int j = 0; j < unknow_robot.who[i].object.size(); j++)
        {
            // if(unknow_robot.who[i].object[j].name != StrE::object[(int)EObject::ball] && unknow_robot.who[i].object[j].name != StrE::object[(int)EObject::goal])break;
            if(unknow_robot.who[i].object[j].name == "")break;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].name = unknow_robot.who[i].object[j].name;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].x = unknow_robot.who[i].object[j].x;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].y = unknow_robot.who[i].object[j].y;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].exist_flag = unknow_robot.who[i].object[j].exist_flag;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].theta.local = unknow_robot.who[i].object[j].theta.local;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].theta.global = unknow_robot.who[i].object[j].theta.global;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].dist.local = unknow_robot.who[i].object[j].dist.local;
            robotcup_info->character_info->who[unknow_robot.who[i].which_robot]->object[unknow_robot.who[i].object[j].name].dist.global = unknow_robot.who[i].object[j].dist.global;
        }
    }
}

void Ros2MultiCommunication::callBackRobot1(const tku_msgs::Ros2MultiCommunication& unknow_robot)
{
    saveCallBackData(unknow_robot);
    // robotcup_info->character_info->changeWitchRobot();
    robotcup_info->character_info->callback_timer[StrE::robot[(int)ERobot::robot1]].initialize();
}
void Ros2MultiCommunication::callBackRobot2(const tku_msgs::Ros2MultiCommunication& unknow_robot)
{
    // std::printf("a222\n");
    saveCallBackData(unknow_robot);
    // robotcup_info->character_info->changeWitchRobot();
    robotcup_info->character_info->callback_timer[StrE::robot[(int)ERobot::robot2]].initialize();
}
void Ros2MultiCommunication::callBackRobot3(const tku_msgs::Ros2MultiCommunication& unknow_robot)
{
    saveCallBackData(unknow_robot);
    // robotcup_info->character_info->changeWitchRobot();
    robotcup_info->character_info->callback_timer[StrE::robot[(int)ERobot::robot3]].initialize();
}
void Ros2MultiCommunication::callBackRobot4(const tku_msgs::Ros2MultiCommunication& unknow_robot)
{
    saveCallBackData(unknow_robot);
    // robotcup_info->character_info->changeWitchRobot();
    robotcup_info->character_info->callback_timer[StrE::robot[(int)ERobot::robot4]].initialize();
}

// void Ros2MultiCommunication::testShowTimer()
// {
//     ROS_INFO("%f", robotcup_info->character_info->callback_timer["robot1"].getTimeMs());
//     ROS_INFO("%f", robotcup_info->character_info->callback_timer["robot2"].getTimeMs());
//     ROS_INFO("%f", robotcup_info->character_info->callback_timer["robot3"].getTimeMs());
//     ROS_INFO("%f", robotcup_info->character_info->callback_timer["robot4"].getTimeMs());
// }

Ros2MultiCommunicationInstance::~Ros2MultiCommunicationInstance()
{

}

Ros2MultiCommunicationInstance* Ros2MultiCommunicationInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new Ros2MultiCommunicationInstance();
    return m_pInstance;
}

void Ros2MultiCommunicationInstance::deleteInstance()
{
    if (m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = nullptr;
    }
}

bool Ros2MultiCommunicationInstance::isInitialize()
{
    if(!is_initialize)ROS_ERROR("Did you forget to initialize for Ros2MultiCommunication?");
    return is_initialize;
}

// TimeClass::TimeClass()
// {
//     // initialize();
// }

// TimeClass::~TimeClass()
// {

// }

// void TimeClass::updateTime()
// {
//     end = ros::WallTime::now().toSec();
//     time_ms = 1000.0*(end - start);
// }

// void TimeClass::initialize()
// {
//     start = ros::WallTime::now().toSec();
//     end = start;
//     time_ms = 0;
// }

// void TimeClass::setTimePass(double check_time_ms, bool init_flag)
// {
//     this->check_time_ms = check_time_ms;
//     if(init_flag)initialize();
// }

// double TimeClass::getTimeMs()
// {
//     updateTime();
//     return time_ms;
// }

// bool TimeClass::checkTimePass()
// {
//     if(getTimeMs() > check_time_ms)
//     {
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }