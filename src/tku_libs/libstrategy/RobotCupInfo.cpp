#include "tku_libs/RobotCupInfo.h"

std::string StrE::object[]    = {"goal", "ball"};
std::string StrE::enemy[]     = {"enemy1", "enemy2", "enemy3", "enemy4"};
std::string StrE::robot[]     = {"robot1", "robot2", "robot3", "robot4"};
std::string StrE::PRS[]       = {"RA", "R12", "R13", "R14", "R23", "R24", "R34", "R1", "R2", "R3", "R4", "R"};
std::string StrE::character[] = {"attacker", "suporter1", "suporter2", "defender", "free", "null", "myself"};
unsigned int StrE::object_size = sizeof(StrE::object)/sizeof(StrE::object[0]);
unsigned int StrE::enemy_size = sizeof(StrE::enemy)/sizeof(StrE::enemy[0]);
unsigned int StrE::robot_size = sizeof(StrE::robot)/sizeof(StrE::robot[0]);
unsigned int StrE::PRS_size = sizeof(StrE::PRS)/sizeof(StrE::PRS[0]);
unsigned int StrE::character_size = sizeof(StrE::character)/sizeof(StrE::character[0]);

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

// void TimeClass::setTimerPass(double check_time_ms, bool init_flag)
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

void ObjectInfoBase::initialize()
{
    x = 0;
    y = 0;
    exist_flag = false;
    theta.initialize();
    dist.initialize();
}

ObjectInfo::ObjectInfo() : ObjectInfoBase()
{

}

ObjectInfo::~ObjectInfo()
{

}

CharacterInfo::CharacterInfo() : ObjectInfoBase()
{
    
    ObjectInfo object_temp;
    for(int i = 0; i < StrE::object_size; i++)
    {
        object_temp.name = StrE::object[i];
        object[StrE::object[i]] = object_temp;
    }
    for(int i = 0; i < StrE::enemy_size; i++)
    {
        object_temp.name = StrE::enemy[i];
        enemy[StrE::enemy[i]] = object_temp;
    }
}

CharacterInfo::~CharacterInfo()
{

}

void CharacterInfo::initialize()
{
    name = StrE::character[(int)ECharacter::null];
    x = 0;
    y = 0;
    exist_flag = false;
    theta.initialize();
    dist.initialize();
    for(std::map<std::string, ObjectInfo>::iterator it = object.begin(); it != object.end(); it++)it->second.initialize();
    for(std::map<std::string, ObjectInfo>::iterator it = enemy.begin(); it != enemy.end(); it++)it->second.initialize();  
}

NomalCharacterBase::NomalCharacterBase()
{
    CharacterInfo *character_temp = new CharacterInfo[StrE::robot_size];
    for(int i = 0; i < StrE::robot_size; i++)
    {
        character_temp[i].which_robot = StrE::robot[i];
        who[StrE::robot[i]] = &character_temp[i];
    }
    TimeClass timeclass_temp;
    timeclass_temp.setTimerPass(3000, false);
    for(int i = 0; i < StrE::robot_size; i++)
    {
        callback_timer[StrE::robot[i]] = timeclass_temp;
    }
}

NomalCharacterBase::~NomalCharacterBase()
{

}

void NomalCharacterBase::changeMyself(std::string name)
{
    who["myself"]->initialize();
    who["myself"]->name = name;
}

void NomalCharacterBase::changeWitchRobot()
{
    // who[who["robot1"]->name] = who["robot1"];
    // who[who["robot2"]->name] = who["robot2"];
    // who[who["robot3"]->name] = who["robot3"];
    // who[who["robot4"]->name] = who["robot4"];
}

bool NomalCharacterBase::checkRobotCharacter()
{
    bool flag = true;
    if(who[StrE::robot[(int)ERobot::robot1]]->name == who[StrE::robot[(int)ERobot::robot2]]->name
     && (who[StrE::robot[(int)ERobot::robot1]]->name != StrE::character[(int)ECharacter::null] && who[StrE::robot[(int)ERobot::robot1]]->name != StrE::character[(int)ECharacter::free]))
    {
        flag = false;
        ROS_ERROR("RobotCharacter redefine, robot1 and robot2 Character are %s", who[StrE::robot[(int)ERobot::robot1]]->name.c_str());
        ros::Duration(1).sleep();
    }
    if(who[StrE::robot[(int)ERobot::robot1]]->name == who[StrE::robot[(int)ERobot::robot3]]->name
     && (who[StrE::robot[(int)ERobot::robot1]]->name != StrE::character[(int)ECharacter::null] && who[StrE::robot[(int)ERobot::robot1]]->name != StrE::character[(int)ECharacter::free]))
    {
        flag = false;
        ROS_ERROR("RobotCharacter redefine, robot1 and robot3 Character are %s", who[StrE::robot[(int)ERobot::robot1]]->name.c_str());
        ros::Duration(1).sleep();
    }
    if(who[StrE::robot[(int)ERobot::robot1]]->name == who[StrE::robot[(int)ERobot::robot4]]->name
     && (who[StrE::robot[(int)ERobot::robot1]]->name != StrE::character[(int)ECharacter::null] && who[StrE::robot[(int)ERobot::robot1]]->name != StrE::character[(int)ECharacter::free]))
    {
        flag = false;
        ROS_ERROR("RobotCharacter redefine, robot1 and robot4 Character are %s", who[StrE::robot[(int)ERobot::robot1]]->name.c_str());
        ros::Duration(1).sleep();
    }
    if(who[StrE::robot[(int)ERobot::robot2]]->name == who[StrE::robot[(int)ERobot::robot3]]->name
     && (who[StrE::robot[(int)ERobot::robot2]]->name != StrE::character[(int)ECharacter::null] && who[StrE::robot[(int)ERobot::robot2]]->name != StrE::character[(int)ECharacter::free]))
    {
        flag = false;
        ROS_ERROR("RobotCharacter redefine, robot2 and robot3 Character are %s", who[StrE::robot[(int)ERobot::robot2]]->name.c_str());
        ros::Duration(1).sleep();
    }
    if(who[StrE::robot[(int)ERobot::robot2]]->name == who[StrE::robot[(int)ERobot::robot4]]->name
     && (who[StrE::robot[(int)ERobot::robot2]]->name != StrE::character[(int)ECharacter::null] && who[StrE::robot[(int)ERobot::robot2]]->name != StrE::character[(int)ECharacter::free]))
    {
        flag = false;
        ROS_ERROR("RobotCharacter redefine, robot2 and robot4 Character are %s", who[StrE::robot[(int)ERobot::robot2]]->name.c_str());
        ros::Duration(1).sleep();
    }
    if(who[StrE::robot[(int)ERobot::robot3]]->name == who[StrE::robot[(int)ERobot::robot4]]->name
     && (who[StrE::robot[(int)ERobot::robot3]]->name != StrE::character[(int)ECharacter::null] && who[StrE::robot[(int)ERobot::robot3]]->name != StrE::character[(int)ECharacter::free]))
    {
        flag = false;
        ROS_ERROR("RobotCharacter redefine, robot3 and robot4 Character are %s", who[StrE::robot[(int)ERobot::robot3]]->name.c_str());
        ros::Duration(1).sleep();
    }
    return flag;
}

std::string NomalCharacterBase::getPRS()
{
    std::string temp = "R";
    for(std::map<std::string, TimeClass>::iterator it = callback_timer.begin(); it != callback_timer.end(); it++)
    {
        if(it->first != who[StrE::character[(int)ECharacter::myself]]->which_robot)
        {
            if(it->second.checkTimePass())
            {
                who[it->first]->initialize();
            }
            else
            {
                temp += it->first[it->first.length()-1];
            }
        }
    }
    return temp;
}

void NomalCharacterBase::setTimerPass(double check_time_ms, bool init_flag)
{
    for(std::map<std::string, TimeClass>::iterator it = callback_timer.begin(); it != callback_timer.end(); it++)
    {
        if(it->first != who[StrE::character[(int)ECharacter::myself]]->which_robot)
        {
            it->second.setTimerPass(check_time_ms, init_flag);
        }
    }
}

void NomalCharacterBase::testShow()
{
    std::printf("\n");
    for(std::map<std::string, CharacterInfo*>::iterator it = who.begin(); it != who.end(); it++)
    {
        while(it->first == StrE::character[(int)ECharacter::myself]  || it->first == StrE::character[(int)ECharacter::attacker]
         || it->first == StrE::character[(int)ECharacter::suporter1] || it->first == StrE::character[(int)ECharacter::suporter2]
         || it->first == StrE::character[(int)ECharacter::defender]  || it->first == StrE::character[(int)ECharacter::free]
         || it->first == StrE::character[(int)ECharacter::null])
        {
            it++;
            if(it == who.end())return;
        }
        std::printf("%-10s {%-10sx = %-8.2f, y = %-8.2f, exist_flag = %-d, theta[ local = %-8.2f, global = %-8.2f], dist[ local = %-8.2f, global = %-8.2f]}\n"
        ,it->second->which_robot.c_str(), it->second->name.c_str(), it->second->x, it->second->y, it->second->exist_flag
        ,it->second->theta.local, it->second->theta.global, it->second->dist.local, it->second->dist.global);

        for(std::map<std::string, ObjectInfo>::iterator itt = it->second->object.begin(); itt != it->second->object.end(); itt++)
        {
            std::printf("%-10s {%-10sx = %-8.2f, y = %-8.2f, exist_flag = %-d, theta[ local = %-8.2f, global = %-8.2f], dist[ local = %-8.2f, global = %-8.2f]}\n"
            ,it->second->name.c_str(), itt->second.name.c_str(), itt->second.x, itt->second.y, itt->second.exist_flag
            ,itt->second.theta.local, itt->second.theta.global, itt->second.dist.local, itt->second.dist.global);
        }
        for(std::map<std::string, ObjectInfo>::iterator itt = it->second->enemy.begin(); itt != it->second->enemy.end(); itt++)
        {
            std::printf("%-10s {%-10sx = %-8.2f, y = %-8.2f, exist_flag = %-d, theta[ local = %-8.2f, global = %-8.2f], dist[ local = %-8.2f, global = %-8.2f]}\n"
            ,it->second->name.c_str(), itt->second.name.c_str(), itt->second.x, itt->second.y, itt->second.exist_flag
            ,itt->second.theta.local, itt->second.theta.global, itt->second.dist.local, itt->second.dist.global);
        }
    }
}

void NomalCharacterBase::testShowTimer()
{
    for(int i = 0; i < StrE::robot_size; i++)
    {
        ROS_INFO("%s_timer = %f", StrE::robot[i].c_str(), callback_timer[StrE::robot[i]].getTimeMs());
    }
}

NomalCharacter::NomalCharacter() : NomalCharacterBase()
{

}

NomalCharacter::~NomalCharacter()
{

}

RobotCupInfoBase::RobotCupInfoBase()
{
    character_info = new NomalCharacter;
    std::string str_temp = "rosparam load " + ros::package::getPath("strategy") + "/Parameter/robotcup_info.yaml /robotcup_info";
    system(str_temp.c_str());
    if (nh.getParam("robotcup_info", param_data) && param_data.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
        for(int i = 0; i < StrE::robot_size; i++)
        {
            character_info->who[StrE::robot[i]]->name = (std::string)param_data[0]["state"][StrE::robot[i]];
            topic_names[StrE::robot[i]] = (std::string)param_data[0]["topic"][StrE::robot[i]];
        }
        character_info->who[StrE::character[(int)ECharacter::myself]] = character_info->who[(std::string)param_data[0]["state"]["which_robot"]];
    }
    // initialize();
}

RobotCupInfoBase::~RobotCupInfoBase()
{
    delete character_info;
}

void RobotCupInfoBase::initialize()
{
    
}

RobotCupInfoInstance* RobotCupInfoInstance::m_pInstance;

RobotCupInfoInstance::RobotCupInfoInstance() : RobotCupInfo()
{

}

RobotCupInfoInstance::~RobotCupInfoInstance()
{

}

RobotCupInfoInstance* RobotCupInfoInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new RobotCupInfoInstance();
    return m_pInstance;
}

void RobotCupInfoInstance::deleteInstance()
{
    if (m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = nullptr;
    }
}