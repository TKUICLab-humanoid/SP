#ifndef ROBOTCUPINFO_H
#define ROBOTCUPINFO_H
#include <string>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <map>
#include "tku_libs/TKU_tool.h"

enum class ECharacter {attacker, suporter1, suporter2, defender, free, null, myself};
enum class EObject {goal, ball};
enum class EEnemy {enemy1, enemy2, enemy3, enemy4};
enum class ERobot {robot1, robot2, robot3, robot4};
//PRS = Parner Robot State
enum class PRS {RA, R12, R13, R14, R23, R24, R34, R1, R2, R3, R4, R};

// class TimeClass
// {
//     public:
//         TimeClass();
//         ~TimeClass();
//         void updateTime();
//         void initialize();
//         void setTimerPass(double check_time_ms, bool init_flag = true);
//         double getTimeMs();
//         bool checkTimePass();

//     private:
//         double start = 0;
//         double end = 0;
//         double time_ms = 0;
//         double check_time_ms = 1000;
// };

class StrE
{
    public:
    StrE(){}
    ~StrE(){}
    static std::string character[];
    static std::string object[];
    static std::string enemy[];
    static std::string robot[];
    static std::string PRS[];
    static unsigned int character_size;
    static unsigned int object_size;
    static unsigned int enemy_size;
    static unsigned int robot_size;
    static unsigned int PRS_size;
};

class ObjectInfoBase
{
    class WhitchData
    {
        public:
        void initialize()
        {
            local = 0;
            global = 0;
        }
        float local = 0;
        float global = 0;
    };

    public:
        virtual void initialize();
        
    public:
        std::string name = "null";
        float x = 0;
        float y = 0;
        bool exist_flag = false;
        WhitchData theta;
        WhitchData dist;
};

class ObjectInfo : public ObjectInfoBase
{
	public:
        ObjectInfo();
        ~ObjectInfo();
};

class CharacterInfo : public ObjectInfoBase
{
	public:
        std::string which_robot = "";
        std::map<std::string, ObjectInfo> object;
        std::map<std::string, ObjectInfo> enemy;
        CharacterInfo();
        ~CharacterInfo();
        void initialize();
};

class NomalCharacterBase
{
    public:
        friend class Ros2MultiCommunication;
        NomalCharacterBase();
        ~NomalCharacterBase();
        void changeMyself(std::string name);
        void changeWitchRobot();
        bool checkRobotCharacter();
        std::string getPRS();
        void setTimerPass(double check_time_ms, bool init_flag = true);
        void testShow();
        void testShowTimer();
        std::map<std::string, CharacterInfo*> who;

    private:
        std::map<std::string, TimeClass> callback_timer;
};

class NomalCharacter : public NomalCharacterBase
{
    public:
        NomalCharacter();
        ~NomalCharacter();
};

class RobotCupInfoBase
{
    public:
        RobotCupInfoBase();
        ~RobotCupInfoBase();
        void initialize();

    public:
        NomalCharacter *character_info;
        std::map<std::string, std::string> topic_names;
    private:
        ros::NodeHandle nh;
        XmlRpc::XmlRpcValue param_data;
};

class RobotCupInfo : public RobotCupInfoBase
{
public:
    RobotCupInfo() : RobotCupInfoBase(){}
    ~RobotCupInfo(){}
};

class RobotCupInfoInstance : public RobotCupInfo
{
    public:
        RobotCupInfoInstance();
        ~RobotCupInfoInstance();
        static RobotCupInfoInstance* getInstance();
        static void deleteInstance();

    private:
        static RobotCupInfoInstance* m_pInstance;
};

#endif