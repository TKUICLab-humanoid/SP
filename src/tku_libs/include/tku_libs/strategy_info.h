#ifndef PARTICLEINFO_H
#define PARTICLEINFO_H

#include <vector>
#include <cv_bridge/cv_bridge.h>

enum class DrawMode
{
    DrawLine, DrawObject
};

enum class LabelModel
{
    Orange, Yellow, Blue, Green, Black, Red, White, Other
};
enum class LabelMark
{
    OrangeLabel = 0x01,
    YellowLabel = 0x02,
    BlueLabel = 0x04,  
    GreenLabel = 0x08,
    BlackLabel = 0x10,
    RedLabel = 0x20,
    WhiteLabel = 0x40,
    OthersLabel = 0x80
};

enum class WalkingMode
{
    SingleStep,
    ContinuousStep,
    LC_Step,
    LC_Down,
    LongJump,
    Single_wood,
    Single_third,
    ContinuousStep_second,
    ContinuousStep_third,
    Right_Kick,
    Left_Kick
};

enum class HeadMotorID
{
    HorizontalID = 1, VerticalID = 2
};

enum class SensorMode   // pitch fix X-direction error, roll fix Y-direction error
{
    None,         //without imu
    Roll,         //only roll
    Pitch,        //only pitch
    RollPitch     //roll & pitch
};

enum class ObjectMode
{
    SOCCER,
    GOAL,
    NOTHING
};
typedef union
{
    unsigned char DInput;
    struct
    {
        unsigned D0 :1;
        unsigned D1 :1;
        unsigned D2 :1;
        unsigned D3 :1;
        unsigned NONE0 :1;
        unsigned NONE2 :1;
        unsigned NONE3 :1;
        unsigned NONE4 :1;
    }Switch;
}DIOInput;

class ObjectData
{
    public:
        ObjectData(){}
        ~ObjectData(){}
        int XMax, XMin, YMax, YMin;
        int X;
        int Y;
        int Width;
        int Height;
        int size;
};

class IMUData
{
    public:
        float Roll = 0;
        float Pitch = 0;
        float Yaw = 0;
};

class ObjectPosition2D
{
    public:
        int x;
        int y;
        float dir;
};

struct ObjectInfo
{
    int x;
    int y;
    int height;
    int width;
    int distance;
    int x_distance;
    int y_distance;
    ObjectMode object_mode;
};


class StrategyInfo
{
    public:
        ObjectPosition2D getRobotPosition();
        ObjectPosition2D getGoalPoint();
        IMUData getIMUValue();
        bool getStrategyStart();
    public:
        StrategyInfo();
        ~StrategyInfo();
        bool get_label_model_flag;
        bool get_image_flag;
        int *color_mask_subject_cnts;//[8];
        ObjectData **color_mask_subject;//[8][320];
        DIOInput DIOValue;
        unsigned char *label_model;
        cv_bridge::CvImagePtr cvimg;
        std::vector<ObjectInfo> soccer_info;

        friend class RosCommunication;
    private:
        bool is_start;
        ObjectPosition2D robot_position;
        ObjectPosition2D goal_point;
        IMUData imu_value;
};

class StrategyInfoInstance : public StrategyInfo
{
public:
    StrategyInfoInstance() : StrategyInfo(){}
    ~StrategyInfoInstance(){}
    static StrategyInfoInstance* getInstance();
    static void deleteInstance();
private:
    static StrategyInfoInstance* m_pInstance;
};

#endif // PARTICLEINFO_H

