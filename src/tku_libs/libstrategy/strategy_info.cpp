#include "tku_libs/strategy_info.h"

StrategyInfoInstance* StrategyInfoInstance::m_pInstance;

StrategyInfo::StrategyInfo()
{
    get_label_model_flag = false;
    get_image_flag = false;
    is_start = false;
    color_mask_subject_cnts = new int[8]{};
    color_mask_subject = new ObjectData*[8];
    for(int i = 0; i < 8; i++)color_mask_subject[i] = new ObjectData[320]{};
    label_model = new unsigned char [320 * 240]{};
    cvimg = NULL;
}

StrategyInfo::~StrategyInfo()
{
    delete[] color_mask_subject_cnts;
    delete[] color_mask_subject;
    delete[] label_model;
}

ObjectPosition2D StrategyInfo::getRobotPosition()
{
    return robot_position;
}
ObjectPosition2D StrategyInfo::getGoalPoint()
{
    return goal_point;
}
IMUData StrategyInfo::getIMUValue()
{
    return imu_value;
}
bool StrategyInfo::getStrategyStart()
{
    return is_start;
}

StrategyInfoInstance* StrategyInfoInstance::getInstance()
{
    if(!m_pInstance)m_pInstance = new StrategyInfoInstance();
    return m_pInstance;
}

void StrategyInfoInstance::deleteInstance()
{
    if (m_pInstance)
    {
        delete m_pInstance;
        m_pInstance = NULL;
    }
}