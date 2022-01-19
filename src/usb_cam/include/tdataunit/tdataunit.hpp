#ifndef TDATAUNIT_H
#define TDATAUNIT_H
#include <fstream>
#include <string>
#include <stdlib.h>
#include <string.h>
#include "tku_libs/TKU_tool.h"

using namespace std;


struct CameraParameter
{
    int  auto_exposure;
    int  auto_white_balance;
    int  auto_Backlight_Compensation;
    int  brightness;
    int  contrast;
    int  saturation;
    int  white_balance;
    string ParameterName;
};


class TdataUnit
{
public:
    TdataUnit();
    ~TdataUnit();
    void SaveCameraSetFile();
    void LoadCameraSetFile();
    ToolInstance *tool;

    CameraParameter* CameraParameterValue;
};
extern TdataUnit* DataUnit;

#endif // TDATAUNIT_H
