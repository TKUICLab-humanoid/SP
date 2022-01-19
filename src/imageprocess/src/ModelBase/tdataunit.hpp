#ifndef TDATAUNIT_H
#define TDATAUNIT_H
#include <fstream>
#include <string>
#include <stdlib.h>
#include <string.h>
#include "tku_libs/TKU_tool.h"
//include "../../../strategy/src/StrategyNameAndPath.h"
using namespace std;


struct ColorRange
{
    float HueMin;
    float HueMax;
    float SaturationMin;
    float SaturationMax;
    float BrightnessMin;
    float BrightnessMax;

    string LabelName;
};


class TdataUnit
{
public:
    TdataUnit();
    ~TdataUnit();
    int strategyname;
    void SaveColorRangeFile();
    void LoadColorRangeFile();
    Tool *tool = new Tool();
    ColorRange** HSVColorRange;
};
extern TdataUnit* DataUnit;

#endif // TDATAUNIT_H
