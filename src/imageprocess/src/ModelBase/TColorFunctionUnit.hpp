#ifndef TCOLORFUNCTIONUNIT_H_
#define TCOLORFUNCTIONUNIT_H_

/*****************************************************************************
** Includes
*****************************************************************************/

/*****************************************************************************
** Class
*****************************************************************************/

class TColorFunctionUnit
{
private:
    float RtmpValue, GtmpValue, BtmpValue;
    float HtmpValue, StmpValue, VtmpValue;
    float maxHSV, minHSV;

    void ExecuteMaxMin(float &rValue, float &gValue, float &bValue);
    void HSVtoRGB(float hValue, float sValue, float vValue);

public:
    TColorFunctionUnit();
    ~TColorFunctionUnit();
        //------HSV to RGB------//
        float RofHSVtoRGB(float hValue, float sValue, float vValue);
        float GofHSVtoRGB(float hValue, float sValue, float vValue);
        float BofHSVtoRGB(float hValue, float sValue, float vValue);
        //------RGB to HSV------//
        float HofRGBtoHSV(float rValue, float gValue, float bValue);
        float SofRGBtoHSV(float rValue, float gValue, float bValue);
        float VofRGBtoHSV(float rValue, float gValue, float bValue);

};
extern TColorFunctionUnit* ColorFunctionUnit;

#endif /* _CLASSINFO_H_ */
