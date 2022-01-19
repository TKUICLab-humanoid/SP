/*****************************************************************************
** Includes
*****************************************************************************/

#include "TColorFunctionUnit.hpp"

TColorFunctionUnit* ColorFunctionUnit = new TColorFunctionUnit();
/*****************************************************************************
** Implementation
*****************************************************************************/

TColorFunctionUnit::TColorFunctionUnit()
{
    this->maxHSV = 0.0;
    this->minHSV = 0.0;

    this->RtmpValue = 0.0;
    this->GtmpValue = 0.0;
    this->BtmpValue = 0.0;

    this->HtmpValue = 0.0;
    this->StmpValue = 0.0;
    this->VtmpValue = 0.0;
}

TColorFunctionUnit::~TColorFunctionUnit()
{

}
//----------------------ExecuteMaxMin--------------------------------
void TColorFunctionUnit::ExecuteMaxMin(float &rValue, float &gValue, float &bValue)
{
    if(rValue > gValue)                     //R > G
    {
        if(rValue > bValue)                 //R > G && R > B
        {
            this->maxHSV = rValue;
            if(gValue > bValue)
                this->minHSV = bValue;      //R > G > B
            else
                this->minHSV = gValue;      //R > B > G
        }
        else                                //B > R > G
        {
            this->maxHSV = bValue;
            this-> minHSV = gValue;
        }
    }
    else                                    //G > R
    {
        if(gValue > bValue)                 //G > R && G > B
        {
            this->maxHSV = gValue;
            if(rValue > bValue)
                this->minHSV = bValue;      //G > R > B
            else
                this->minHSV = rValue;      //G > B > R
        }
        else                                //B > G > R
        {
            this->maxHSV = bValue;
            this->minHSV = rValue;
        }
    }
}
//-----------------------HSVtoRGB-------------------
void TColorFunctionUnit::HSVtoRGB(float hValue, float sValue, float vValue)
{
    if(sValue == 0.0)
    {
        this->RtmpValue = vValue;
        this->GtmpValue = vValue;
        this->BtmpValue = vValue;
    }
    else
    {
        int HueInt;
        float Huefloat;
        float pValue, qValue, tValue;

        HueInt = ((int)hValue / 60) % 6;
        Huefloat = (float)((hValue / 60.0) - (float)HueInt);
        pValue = (float)(vValue * (1.0 - sValue));
        qValue = (float)(vValue * (1.0 - Huefloat * sValue));
        tValue = (float)(vValue * (1.0 - (1.0 - Huefloat) * sValue));

        switch(HueInt)
        {
            case 0:
            {
                this->RtmpValue = vValue;
                this->GtmpValue = tValue;
                this->BtmpValue = pValue;
                break;
            }
            case 1:
            {
                this->RtmpValue = qValue;
                this->GtmpValue = vValue;
                this->BtmpValue = pValue;
                break;
            }
            case 2:
            {
                this->RtmpValue = pValue;
                this->GtmpValue = vValue;
                this->BtmpValue = tValue;
                break;
            }
            case 3:
            {
                this->RtmpValue = pValue;
                this->GtmpValue = qValue;
                this->BtmpValue = vValue;
                break;
            }
            case 4:
            {
                this->RtmpValue = tValue;
                this->GtmpValue = pValue;
                this->BtmpValue = vValue;
                break;
            }
            case 5:
            {
                this->RtmpValue = vValue;
                this->GtmpValue = pValue;
                this->BtmpValue = qValue;
                break;
            }
        }
    }
    this->HtmpValue = hValue;
    this->StmpValue = sValue;
    this->VtmpValue = vValue;
}
//----------(R,G,B) of HSV to RGB-------------
float TColorFunctionUnit::RofHSVtoRGB(float hValue, float sValue, float vValue)
{
    if(hValue == this->HtmpValue && sValue == this->StmpValue && vValue == this->VtmpValue)
        return this->RtmpValue;
    this->HSVtoRGB(hValue, sValue, vValue);
    return this->RtmpValue;
}

float TColorFunctionUnit::GofHSVtoRGB(float hValue, float sValue, float vValue)
{
    if(hValue == this->HtmpValue && sValue == this->StmpValue && vValue == this->VtmpValue)
        return this->GtmpValue;
    this->HSVtoRGB(hValue, sValue, vValue);
    return this->GtmpValue;
}

float TColorFunctionUnit::BofHSVtoRGB(float hValue, float sValue, float vValue)
{
    if(hValue == this->HtmpValue && sValue == this->StmpValue && vValue == this->VtmpValue)
        return this->BtmpValue;
    this->HSVtoRGB(hValue, sValue, vValue);
    return this->BtmpValue;
}
//--------------------------------------------
//-----------(H,S,V) of RGB to HSV------------
float TColorFunctionUnit::HofRGBtoHSV(float rValue, float gValue, float bValue)
{
    this->ExecuteMaxMin(rValue, gValue, bValue);
    float HueValue;

    if(this->maxHSV == this->minHSV)
        return 0.0;
    if(rValue == this->maxHSV)
    {
        HueValue = 60.0 * ((gValue - bValue) / (this->maxHSV - this->minHSV));
        if(gValue < bValue)
            HueValue += 360.0;
    }
    else if(gValue == this->maxHSV)
        HueValue = (float)(60.0 * ((bValue - rValue) / (this->maxHSV - this->minHSV)) + 120.0);
    else
        HueValue = (float)(60.0 * ((rValue - gValue) / (this->maxHSV - this->minHSV)) + 240.0);
    if(HueValue >= 360.0)
        HueValue -= 360.0;
    if(HueValue < 0.0)
        HueValue += 360.0;
    return HueValue;
}

float TColorFunctionUnit::SofRGBtoHSV(float rValue, float gValue, float bValue)
{
    this->ExecuteMaxMin(rValue, gValue, bValue);
    return ((this->maxHSV == 0.0) || (this->maxHSV == this->minHSV)) ? 0:((this->maxHSV - this->minHSV) / this->maxHSV);
}

float TColorFunctionUnit::VofRGBtoHSV(float rValue, float gValue, float bValue)
{
    this->ExecuteMaxMin(rValue, gValue, bValue);
    return this->maxHSV;
}
//-------------------------------------------
