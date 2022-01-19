#include "tmodelunit.hpp"

TModelUnit* ModelUnit = new TModelUnit();
TModelUnit::TModelUnit()
{
    this->ImageSize = IMAGEHEIGHT * IMAGEWIDTH * 3;
    this->bmpSample = new unsigned char [ ColorDeep * ColorDeep * ColorDeep ];
    this->TableBMPtoHSV = new unsigned char[ ColorDeep * ColorDeep * ColorDeep * 3];
    this->Label_Model = new unsigned char [ IMAGEHEIGHT * IMAGEWIDTH];
    this->BuiltTable = false;
    this->ViewColorModel = true;
    this->isBuildModel = false;
    this->ColorSelected = 0;

}

TModelUnit::~TModelUnit()
{
    delete [] this->bmpSample;
    delete [] this->Label_Model;
}

void TModelUnit::HSV_BuildingTable()
{

    unsigned char *tmpHere;
    float rValue, gValue, bValue;

    tmpHere = this->TableBMPtoHSV;


    for(int rValue_tmp = 0; rValue_tmp < ColorDeep; rValue_tmp++){
        for( int gValue_tmp = 0; gValue_tmp < ColorDeep; gValue_tmp++ ){
            for( int bValue_tmp = 0; bValue_tmp < ColorDeep; bValue_tmp++ ){
                rValue = float( rValue_tmp / 255.0 );
                gValue = float( gValue_tmp / 255.0 );
                bValue = float( bValue_tmp / 255.0 );

                *tmpHere++ = (unsigned char)( ColorFunctionUnit->HofRGBtoHSV( float( rValue ), float( gValue ), float( bValue ) ) / 360.0 * 255.0 );
                *tmpHere++ = (unsigned char)( ColorFunctionUnit->SofRGBtoHSV( float( rValue ), float( gValue ), float( bValue ) ) * 255.0 );
                *tmpHere++ = (unsigned char)( ColorFunctionUnit->VofRGBtoHSV( float( rValue ), float( gValue ), float( bValue ) ) * 255.0 );
            }
        }
    }
    this->BuiltTable = true;
    ROS_INFO("BuildingTable");
}

void TModelUnit::HSV_BuildingColorModel()
{

    unsigned char *tmpHere;
    float hValue, sValue, vValue;
    float rValue, gValue, bValue;

    tmpHere = this->bmpSample;
    for( int rValue_tmp = 0; rValue_tmp < ColorDeep; rValue_tmp++ ){
        for( int gValue_tmp = 0; gValue_tmp < ColorDeep; gValue_tmp++ ){
            for( int bValue_tmp = 0; bValue_tmp < ColorDeep; bValue_tmp++ ){
                rValue = float( rValue_tmp / 255.0 );
                gValue = float( gValue_tmp / 255.0 );
                bValue = float( bValue_tmp / 255.0 );

                hValue = float( ColorFunctionUnit->HofRGBtoHSV(float( rValue ), float( gValue ), float( bValue ) ) / 360.0 );
                sValue = ColorFunctionUnit->SofRGBtoHSV(float(rValue), float(gValue), float(bValue));
                vValue = ColorFunctionUnit->VofRGBtoHSV(float(rValue), float(gValue), float(bValue));

                if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etYellow], hValue, sValue, vValue ) )
                    *tmpHere++ = YellowLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etBlue], hValue, sValue, vValue ) )
                    *tmpHere++ = BlueLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etOrange], hValue, sValue, vValue ) )
                    *tmpHere++ = OrangeLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etGreen], hValue, sValue, vValue ) )
                    *tmpHere++ = GreenLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etWhite], hValue, sValue, vValue ) )
                    *tmpHere++ = WhiteLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etBlack], hValue, sValue, vValue ) )
                    *tmpHere++ = BlackLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etRed], hValue, sValue, vValue ) )
                    *tmpHere++ = RedLabel;
                else if( this->HSV_hsvCheckRange( DataUnit->HSVColorRange[etOther], hValue, sValue, vValue ) )
                    *tmpHere++ = OthersLabel;
                else
                    *tmpHere++ = 0x00;
            }
        }
    }
    *this->bmpSample = 0x00;
    ROS_INFO("BuildingColorModel");
}

void TModelUnit::ColorModeling(cv::Mat pImage)
{
    unsigned char rValue, gValue, bValue;
    unsigned char rMask, gMask, bMask;
    float hValue, sValue, vValue;
    unsigned char *tmpTable;

    switch (this->ColorSelected)
    {
    case etBlack:
        rMask = 255;
        gMask = 0;
        bMask = 255;
        break;
    case etBlue:
        rMask = 128;
        gMask = 0;
        bMask = 128;
        break;
    case etGreen:
        rMask = 128;
        gMask = 0;
        bMask = 0;
        break;
    case etOrange:
        rMask = 0;
        gMask = 0;
        bMask = 128;
        break;
    case etRed:
        rMask = 0;
        gMask = 255;
        bMask = 255;
        break;
    case etYellow:
        rMask = 0;
        gMask = 128;
        bMask = 128;
        break;
    case etWhite:
        rMask = 255;
        gMask = 255;
        bMask = 0;
        break;
    case etOther:
        rMask = 128;
        gMask = 0;
        bMask = 255;
        break;
    default:
        break;
    }
    for( int HeightCnt  = 0; HeightCnt < IMAGEHEIGHT; HeightCnt++ )
    {
        for( int WidthCnt = 0; WidthCnt < IMAGEWIDTH; WidthCnt++ )
        {
            bValue = *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0));
            gValue = *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1));
            rValue = *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2));
            tmpTable = this->TableBMPtoHSV + 3 * (rValue * ColorDeep * ColorDeep + gValue * ColorDeep + bValue);

            hValue = ((float)(*tmpTable++) / 255);
            sValue = ((float)(*tmpTable++) / 255);
            vValue = ((float)(*tmpTable++) / 255);

            if(this->HSV_hsvCheckRange(this->hsvColorRange, hValue, sValue, vValue))
            {
                *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0)) = bMask;
                *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1)) = gMask;
                *(pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2)) = rMask;
            }
        }
    }
}

void TModelUnit::ChangeToColorModel(cv::Mat pImage)
{

    unsigned char *rValue, *gValue, *bValue;
    unsigned char *tmpValue;
    for( int HeightCnt  = 0; HeightCnt < IMAGEHEIGHT; HeightCnt++ )
    {
        for( int WidthCnt = 0; WidthCnt < IMAGEWIDTH; WidthCnt++ )
        {
            bValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0));
            gValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1));
            rValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2));

            tmpValue = this->bmpSample + (*rValue * ColorDeep * ColorDeep + *gValue * ColorDeep + *bValue);
            if(*tmpValue == BlackLabel)
            {
                *bValue = 255;
                *gValue = 0;
                *rValue = 255;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = BlackLabel;
            }else if(*tmpValue == BlueLabel){
                *bValue = 128;
                *gValue = 0;
                *rValue = 128;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = BlueLabel;
            }else if(*tmpValue == GreenLabel){
                *bValue = 0;
                *gValue = 0;
                *rValue = 128;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = GreenLabel;
            }else if(*tmpValue == OrangeLabel){
                *bValue = 128;
                *gValue = 0;
                *rValue = 0;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = OrangeLabel;
            }else if(*tmpValue == RedLabel){
                *bValue = 255;
                *gValue = 255;
                *rValue = 0;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = RedLabel;
            }else if(*tmpValue == YellowLabel){
                *bValue = 128;
                *gValue = 128;
                *rValue = 0;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = YellowLabel;
            }else if(*tmpValue == WhiteLabel){
                *bValue = 0;
                *gValue = 255;
                *rValue = 255;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = WhiteLabel;
            }else if(*tmpValue == OthersLabel){
                *bValue = 255;
                *gValue = 0;
                *rValue = 128;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = OthersLabel;
            }else{
                *bValue = 0;
                *gValue = 0;
                *rValue = 0;
                *((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)) = 0x00;
            }
//            this->labelmodel.LabelModel.push_back(*((this->Label_Model)+(HeightCnt*IMAGEWIDTH + WidthCnt)));
        }
    }

}

void TModelUnit::ChangeToColorModel_ImageProcess(cv::Mat pImage)
{
    unsigned char *rValue, *gValue, *bValue;
    unsigned char *tmpValue;
    for( int HeightCnt  = 0; HeightCnt < IMAGEHEIGHT; HeightCnt++ )
    {
        for( int WidthCnt = 0; WidthCnt < IMAGEWIDTH; WidthCnt++ )
        {
            bValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 0));
            gValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 1));
            rValue = (pImage.data + ((HeightCnt*IMAGEWIDTH + WidthCnt) * 3 + 2));
            tmpValue = this->Label_Model + HeightCnt * IMAGEWIDTH + WidthCnt;
            this->labelmodel.LabelModel.push_back(*tmpValue);
            if(*tmpValue == BlackLabel)
            {
                *bValue = 255;
                *gValue = 0;
                *rValue = 255;
            }else if(*tmpValue == BlueLabel){
                *bValue = 128;
                *gValue = 0;
                *rValue = 128;
            }else if(*tmpValue == GreenLabel){
                *bValue = 0;
                *gValue = 0;
                *rValue = 128;
            }else if(*tmpValue == OrangeLabel){
                *bValue = 128;
                *gValue = 0;
                *rValue = 0;
            }else if(*tmpValue == RedLabel){
                *bValue = 255;
                *gValue = 255;
                *rValue = 0;
            }else if(*tmpValue == YellowLabel){
                *bValue = 128;
                *gValue = 128;
                *rValue = 0;
            }else if(*tmpValue == WhiteLabel){
                *bValue = 0;
                *gValue = 255;
                *rValue = 255;
            }else if(*tmpValue == OthersLabel){
                *bValue = 255;
                *gValue = 0;
                *rValue = 128;
            }
        }
    }
}

bool TModelUnit::HSV_hsvCheckRange(ColorRange *hsvRange, float &hValue, float &sValue, float &vValue)
{

    if( hsvRange->HueMax >= hsvRange->HueMin ){
        if( hsvRange->HueMax >= hValue && hsvRange->HueMin <= hValue
                && hsvRange->SaturationMax >= sValue && hsvRange->SaturationMin <= sValue
                && hsvRange->BrightnessMax >= vValue && hsvRange->BrightnessMin <= vValue )
            return true;
    }else{
        if( ( hsvRange->HueMax >= hValue || hsvRange->HueMin <= hValue )
                && hsvRange->SaturationMax >= sValue && hsvRange->SaturationMin <= sValue
                && hsvRange->BrightnessMax >= vValue && hsvRange->BrightnessMin <= vValue )
            return true;
    }

    return false;
}

