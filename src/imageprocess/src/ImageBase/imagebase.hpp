#ifndef IMAGEBASE_H
#define IMAGEBASE_H


#define DefBeginMark 1
#define DefRunMark 2
#define DefCheckMark 3
#define DefEndMark 4

#include "../ModelBase/modelparameter.h"
#include "../Coordinate/tcoordinate.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

using namespace std;
class TObjectInfo
{
public:
    TObjectInfo();
    TObjectInfo( int xValue, int yValue );
    unsigned char MaskLabel;					//Object Color Label
    int XMax, XMin, YMax, YMin;					//Object Margin
    int Area(void);								//Object Area
    TCoordinate Center;							//Object Center
    int CoordinateCnt;
    int AreaSize;
    int ObjectWidth,ObjectHigh;
    vector<TCoordinate> CoordinateList;			//Object Coordinate Data Base

};
class TImageInfo
{
public:
    TImageInfo(void);
    int ObjectCnt;
    void ClearInfo( int ObjectOffset );
    vector<TObjectInfo> ObjectInfoList;
};
class TImageBase
{
private:
    unsigned char *Label_Model;
    int ColorLable;

public:
    int ImageSize;
    unsigned char* tmpModelBuffer;
    int ColorMaskSubjectCnt[DefLabelMarkSize];
    int ColorMask[IMAGEWIDTH][IMAGEHEIGHT];
    int ProcessLabelValue;
    bool bNewFlag;
    vector<TImageInfo*> ObjectList;
    TCoordinate ColorMaskSubject[DefLabelMarkSize][IMAGEWIDTH];

    TImageBase();
    void TDilation(unsigned char* ModelBuffer);
    void TErosionMain(unsigned char* ModelBuffer);
    void Boundary( TObjectInfo &ObjectInfo, TCoordinate Coordinate);
    void ClearAllObject(void);
    bool TSegmentationMain(unsigned char* modelbuffer);
    //void TModelClassifyMain(unsigned char* ModelBuffer);
};
extern TImageBase* ImageBase;

#endif // IMAGEBASE_H
