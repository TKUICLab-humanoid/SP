#ifndef TMODELUNIT_H
#define TMODELUNIT_H

#define ColorDeep 256

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "modelparameter.h"
#include "TColorFunctionUnit.hpp"
#include "tdataunit.hpp"
#include "tku_msgs/LabelModelObjectList.h"


class TModelUnit
{
private:
    int ImageSize;
    unsigned char *bmpSample;
public:
    TModelUnit();
    ~TModelUnit();
    ColorRange* hsvColorRange;
    unsigned char *TableBMPtoHSV;
    unsigned char *Label_Model;
    bool BuiltTable;
    bool isBuildModel;
    bool ViewColorModel;
    int ColorSelected;

    void HSV_BuildingTable();
    void HSV_BuildingColorModel();
    void ColorModeling(cv::Mat colorimage);             //while color modeling
    void ChangeToColorModel(cv::Mat pImage);        //before Erosion and Dilation
    void ChangeToColorModel_ImageProcess(cv::Mat pImage);   //after Erosion and Dilation
    bool HSV_hsvCheckRange( ColorRange *hsvRange, float &hValue, float &sValue, float &vValue );

    tku_msgs::LabelModelObjectList labelmodel;
//    std_msgs::Int16MultiArray labelmodel;
};
extern TModelUnit* ModelUnit;
#endif // TMODELUNIT_H
