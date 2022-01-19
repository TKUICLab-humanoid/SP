#ifndef MODELPARAMETER_H
#define MODELPARAMETER_H

#define IMAGEHEIGHT 240
#define IMAGEWIDTH 320
#define DefLabelMarkSize 8
#define HueScrollBarMax 360
#define SaturationScrollBarMax 100
#define BrightnessScrollBarMax 100


typedef enum
{
    etOrange,
    etYellow,
    etBlue,
    etGreen,
    etBlack,
    etRed,
    etWhite,
    etOther
}etLabelModel;

typedef enum
{
    OrangeLabel = 0x01,// 0000 0001
    YellowLabel = 0x02,// 0000 0010
    BlueLabel = 0x04,  // 0000 0100
    GreenLabel = 0x08, // 0000 1000
    BlackLabel = 0x10, // 0001 0000
    RedLabel = 0x20,
    WhiteLabel = 0x40,
    OthersLabel = 0x80
}etLabelMark;


#endif // MODELPARAMETER_H

