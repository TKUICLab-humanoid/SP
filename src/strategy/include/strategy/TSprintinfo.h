#pragma once

#ifndef TSprintInfoH
#define TSprintInfoH



typedef struct
{

  int tmp_theta,
      send_x,
      send_y,
      send_theta,
      ArrayYellow[5],
      ArrayBlue[5];

  char checkback_cnt;

  bool get_target,
  SprForWard; //

}TSpintInfomation;


typedef enum{
    etXMin, //0
    etXMax, //1
    etYMin, //2
    etYMax, //3
    etSize, //4
}teSprintInfoRange;

#define TargetNumber 3

class TSprintInfo
{
 public:
    TSprintInfo();

        TSpintInfomation* SpintInfomation;

        int fram_count;

        int head_motor_x;
        int head_motor_y;

        int XMax[TargetNumber],
            XMin[TargetNumber],
            YMax[TargetNumber],
            YMin[TargetNumber],
            TargetArea;

    double center_x;
    double center_y;

    double backward_310L;
    double backward_310R;
    double backward_300L;
    double backward_300R;
    double backward_250L;
    double backward_250R;
    double backward_200L;
    double backward_200R;
    double backward_150L;
    double backward_150R;
    double backward_100L;
    double backward_100R;
    double backward_50L;
    double backward_50R;
        //IMU
    float IMU_now;
    float IMU_right;
    float IMU_left;
    float IMU_start;
};
extern TSprintInfo* SprintInfo;
#endif
