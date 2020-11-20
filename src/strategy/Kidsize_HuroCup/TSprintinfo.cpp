#include"strategy/TSprintinfo.h"

TSprintInfo* SprintInfo = new TSprintInfo();

TSprintInfo::TSprintInfo(void)
{

 SpintInfomation = new TSpintInfomation;

 SpintInfomation->send_x=SpintInfomation->send_y=SpintInfomation->tmp_theta = 0;
 SpintInfomation->checkback_cnt = 0;

}
