#include "DrawImage.hpp"
#include <stdio.h>
drawimage::drawimage()
{

}
drawimage::~drawimage()
{

}

bool drawimage::isnewdraw(int cnt,std::vector<drawImageInfo> DrawImageArray){
    if(DrawImageArray.empty()) return true;
    
    for(int i=0 ;i < DrawImageArray.size();i++){
        if(DrawImageArray[i].cnt==cnt){
                return false;
        }
    }
    return true;
}