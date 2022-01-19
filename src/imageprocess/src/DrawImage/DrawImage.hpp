#include <vector>
struct drawImageInfo
{
    int Mode;
    int cnt;
    int XMin;
    int XMax;
    int YMin;
    int YMax;
    int rValue;
    int gValue;
    int bValue;
};


class drawimage
{
public:
    drawimage();
    ~drawimage();
    bool isnewdraw(int cnt,std::vector<drawImageInfo> DrawImageArray);
};
