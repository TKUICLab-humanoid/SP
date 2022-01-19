#include "tcoordinate.hpp"
TCoordinate::TCoordinate( int xValue, int yValue )
{
        this->X = xValue;
        this->Y = yValue;
}
//------------------------------------------------------------------------------
TCoordinate::TCoordinate()
{
        this->X = 0;
        this->Y = 0;
}
//------------------------------------------------------------------------------
TCoordinate::~TCoordinate()
{
}
