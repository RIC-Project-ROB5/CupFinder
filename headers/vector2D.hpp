//class for working with 2Dvectors
#pragma once
#include <cmath>
#include "point.hpp"
#include <cstdlib>

class vector2D
{
  private:
    point point_1;
    point point_2;
  public:
    point getStartPoint() const;
    point getEndPoint() const ;
    vector2D(point startpoint, point endpoint);
    float GetLenght() const;
    float distanceToPoint(point p) const;

};
