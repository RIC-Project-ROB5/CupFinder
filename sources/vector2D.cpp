#include "../headers/vector2D.hpp"


point vector2D::getStartPoint() const
{
  return point_1;
}
point vector2D::getEndPoint() const
{
  return point_2;
}
vector2D::vector2D(point startpoint, point endpoint)
{
  point_1 = startpoint;
  point_2 = endpoint;
}

float vector2D::GetLenght() const
{
  return sqrt( pow(point_1.x - point_2.x, 2) + pow(point_1.y - point_2.y, 2));
}

float vector2D::distanceToPoint(point p) const
{
  float num = (point_2.y - point_1.y) * p.x - (point_2.x - point_1.x) * p.y;
  num += point_2.x * point_1.y - point_2.y * point_1.x;
  num = fabs(num);
  float den = sqrt(pow(point_2.y - point_1.y, 2) + pow(point_2.x - point_1.x, 2));
  return num / den;
}
