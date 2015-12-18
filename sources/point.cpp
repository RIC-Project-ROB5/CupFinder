#include "point.hpp"
#include <cmath>

float point::GetDistance(const point &p) const
{
    auto x_dif = this->x - p.x;
    auto y_dif = this->y - p.y;
    return sqrt(x_dif * x_dif + y_dif * y_dif);
}

float point::GetDistance(const point p1, const point p2)
{
  return p1.GetDistance(p2);
}

bool point::isNeighbour(const point &p)
{
  for(int i = -1; i <= 1; i++)
  {
    for(int j = -1; j <=1; j++)
    {
      if(j == 0 && i == 0) continue;
      point p1 = {this->x + i, this->y + j};
      if(p1 == p) return true;
    }
  }
  return false;
}

std::ostream& operator<<(std::ostream &os, const point &p)
{
    os << "[" << p.x << ',' << p.y << ']';
    return os;
}
