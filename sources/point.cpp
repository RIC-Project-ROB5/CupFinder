#include "point.hpp"


float point::GetDistance(const point p1, const point p2)
{
  return p1.GetDistance(p2);
}

bool point::isNeighbour(const point &p)
{
    if(abs(this->x - p.x) <= 1)
    {
        if(abs(this->y - p.y) <= 1)
        {
            if(*this == p) return false;
            return true;
        }
    }
    return false;
}

std::ostream& operator<<(std::ostream &os, const point &p)
{
    os << "[" << p.x << ',' << p.y << ']';
    return os;
}
