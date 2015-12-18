#pragma once
#include <ostream>
#include <cmath>
class point
{
  public:
	  point(){}
	  point(int _x, int _y){
		  x = _x;
		  y = _y;
	  }
  int x;
  int y;
  inline bool operator ==(const point &p) const
  {
    return (this->x == p.x && this->y == p.y);
  }
  inline bool operator !=(const point &p) const
  {
    return !(*this == p);
  }

  inline point operator +(const point &p) const
  {
    return {p.x + this->x, p.y + this->y};
  }
  inline void operator +=(const point &p)
  {
      this->x += p.x;
      this->y += p.y;
  }

  float GetDistance(const point &p) const
  {
      auto x_dif = this->x - p.x;
      auto y_dif = this->y - p.y;
      return sqrt(x_dif * x_dif + y_dif * y_dif);
  }

  static float GetDistance(const point p1, const point p2);
  bool isNeighbour(const point &p);
};

std::ostream& operator<<(std::ostream &os, const point &p);
