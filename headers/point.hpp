#pragma once
class point
{
  public:
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

  float GetDistance(const point &p) const;
  static float GetDistance(const point p1, const point p2);
  bool isNeighbour(const point &p);
};
