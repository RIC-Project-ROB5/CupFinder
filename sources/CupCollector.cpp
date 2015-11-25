#include "CupCollector.hpp"
#include <limits>
#include <cassert>

static point neighbours[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {-1, -1}, {1, 1}, {1, -1}, {-1, 1}}; //the neighbours a cell have

CupCollector::CupCollector(__attribute__((unused))rw::sensor::Image* map)
{
    //Convert image into mapspace map of the workspace
    //convert workspace into configurationspace
    //Do cell decomposition, create waypoints and cells.
    //Connect the waypoints and cells into a graph

}

CupCollector::~CupCollector()
{
    //Do cleanup
}

void CupCollector::SearchCell(__attribute__((unused))const Waypoint &startpoint, __attribute__((unused))const Waypoint &endpoint, __attribute__((unused))Cell &cell)
{   //Searches the given cell for cups, collects them, return them to dropoff
    //Start at startpoint and exit at endpoint.

    //First, find out if we have entered the room from the top, bottom, left or right

}

void CupCollector::WalkLine(vector2D const &line)
{   //Walk along the line from startpoint to endpoint
    //There should be a clear path, e.g. no obstacles.
    while(current_point != line.getEndPoint())
    {
        point next_point = FindNextPointOnLine(line);
        current_point = next_point;
    }
}

point CupCollector::FindNextPointOnLine(const vector2D &line) const
{
    //check all 8 directions collect the ones which are closer to the endpoint of the line
    // to the endpoint of the linethan the current.
    point closestpoint = {0, 0};
    float curdistance = current_point.GetDistance(line.getEndPoint());
    float mindistance = std::numeric_limits<float>::infinity();
    for(uint8_t i = 0; i < sizeof(neighbours) / sizeof(neighbours[i]); i++)
    {
        point this_point = current_point + neighbours[i];
        if(IsOutsideMap(this_point)) continue;

        if(this_point.GetDistance(line.getEndPoint()) <= curdistance)
        {   //select the wanted point as the one closest to the straight line to end.
            float distance_to_line = line.distanceToPoint(this_point);
            if(distance_to_line < mindistance)
            {
                closestpoint = this_point;
                mindistance = distance_to_line;
            }
        }
    }
    return closestpoint;
}

bool CupCollector::IsOutsideMap(const point &p) const
{
    if(p.y < 0 || p.x < 0 || p.y > size_y || p.x > size_x)
        return true;
    return false;
}
