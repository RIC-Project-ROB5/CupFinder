#pragma once
#include "point.hpp"
#include "Image.hpp"
#include "Map.hpp"
#include <vector>
#include <list>

//Forward declare structs and classes
struct Cell;
struct Waypoint_connection;
struct WayPoint;
class CupCollector;

struct Cell
{
    point lower_left;
    point lower_right;
    point upper_left;
    point upper_right;
    bool searched; //Have this cell been covered yet?
};

struct Waypoint
{
    //list of connections to other cells
    std::list<Waypoint_connection> connections;
};

struct Waypoint_connection
{
    Waypoint *linkptr; //Pointer to the connected Waypoint
    Cell *connection_cell; //the cell which connects the two Waypoints
    uint32_t cost;         //The cost of traveling through this connection
};

/*
Class for the robot collecting cups
*/
class CupCollector
{
    public:
        std::vector<point> get_path(); //Gives the path for cup collecting.
        //the collection starts at one of the drop of areas.

        CupCollector(rw::sensor::Image *map);
        ~CupCollector();

    private:
        point currentpoint;
        std::vector<point> move_path;
        std::vector< std::vector< mapSpace> > workspace;
        std::vector< std::vector< mapSpace> > configurationspace;
        Waypoint *dropoffs[2] = {nullptr, nullptr};
};
