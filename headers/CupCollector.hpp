#pragma once
#include "point.hpp"
#include "Image.hpp"
#include "Map.hpp"
#include "vector2D.hpp"
#include <vector>
#include <list>

#define PIXEL_PER_METER 10
#define ROB_RADIUS  (0.4 * PIXEL_PER_METER)
#define ROB_VIEW_RANGE (2 * PIXEL_PER_METER)
#define ROB_PICKUP_RANGE (1 * PIXEL_PER_METER)
#define MAX_CUPS 20

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
    point coord;
    std::vector<Waypoint_connection> connections; //list of connections to other cells
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
        point current_point;
        int32_t size_x; //size of the map (x axis)
        int32_t size_y; //size of the map (y axis)
        uint32_t current_cups = 0; //number of cups the robot currently holds.
        uint32_t total_cups = 0;
        std::vector< point > move_path;
        std::vector< std::vector< mapSpace> > workspace;
        std::vector< std::vector< mapSpace> > configurationspace;
        std::vector< Cell > cells;
        std::vector< Waypoint > wayPoints;

        Waypoint *dropoffs[2] = {nullptr, nullptr};

        void SearchCell(const Waypoint &startpoint, const Waypoint &endpoint, Cell &cell);
        void WalkLine(vector2D const &line);
        point FindNextPointOnLine(const vector2D &line) const;
        bool IsOutsideMap(const point &p) const;
        void CreateWorkspaceMap(rw::sensor::Image* map);
        bool IsObstacle(const point &p) const;
        void cellDecomposition();
        void graphConnecting();
        void findWaypoints(size_t id);
        void findCells(int id);
};
