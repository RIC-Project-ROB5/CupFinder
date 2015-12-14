#pragma once
#include "point.hpp"
#include "Image.hpp"
#include "Map.hpp"
#include "vector2D.hpp"
#include <vector>
#include <list>
#include <iostream>

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
    //list of connections to other cells
    point position;
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
    private: //private functions
        void ExpandPixel(const point p);
        void prepare_wavefront();
        inline void setDistance(const point &p, const uint64_t value)
        {
            wavefront[p.x][p.y] = value;
        }
        void check_neighbour(const point &this_point, const point &neighbour, std::vector<point> &expand_points_next);
        inline uint64_t getDistance(const point &p) const
        { //returns distance to goal
            return wavefront[p.x][p.y];
        }

    public: //public functions
        std::vector<point> get_path(); //Gives the path for cup collecting.
        //the collection starts at one of the drop of areas.

        CupCollector(rw::sensor::Image *map, const point &inDropoff);
        ~CupCollector();

        //Searches the given cell for cups, collects them, return them to dropoff
        //Start at startpoint and exit at endpoint.
        void SearchCell(const Waypoint &startpoint, const Waypoint &endpoint, Cell &cell);
        void WalkLine(vector2D const &line);
        point FindNextPointOnLine(const vector2D &line) const;
        bool IsOutsideMap(const point &p) const;
        void CreateWorkspaceMap(rw::sensor::Image* map);
        bool IsObstacleWS(const point &p) const;
        bool IsObstacleCS(const point &p) const;
        void CreateConfigurationspaceMap();
        void compute_wavefront(); //(pre)computes the wavefront
        void SaveMaps(rw::sensor::Image* map);

    private: //private objects/variables
        int32_t size_x; //size of the map (x axis)
        int32_t size_y; //size of the map (y axis)
        uint32_t current_cups = 0; //number of cups the robot currently holds.
        uint32_t total_cups = 0;
        point current_point; //point where the robot is right now
        std::vector<point> move_path;
        std::vector< std::vector< mapSpace> > workspace;
        std::vector< std::vector< mapSpace> > configurationspace;
        Waypoint *dropoffs[2] = {nullptr, nullptr};
        point dropoff;
        uint64_t **wavefront = nullptr; //the wavefront map

        bool debug = true;
};
