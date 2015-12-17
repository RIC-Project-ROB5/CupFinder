#pragma once
#include "point.hpp"
#include "Image.hpp"
// #include "PPMLoader.hpp"
#include "Map.hpp"
#include "vector2D.hpp"
#include <vector>
#include <list>
#include <iostream>
#include <limits>


#define PIXEL_PER_METER 10
#define ROB_RADIUS  (0.4 * PIXEL_PER_METER)
#define ROB_VIEW_RANGE (2 * PIXEL_PER_METER)
#define ROB_PICKUP_RANGE (1 * PIXEL_PER_METER)
#define COVER_RANGE ROB_VIEW_RANGE //int(ROB_RADIUS) //How close the robot should circle the cells.
#define MAX_CUPS 20

//Forward declare structs and classes
struct Cell;
struct Waypoint_connection;
struct WayPoint;
struct RGB;
class CupCollector;

RGB mapcolour(uint64_t value, uint64_t max);


struct RGB
{
  uint8_t r, g, b;
};

struct Cell
{
    point lower_left;
    point lower_right;
    point upper_left;
    point upper_right;
    bool searched = false; //Have this cell been covered yet?
};

struct Waypoint
{
    point coord;
    std::vector<Waypoint_connection> connections; //list of connections to other cells
    bool visited = false;
};

struct Waypoint_connection
{
    size_t index; //index of connected waypoint
    Cell *connection_cell; //the cell which connects the two Waypoints, nullptr if no cell
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
        void setDistance(const point &p, const uint64_t value);

        void check_neighbour(const point &this_point, const point &neighbour, std::vector<point> &expand_points_next);
        uint64_t getDistance(const point &p) const;
        std::vector<point> SearchCell(const Waypoint &startpoint, const Waypoint &endpoint, const Cell &cell);
        std::vector<point> WalkLine(vector2D const &line, const std::vector< std::vector< mapSpace> > &map, float distance = -1) const;
        point FindNextPointOnLine(const vector2D &line, const point &cur, const std::vector< std::vector< mapSpace> > &map, bool *success = nullptr) const;
        bool IsOutsideMap(const point &p) const;
        void CreateWorkspaceMap(rw::sensor::Image* map);
        bool IsObstacle(const point &p, const std::vector< std::vector< mapSpace> > &map) const;
        void CreateConfigurationspaceMap();
        void compute_wavefront(); //(pre)computes the wavefront
        void cellDecomposition();
        void graphConnecting();
        void findWaypoints(int64_t id);
        void findCells(int id);
        void SaveWorkspaceMap(std::string name);
        void SaveConfigurationspaceMap(std::string name);
        void SaveWaypointMap(std::string name);
        void SaveWalkMap(std::string name);
        void SaveSearchedMap(std::string name);
        void SaveWavefrontMap(std::string name);
        void SaveConnectionMap(std::string name);
        void SaveCellMap(std::string name);
        void cleanCellMap();
        void prepareCellDecomposition();
        void cellDecomposition(Cell &seedcell, int64_t id);
        bool isGraphConnected();
        bool validateMap();
        void connectNeighbours(size_t id);
        void traverseGraphRec(Waypoint &wp);
        std::vector<point> SearchGraph(Waypoint &wp);
        std::vector<point> getGoalPath(const point &start) const;
        point get_next_point(const point &curpoint, bool *success) const;
        bool isblocked(vector2D &line) const;
        std::vector<point> GetCup(point &p_start, point &p_cup);
        std::vector<point> SearchForCups(point &p, float distance);
        std::vector<point> SearchLine(vector2D const &line, float distance);


    public: //public functions
        std::vector<point> get_path(); //Gives the path for cup collecting.
        //the collection starts at one of the drop of areas.

        CupCollector(rw::sensor::Image *map);
        ~CupCollector();
        void SaveMaps();

    private:
        point current_point;
        int32_t size_x; //size of the map (x axis)
        int32_t size_y; //size of the map (y axis)
        uint32_t current_cups = 0; //number of cups the robot currently holds.
        uint32_t total_cups = 0;
        std::vector< point > move_path;
        std::vector< std::vector< mapSpace> > workspace;
        std::vector< std::vector< mapSpace> > searchmap;
        std::vector< std::vector< mapSpace> > configurationspace;
        std::vector< std::vector< int64_t > > cellDecompMap;
        std::vector< Cell > cells;
        std::vector< Waypoint > wayPoints;
        int64_t cellid = 2;
        uint64_t **wavefront = nullptr; //the wavefront map

        bool debug = true;

};
