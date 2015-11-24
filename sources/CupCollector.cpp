#include "CupCollector.hpp"


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
