#include "CupCollector.hpp"
#include <limits>
#include <cassert>
#include <cstdlib>

using namespace std;
using namespace rw::sensor;

static point neighbours[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {-1, -1}, {1, 1}, {1, -1}, {-1, 1}}; //the neighbours a cell have


static point expand_points[] = {{1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {2, -2}, {2, -1}, {2, 0}, {2, 1}, {2, 2}, {1, 2}, {0, 2}, {-1, 2}, {-2, 2}, {-2, -1}, {-2, 0}, {-2, -1}, {-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {3, -3}, {3, -2}, {3, -1}, {3, 0}, {3, 1}, {3, 2}, {3, 3}, {2, 3}, {1, 3}, {0, 3}, {-1, 3}, {-2, 3}, {-3, 3}, {-3, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-3, -2}, {-3, -3}, {-2, -3}, {-1, -3}, {0, -3}, {1, -3}, {2, -3}, {4, -2}, {4, -1}, {4, 0}, {4, 1}, {4, 2}, {2, 4}, {1, 4}, {0, 4}, {-1, 4}, {-2, 4}, {-4, 2}, {-4, 1}, {-4, 0}, {-4, -1}, {-4, -2}, {-2, -4}, {-1, -4}, {0, -4}, {1, -4}, {2, -4}};   //68 points


void DrawSquare(Image *img, Cell &c, RGB colour)
{   //this only works for "perfect" squares
    for(int x = c.upper_left.x; x <= c.lower_right.x ; x++)
    {
        for(int y = c.upper_right.y; y <= c.lower_left.y ; y++)
        {
            img->setPixel8U(x, y, colour.r, colour.g, colour.b);
        }
    }
}

RGB mapcolour(uint64_t value, uint64_t max)
{ //map colours according to value and maxvalue
  //the returned colour will be more red the closer it is to 0
  //and more blue the closer it is to max.
  //there are exeption however, if the value is 1, the returned colour will be black
  //if value is 3, the returned colour will be green
  //if value is 2  the colour will be dark green
  if(value == 1) return {0, 0, 0};
  if(value == 3) return {0, 255, 0};
  if(value == 2) return {100, 255, 100};

  RGB col;
  col.r = 255 - (value * 255) / max;
  col.g = 0 ;

  col.b = (value * 255) / max;
  return col;
}


CupCollector::CupCollector(Image* map)
{
    //Save image dimensions
    //cells.reserve(100000);
    //wayPoints.reserve(100000);
    size_x = map->getWidth();
    size_y = map->getHeight();
    if (debug)
        std::cout << "Image size is " << size_x << ", " << size_y << std::endl;

    //Convert image into mapspace map of the workspace
    CreateWorkspaceMap(map);
    if (debug)
        std::cout << "Workspace created" << std::endl;

    //Convert workspace into configurationspace
    CreateConfigurationspaceMap();
    if (debug)
        std::cout << "Configurationspace created" << std::endl;

    compute_wavefront();
    if (debug)
        std::cout << "Wavefront created" << std::endl;

    prepareCellDecomposition();
    Cell seedc;
    seedc.upper_left = {2000, 1280};
    seedc.lower_right = seedc.upper_left + point(0, 10);
    cellDecomposition(seedc, cellid);
    if(debug)
        std::cout << "Created " << cells.size() << " cells." << std::endl;
    //set the right id's in the cell decomposition map
    cleanCellMap();
	graphConnecting();
    if(!validateMap())
    {
        std::cout << "Generated waypoint map is invalid, bailing out" << std::endl;
        exit(1);
    }
    if(debug)
        std::cout << "Generated waypoint map is valid" << std::endl;
    if(!isGraphConnected())
    {
        std::cout << "Not all generated waypoints are connected, bailing out" << std::endl;
        exit(1);
    }
    if(debug)
        std::cout << "All generated waypoints are connected." << std::endl;

    //Create output image
    SaveMaps();
    if (debug)
        std::cout << "Maps have been saved" << std::endl;

}

CupCollector::~CupCollector()
{
    //Deallocate the wavefront map
    if(wavefront != nullptr)
    {
        for(long i = 0; i < size_x; i++)
        {
            delete[] wavefront[i];
        }
        delete[] wavefront;
    }
}


void CupCollector::CreateWorkspaceMap(rw::sensor::Image* map)
{
    int channel = 0; //the map is grayscale, so channel is 0
    for(uint32_t x = 0; x < map->getWidth(); x++)
    {
        std::vector<mapSpace> y_line; //Contains a single line on the y-axis
        for(uint32_t y = 0; y < map->getHeight(); y++)
        {
            mapSpace pixeltype;
            switch(map->getPixelValuei(x, y, channel))
            {
                case 0: case 128: case 129: case 130: case 131: case 132:
                    pixeltype = obstacle;
                    break;
                case 100:
                    pixeltype = dropoff;
                    break;
                case 150:
                    pixeltype = cup;
                    break;
                default:
                    pixeltype = freespace;
                    break;
            }
            y_line.push_back(pixeltype);
        }
        workspace.push_back(y_line);
    }
}

bool CupCollector::validateMap()
{
    //check if the map is valid. First check if all connections are bidirectional.
    for(size_t i = 0; i < wayPoints.size(); i++)
    {
        auto &wp = wayPoints[i];
        for(auto &con : wp.connections)
        {
            bool bidir = false;
            for(auto &con2 : wayPoints[con.index].connections)
            {
                if(con2.index == i)
                {
                    bidir = true;
                    break;
                }
            }
            if(!bidir)
            {
                return false;
            }
        }
    }
    return true;
}

bool CupCollector::isGraphConnected()
{//In this function we traverse the entire graph. We do this by using a depth first Search
    //Mark all waypoints as unvisited
    for(auto &wp : wayPoints)
        wp.visited = false;
    traverseGraphRec(wayPoints[0]);
    for(auto &wp : wayPoints)
    {
        if(wp.visited == false) return false;
    }
    return true;
}
void CupCollector::traverseGraphRec(Waypoint &wp)
{
    wp.visited = true;
    for(auto &con : wp.connections)
    {
        if(wayPoints[con.index].visited == false)
        {
            traverseGraphRec(wayPoints[con.index]);
        }
    }
}

void CupCollector::SearchCell(__attribute__((unused))const Waypoint &startpoint, __attribute__((unused))const Waypoint &endpoint, __attribute__((unused))Cell &cell)
{   //Searches the given cell for cups, collects them, return them to dropoff
    //Start at startpoint and exit at endpoint.

    //Find the nearest corner

    //Go to that, find out which direction we are going.

    //Cover the cell in a shrinking spiral pattern, untill we get to the middle. For each round, define a new "smaller" cell to cover the next time

    //Go to the endpoint.
}

std::vector<point> CupCollector::WalkLine(vector2D const &line) const
{   //Walk along the line from startpoint to endpoint
    //There should be a clear path, e.g. no obstacles.
    //std::cout << "[" << line.getStartPoint().x << ", " << line.getStartPoint().y << "]-[" <<
    //line.getEndPoint().x << "," << line.getEndPoint().y << "]" <<std::endl;
    std::vector<point> line_points;
    point cur = line.getStartPoint();
    line_points.push_back(cur);
    while(cur != line.getEndPoint())
    {
        bool success = false;
        point next_point = FindNextPointOnLine(line, cur, &success);
        if(!success) return std::vector<point>();
        cur = next_point;
        //std::cout << cur << std::endl;
        line_points.push_back(cur);
        //if(line_points.size() > 100) exit(0);
    }
    return line_points;
}

point CupCollector::FindNextPointOnLine(const vector2D &line, const point &cur, bool *success ) const
{
    //check all 8 directions collect the ones which are closer to the endpoint of the line
    // to the endpoint of the linethan the current.
    if(success != nullptr) *success = true;
    point closestpoint(-1, -1);
    float curdistance = cur.GetDistance(line.getEndPoint());
    float mindistance = std::numeric_limits<float>::infinity();
    for(uint8_t i = 0; i < sizeof(neighbours) / sizeof(neighbours[i]); i++)
    {
        point this_point = cur + neighbours[i];
        if(IsOutsideMap(this_point) || IsObstacleCS(this_point)) continue;

        if(this_point.GetDistance(line.getEndPoint()) < curdistance)
        {   //select the wanted point as the one closest to the straight line to end.
            float distance_to_line = line.distanceToPoint(this_point);
            if(distance_to_line < mindistance)
            {
                closestpoint = this_point;
                mindistance = distance_to_line;
            }
        }
    }
    if(success != nullptr && closestpoint == point(-1, -1)) *success = false;
    return closestpoint;
}

bool CupCollector::IsOutsideMap(const point &p) const
{
    if(p.y < 0 || p.x < 0 || p.y > size_y || p.x > size_x)
        return true;
    return false;
}

bool CupCollector::IsObstacleWS(const point &p) const
{
    if(workspace[p.x][p.y] == obstacle)
        return true;
    return false;
}

bool CupCollector::IsObstacleCS(const point &p) const
{
    if(configurationspace[p.x][p.y] == obstacle)
        return true;
    return false;
}

void CupCollector::CreateConfigurationspaceMap()
// Creates a configurationspace map from input workspace map
// Saves configurationspace map as private variable configurationspace
{
    /* Find the coordinates for all obstacles and save these in a vector */
    std::vector< point > obstacles_unfiltered;
    for (int32_t x = 0; x < size_x; x++) {
      for (int32_t y = 0; y < size_y; y++) {
        if (workspace[x][y] == obstacle)
          obstacles_unfiltered.push_back(point(x,y));
      }
    }
    if (debug)
        std::cout << "There are " << obstacles_unfiltered.size() << " obstacle pixels in the map" << std::endl;

    /* Filter vector only to include edges (not 'internal' obstacels) */
    std::vector< point > obstacles_filtered;
    for (size_t index = 0; index < obstacles_unfiltered.size(); index++) {
      for (int32_t k = 0; k < 8; k++) {
        if (!IsObstacleWS(obstacles_unfiltered[index]+neighbours[k])) {
          obstacles_filtered.push_back(point(obstacles_unfiltered[index].x,obstacles_unfiltered[index].y));
          break;
        }
      }
    }
    if (debug)
        std::cout << "There are " << obstacles_filtered.size() << " filtered obstacle pixels in the map" << std::endl;

    /* Create configurationspace from workspace and expand the filtered obstacles */
    configurationspace = workspace;
    for (size_t index = 0; index < obstacles_filtered.size(); index++) {
        ExpandPixel(obstacles_filtered[index]);
    }
}

void CupCollector::ExpandPixel(const point p)
{
    if (!IsOutsideMap(p))
        for (size_t i = 0; i < 68; i++)
            configurationspace[p.x + expand_points[i].x][p.y + expand_points[i].y] = obstacle;
    else
        for (size_t i = 0; i < 68; i++)
            if (p.x+expand_points[i].x < size_x and p.x-expand_points[i].x >= 0 and p.y+expand_points[i].y < size_y and p.y-expand_points[i].y >= 0)
                configurationspace[p.x + expand_points[i].x][p.y + expand_points[i].y] = obstacle;
}

void CupCollector::compute_wavefront()
{
  prepare_wavefront();
  auto expand_points_this = new std::vector<point>; //points to expand from in this run
  auto expand_points_next = new std::vector<point>; //points to expand from in the next run

  for(int32_t x = 0; x < size_x; x++)
  {
    for(int32_t y = 0; y < size_y; y++)
    {
        if(configurationspace[x][y] == dropoff)
            expand_points_this->push_back(point(x, y));  //start expansion from all dropoff points.
    }
  }

  while(expand_points_this->size())
  {
    for( auto this_point : *expand_points_this) //run through all the points
    {
      for(uint8_t i = 0; i < sizeof(neighbours) / sizeof(neighbours[i]); i++)
        check_neighbour(this_point, this_point + neighbours[i], *expand_points_next);
    }
    //clear list of points to check, and switch to next list.
    expand_points_this->clear();
    std::swap(expand_points_this, expand_points_next);
  }
  delete expand_points_this;
  delete expand_points_next;
}

void CupCollector::prepare_wavefront()
{ //prepare wavefront structure for computations
  //First allocate the required space
  if(wavefront == nullptr)
  {
    wavefront = new uint64_t *[size_x];
    for(long i = 0; i < size_x; i++)
    {
      wavefront[i] = new uint64_t[size_y];
    }
  }
  //fill with initial values according to map (obstacles become 1, freespace 0)
  for(int x = 0; x < size_x; x++)
  {
    for(int y = 0; y < size_y; y++)
    {
      setDistance({x, y}, (uint64_t)configurationspace[x][y]);
    }
  }
}

void CupCollector::setDistance(const point &p, const uint64_t value)
{
    assert(!IsOutsideMap(p));
    wavefront[p.x][p.y] = value;
}

uint64_t CupCollector::getDistance(const point &p) const
{ //returns distance to goal
    assert(!IsOutsideMap(p));
    return wavefront[p.x][p.y];
}

void CupCollector::check_neighbour(const point &this_point, const point &neighbour, std::vector<point> &expand_points_next)
{ //check if we should expand in to the given neighbour, and if so, add it
  //to the list of points to check on next run.

  //First check if we are out of bounds
  if(neighbour.y < 0 || neighbour.y > size_y - 1 ||
      neighbour.x < 0 || neighbour.x > size_x - 1)
  {
    return;
  }
  //only expand into point if value is either 0 or at least 2 larger than current value.
  if(getDistance(neighbour) == 0 ||
    getDistance(neighbour) > getDistance(this_point) + 1)
  {
    //set point to current value + 1
    setDistance(neighbour, getDistance(this_point) + 1);
    expand_points_next.push_back(neighbour);
  }
}



void CupCollector::cellDecomposition(Cell &seedcell, int64_t id)
{
    //This algorithm performs celldecomposition in a rather special way:
    //We start from a set of points, and try to expand out from these in a square-like shape.
    //if we hid an obstacle on one of the sides we stop expanding this way.
    //However, if not all of the side is obstacles, we start expanding a new cell from this.
    //we only use upper_left and lower_right untill the end.

    auto &up_l = seedcell.upper_left;
    auto &lo_r = seedcell.lower_right;
    //std::cout << up_l << "\t" << lo_r << std::endl;
    //check if allready occupied
    for(int x = up_l.x; x <= lo_r.x; x++)
        for(int y = up_l.y; y <= lo_r.y ; y++)
        {
            if(cellDecompMap[x][y] != 0 && cellDecompMap[x][y] != id)
            {
                return;
            }
        }

    bool expanded = false;
    std::vector<Cell> expandcells;
    //int i = 0;
    do
    {
        //std::cout << i++ << std::endl;
        //std::cout << up_l << "\t" << lo_r << std::endl;
        expandcells.clear();
        expanded = false;
        //try to expand left quite ugly, should be cleaned up a bit.
        if(!IsOutsideMap(up_l + point(-1, 0)))
        {
            int x = up_l.x - 1;

            bool lastfree = false;
            Cell newcell;
            bool hit_freespace = false;
            for(int y = up_l.y; y <= lo_r.y; y++)
            {
                if(cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id)
                {
                    if(lastfree)
                        newcell.lower_right += point(0, 1);
                    else if(hit_freespace)
                    {
                        expandcells.push_back(newcell);
                        newcell.upper_left = {up_l.x - 1, y};
                        newcell.lower_right = newcell.upper_left;
                    }
                    else
                    {
                        hit_freespace = true;
                        newcell.upper_left = {up_l.x - 1, y};
                        newcell.lower_right = newcell.upper_left;
                    }
                }

                lastfree = (cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id);
            }
            if(hit_freespace) expandcells.push_back(newcell);
            if(lastfree)
            {
                if(newcell.upper_left.y == up_l.y && newcell.lower_right.y == lo_r.y)
                {
                    expandcells.pop_back();
                    up_l.x -= 1;
                    expanded = true;
                }
            }
        }

        //expand right
        if(!IsOutsideMap(lo_r + point(1, 0)))
        {
            int x = lo_r.x + 1;

            bool lastfree = false;
            Cell newcell;
            bool hit_freespace = false;
            for(int y = up_l.y; y <= lo_r.y; y++)
            {
                if(cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id)
                {
                    if(lastfree)
                        newcell.lower_right += point(0, 1);
                    else if(hit_freespace)
                    {
                        expandcells.push_back(newcell);
                        newcell.upper_left = {lo_r.x + 1, y};
                        newcell.lower_right = newcell.upper_left;
                    }
                    else
                    {
                        hit_freespace = true;
                        newcell.upper_left = {lo_r.x + 1, y};
                        newcell.lower_right = newcell.upper_left;
                    }
                }
                lastfree = (cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id);
            }
            if(hit_freespace) expandcells.push_back(newcell);
            if(lastfree)
            {
                if(newcell.upper_left.y == up_l.y && newcell.lower_right.y == lo_r.y)
                {
                    lo_r.x += 1;
                    expanded = true;
                }
                else expandcells.push_back(newcell);
            }
        }

        //expand up
        if(!IsOutsideMap(up_l + point(0, -1)))
        {
            int y = up_l.y - 1;

            bool lastfree = false;
            Cell newcell;
            bool hit_freespace = false;
            for(int x = up_l.x; x <= lo_r.x; x++)
            {
                if(cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id)
                {
                    if(lastfree)
                        newcell.lower_right += point(1, 0);
                    else if(hit_freespace)
                    {
                        expandcells.push_back(newcell);
                        newcell.upper_left = {x, up_l.y - 1};
                        newcell.lower_right = newcell.upper_left;
                    }
                    else
                    {
                        hit_freespace = true;
                        newcell.upper_left = {x, up_l.y - 1};
                        newcell.lower_right = newcell.upper_left;
                    }
                }
                lastfree = (cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id);
            }
            if(hit_freespace) expandcells.push_back(newcell);
            if(lastfree)
            {
                if(newcell.upper_left.x == up_l.x && newcell.lower_right.x == lo_r.x)
                {
                    up_l.y -= 1;
                    expanded = true;
                }
                else expandcells.push_back(newcell);
            }
        }
        //expand down
        if(!IsOutsideMap(lo_r + point(0, 1)))
        {
            int y = lo_r.y + 1;

            bool lastfree = false;
            Cell newcell;
            bool hit_freespace = false;
            for(int x = up_l.x; x <= lo_r.x; x++)
            {
                if(cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id)
                {
                    if(lastfree)
                        newcell.lower_right += point(1, 0);
                    else if(hit_freespace)
                    {
                        expandcells.push_back(newcell);
                        newcell.upper_left = {x, lo_r.y + 1};
                        newcell.lower_right = newcell.upper_left;
                    }
                    else
                    {
                        hit_freespace = true;
                        newcell.upper_left = {x, lo_r.y + 1};
                        newcell.lower_right = newcell.upper_left;
                    }
                }
                lastfree = (cellDecompMap[x][y] == 0 || cellDecompMap[x][y] == id);
            }
            if(hit_freespace) expandcells.push_back(newcell);
            if(lastfree)
            {
                if(newcell.upper_left.x == up_l.x && newcell.lower_right.x == lo_r.x)
                {
                    lo_r.y += 1;
                    expanded = true;
                }
                else expandcells.push_back(newcell);
            }
        }

    } while(expanded);
    //make all in current cell member of itself

    for(int x = up_l.x; x <= lo_r.x; x++)
        for(int y = up_l.y; y <= lo_r.y ; y++)
        {
            cellDecompMap[x][y] = id;
        }

        seedcell.upper_right = {seedcell.lower_right.x, seedcell.upper_left.y};
        seedcell.lower_left = {seedcell.upper_left.x, seedcell.lower_right.y};
        seedcell.searched = false;
        cells.push_back(seedcell);
        for(auto &c : expandcells)
        {
            cellDecomposition(c, ++cellid);
        }
}
void CupCollector::prepareCellDecomposition()
{
    //Prepare the cell decomposition
    for(int x = 0; x < size_x; x++)
    {
        cellDecompMap.push_back(vector<int64_t>());
        for(int y = 0; y < size_y; y++)
        {
            if(configurationspace[x][y] != obstacle) cellDecompMap[x].push_back(0); //free
            else cellDecompMap[x].push_back(-1); //obstacle
        }
    }
}


void CupCollector::findWaypoints(int64_t __attribute__((unused))id){
    vector<point> points;

	// finding all waypoints at the intersection lines between the id cell and neighbor cells
    auto &c = cells[id];
    //std::cout << c.upper_left << " " << c.lower_left << " " << c.upper_right << " " << c.lower_right << std::endl;
	//find points to the left
    if(!IsOutsideMap(c.upper_left + point(-1, 0)))
    {
        int start = c.upper_left.y;
        int end = start;
        int x = c.upper_left.x - 1;
        int64_t lastid = id;
        for(int y = c.upper_left.y; y <= c.lower_left.y; y++)
        {
            if(cellDecompMap[x][y] == -1)//there is an obstacle
            {
                if(lastid != id && lastid != -1) points.push_back(point(c.upper_left.x, (start + end) / 2));
            }
            else
            {
                if(lastid == -1 || lastid == id)
                {   //start a new connection
                    start = y;
                    end = y;
                }
                else if(lastid != cellDecompMap[x][y])
                {
                    points.push_back(point(c.upper_left.x, (start + end) / 2));
                    //start a new connection
                    start = y;
                    end = y;
                }
                else end++;
            }
            lastid = cellDecompMap[x][y];
        }
        if(lastid != -1)
        {
            points.push_back(point(c.upper_left.x, (start + end) / 2));
        }
    }


    //find points to the right
    if(!IsOutsideMap(c.upper_right + point(1, 0)))
    {
        int start = c.upper_right.y;
        int end = start;
        int x = c.upper_right.x + 1;
        int64_t lastid = id;
        for(int y = c.upper_right.y; y <= c.lower_right.y; y++)
        {
            if(cellDecompMap[x][y] == -1)//there is an obstacle
            {
                if(lastid != id && lastid != -1) points.push_back(point(c.upper_right.x, (start + end) / 2));
            }
            else
            {
                if(lastid == -1 || lastid == id)
                {   //start a new connection
                    start = y;
                    end = y;
                }
                else if(lastid != cellDecompMap[x][y])
                {
                    points.push_back(point(c.upper_right.x, (start + end) / 2));
                    //start a new connection
                    start = y;
                    end = y;
                }
                else end++;
            }
            lastid = cellDecompMap[x][y];
        }
        if(lastid != -1)
        {
            points.push_back(point(c.upper_right.x, (start + end) / 2));
        }
    }

    //find points up
    if(!IsOutsideMap(c.upper_left + point(0, -1)))
    {
        int start = c.upper_left.x;
        int end = start;
        int y = c.upper_left.y - 1;
        int64_t lastid = id;
        for(int x = c.upper_left.x; x <= c.upper_right.x; x++)
        {
            if(cellDecompMap[x][y] == -1)//there is an obstacle
            {
                if(lastid != id && lastid != -1) points.push_back(point((start + end) / 2, c.upper_right.y));
            }
            else
            {
                if(lastid == -1 || lastid == id)
                {   //start a new connection
                    start = x;
                    end = x;
                }
                else if(lastid != cellDecompMap[x][y])
                {
                    points.push_back(point((start + end) / 2, c.upper_right.y));
                    //start a new connection
                    start = x;
                    end = x;
                }
                else end++;
            }
            lastid = cellDecompMap[x][y];
        }
        if(lastid != -1)
        {
            points.push_back(point((start + end) / 2, c.upper_right.y));
        }
    }

    //find points down
    if(!IsOutsideMap(c.lower_right + point(0, 1)))
    {
        int start = c.lower_left.x;
        int end = start;
        int y = c.lower_left.y + 1;
        int64_t lastid = id;
        for(int x = c.lower_left.x; x <= c.lower_right.x; x++)
        {
            if(cellDecompMap[x][y] == -1)//there is an obstacle
            {
                if(lastid != id && lastid != -1) points.push_back(point((start + end) / 2, c.lower_right.y));
            }
            else
            {
                if(lastid == -1 || lastid == id)
                {   //start a new connection
                    start = x;
                    end = x;
                }
                else if(lastid != cellDecompMap[x][y])
                {
                    points.push_back(point((start + end) / 2, c.lower_right.y));
                    //start a new connection
                    start = x;
                    end = x;
                }
                else end++;
            }
            lastid = cellDecompMap[x][y];
        }
        if(lastid != -1)
        {
            points.push_back(point((start + end) / 2, c.lower_right.y));
        }
    }
    //create all the waypoints
    //std::cout << id << " " << points.size() << std::endl;
    size_t start_index = wayPoints.size();
    for(auto &p : points)
    {
        Waypoint tmp;
        tmp.coord = p;
        //std::cout << p << std::endl;
        wayPoints.push_back(tmp);
    }
    size_t end_index = wayPoints.size();
    if(points.size() == 1) //if only a single wp in the cell, make a connection to itself.
    {
        Waypoint_connection tmp;
        tmp.index = start_index;
        tmp.connection_cell = &(cells[id]);
        tmp.cost = 0;
        wayPoints[start_index].connections.push_back(tmp);
    }
    else
    {
        //connect all waypoints in the cell
        //std::cout << start_index << " " << end_index << std::endl;
        for(size_t i = start_index; i < end_index; i++)
        {
            for(size_t j = start_index; j < end_index; j++)
            {
                if(i == j) continue;
                Waypoint_connection tmp;
                tmp.index = j;
                tmp.connection_cell = &(cells[id]);
                vector2D line(wayPoints[i].coord, wayPoints[j].coord);
                tmp.cost = WalkLine(line).size();
                wayPoints[i].connections.push_back(tmp);
            }
        }
    }

}

void CupCollector::connectNeighbours(size_t id)
{
    //check for all waypoints if we should connect and if we are allready connected
    for(size_t i = 0; i < wayPoints.size(); i++)
    {
        if(i == id) continue;
        bool connected = false;
        for(auto con : wayPoints[id].connections)
        {
            if(con.index == i)
            {
                connected = true;
                break;
            }
        }
        if(connected) continue;
        if(wayPoints[id].coord.GetDistance(wayPoints[i].coord) < 2)
        {   //create connection
            Waypoint_connection tmp;
            tmp.index = i;
            tmp.connection_cell = nullptr;
            vector2D line(wayPoints[id].coord, wayPoints[i].coord);
            tmp.cost = WalkLine(line).size();
            wayPoints[id].connections.push_back(tmp);
        }
    }
}
void CupCollector::graphConnecting(){
	for (size_t i = 0; i < cells.size(); i++)
    {
        findWaypoints(i);
    }
    //connect all waypoints which can connect to each other, and are neighbours
    for(size_t i = 0; i < wayPoints.size(); i++)
    {
        connectNeighbours(i);
    }
    if(debug)
        std::cout << "Created " << wayPoints.size() << " waypoints." << std::endl;
}

void CupCollector::cleanCellMap()
{
    for(size_t i = 0; i < cells.size(); i++)
    {
        auto &c = cells[i];
        for(int x = c.upper_left.x; x <= c.lower_right.x; x++)
        {
            for(int y = c.upper_left.y; y <= c.lower_right.y ; y++)
            {
                cellDecompMap[x][y] = i;
            }
        }
    }
}

void CupCollector::SaveWorkspaceMap(std::string name)
{
    Image workspace_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the workspace
    //Save workspace
    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            RGB col = {0, 0, 0};
            switch(workspace[x][y])
            {
                case freespace:
                    col = {255, 255, 255};
                    break;
                case obstacle:
                    col = {0, 0, 0};
                    break;
                case dropoff:
                    col = {0, 255, 0};
                    break;
                case cup:
                    col = {100, 255, 100};
                    break;
                default:
                    assert(false);
            }
            workspace_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }
    workspace_img.saveAsPPM(name);

}

void CupCollector::SaveConfigurationspaceMap(std::string name)
{
    //Save configurationspace
    Image configurationspace_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the workspace

    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            RGB col = {0, 0, 0};
            switch(configurationspace[x][y])
            {
                case freespace:
                    col = {255, 255, 255};
                    break;
                case obstacle:
                    col = {0, 0, 0};
                    break;
                case dropoff:
                    col = {0, 255, 0};
                    break;
                case cup:
                    col = {100, 255, 100};
                    break;
                default:
                    assert(false);
            }
            configurationspace_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }
    configurationspace_img.saveAsPPM(name);

}

void CupCollector::SaveConnectionMap(std::string name)
{
    //same as configuration space
    Image connections_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the workspace

    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            RGB col = {0, 0, 0};
            switch(configurationspace[x][y])
            {
                case freespace:
                    col = {255, 255, 255};
                    break;
                case obstacle:
                    col = {0, 0, 0};
                    break;
                case dropoff:
                    col = {0, 255, 0};
                    break;
                case cup:
                    col = {100, 255, 100};
                    break;
                default:
                    assert(false);
            }
            connections_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }
    //Put in connections
    for(auto &wp : wayPoints)
    {
        for(auto &con : wp.connections)
        {
            vector2D line(wp.coord, wayPoints[con.index].coord);
            //std::cout << line.getStartPoint() << " " << line.getEndPoint() << std::endl;
            for(auto &p : WalkLine(line))
                connections_img.setPixel8U(p.x, p.y, 0, 0, 255);
        }
    }

    connections_img.saveAsPPM(name);
}


void CupCollector::SaveWaypointMap(std::string name)
{
    //same as configuration space
    Image waypoints_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the workspace

    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            RGB col = {0, 0, 0};
            switch(configurationspace[x][y])
            {
                case freespace:
                    col = {255, 255, 255};
                    break;
                case obstacle:
                    col = {0, 0, 0};
                    break;
                case dropoff:
                    col = {0, 255, 0};
                    break;
                case cup:
                    col = {100, 255, 100};
                    break;
                default:
                    assert(false);
            }
            waypoints_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }
    //put in waypoints
    for(auto &wp : wayPoints)
    {
        assert(!IsOutsideMap(wp.coord));
        waypoints_img.setPixel8U(wp.coord.x, wp.coord.y, 255, 0, 0);
    }

    waypoints_img.saveAsPPM(name);
}
void CupCollector::SaveCellMap(std::string name)
{
    Image cell_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the wavefront
    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            RGB col = {0, 0, 0};
            switch(configurationspace[x][y])
            {
                case freespace:
                    col = {255, 0, 0};
                    break;
                case obstacle:
                    col = {0, 0, 0};
                    break;
                case dropoff:
                    col = {0, 255, 0};
                    break;
                case cup:
                    col = {100, 255, 100};
                    break;
                default:
                    assert(false);
            }
            cell_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }

    for(auto &c : cells)
    {
        RGB col;
        col.r = 0; col.g = rand()%200; col.b = rand()%200;
        DrawSquare(&cell_img, c, col);
    }
    cell_img.saveAsPPM(name);
}

void CupCollector::SaveWavefrontMap(std::string name)
{
    //Save wavefront
    Image wavefront_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the wavefront

    //First, find the max wavefront value
    uint64_t max = 0;
    for(unsigned int x = 0; x < wavefront_img.getWidth(); x++)
    {
      for(unsigned int y = 0; y < wavefront_img.getHeight(); y++)
      {
        if(getDistance({(int)x, (int)y}) > max)
          max = getDistance({(int)x, (int)y});
      }
    }
    //map the colours
    for(unsigned int x = 0; x < wavefront_img.getWidth(); x++)
    {
      for(unsigned int y = 0; y < wavefront_img.getHeight(); y++)
      {
        auto col = mapcolour(getDistance({(int)x, (int)y}), max);
        wavefront_img.setPixel8U(x, y, col.r, col.g, col.b);
      }
    }
    wavefront_img.saveAsPPM(name);

}

void CupCollector::SaveMaps() {

    SaveWaypointMap("waypoints.ppm");
    SaveCellMap("cells.ppm");
    SaveConnectionMap("connections.ppm");
    SaveWorkspaceMap("workspace.ppm");
    SaveWavefrontMap("wavefront.ppm");
    SaveConfigurationspaceMap("configurationspace.ppm");

}
