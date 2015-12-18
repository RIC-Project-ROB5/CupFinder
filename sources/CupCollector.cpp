#include "CupCollector.hpp"
#include <cassert>
#include <cstdlib>
#include <algorithm>
#include <cmath>

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


CupCollector::CupCollector(Image* map, bool _verbose)
{
    verbose = _verbose;
    //Save image dimensions
    size_x = map->getWidth();
    size_y = map->getHeight();
    if(verbose)
        std::cout << "Image size is " << size_x << ", " << size_y << std::endl;

    //Convert image into mapspace map of the workspace
    CreateWorkspaceMap(map);
    if(verbose)
        std::cout << "Workspace created" << std::endl;
    //copy workspace map into searchmap
    searchmap = workspace;
    //Convert workspace into configurationspace
    CreateConfigurationspaceMap();
    if(verbose)
        std::cout << "Configurationspace created" << std::endl;

    compute_wavefront();
    if(verbose)
        std::cout << "Wavefront created" << std::endl;

    prepareCellDecomposition();
    Cell seedc;
    seedc.upper_left = {2000, 1280};
    seedc.lower_right = seedc.upper_left;
    cellDecomposition(seedc, cellid);
    if(verbose)
        std::cout << "Created " << cells.size() << " cells." << std::endl;
    //set the right id's in the cell decomposition map
    cleanCellMap();
	graphConnecting();
    if(!validateMap())
    {
        std::cout << "Generated waypoint map is invalid, bailing out" << std::endl;
        exit(1);
    }
    if(verbose)
        std::cout << "Generated waypoint map is valid" << std::endl;
    if(!isGraphConnected())
    {
        std::cout << "Not all generated waypoints are connected, bailing out" << std::endl;
        exit(1);
    }
    if(verbose)
        std::cout << "All generated waypoints are connected." << std::endl;
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
    int cups = 0;
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
                    cups++;
                    break;
                default:
                    pixeltype = freespace;
                    break;
            }
            y_line.push_back(pixeltype);
        }
        workspace.push_back(y_line);
    }
    if(verbose)
        std::cout << "Number of cups in map: " << cups << std::endl;
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

void CupCollector::add_paths(std::vector<point> &v1, std::vector<point> &v2)
{   //Adds the two paths in the vectors, and applies some hacks to make it continues
    while(v1.size() && v2.size() && v1.back() == v2.front()) v1.pop_back(); //remove double points
    if(v1.size() && v2.size() && !(v1.back().isNeighbour(v2.front()))) //make continues
    {
        vector2D line(v1.back(), v2.front());
        auto w = SearchLine(line, ROB_VIEW_RANGE);
        w.pop_back();
        v1.insert(v1.end(), w.begin(), w.end());
    }
    v1.insert(v1.end(), v2.begin(), v2.end());
}

std::vector<point> CupCollector::SearchGraph(Waypoint &wp)
{
    wp.visited = true;
    //For all connections, travel to the if they are unvisited.
    //if the cell they are connected to are unsearched, search it
    std::vector<point> walkpath;
    for(auto &con : wp.connections)
    {
        auto &wp_c = wayPoints[con.index];
        auto &c = con.connection_cell;
        vector2D con_line(wp.coord, wp_c.coord);
        if(c == nullptr && !wp_c.visited) //simply go to the cell
        {
            auto w = WalkLine(con_line, configurationspace); //get path to waypoint
            auto next_path = SearchGraph(wp_c); //get path from next recursion
            //add to our path : the way to the waypoint,
            //the path traveled further down the graph, and the reverse path from the Cell
            add_paths(walkpath, w);
            add_paths(walkpath, next_path);
            std::reverse(w.begin(), w.end());
            add_paths(walkpath, w);
        }
        else if(c != nullptr && !wp_c.visited)
        {
            //Travel to the cell, but do it through a search of the cell if it haven't been searched.
            //When returning, just walk in straigh line
            auto w = WalkLine(con_line, configurationspace); //get path to waypoint
            std::vector<point> searchpath;
            if(!c->searched)
            {
                searchpath = SearchCell(wp, wp_c, *c);
                c->searched = true;
            }
            else searchpath = w;
            auto next_path = SearchGraph(wp_c); //get path from next recursion
            add_paths(walkpath, searchpath);
            add_paths(walkpath, next_path);
            std::reverse(w.begin(), w.end());
            add_paths(walkpath, w);
        }
        else if(c != nullptr && !c->searched && wp_c.visited)
        {
            //simply search the cell and return to the same spot.
            auto searchpath = SearchCell(wp, wp, *c);
            add_paths(walkpath, searchpath);
            for(size_t i = 1; i < walkpath.size(); i++)
                assert(walkpath[i].isNeighbour(walkpath[i - 1]));
            c->searched = true;
        }
        else
        {
            assert(wp_c.visited);
            assert(c == nullptr || c->searched);
        }
    }
    #ifndef NDEBUG
    bool fail = false;
    for(size_t i = 1; i < walkpath.size(); i++)
        if(!walkpath[i].isNeighbour(walkpath[i - 1])){ fail = true; std::cout << i << std::endl;}
    if(fail) for(auto &p : walkpath) std::cout << p << std::endl;
    #endif
    for(size_t i = 1; i < walkpath.size(); i++)
        assert(walkpath[i].isNeighbour(walkpath[i - 1]));
    return walkpath;
}


std::vector<point> CupCollector::get_path()
{
    //Mark all waypoints and cells unvisited
    for(auto &wp : wayPoints)
        wp.visited = false;
    for(auto &c : cells)
        c.searched = false;
    auto ret_vec = SearchGraph(wayPoints[0]); //start at waypoint 0
    //make sure all cells are searchd and all graphs visited (only complains if asserts are enabled)
    for(auto __attribute__((unused))&wp : wayPoints)
        assert(wp.visited);
    for(auto __attribute__((unused))&c : cells)
        assert(c.searched);
    for(size_t i = 1; i < ret_vec.size(); i++) //make sure path is continues.
    {
        assert(ret_vec[i].isNeighbour(ret_vec[i - 1]));
    }

    if(verbose)
        std::cout << "Collected " << total_cups + current_cups << " cups." << std::endl;

    return ret_vec;
}

bool CupCollector::isblocked(vector2D &line) const
{   //Check if we can walk along the lines
    if(line.getStartPoint() == line.getEndPoint()) return false;
    if(WalkLine(line, searchmap).size() > 0) return false; //Walkline returns a empty vec if we can't walk to the point.
    return true;
}

std::vector<point> CupCollector::GetCup(point &p_start, point &p_cup)
{   //Walk towards the cup while it's outside pickup range.
    auto cup_path = WalkLine(vector2D(p_start, p_cup), configurationspace, ROB_PICKUP_RANGE);
    //pickup the cup
    current_cups++;

    auto ret_vec = cup_path;
    //go back
    if(cup_path.size()) cup_path.pop_back();
    ret_vec.insert(ret_vec.end(), cup_path.rbegin(), cup_path.rend());
    //if we have more than max_cups, drop them of in dropoff
    if(current_cups >= MAX_CUPS)
    {
        auto dropoff_path = getGoalPath(p_start);
        //Add the path, and add it in reverse afterwards.
        ret_vec.insert(ret_vec.end(), dropoff_path.begin(), dropoff_path.end());
        dropoff_path.pop_back();
        ret_vec.insert(ret_vec.end(), dropoff_path.rbegin(), dropoff_path.rend());
        total_cups += current_cups;
        current_cups = 0;
    }
    for(size_t i = 1; i < ret_vec.size(); i++)
        assert(ret_vec[i].isNeighbour(ret_vec[i - 1]));
    return ret_vec;
}

std::vector<point> CupCollector::SearchForCups(point &p, float distance)
{   //search and collect cups in distance
    std::vector<point> ret_vec;
    for(int x = -int(distance + 1); x <= int(distance + 1); x++)
    {
        for(int y = -int(distance + 1); y <= int(distance + 1); y++)
        {
            point check_p = p + point(x, y);
            if(IsOutsideMap(check_p) || p.GetDistance(check_p) >= distance) //check if inside map and within range of robot
                continue;

            vector2D line(p, check_p);
            //if(isblocked(line)) continue;
            if(searchmap[check_p.x][check_p.y] == cup && !isblocked(line))
            {   //The point is a cup and there is no obstacles in the way
                //Retrieve the cup.
                auto v = GetCup(p, check_p);
                if(ret_vec.size()) while(ret_vec.back() == v.front()) ret_vec.pop_back(); //ugly hack
                ret_vec.insert(ret_vec.end(), v.begin(), v.end());
                searchmap[check_p.x][check_p.y] = searched; //mark as searched.
            }
            //assert
            if(searchmap[check_p.x][check_p.y] != obstacle && searchmap[check_p.x][check_p.y] != cup)
            {
                searchmap[check_p.x][check_p.y] = searched;
            }
        }
    }
    for(size_t i = 1; i < ret_vec.size(); i++)
        assert(ret_vec[i].isNeighbour(ret_vec[i - 1]));
    return ret_vec;
}

void CupCollector::minimise_cell(Cell &c)
{//Make the cell smaler by checking all around it if it's edges have been searched.
    bool shrinked = true;
    while(shrinked)
    {
        shrinked = false;
        //check top
        bool allsearched = true;
        for(int x = c.upper_left.x - ROB_RADIUS; x <= c.lower_right.x + ROB_RADIUS; x++)
            if(!IsOutsideMap(point(x, c.upper_left.y)))
                if(searchmap[x][c.upper_left.y] != searched) allsearched = false;
        if(allsearched == true && c.upper_left.y < c.lower_right.y)
        {
            c.upper_left.y++;
            shrinked = true;
        }
        //check bottom
        allsearched = true;
        for(int x = c.upper_left.x - ROB_RADIUS; x <= c.lower_right.x + ROB_RADIUS; x++)
            if(!IsOutsideMap(point(x, c.lower_right.y)))
                if(searchmap[x][c.lower_right.y] != searched) allsearched = false;
        if(allsearched == true && c.upper_left.y < c.lower_right.y)
        {
            c.lower_right.y--;
            shrinked = true;
        }
        //check left
        allsearched = true;
        for(int y = c.upper_left.y - ROB_RADIUS; y <= c.lower_right.y + ROB_RADIUS; y++)
            if(!IsOutsideMap(point(c.upper_left.x, y)))
                if(searchmap[c.upper_left.x][y] != searched) allsearched = false;
        if(allsearched == true && c.upper_left.x < c.lower_right.x)
        {
            c.upper_left.x++;
            shrinked = true;
        }
        //check right
        allsearched = true;
        for(int y = c.upper_left.y - ROB_RADIUS; y <= c.lower_right.y + ROB_RADIUS; y++)
            if(!IsOutsideMap(point(c.lower_right.x, y)))
                if(searchmap[c.lower_right.x][y] != searched) allsearched = false;
        if(allsearched == true && c.upper_left.x < c.lower_right.x)
        {
            c.lower_right.x--;
            shrinked = true;
        }
    }
    c.upper_right = {c.lower_right.x, c.upper_left.y};
    c.lower_left = {c.upper_left.x, c.lower_right.y};
}

std::vector<point> CupCollector::SearchCell(const Waypoint &startpoint, const Waypoint &endpoint, const Cell &cell)
{
    //Searches the given cell for cups, collects them, return them to dropoff
    //Start at startpoint and exit at endpoint.
    //The cell is covered in a shrinking spiraling pattern.
    std::vector<point> ret_vec;
    //Minimise the cell so we don't search allready searched areas, and make a copy
    Cell this_cell = cell;
    minimise_cell(this_cell);
    //in turns, go to each corner, search allong the way, make them smaller(to cirkel invard).
    //Stop and go to end when the corners are closer than COVER_RANGE fom each other.
    bool continue_coverage = true;
    for(int i = 0; continue_coverage == true; i++)
    {
        //std::cout << i << std::endl;
        //std::cout << this_cell.upper_left << " " << this_cell.lower_right << std::endl;
        //Make the cell smaller according to COVER_RANGE
        continue_coverage = (this_cell.upper_left.x + COVER_RANGE <= this_cell.lower_right.x - COVER_RANGE) &&
                            (this_cell.upper_left.y + COVER_RANGE <= this_cell.lower_right.y - COVER_RANGE);

        if(i == 0) //first time, go from startpoint to first corner
        {
            vector2D line(startpoint.coord, this_cell.upper_left);
            auto v = SearchLine(line, ROB_VIEW_RANGE);
            add_paths(ret_vec, v);
            for(size_t j = 1; j < ret_vec.size(); j++)
                assert(ret_vec[j].isNeighbour(ret_vec[j - 1]));

        }
        else //else go from current point (back of ret_vec) to upper left corner.
        {
            assert(ret_vec.size());
            vector2D line(ret_vec.back(), this_cell.upper_left);
            auto v = SearchLine(line, ROB_VIEW_RANGE);
            add_paths(ret_vec, v);
            for(size_t j = 1; j < ret_vec.size(); j++)
                assert(ret_vec[j].isNeighbour(ret_vec[j - 1]));

        }
        //continue to the rest of the corners
        if(ret_vec.size() != 0 && ret_vec.back() != this_cell.upper_left) //hack to fix some small bug
        {
            vector2D line(ret_vec.back(), this_cell.upper_left);
            auto v = SearchLine(line, ROB_VIEW_RANGE);
            add_paths(ret_vec, v);
        }
        assert(ret_vec.size() == 0 || ret_vec.back() == this_cell.upper_left);
        vector2D line1(this_cell.upper_left, this_cell.upper_right);
        vector2D line2(this_cell.upper_right, this_cell.lower_right);
        vector2D line3(this_cell.lower_right, this_cell.lower_left);
        vector2D line4(this_cell.lower_left, this_cell.upper_left);

        auto v = SearchLine(line1, ROB_VIEW_RANGE);
        add_paths(ret_vec, v);
        v = SearchLine(line2, ROB_VIEW_RANGE);
        add_paths(ret_vec, v);
        v = SearchLine(line3, ROB_VIEW_RANGE);
        add_paths(ret_vec, v);
        v = SearchLine(line4, ROB_VIEW_RANGE);
        add_paths(ret_vec, v);
        //check if this is the last time we should cover the cell.
        this_cell.upper_left = point(min(this_cell.upper_left.x + max(1, int(COVER_RANGE + COVER_RANGE / sqrt(2))), this_cell.lower_right.x),
                                     min(this_cell.upper_left.y + max(1, int(COVER_RANGE + COVER_RANGE / sqrt(2))), this_cell.lower_right.y));
        this_cell.lower_right = point(max(this_cell.lower_right.x - max(1, int(COVER_RANGE + COVER_RANGE / sqrt(2))), this_cell.upper_left.x),
                                      max(this_cell.lower_right.y - max(1, int(COVER_RANGE + COVER_RANGE / sqrt(2))), this_cell.upper_left.y));

        this_cell.upper_right = {this_cell.lower_right.x, this_cell.upper_left.y};
        this_cell.lower_left = {this_cell.upper_left.x, this_cell.lower_right.y};
    }

    vector2D line(startpoint.coord, endpoint.coord);
    //Go to the endpoint.
    if(ret_vec.size() > 0)
         line = vector2D(ret_vec.back(), endpoint.coord);
    auto v = WalkLine(line, configurationspace);
    ret_vec.insert(ret_vec.end(), v.begin(), v.end());

    for(size_t i = 1; i < ret_vec.size(); i++)
        assert(ret_vec[i].isNeighbour(ret_vec[i - 1]));
    return ret_vec;

}

std::vector<point> CupCollector::SearchLine(vector2D const &line, float distance)
{
    std::vector<point> searchvec;
    point cur = line.getStartPoint();
    auto s = SearchForCups(cur, distance);
    searchvec.insert(searchvec.end(), s.begin(), s.end());
    if(s.size() && s.back() != cur) searchvec.push_back(cur);
    for(size_t i = 1; i < searchvec.size(); i++)
        assert(searchvec[i].isNeighbour(searchvec[i - 1]));

    while(cur != line.getEndPoint())
    {
        bool success = false;
        point next_point = FindNextPointOnLine(line, cur, configurationspace, &success);
        assert(success);
        assert(cur.isNeighbour(next_point));
        cur = next_point;
        if(searchvec.size()) //ugly hack
        {
            while(searchvec.back() == cur)
            {
                searchvec.pop_back();
            }
            if(!searchvec.back().isNeighbour(cur))
            {
                vector2D line2(searchvec.back(), cur);
                auto v = SearchLine(line2, distance);
                if(v.size()) v.pop_back();
                searchvec.insert(searchvec.end(), v.begin(), v.end());
            }
        }
        searchvec.push_back(cur);
        auto s2 = SearchForCups(cur, distance);
        searchvec.insert(searchvec.end(), s2.begin(), s2.end());
    }
    for(size_t i = 1; i < searchvec.size(); i++)
        assert(searchvec[i].isNeighbour(searchvec[i - 1]));
    return searchvec;
}

std::vector<point> CupCollector::WalkLine(vector2D const &line, const std::vector< std::vector< mapSpace> > &map, float distance) const
{   //Walk along the line from startpoint to endpoint, or untill within distance from endpoint
    //There should be a clear path, e.g. no obstacles.
    //std::cout << "[" << line.getStartPoint().x << ", " << line.getStartPoint().y << "]-[" <<
    //line.getEndPoint().x << "," << line.getEndPoint().y << "]" <<std::endl;
    std::vector<point> line_points;
    point cur = line.getStartPoint();
    //line_points.push_back(cur); //don't include first point, as we are already on it
    while(cur != line.getEndPoint() && cur.GetDistance(line.getEndPoint()) > distance )
    {
        bool success = false;
        point next_point = FindNextPointOnLine(line, cur, map, &success);
        if(!success) return std::vector<point>();
        cur = next_point;
        //std::cout << cur << std::endl;
        line_points.push_back(cur);
        //if(line_points.size() > 100) exit(0);
    }

    for(size_t i = 1; i < line_points.size(); i++)
        assert(line_points[i].isNeighbour(line_points[i - 1]));
    return line_points;
}

point CupCollector::FindNextPointOnLine(const vector2D &line, const point &cur, const std::vector< std::vector< mapSpace> > &map, bool *success) const
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
        if(IsOutsideMap(this_point) || IsObstacle(this_point, map)) continue;

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

std::vector<point> CupCollector::getGoalPath(const point &start) const
{
  std::vector<point> path;
  path.push_back(start);
  while(getDistance(path.back()) != 1 && getDistance(path.back()) != 2) //continue until we are either in an obstacle or at the goal
  {
    bool success;
    point next_point = get_next_point(path.back(), &success);
    if(success) path.push_back(next_point);
    else break;
  }
  return path;
}

point CupCollector::get_next_point(const point &curpoint, bool *success) const
{
  for(uint8_t i = 0; i < sizeof(neighbours) / sizeof(neighbours[i]); i++) //go through all possible neighbours
  {
    //Check if we are out of bounds
    auto neighbour = curpoint + neighbours[i];
    if(neighbour.y < 0 || neighbour.y > size_y - 1 ||
        neighbour.x < 0 || neighbour.x > size_x - 1)
    {
      continue;
    }
    if(getDistance(neighbour) < getDistance(curpoint) && getDistance(neighbour) != 1)
    {
      *success = true;
      return neighbour;
    }
  }
  //if we didn't find a point, we are probably in an obstacle
  //set success to false and return current point
  *success = false;
  return curpoint;
}

bool CupCollector::IsOutsideMap(const point &p) const
{
    if(p.y < 0 || p.x < 0 || p.y >= size_y || p.x >= size_x)
        return true;
    return false;
}

bool CupCollector::IsObstacle(const point &p, const std::vector< std::vector< mapSpace> > &map) const
{
    if(map[p.x][p.y] == obstacle)
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
    if(verbose)
        std::cout << "There are " << obstacles_unfiltered.size() << " obstacle pixels in the map" << std::endl;

    /* Filter vector only to include edges (not 'internal' obstacels) */
    std::vector< point > obstacles_filtered;
    for (size_t index = 0; index < obstacles_unfiltered.size(); index++) {
      for (int32_t k = 0; k < 8; k++) {
        point tmp_point = obstacles_unfiltered[index]+neighbours[k];
        if(IsOutsideMap(tmp_point)) continue;
        if (!IsObstacle(tmp_point, workspace)) {
          obstacles_filtered.push_back(point(obstacles_unfiltered[index]));
          break;
        }
      }
    }
    if(verbose)
        std::cout << "There are " << obstacles_filtered.size() << " filtered obstacle pixels in the map" << std::endl;

    /* Create configurationspace from workspace and expand the filtered obstacles */
    configurationspace = workspace;
    for (size_t index = 0; index < obstacles_filtered.size(); index++) {
        ExpandPixel(obstacles_filtered[index]);
    }
}

void CupCollector::ExpandPixel(const point p)
{
    for (size_t i = 0; i < 68; i++)
    {
        point expand_point = p + expand_points[i];
        if(!IsOutsideMap(expand_point))
        {
            //assert(configurationspace[expand_point.x][expand_point.y] != cup);
            configurationspace[expand_point.x][expand_point.y] = obstacle;
        }
    }
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
    //The function is pretty long, but most of it is repetition for up/down/left/right
    //Should probably be split up in multiple functions
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
    //The function is pretty long, but most of it is repetition for up/down/left/right
    //Should probably be split up in multiple functions

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
                tmp.cost = WalkLine(line, configurationspace).size();
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
            tmp.cost = WalkLine(line, configurationspace).size();
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
    if(verbose)
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
            for(auto &p : WalkLine(line, configurationspace))
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
        col.r = 50; col.g = rand()%200 + 50; col.b = rand()%200 + 50;
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

void CupCollector::SaveSearchedMap(std::string name)
{
    //All is white apart from obstacles.
    //Walkpath is green.
    Image search_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the workspace

    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            RGB col = {0, 0, 0};
            switch(searchmap[x][y])
            {
                case freespace:
                    col = {255, 255, 255};
                    break;
                case obstacle:
                    col = {0, 0, 0};
                    break;
                case searched:
                    col = {255, 0, 0};
                    break;
                case cup:
                    col = {0, 255, 0};
                    break;
                default:
                    col = {255, 255, 255};
            }
            search_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }
    search_img.saveAsPPM(name);

}
void CupCollector::SaveWalkMap(std::string name, std::vector<point> &path)
{
    //clean the decomp map, and use it for plotting walk
    int64_t max_path = 0;
    for(auto &y_line : cellDecompMap)
        for(auto &pixel : y_line)
            pixel = 0;
    for(auto &p : path)
        max_path = max(max_path, ++cellDecompMap[p.x][p.y]);
    //All is white apart from obstacles.
    //Walkpath is green.
    Image walk_img(size_x, size_y, Image::ColorCode::RGB, Image::PixelDepth::Depth8U); //image object for plotting the workspace
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
                    col = {255, 255, 255};
                    break;
                case cup:
                    col = {255, 255, 255};
                    break;
                default:
                    assert(false);
            }
            walk_img.setPixel8U(x, y, col.r, col.g, col.b);
        }
    }
    for(auto &p : path)
    {
        RGB col;
        col.r = 0;
        col.g = 255 - uint8_t((200. / max_path) * cellDecompMap[p.x][p.y]);
        col.b = 255 - uint8_t((200. / max_path) * cellDecompMap[p.x][p.y]);
        walk_img.setPixel8U(p.x, p.y, col.r, col.g, col.b);
    }
    walk_img.saveAsPPM(name);

}
