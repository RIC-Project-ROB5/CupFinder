#include "CupCollector.hpp"
#include <limits>
#include <cassert>

using namespace std;

static point neighbours[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {-1, -1}, {1, 1}, {1, -1}, {-1, 1}}; //the neighbours a cell have
static point expand_points[] = {{1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {2, -2}, {2, -1}, {2, 0}, {2, 1}, {2, 2}, {1, 2}, {0, 2}, {-1, 2}, {-2, 2}, {-2, -1}, {-2, 0}, {-2, -1}, {-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {3, -3}, {3, -2}, {3, -1}, {3, 0}, {3, 1}, {3, 2}, {3, 3}, {2, 3}, {1, 3}, {0, 3}, {-1, 3}, {-2, 3}, {-3, 3}, {-3, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-3, -2}, {-3, -3}, {-2, -3}, {-1, -3}, {0, -3}, {1, -3}, {2, -3}, {4, -2}, {4, -1}, {4, 0}, {4, 1}, {4, 2}, {2, 4}, {1, 4}, {0, 4}, {-1, 4}, {-2, 4}, {-4, 2}, {-4, 1}, {-4, 0}, {-4, -1}, {-4, -2}, {-2, -4}, {-1, -4}, {0, -4}, {1, -4}, {2, -4}};   //68 points



CupCollector::CupCollector(rw::sensor::Image* map, const point &inDropoff)
{
    //Save dropoff point
    dropoff = inDropoff;
    //Save image dimensions
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
        std::cout << "Wavefront from " << dropoff.x << ", " << dropoff.y << " created" << std::endl;

    cellDecomposition();

	graphConnecting();

    //Create output image
    SaveMaps(map);
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
                    pixeltype = mapSpace::obstacle;
                    break;
                case 100:
                    pixeltype = mapSpace::dropoff;
                    break;
                case 150:
                    pixeltype = mapSpace::cup;
                    break;
                default:
                    pixeltype = mapSpace::freespace;
                    break;
            }
            y_line.push_back(pixeltype);
        }
        workspace.push_back(y_line);
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
    point closestpoint(-1, -1);
    float curdistance = current_point.GetDistance(line.getEndPoint());
    float mindistance = std::numeric_limits<float>::infinity();
    for(uint8_t i = 0; i < sizeof(neighbours) / sizeof(neighbours[i]); i++)
    {
        point this_point = current_point + neighbours[i];
        if(IsOutsideMap(this_point) || IsObstacleCS(this_point)) continue;

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
    assert(closestpoint != point(-1, -1) );
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
    if(workspace[p.x][p.y] == mapSpace::obstacle)
        return true;
    return false;
}

bool CupCollector::IsObstacleCS(const point &p) const
{
    if(configurationspace[p.x][p.y] == mapSpace::obstacle)
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
        if (workspace[x][y] == mapSpace::obstacle)
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
            configurationspace[p.x + expand_points[i].x][p.y + expand_points[i].y] = mapSpace::obstacle;
    else
        for (size_t i = 0; i < 68; i++)
            if (p.x+expand_points[i].x < size_x and p.x-expand_points[i].x >= 0 and p.y+expand_points[i].y < size_y and p.y-expand_points[i].y >= 0)
                configurationspace[p.x + expand_points[i].x][p.y + expand_points[i].y] = mapSpace::obstacle;
}

void CupCollector::compute_wavefront()
{
  prepare_wavefront();
  auto expand_points_this = new std::vector<point>; //points to expand from in this run
  auto expand_points_next = new std::vector<point>; //points to expand from in the next run
  expand_points_this->push_back(dropoff);  //start expansion from goal.
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
  setDistance(dropoff, 2); //set goal
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

void CupCollector::cellDecomposition(){

	int prevPixel = obstacle;

	for (size_t x = 0; x < configurationspace.size(); x++){
		int yStart = 0;
		int yEnd = 0;
		for (size_t y = 0; y < configurationspace[x].size(); y++){

			if (configurationspace[x][y] != obstacle && prevPixel == obstacle){

				yStart = y;
			}
			else if (configurationspace[x][y] == obstacle && prevPixel != obstacle){

				yEnd = y - 1;

				bool cellMatch = false;

				for (size_t i = 0; i < cells.size(); i++){
					if (cells[i].upper_left.y == yStart && cells[i].lower_left.y == yEnd){
						cellMatch = true;
						cells[i].upper_right.x++;
						cells[i].lower_right.x++;
					}
				}

				if (!cellMatch){
					Cell newCell;
					newCell.upper_left = point(x, yStart);
					newCell.lower_left = point(x, yEnd);
					newCell.upper_right = point(x, yStart);
					newCell.lower_right = point(x, yEnd);
					newCell.searched = false;
					cells.push_back(newCell);
				}
			}

			prevPixel = configurationspace[x][y];
		}
	}

	//Do cell decomposition, create waypoints and cells.
	//Connect the waypoints and cells into a graph
}

void CupCollector::findWaypoints(size_t id){
	vector<point> coord;
	for (size_t i = 0; i < cells.size(); i++){
		if (i == id) continue;
		if ((cells[i].lower_right.x + 1 == cells[id].upper_left.x &&
			(cells[i].lower_right.y > cells[id].upper_left.y &&
			cells[i].upper_right.y < cells[id].lower_left.y))
		) coord.push_back(point(cells[i].lower_right.x, (min(cells[i].lower_left.y, cells[id].lower_left.y) - max(cells[i].upper_left.y, cells[id].upper_left.y)) / 2 + max(cells[i].upper_left.y, cells[id].upper_left.y)));
		else if ((cells[i].upper_left.x - 1 == cells[id].lower_right.x &&
			(cells[i].lower_left.y > cells[id].upper_right.y &&
			cells[i].upper_left.y < cells[id].lower_right.y))
		) coord.push_back(point(cells[id].lower_right.x, (min(cells[i].lower_left.y, cells[id].lower_left.y) - max(cells[i].upper_left.y, cells[id].upper_left.y)) / 2 + max(cells[i].upper_left.y, cells[id].upper_left.y)));
	}

	vector<Waypoint*> waypointPtr;

	for (size_t i = 0; i < coord.size(); i++){
		bool match = false;
		for (size_t j = 0; j < wayPoints.size(); j++){
			if (coord[i] == wayPoints[j].coord){
				waypointPtr.push_back(&wayPoints[j]);
				match = true;
				break;
			}
		}
		if (!match){
			Waypoint temp;
			temp.coord = coord[i];
			wayPoints.push_back(temp);
			waypointPtr.push_back(&wayPoints[wayPoints.size() - 1]);
		}
	}

	for (size_t i = 0; i < waypointPtr.size(); i++){
		for (size_t j = 0; j < waypointPtr.size(); j++){
			if (i == j) continue;
			Waypoint_connection temp;
			temp.linkptr = waypointPtr[j];
			temp.connection_cell = &cells[id];
			(*waypointPtr[i]).connections.push_back(temp);
		}
	}
}

void CupCollector::graphConnecting(){
	for (size_t i = 0; i < cells.size(); i++) findWaypoints(i);
}

void CupCollector::SaveMaps(rw::sensor::Image* map) {
    int value;

    //Save workspace
    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            if (workspace[x][y] == 1) {
                value = 0;
            } else {
                value = 255-workspace[x][y];
            }
            map->setPixel8U(x, y, value);
        }
    }
    map->saveAsPGM("workspace.pgm");

    //Save configurationspace
    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            if (configurationspace[x][y] == 1) {
                value = 0;
            } else {
                value = 255-configurationspace[x][y];
            }
            map->setPixel8U(x, y, value);
        }
    }
    map->saveAsPGM("configurationspace.pgm");

    //Save wavefront
    for (int32_t x = 0; x < size_x; x++) {
        for (int32_t y = 0; y < size_y; y++) {
            if (wavefront[x][y] == 0) {
                value = 255;
            } else {
                value = wavefront[x][y] % 255;;
            }
            map->setPixel8U(x, y, value);
        }
    }
    if (true) { //Create white circle around dropoff
        value = 255;
        map->setPixel8U(dropoff.x, dropoff.y-3, value);
        map->setPixel8U(dropoff.x+1, dropoff.y-3, value);
        map->setPixel8U(dropoff.x+2, dropoff.y-2, value);
        map->setPixel8U(dropoff.x+3, dropoff.y-1, value);
        map->setPixel8U(dropoff.x+3, dropoff.y, value);
        map->setPixel8U(dropoff.x+3, dropoff.y+1, value);
        map->setPixel8U(dropoff.x+2, dropoff.y+2, value);
        map->setPixel8U(dropoff.x+1, dropoff.y+3, value);
        map->setPixel8U(dropoff.x, dropoff.y+3, value);
        map->setPixel8U(dropoff.x-1, dropoff.y+3, value);
        map->setPixel8U(dropoff.x-2, dropoff.y+2, value);
        map->setPixel8U(dropoff.x-3, dropoff.y+1, value);
        map->setPixel8U(dropoff.x-3, dropoff.y, value);
        map->setPixel8U(dropoff.x-3, dropoff.y-1, value);
        map->setPixel8U(dropoff.x-2, dropoff.y-2, value);
        map->setPixel8U(dropoff.x-1, dropoff.y-3, value);
    }
    map->saveAsPGM("wavefront.pgm");
}
