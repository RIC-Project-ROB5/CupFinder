#include "CupCollector.hpp"
#include <limits>
#include <cassert>

using namespace std;
using namespace rw::sensor;

static point neighbours[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {-1, -1}, {1, 1}, {1, -1}, {-1, 1}}; //the neighbours a cell have


static point expand_points[] = {{1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {2, -2}, {2, -1}, {2, 0}, {2, 1}, {2, 2}, {1, 2}, {0, 2}, {-1, 2}, {-2, 2}, {-2, -1}, {-2, 0}, {-2, -1}, {-2, -2}, {-1, -2}, {0, -2}, {1, -2}, {3, -3}, {3, -2}, {3, -1}, {3, 0}, {3, 1}, {3, 2}, {3, 3}, {2, 3}, {1, 3}, {0, 3}, {-1, 3}, {-2, 3}, {-3, 3}, {-3, 2}, {-3, 1}, {-3, 0}, {-3, -1}, {-3, -2}, {-3, -3}, {-2, -3}, {-1, -3}, {0, -3}, {1, -3}, {2, -3}, {4, -2}, {4, -1}, {4, 0}, {4, 1}, {4, 2}, {2, 4}, {1, 4}, {0, 4}, {-1, 4}, {-2, 4}, {-4, 2}, {-4, 1}, {-4, 0}, {-4, -1}, {-4, -2}, {-2, -4}, {-1, -4}, {0, -4}, {1, -4}, {2, -4}};   //68 points



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

    cellDecomposition();

	graphConnecting();

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
    std::cout << cups << std::endl;
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
    std::cout << line.getStartPoint().x << ", " << line.getStartPoint().x << "]-[" <<
    line.getEndPoint().x << "," << line.getEndPoint().y << std::endl;
    std::vector<point> line_points;
    point cur = line.getStartPoint();
    line_points.push_back(cur);
    while(cur != line.getEndPoint())
    {
        point next_point = FindNextPointOnLine(line, cur);
        cur = next_point;
        std::cout << cur.x << " " << cur.y << std::endl;
        line_points.push_back(cur);
        if(line_points.size() > 100) exit(0);
    }
    return line_points;
}

point CupCollector::FindNextPointOnLine(const vector2D &line, const point &cur) const
{
    //check all 8 directions collect the ones which are closer to the endpoint of the line
    // to the endpoint of the linethan the current.
    point closestpoint(-1, -1);
    float curdistance = cur.GetDistance(line.getEndPoint());
    float mindistance = std::numeric_limits<float>::infinity();
    for(uint8_t i = 0; i < sizeof(neighbours) / sizeof(neighbours[i]); i++)
    {
        point this_point = cur + neighbours[i];
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

	//remove all nonaccesible cells
    auto tmpcells = cells;
    cells.clear();
    for( auto &cell : tmpcells)
    {
        if (getDistance(cell.lower_left) != freespace)
            cells.push_back(cell);
    }
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

void CupCollector::SaveWaypointMap(__attribute__((unused))std::string name)
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
    int i = 0;
    for(auto &wp : wayPoints)
    {
        std::cout << std::endl;
        std::cout << "waypoint " << i++ << " Out of " << wayPoints.size() <<
        " Got " << wp.connections.size() << " connections." << " Coords are ["
        <<wp.coord.x << "," << wp.coord.y << "]" << std::endl;
        //put in waypoint
        waypoints_img.setPixel8U(wp.coord.x, wp.coord.y, 255, 0, 0);
        //put in connections
        for(auto &con : wp.connections)
        {
            //create line
            vector2D line(wp.coord, con.linkptr->coord);
            std::cout << "[" <<  con.linkptr->coord.x << "," << con.linkptr->coord.y << "]";
            std::cout << "\tPointer address is: " << con.linkptr << std::endl;
            //for(auto &p : WalkLine(line))
            //{
            //    waypoints_img.setPixel8U(p.x, p.y, 0, 0, 255);
            //}
        }
    }
    waypoints_img.saveAsPPM(name);
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
    SaveWorkspaceMap("workspace.ppm");
    SaveWavefrontMap("wavefront.ppm");
    SaveConfigurationspaceMap("configurationspace.ppm");

}
