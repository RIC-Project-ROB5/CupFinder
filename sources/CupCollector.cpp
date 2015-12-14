#include "CupCollector.hpp"


using namespace std;

CupCollector::CupCollector(__attribute__((unused))rw::sensor::Image* map)
{
    //Convert image into mapspace map of the workspace
    CreateWorkspaceMap(map);

    //convert workspace into configurationspace
    configurationspace = workspace; //just set them as equal for now.

    cellDecomposition();

	graphConnecting();

}

CupCollector::~CupCollector()
{
    //Do cleanup
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
        if(IsOutsideMap(this_point) || IsObstacle(this_point)) continue;

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

bool CupCollector::IsObstacle(const point &p) const
{
    if(configurationspace[p.x][p.y] == mapSpace::obstacle)
        return true;
    return false;
}

void CupCollector::cellDecomposition(){

	int prevPixel = obstacle;

	for (int x = 0; x < configurationspace.size(); x++){
		int yStart = 0;
		int yEnd = 0;
		for (int y = 0; y < configurationspace[x].size(); y++){

			if (configurationspace[x][y] != obstacle && prevPixel == obstacle){

				yStart = y;
			}
			else if (configurationspace[x][y] == obstacle && prevPixel != obstacle){

				yEnd = y - 1;

				bool cellMatch = false;

				for (int i = 0; i < cells.size(); i++){
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

void CupCollector::findWaypoints(int id){
	vector<point> coord;
	for (int i = 0; i < cells.size(); i++){
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

	for (int i = 0; i < coord.size(); i++){
		bool match = false;
		for (int j = 0; j < wayPoints.size(); j++){
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

	for (int i = 0; i < waypointPtr.size(); i++){
		for (int j = 0; j < waypointPtr.size(); j++){
			if (i == j) continue;
			Waypoint_connection temp;
			temp.linkptr = waypointPtr[j];
			temp.connection_cell = &cells[id];
			(*waypointPtr[i]).connections.push_back(temp);
		}
	}
}

void CupCollector::graphConnecting(){
	for (int i = 0; i < cells.size(); i++) findWaypoints(i);
}
