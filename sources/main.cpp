#include "Image.hpp"
#include "PPMLoader.hpp"
#include "CupCollector.hpp"
#include "point.hpp"
#include <ctime>
#include <cstdlib>
#include <iostream>

using namespace std;
using namespace rw::loaders;
using namespace rw::sensor;

int main(int argc, char const *argv[]) {
    if(argc < 2)
    {
        cout << "Not enough arguments!" << endl;
        cout << "Arguments should be:" << endl;
        cout << "<MapFile>" << endl;
    }
    srand(time(NULL));
    //load the Map
    string filename(argv[1]);
    cout << "Loading image" << endl;
    Image *map = PPMLoader::load(filename);
    //Init Cupcollector
    CupCollector CCollector(map);
    std::cout << "Saving illustrations" << std::endl;
    CCollector.SaveWaypointMap("waypoints.ppm");
    CCollector.SaveCellMap("cells.ppm");
    CCollector.SaveConnectionMap("connections.ppm");
    CCollector.SaveWorkspaceMap("workspace.ppm");
    CCollector.SaveWavefrontMap("wavefront.ppm");
    CCollector.SaveConfigurationspaceMap("configurationspace.ppm");
    std::cout << "Finding path" << std::endl;
    auto path = CCollector.get_path();
    CCollector.SaveWalkMap("RobWalkMap.ppm", path);
    CCollector.SaveSearchedMap("SearchMap.ppm");

    std::cout << "Path is " << path.size() << " long." << std::endl;


    cout << "Done finding cups" << endl;

    //clean up
    delete map;

    return 0;
}
