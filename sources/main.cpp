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
    //Find the cups
    cout << "Finding cups" << endl;
    CupCollector CCollector(map);
    cout << "Done finding cups" << endl;

    //clean up
    delete map;

    return 0;
}
