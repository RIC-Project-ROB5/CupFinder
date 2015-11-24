#include "Image.hpp"
#include "PPMLoader.hpp"
#include "CupCollector.hpp"
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

    //load the Map
    string filename(argv[1]);
    cout << "Loading image" << endl;
    Image *map = PPMLoader::load(filename);
    //Find the cups
    cout << "Finding cups" << endl;
    CupCollector CCollector(map);

    //clean up
    delete map;

    return 0;
}
