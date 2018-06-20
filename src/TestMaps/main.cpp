#include <QCoreApplication>

#include "base/geometry/polygon_2DC.h"

#include "maps/octomap_wrapper.h"

using namespace mace ;
using namespace geometry;


int main(int argc, char *argv[])
{
    mace::maps::OctomapWrapper *wrapper = new mace::maps::OctomapWrapper();


    delete wrapper;
}
