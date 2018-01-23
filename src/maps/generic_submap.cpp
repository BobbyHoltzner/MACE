#include "generic_submap.h"

namespace mace {
namespace maps {

GenericSubMap::GenericSubMap(const BaseGridMap* map, const pose::CartesianPosition_2D &position, const double xSize, const double ySize, bool &valid)
{
    parentMap = map;

}

} //end of namespace maps
} //end of namepsace mace
