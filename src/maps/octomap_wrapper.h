#ifndef OCTOMAP_WRAPPER_H
#define OCTOMAP_WRAPPER_H

#include "octomap/OcTree.h"
#include "octomap_sensor_definition.h"

#include "dynamic_2D_grid.h"

namespace mace{
namespace maps{

class OctomapWrapper
{
public:

    enum class OccupiedResult
    {
        NO_DATA,
        OUTSIDE_ENVIRONMENT,
        OCCUPIED,
        NOT_OCCUPIED
    };

public:
    OctomapWrapper(const double &resolution = 0.5, const OctomapSensorDefinition &sensorProperties = OctomapSensorDefinition());

    bool is2DProjectionEnabled() const;

    void set2DProjection(const bool enable);

    void loadOctreeFromBT(const std::string &path);


private:
    bool enabled2DProjection = false;
    double treeResolution = 0.5;
    unsigned int treeDepth = 0;


    octomap::OcTree* m_Tree;
    maps::Dynamic2DGrid* m_Map;

    OctomapSensorDefinition* m_sensorProperties;
};

} //end of namespace maps
} //end of namespace mace


#endif // OCTOMAP_WRAPPER_H
