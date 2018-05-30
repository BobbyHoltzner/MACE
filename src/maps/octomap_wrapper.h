#ifndef OCTOMAP_WRAPPER_H
#define OCTOMAP_WRAPPER_H


#include "octomap/OcTree.h"
#include "octomap/OcTreeIterator.hxx"

#include "octomap_sensor_definition.h"

#include "data_2d_grid.h"
#include <iostream>

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
    OctomapWrapper(const double &resolution = 0.05, const OctomapSensorDefinition &sensorProperties = OctomapSensorDefinition());

    bool is2DProjectionEnabled() const;

    void set2DProjection(const bool enable);

    bool loadOctreeFromBT(const std::string &path);

    void updateSensorProperties(const OctomapSensorDefinition &sensorProperties);

    void updateFromPointCloud(octomap::Pointcloud *pc, const octomap::pose6d &origin = octomap::pose6d());

    void updateFromLaserScan(octomap::Pointcloud* pc, const octomap::pose6d &origin = octomap::pose6d());

public:
    maps::Data2DGrid<OccupiedResult>* get2DOccupancyMap();
    octomap::OcTree* get3DOccupancyMap();

private:
    void updateMapContinuity();

    void updateMapFromTree();

    void updateMapOccupancy(const octomap::OcTreeKey &key, const bool &occupancy);

    void updateMapOccupancyRecursiveCheck(const double &xPos, const double &yPos, const unsigned int &depth, const bool &occupancy);

    void updateMapOccupancyRecursiveCheck(const octomap::OcTree::iterator &it, const bool &occupancy);

private:
    //void filterGroundPlane(const octomap::Pointcloud& pc, octomap::Pointcloud& ground, octomap::Pointcloud& nonground);
    void updateOccupiedNode(const octomap::OcTree::iterator &it);
    void updateFreeNode(const octomap::OcTree::iterator &it);

private:
    inline void mapIndex(const octomap::OcTreeKey& key, unsigned int &rows, unsigned int &colums) const {
        rows = key[0] - paddedMinKey[0];
        colums = key[1] - paddedMinKey[1];
    }

private:
    bool enabled2DProjection = true;
    double treeResolution = 0.05;
    unsigned int treeDepth = 0;
    unsigned int maxTreeDepth = 0;

    unsigned int mapScaling = 0;

    octomap::OcTree* m_Tree;
    octomap::OcTreeKey keyBBXMin;
    octomap::OcTreeKey keyBBXMax;
    octomap::OcTreeKey paddedMinKey;
    octomap::OcTreeKey paddedMaxKey;

    maps::Data2DGrid<OccupiedResult>* m_Map;

    OctomapSensorDefinition* m_sensorProperties;

};

} //end of namespace maps
} //end of namespace mace


#endif // OCTOMAP_WRAPPER_H
