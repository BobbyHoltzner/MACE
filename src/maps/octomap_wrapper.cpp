#include "octomap_wrapper.h"
namespace mace{
namespace maps{

OctomapWrapper::OctomapWrapper(const double &resolution, const OctomapSensorDefinition &sensorProperties):
    treeResolution(resolution),
    m_Tree(nullptr),
    m_Map(nullptr),
    m_sensorProperties(sensorProperties)
{
    m_Tree = new octomap::OcTree(treeResolution);
    m_Map = new maps::Dynamic2DGrid<OccupiedResult>();
}

bool OctomapWrapper::is2DProjectionEnabled() const
{
    return this->enabled2DProjection;
}

void OctomapWrapper::set2DProjection(const bool enable)
{
    this->enabled2DProjection = enable;
    //if(this->enabled2DProjection)
    //for (octomap::OcTree::iterator it = m_Tree->begin(m_maxTreeDepth), end = m_Tree->end(); it != end; ++it)
}

void OctomapWrapper::loadOctreeFromBT(const std::string &path)
{
    m_Tree->clear();
    m_Tree->readBinary(path);
    this->treeDepth = m_Tree->getTreeDepth();
    this->treeResolution = m_Tree->getResolution();
    double minX, minY, minZ;
    double maxX, maxY, maxZ;
    m_Tree->getMetricMin(minX,minY,minZ);
    m_Tree->getMetricMax(maxX,maxY,maxZ);
    m_Map->setGridSize(minX,maxX,minY,maxY,treeResolution,treeResolution);

}



} //end of namespace maps
} //end of namespace mace
