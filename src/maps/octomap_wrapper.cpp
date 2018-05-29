#include "octomap_wrapper.h"
namespace mace{
namespace maps{

OctomapWrapper::OctomapWrapper(const double &resolution, const OctomapSensorDefinition &sensorProperties):
    treeResolution(resolution),
    m_Tree(nullptr),
    m_Map(nullptr),
    m_sensorProperties(nullptr)
{
    m_sensorProperties = new OctomapSensorDefinition(sensorProperties);
    m_Tree = new octomap::OcTree(treeResolution);
    m_Tree->enableChangeDetection(true);

    OccupiedResult fillValue = OccupiedResult::NO_DATA;
    m_Map = new maps::Data2DGrid<OccupiedResult>(&fillValue);
}

void OctomapWrapper::updateSensorProperties(const OctomapSensorDefinition &sensorProperties)
{
    m_sensorProperties = new OctomapSensorDefinition(sensorProperties);
    m_Tree->setProbHit(m_sensorProperties->getProbHit());
    m_Tree->setProbMiss(m_sensorProperties->getProbMiss());
    m_Tree->setClampingThresMax(m_sensorProperties->getThreshMax());
    m_Tree->setClampingThresMin(m_sensorProperties->getThreshMin());
}

void OctomapWrapper::updateFromPointCloud(octomap::Pointcloud *pc, const octomap::pose6d &origin)
{
//    octomap::KeySet occupiedKeySet;
//    octomap::KeySet freeKeySet;
    m_Tree->insertPointCloud(*pc,origin.trans(),m_sensorProperties->getMaxRange());
    //The following function is already called when calling the insertPointCloud function, this would be inefficient
    //As it would perform the ray trace operation twice. It may be better at some future date to modify the octomap
    //library to merely deliver the changed values and keys on insertion and/or store as members.
    //m_Tree->computeUpdate(*pc,origin.trans(),freeKeySet,occupiedKeySet,m_sensorProperties->getMaxRange());
    if(m_Tree->numChangesDetected() > 0)
    {
        if(enabled2DProjection)
        {
            updateMapContinuity();
            for (octomap::KeyBoolMap::const_iterator it = m_Tree->changedKeysBegin(), end = m_Tree->changedKeysEnd(); it != end; ++it)
            {
                octomap::OcTreeKey key = it->first;
                bool occupied = m_Tree->isNodeOccupied(m_Tree->search(key,m_Tree->getTreeDepth()));
                updateMapOccupancyRecursiveCheck();
            }
        }
        m_Tree->resetChangeDetection();
    }
}

void OctomapWrapper::updateFromLaserScan(octomap::Pointcloud *pc, const octomap::pose6d &origin)
{
    octomap::ScanGraph scan;
    scan.addNode(pc,origin);
    octomap::ScanGraph::iterator it;
    for (it = scan.begin(); it != scan.end(); it++) {
      m_Tree->insertPointCloud(**it, m_sensorProperties->getMaxRange());
    }
}

bool OctomapWrapper::is2DProjectionEnabled() const
{
    return this->enabled2DProjection;
}

void OctomapWrapper::set2DProjection(const bool enable)
{
    this->enabled2DProjection = enable;
    if(this->enabled2DProjection)
    {
        updateMapContinuity();
        updateMapFromTree();
    }
}

bool OctomapWrapper::loadOctreeFromBT(const std::string &path)
{
    std::string suffix = path.substr(path.length()-3, 3);
    if (suffix== ".bt"){
        if (!m_Tree->readBinary(path)){
            return false;
        }
    } else if (suffix == ".ot"){
        octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(path);
        if (!tree){
            return false;
        }
        if (m_Tree){
            delete m_Tree;
            m_Tree = nullptr;
        }
        m_Tree = dynamic_cast<octomap::OcTree*>(tree);
        if (!m_Tree){
            throw std::runtime_error("Could not read the supplied file.");
            return false;
        }

    } else{
        return false;
    }

    double minX, minY, minZ, maxX, maxY, maxZ;

    m_Tree->getMetricMin(minX, minY, minZ);
    m_Tree->getMetricMax(maxX, maxY, maxZ);

    treeDepth = m_Tree->getTreeDepth();
    maxTreeDepth = treeDepth;
    treeResolution = m_Tree->getResolution();
    keyBBXMin[0] = m_Tree->coordToKey(minX);
    keyBBXMin[1] = m_Tree->coordToKey(minY);
    keyBBXMin[2] = m_Tree->coordToKey(minZ);

    keyBBXMax[0] = m_Tree->coordToKey(maxX);
    keyBBXMax[1] = m_Tree->coordToKey(maxY);
    keyBBXMax[2] = m_Tree->coordToKey(maxZ);
    return true;
}

void OctomapWrapper::updateMapContinuity()
{
    double minX, minY, minZ, maxX, maxY, maxZ;

    m_Tree->getMetricMin(minX, minY, minZ);
    m_Tree->getMetricMax(maxX, maxY, maxZ);

    double halfPaddedX = 0.5*m_sensorProperties->getMinSizeX();
    double halfPaddedY = 0.5*m_sensorProperties->getMinSizeY();
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey = m_Tree->coordToKey(minPt, maxTreeDepth);
    octomap::OcTreeKey maxKey = m_Tree->coordToKey(maxPt, maxTreeDepth);

    bool minKeyCheck = m_Tree->coordToKeyChecked(minPt, maxTreeDepth, paddedMinKey);
    bool maxKeycheck = m_Tree->coordToKeyChecked(maxPt, maxTreeDepth, paddedMaxKey);

    mapScaling = 1 << (treeDepth - maxTreeDepth);
    unsigned int width = (paddedMaxKey[0] - paddedMinKey[0])/mapScaling;
    unsigned int height = (paddedMaxKey[0] - paddedMinKey[0])/mapScaling; //I dont know if this should match the grid size
    double gridRes = m_Tree->getNodeSize(maxTreeDepth);
    m_Map->updateGridSize(minX,maxX,minY,maxY,gridRes,gridRes);

    int mapOriginX = minKey[0] - paddedMinKey[0];
    int mapOriginY = minKey[1] - paddedMinKey[1];

    // might not exactly be min / max of octree:
    octomap::point3d origin = m_Tree->keyToCoord(paddedMinKey, treeDepth);
    //    m_projectCompleteMap = (!m_incrementalUpdate || (std::abs(gridRes-m_gridmap.info.resolution) > 1e-6));
    //    m_gridmap.info.resolution = gridRes;
    std::cout<<"X Position Origin: "<<origin.x() - gridRes*0.5<<std::endl;
    std::cout<<"Y Position Origin: "<<origin.y() - gridRes*0.5<<std::endl;

    if (maxTreeDepth != treeDepth){
        std::cout<<"Was this true"<<std::endl;
        //        m_gridmap.info.origin.position.x -= m_res/2.0;
        //        m_gridmap.info.origin.position.y -= m_res/2.0;
    }
}

void OctomapWrapper::updateMapFromTree()
{
    int counter = 0;

    for (octomap::OcTree::iterator it = m_Tree->begin(maxTreeDepth), end = m_Tree->end(); it != end; ++it)
    {
        if(it.getZ() > 0.5){
            counter++;
            if(m_Tree->isNodeOccupied(*it))
            {
                updateMapOccupancyRecursiveCheck(it,true);
            }
            else
            {
                updateMapOccupancyRecursiveCheck(it,false);
            }
        }
    }
}

void OctomapWrapper::updateMapOccupancy(const octomap::OcTreeKey &key, const bool &occupancy)
{
    octomap::point3d point = m_Tree->keyToCoord(key,maxTreeDepth);
    OccupiedResult* ptr = m_Map->getCellByPos(point.x,point.y);
    if(occupancy)
        *ptr = OccupiedResult::OCCUPIED;
    else
        *ptr = OccupiedResult::NOT_OCCUPIED;
}

void OctomapWrapper::updateMapOccupancyRecursiveCheck(const octomap::OcTree::iterator &it, const bool &occupancy)
{
    if(it.getDepth() == maxTreeDepth)
    {
        OccupiedResult* newPtr = m_Map->getCellByPos(it.getX(),it.getY());
        if(occupancy && ((*newPtr == OccupiedResult::NO_DATA) || (*newPtr == OccupiedResult::NOT_OCCUPIED)))
        {
            *newPtr = OccupiedResult::OCCUPIED;
        }
        else
        {
            if((*newPtr == OccupiedResult::NO_DATA) || (*newPtr == OccupiedResult::NOT_OCCUPIED))  //if we had no data before or was already unoccupied we can go ahead and mark it as occupied
                *newPtr = OccupiedResult::NOT_OCCUPIED;
            else
            {
                //this means this cell was previously occupied and now we are saying it is unoccuppied
                //we cannot necessarily just update the cell, we need to create a bbx and check all the z
                //conditions to see if it holds true

                //first we need to define a bbx
                double minX, minY, minZ, maxX, maxY, maxZ;

                m_Tree->getMetricMin(minX, minY, minZ);
                m_Tree->getMetricMax(maxX, maxY, maxZ);
                double width = it.getSize()/2.0;
                octomap::point3d minPt(it.getX() - width, it.getY() - width, minZ);
                octomap::point3d maxPt(it.getX() + width, it.getY() + width, maxZ);
                for(octomap::OcTree::leaf_bbx_iterator it = m_Tree->begin_leafs_bbx(minPt,maxPt,maxTreeDepth), end=m_Tree->end_leafs_bbx();
                    it!=end;++it)
                {
                    if(m_Tree->isNodeOccupied(*it)) //if another node is occupied in the tree somewhere different in height, we cannot mark the point in the map as unoccupied
                        return;
                }
                //if we have reached here, there are no nodes in the bbx that are occupied and therefore the node can be marked as free
                *newPtr = OccupiedResult::NOT_OCCUPIED;
            }

        }
    }else
    {

    }
}

maps::Data2DGrid<OctomapWrapper::OccupiedResult>* OctomapWrapper::get2DOccupancyMap()
{
    return this->m_Map;
}

octomap::OcTree* OctomapWrapper::get3DOccupancyMap()
{
    return this->m_Tree;
}

void OctomapWrapper::updateFreeNode(const octomap::OcTree::iterator &it)
{
    updateMapOccupancyRecursiveCheck(it,false);
}

void OctomapWrapper::updateOccupiedNode(const octomap::OcTree::iterator &it)
{
    updateMapOccupancyRecursiveCheck(it,true);

}

//void OctomapWrapper::filterGroundPlane(const octomap::Pointcloud& pc, octomap::Pointcloud& ground, octomap::Pointcloud& nonground) const
//{
//  ground.header = pc.header;
//  nonground.header = pc.header;

//  if (pc.size() < 50){
//    ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
//    nonground = pc;
//  } else {
//    // plane detection for ground plane removal:
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

//    // Create the segmentation object and set up:
//    pcl::SACSegmentation<PCLPoint> seg;
//    seg.setOptimizeCoefficients (true);
//    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
//    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(200);
//    seg.setDistanceThreshold (m_groundFilterDistance);
//    seg.setAxis(Eigen::Vector3f(0,0,1));
//    seg.setEpsAngle(m_groundFilterAngle);


//    PCLPointCloud cloud_filtered(pc);
//    // Create the filtering object
//    pcl::ExtractIndices<PCLPoint> extract;
//    bool groundPlaneFound = false;

//    while(cloud_filtered.size() > 10 && !groundPlaneFound){
//      seg.setInputCloud(cloud_filtered.makeShared());
//      seg.segment (*inliers, *coefficients);
//      if (inliers->indices.size () == 0){
//        break;
//      }

//      extract.setInputCloud(cloud_filtered.makeShared());
//      extract.setIndices(inliers);

//      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
//        extract.setNegative (false);
//        extract.filter (ground);

//        // remove ground points from full pointcloud:
//        // workaround for PCL bug:
//        if(inliers->indices.size() != cloud_filtered.size()){
//          extract.setNegative(true);
//          PCLPointCloud cloud_out;
//          extract.filter(cloud_out);
//          nonground += cloud_out;
//          cloud_filtered = cloud_out;
//        }

//        groundPlaneFound = true;
//      } else{
//        pcl::PointCloud<PCLPoint> cloud_out;
//        extract.setNegative (false);
//        extract.filter(cloud_out);
//        nonground +=cloud_out;
//        // debug
//        //            pcl::PCDWriter writer;
//        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

//        // remove current plane from scan for next iteration:
//        // workaround for PCL bug:
//        if(inliers->indices.size() != cloud_filtered.size()){
//          extract.setNegative(true);
//          cloud_out.points.clear();
//          extract.filter(cloud_out);
//          cloud_filtered = cloud_out;
//        } else{
//          cloud_filtered.points.clear();
//        }
//      }

//    }
//    // TODO: also do this if overall starting pointcloud too small?
//    if (!groundPlaneFound){ // no plane found or remaining points too small

//      // do a rough fitlering on height to prevent spurious obstacles
//      pcl::PassThrough<PCLPoint> second_pass;
//      second_pass.setFilterFieldName("z");
//      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
//      second_pass.setInputCloud(pc.makeShared());
//      second_pass.filter(ground);

//      second_pass.setFilterLimitsNegative (true);
//      second_pass.filter(nonground);
//    }

//  }
//}

} //end of namespace maps
} //end of namespace mace