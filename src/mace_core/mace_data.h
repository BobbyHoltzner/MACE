#ifndef I_MACE_DATA_H
#define I_MACE_DATA_H

#include <iostream>
#include <string>
#include <map>
#include <unordered_map>
#include <stdexcept>
#include <functional>
#include <mutex>
#include <list>
#include <vector>

#include "vehicle_data.h"

#include "observation_history.h"

#include "matrix_operations.h"

#include "topic.h"

#include "data_generic_item/data_generic_item_components.h"
#include "data_generic_state_item/state_item_components.h"
#include "data_generic_command_item/command_item_components.h"

#include "data/system_description.h"
#include "data/mission_execution_state.h"


#include "maps/octomap_wrapper.h"
#include "maps/data_2d_grid.h"

#include "octomap/octomap.h"
#include "octomap/OcTree.h"

#include "base/pose/cartesian_position_3D.h"
#include "base/pose/orientation_3D.h"
#include "base/geometry/cell_2DC.h"

namespace MaceCore
{

class MaceCore;

//!
//! \brief Object that is to hold all data that MACE modules may need
//!
//! It it intended that a single MaceData object will be created and passed to each module.
//! Therefore all of this objects methods are thread safe as they may be stimulated from multiple module's threads.
//!
//! The reason for a single data storage object over simply passing data in command methods was done for three reasons:
//!     1) Some modules may require the same data, so a centrailized container is simplier over distrubtiting idential data.
//!     2) Some data may be updated at a high rate, so changing one and calling a notify modules of new data is prefered over sending new data
//!     3) Commands may be marsheled at a different rate than they are generated, so a centralize data bin gives the module full access to data without over-invoking
//!
//! Only MaceCore object will be able of manipulating data inside this object, while any module can read data out of it.
//! If a module must write to this object it must issue an event to MaceCore first.
//!
//! Use of the MaceData object requires implementaiton of various Fuse_* methods.
//! These methods describe how to combine multiple observations and used to expose a continuous-time data-space with descrite-time observations.
//! The exact method in how this interpolation is done (pick closests, linear, more advanced methods) is left to the user of MaceData.
//!
class MACE_CORESHARED_EXPORT MaceData
{
    friend class MaceCore;

    static const uint64_t DEFAULT_MS_RECORD_TO_KEEP = 1000;

private:
    uint64_t m_MSTOKEEP;

public:

    MaceData() :
        m_MSTOKEEP(DEFAULT_MS_RECORD_TO_KEEP), flagBoundaryVerts(false)
    {
        m_OctomapWrapper = new mace::maps::OctomapWrapper();
    }

    MaceData(uint64_t historyToKeepInms) :
        m_MSTOKEEP(historyToKeepInms), flagBoundaryVerts(false)
    {
        m_OctomapWrapper = new mace::maps::OctomapWrapper();
    }

    /////////////////////////////////////////////////////////
    /// DATA FUSION METHODS
    /////////////////////////////////////////////////////////


    //!
    //! \brief Abstract method to interpolate vehicle dynamics
    //!
    //! This method is to be implemented by the instantiator of MaceData
    //! \param time Time to interpolate to
    //! \param v0 Value0
    //! \param t0 Time0
    //! \param v1 Value1
    //! \param t1 Time1
    //! \return Interpolated vehicle dynamics
    //!
    virtual VectorDynamics Fuse_VehicleDynamics(const TIME &time, const VectorDynamics &v0, const TIME &t0, const VectorDynamics &v1, const TIME &t1) const = 0;


    //!
    //! \brief Abstract method to interpolate vehicle life
    //!
    //! This method is to be implemented by the instantiator of MaceData
    //! \param time Time to interpolate to
    //! \param v0 Value0
    //! \param t0 Time0
    //! \param v1 Value1
    //! \param t1 Time1
    //! \return Interpolated vehicle life
    //!
    virtual VehicleLife Fuse_VehicleLife(const TIME &time, const VehicleLife &v0, const TIME &t0, const VehicleLife &v1, const TIME &t1) const = 0;


    /////////////////////////////////////////////////////////
    /// VEHICLE DATA
    /////////////////////////////////////////////////////////

public:
    void GetAvailableVehicles(std::vector<int> &vehicleIDs) const
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        vehicleIDs = m_AvailableVehicles;
    }

    void GetLocalVehicles(std::vector<int> &vehicleIDs) const
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        vehicleIDs = m_LocalVehicles;
    }

    CommandItem::SpatialHome GetVehicleHomePostion(const int &vehicleID) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        CommandItem::SpatialHome vehicleHome = m_VehicleHomeMap.at(vehicleID);
        return vehicleHome;
    }


    mace::pose::GeodeticPosition_3D GetGlobalOrigin() const
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        return m_GlobalOrigin;
    }

    double GetGridSpacing() const
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        double gridSpacing = m_GridSpacing;
        return gridSpacing;
    }


    std::vector<DataState::StateGlobalPosition> GetEnvironmentBoundary() const
    {
        std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
        std::vector<DataState::StateGlobalPosition> boundaryVerts = m_BoundaryVerts;
        return boundaryVerts;
    }

    std::vector<BoundaryItem::BoundaryList> GetVehicleBoundaryList() const
    {
        std::lock_guard<std::mutex> guard(m_VehicleBoundaryMutex);
        std::vector<BoundaryItem::BoundaryList> boundaryList = m_vehicleBoundaryList;
        return boundaryList;
    }

private:

    void AddAvailableVehicle(const int &vehicleID, bool internal)
    {
        std::lock_guard<std::mutex> guard(m_AvailableVehicleMutex);
        m_AvailableVehicles.push_back(vehicleID);
        std::sort( m_AvailableVehicles.begin(), m_AvailableVehicles.end());
        m_AvailableVehicles.erase( unique( m_AvailableVehicles.begin(), m_AvailableVehicles.end() ), m_AvailableVehicles.end() );

        if(internal == true)
        {
            m_LocalVehicles.push_back(vehicleID);
            m_LocalVehicles.erase( unique( m_LocalVehicles.begin(), m_LocalVehicles.end() ), m_LocalVehicles.end() );
        }
    }

    void UpdateVehicleHomePosition(const CommandItem::SpatialHome &vehicleHome)
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        m_VehicleHomeMap[vehicleHome.getOriginatingSystem()] = vehicleHome;
    }

    void UpdateGlobalOrigin(const mace::pose::GeodeticPosition_3D &globalOrigin)
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        double bearingTo = m_GlobalOrigin.polarBearingTo(globalOrigin);
        double distanceTo = m_GlobalOrigin.distanceBetween2D(globalOrigin); //in this case we need the 2D value since we are going to update only 2D position of boundaries
        m_GlobalOrigin = globalOrigin;
        updateBoundariesNewOrigin(distanceTo, bearingTo);
    }

    void UpdateGridSpacing(const double &gridSpacing)
    {
        std::lock_guard<std::mutex> guard(m_VehicleHomeMutex);
        m_GridSpacing = gridSpacing;
    }


    //Gets called when you draw on the GUI
    void UpdateEnvironmentVertices(const std::vector<DataState::StateGlobalPosition> &boundaryVerts) {
        std::lock_guard<std::mutex> guard(m_EnvironmentalBoundaryMutex);
        m_BoundaryVerts = boundaryVerts;
        flagBoundaryVerts = true;
    }

    //This is only called via RTA
    void UpdateVehicleCellMap(const std::map<int, mace::geometry::Cell_2DC> &vehicleMap) {
        std::lock_guard<std::mutex> gaurd(m_VehicleBoundaryMutex);
        m_vehicleCellMap = vehicleMap;
    }

    //This is only called via RTA
    void UpdateVehicleBoundaryList(const std::vector<BoundaryItem::BoundaryList> &boundaryList) {
        std::lock_guard<std::mutex> gaurd(m_VehicleBoundaryMutex);
        m_vehicleBoundaryList = boundaryList;
    }


    void RemoveVehicle(const std::string &rn)
    {
        if(m_PositionDynamicsHistory.find(rn) == m_PositionDynamicsHistory.cend())
            throw std::runtime_error("resource name does not exists");

        m_PositionDynamicsHistory.erase(rn);
        m_AttitudeDynamicsHistory.erase(rn);
        m_VehicleLifeHistory.erase(rn);
    }

    void AddVehicleLife(const std::string &rn, const TIME &time, const VehicleLife &life)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        m_VehicleLifeHistory.at(rn).InsertObservation(time, life);
    }


    //!
    //! \brief set the Target list of a specific vehicle
    //!
    //! The target is a macro-level list, a vehicle should not be flown based on targets.
    //! \param vehicleID Id of Vehicle
    //! \param targetPos List of targeted positions.
    //!
    void setVehicleTarget(const std::string vehicleID, std::vector<Eigen::Vector3d> targetPos)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        m_VehicleTargetPositionList[vehicleID] = targetPos;
    }


    //!
    //! \brief set the dynamics for a specific vehicle.
    //!
    //! Commands are micro-level list of dynamics the vehicle is to physically achieve to reach a target.
    //! \param vehicleID ID of vehicle whose commands are being modified on
    //! \param locationalCommands List of commands to asign to vehicle
    //!
    void setVehicleDynamicsCommands(const std::string vehicleID, std::vector<FullVehicleDynamics> commands)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        m_VehicleCommandDynamicsList[vehicleID] = commands;
    }

public:

    void setTopicDatagram(const std::string &topicName, const int senderID, const TIME &time, const TopicDatagram &value) {

        std::lock_guard<std::mutex> guard(m_TopicMutex);

        if(m_LatestTopic.find(topicName) == m_LatestTopic.cend()) {
            m_LatestTopic.insert({topicName, {}});
        }
        if(m_LatestTopic[topicName].find(senderID) == m_LatestTopic[topicName].cend()) {
            m_LatestTopic[topicName].insert({senderID, TopicDatagram()});
        }
        m_LatestTopic[topicName][senderID].MergeDatagram(value);

        std::vector<std::string> terminalNames = value.ListTerminals();
        std::vector<std::string> nonTerminalNames = value.ListNonTerminals();
        for(size_t i = 0 ; i < terminalNames.size() ; i++) {
            m_LatestTopicComponentUpdateTime[topicName][senderID][terminalNames.at(i)] = time;
        }
        for(size_t i = 0 ; i < nonTerminalNames.size() ; i++) {
            m_LatestTopicComponentUpdateTime[topicName][senderID][nonTerminalNames.at(i)] = time;
        }
    }

    std::unordered_map<std::string, TopicDatagram> getAllLatestTopics(const int &targetID)
    {
        std::lock_guard<std::mutex> guard(m_TopicMutex);
        std::unordered_map<std::string, TopicDatagram> topicMap;
        for(auto it = m_LatestTopic.cbegin() ; it != m_LatestTopic.cend() ; ++it) {
            for(auto local_it = m_LatestTopic[it->first].cbegin(); local_it != m_LatestTopic[it->first].cend(); ++local_it)
            {
                if(local_it->first == targetID)
                {
                    topicMap[it->first] = local_it->second;
                }
            }
        }
        return topicMap;

    }

    TopicDatagram GetCurrentTopicDatagram(const std::string &topicName, const int senderID) const {
        std::lock_guard<std::mutex> guard(m_TopicMutex);
        return m_LatestTopic.at(topicName).at(senderID);
    }


public:

    bool GetPositionDynamics(const std::string rn, const TIME &time, Eigen::Vector3d &pos, Eigen::Vector3d &velocity) const
    {
        UNUSED(pos);
        UNUSED(velocity);
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics vec;
        bool success = GetObservation<VectorDynamics>(m_PositionDynamicsHistory.at(rn), time, vec, [&](const TIME& t, const VectorDynamics& d0, const TIME& t0, const VectorDynamics& d1, const TIME& t1){ return Fuse_VehicleDynamics(t, d0, t0, d1, t1);});

        if(success == false)
            return false;
        return true;
    }

    bool GetAttitudeDynamics(const std::string rn, const TIME &time, Eigen::Vector3d &att, Eigen::Vector3d &att_rates) const
    {
        UNUSED(att);
        UNUSED(att_rates);
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics vec;
        bool success = GetObservation<VectorDynamics>(m_AttitudeDynamicsHistory.at(rn), time, vec, [&](const TIME& t, const VectorDynamics& d0, const TIME& t0, const VectorDynamics& d1, const TIME& t1){ return Fuse_VehicleDynamics(t, d0, t0, d1, t1);});

        if(success == false)
            return false;
        return true;
    }


    //!
    //! \brief get the list of targets that a specific vehicle is to move to
    //!
    //! The target is a macro-level list, of general positions a vehicle is to acheive.
    //! Targets may not express the actual path a vehicle is to take, therefore a vehicle should not be flown based on targets.
    //! Will return empty array if no targets are desired for vehicle
    //! \param vehicleID Id of Vehicle
    //! \return List of targeted positions.
    //!
    std::vector<Eigen::Vector3d> getVehicleTarget(const std::string vehicleID) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        if(m_VehicleTargetPositionList.find(vehicleID) == m_VehicleTargetPositionList.cend())
            return {};

        return m_VehicleTargetPositionList.at(vehicleID);
    }


    //!
    //! \brief get the command dynamics for a specific vehicle.
    //!
    //! Commands are micro-level list of positions/attitude the vehicle is to physically achieve to reach a target.
    //! Will return empty array if no attitudes are commanded for vehicle
    //! \param vehicleID ID of Vehicle
    //! \return list of commanded dynamics
    //!
    std::vector<FullVehicleDynamics> getVehicleDynamicsCommands(const std::string vehicleID) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        if(m_VehicleCommandDynamicsList.find(vehicleID) == m_VehicleCommandDynamicsList.cend())
            return {};

        return m_VehicleCommandDynamicsList.at(vehicleID);
    }


    /////////////////////////////////////////////////////////
    /// PATH PLANNING DATA
    /////////////////////////////////////////////////////////

private:


    //!
    //! \brief Entirely replaces the stored Resource map with given matrix
    //! \param occupancy map to replace with
    //!
    void ResourceMap_ReplaceMatrix(const Eigen::MatrixXd &newResourceMap)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        m_ResourceMap = newResourceMap;
    }


    //!
    //! \brief Replace a discrete set of cells in the resource map
    //!
    //! Thread Safe
    //! May be faster than ResourceMap_ReplaceMatrix if operations are sparse
    //! \param cells Vector of cells to replace in the resourse map
    //!
    void ResourceMap_ReplaceCells(const std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        ReplaceCellsInMatrix(m_ResourceMap, cells);
    }


    //!
    //! \brief Call lambda function to modify components of Resource map
    //!
    //! Thread Safe
    //! May be faster than ResourceMap_ReplaceMatrix if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void ResourceMap_GenericOperation(const std::function<void(Eigen::MatrixXd &)> &func)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        func(m_ResourceMap);
    }





    //!
    //! \brief Entirely replaces the stored occupancy map with given matrix
    //! \param occupancy map to replace with
    //!
    void OccupancyMap_ReplaceMatrix(const Eigen::MatrixXd &newOccupancyMap)
    {
        //m_OccupancyMap = newOccupancyMap;
    }


    //!
    //! \brief Replace a discrete set of cells in the occupancy map
    //!
    //! Thread Safe
    //! May be faster than OccupancyMap_ReplaceMatrix if operations are sparse
    //! \param cells Vector of cells to replace in the resourse map
    //!
    void OccupanyMap_ReplaceCells(const std::vector<MatrixCellData<double>> &cells)
    {
        //ReplaceCellsInMatrix(m_OccupancyMap, cells);
    }


    //!
    //! \brief Call lambda function to modify components of Occupancy map
    //!
    //! Thread Safe
    //! May be faster than OccupancyMap_ReplaceMatrix if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void OccupancyMap_GenericOperation(const std::function<void(Eigen::MatrixXd &)> &func)
    {
        //func(m_OccupancyMap);
    }

    //!
    //! \brief Entirely replaces the stored Probility map with given matrix
    //! \param occupancy map to replace with
    //!
    void ProbabilityMap_ReplaceMatrix(const Eigen::MatrixXd &newProbabilityMap)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        m_ProbabilityMap = newProbabilityMap;
    }


    //!
    //! \brief Replace a discrete set of cells in the probibility map
    //!
    //! Thread Safe
    //! May be faster than ProbabilityMap_ReplaceMatrix if operations are sparse
    //! \param cells Vector of cells to replace in the resourse map
    //!
    void ProbabilityMap_ReplaceCells(const std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        ReplaceCellsInMatrix(m_ProbabilityMap, cells);
    }


    //!
    //! \brief Call lambda function to modify components of Probility map
    //!
    //! Thread Safe
    //! May be faster than ProbabilityMap_ReplaceMatrix if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void ProbabilityMap_GenericOperation(const std::function<void(Eigen::MatrixXd &)> &func)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        func(m_ProbabilityMap);
    }

public:

    //!
    //! \brief Retreive a copy of the Resource map
    //!
    //! Thread safe
    //! \return Copy of Resource map
    //!
    Eigen::MatrixXd ResourceMap_GetCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        return m_ResourceMap;
    }


    //!
    //! \brief Read specific cells from resourse map
    //! \param cells Vector of cells to read
    //!
    void ResourceMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        ReadCellsInMatrix(m_ResourceMap, cells);
    }


    //!
    //! \brief Call lambda to perform a generic const operation on Resource map
    //!
    //! Thread Safe
    //! May be faster than GetResourceMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void ResourceMap_GenericConstOperation(std::function<void(const Eigen::MatrixXd &)> &func) const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ResourceMap);

        func(m_ResourceMap);
    }

    void insertGlobalObservation(octomap::Pointcloud& obj, const mace::pose::Position<mace::pose::CartesianPosition_3D> &position)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
        m_OctomapWrapper->updateFromPointCloud(&obj, position);
    }

    void insertObservation(octomap::Pointcloud& obj, const mace::pose::Position<mace::pose::CartesianPosition_3D> &position, const mace::pose::Orientation_3D &orientation)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMaps);
        m_OctomapWrapper->updateFromPointCloud(&obj, position, orientation);
        //m_OctomapWrapper->updateFromPointCloud(&obj);
        // TODO: Test insert and origin point. We'll have to ensure a (0,0,0) origin works, or we'll have to calculate the sensor origin as it moves
        //          - One option is instead of transforming to the world frame, we can pass in a sensor origin AND frame origin point, and the
        //              overloaded version of insertPointCloud() will transform for us. I'm partial to all sensor data being reported in the world
        //              frame in this case though

        //m_OccupancyMap.insertPointCloud(obj, octomap::point3d(0,0,0));
        //        m_OccupancyMap.insertPointCloudRays(obj, octomap::point3d(0,0,0)); // Less efficient than insertPointCloud() accordoing to docs
    }


    //!
    //! \brief checkForOccupancy rather than copying the entire occupancy map
    //! to the module, the check can be done right from the core
    //!
    bool checkForOccupancy()
    {

    }

    //!
    //! \brief Read specific cells from occupancy map
    //! \param cells Vector of cells to read
    //!
    void OccupancyMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        //ReadCellsInMatrix(m_OccupancyMap, cells);
    }


    //!
    //! \brief Call lambda to perform a generic const operation on occupancy map
    //!
    //! Thread Safe
    //! May be faster than GetOccupancyMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void OccupancyMap_GenericConstOperation(std::function<void(const Eigen::MatrixXd &)> &func) const
    {
        //func(m_OccupancyMap);
    }




    //!
    //! \brief Retreive a copy of the Probibility map
    //!
    //! Thread safe
    //! \return Copy of Probibility map
    //!
    Eigen::MatrixXd ProbibilityMap_GetCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        return m_ProbabilityMap;
    }


    //!
    //! \brief Read specific cells from Probibility map
    //! \param cells Vector of cells to read
    //!
    void ProbibilityMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        ReadCellsInMatrix(m_ProbabilityMap, cells);
    }


    //!
    //! \brief Call lambda to perform a generic const operation on Probibility map
    //!
    //! Thread Safe
    //! May be faster than GetProbibilityMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void ProbibilityMap_GenericConstOperation(std::function<void(const Eigen::MatrixXd &)> &func) const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbabilityMap);

        func(m_ProbabilityMap);
    }





private:


    template <typename T>
    bool GetObservation(const ObservationHistory<TIME, T> &history, const TIME time, T &data, const std::function<T(const TIME&, const T&, const TIME&, const T&, const TIME&)> &reckoning) const
    {
        std::vector<TIME> prevTimes;
        std::vector<T> prevData;
        history.GetClosestObservation(time, BACKWARD_EXCLUSION, 1, prevData, prevTimes);

        std::vector<TIME> nextTimes;
        std::vector<T> nextData;
        history.GetClosestObservation(time, FORWARD_INCLUSION, 1, nextData, nextTimes);

        //if neither vector has data then we cant get anything so return false.
        if(prevTimes.size() == 0 && nextTimes.size() == 0)
            return false;

        //if previous is empty then return what is in next
        if(prevTimes.size() == 0)
        {
            data = nextData.at(0);
            return true;
        }

        //if next is empty then return what is in prev
        if(nextTimes.size() == 0)
        {
            data = prevData.at(0);
            return true;
        }

        //neither is empty so call fusion method and return
        data = reckoning(time, prevData.at(0), prevTimes.at(0), nextData.at(0), nextTimes.at(0));
        return true;
    }

    //std::map<int, std::shared_ptr<VehicleObject>> m_VehicleData;

    std::unordered_map<std::string, std::unordered_map<int, TopicDatagram>> m_LatestTopic;
    std::unordered_map<std::string, std::unordered_map<int, std::unordered_map<std::string, TIME>>> m_LatestTopicComponentUpdateTime;

    mutable std::mutex m_AvailableVehicleMutex;
    std::vector<int> m_AvailableVehicles;
    std::vector<int> m_LocalVehicles;

    mutable std::mutex m_VehicleHomeMutex;
    std::map<int, CommandItem::SpatialHome> m_VehicleHomeMap;
    mace::pose::GeodeticPosition_3D m_GlobalOrigin;
    double m_GridSpacing = -1;

    mutable std::mutex m_VehicleBoundaryMutex;
    std::vector<DataState::StateGlobalPosition> m_BoundaryVerts;
    std::map<int, mace::geometry::Cell_2DC> m_vehicleCellMap;
    std::vector<BoundaryItem::BoundaryList> m_vehicleBoundaryList;
    bool flagBoundaryVerts;

    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_PositionDynamicsHistory;
    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_AttitudeDynamicsHistory;
    std::map<std::string, ObservationHistory<TIME, VehicleLife> > m_VehicleLifeHistory;
    std::map<std::string, std::vector<Eigen::Vector3d> > m_VehicleTargetPositionList;
    std::map<std::string, std::vector<FullVehicleDynamics> > m_VehicleCommandDynamicsList;


    Eigen::MatrixXd m_ResourceMap;
    Eigen::MatrixXd m_ProbabilityMap;


    mutable std::mutex m_VehicleDataMutex;

    mutable std::mutex m_Mutex_ResourceMap;
    mutable std::mutex m_Mutex_ProbabilityMap;
    mutable std::mutex m_TopicMutex;

    /////////////////////////////////////////////////////////
    /// PATH PLANNING DATA
    /////////////////////////////////////////////////////////

private:
    mutable std::mutex m_Mutex_OccupancyMaps;
    mace::maps::OctomapWrapper* m_OctomapWrapper;

public:
    bool loadOccupancyEnvironment(const std::string &filePath);
    octomap::OcTree getOccupancyGrid3D() const;
    mace::maps::Data2DGrid<mace::maps::OccupiedResult> getCompressedOccupancyGrid2D() const;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// VEHICLE MISSION METHODS: The following methods are in support of accessing the mission items stored within MaceData.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*
    The following methods aid a MACE instance in assigning an appropriate missionID to the mission in the core data.
    The data class is responsible for reporting an updated missionKey to the calling agent attempting to update
    the core data structure.
    */
    //variables
private:
    mutable std::mutex MUTEXMissionID;
    //!
    //! \brief mapMissionID
    //! The map structure is broken down as systemID(the system for which the mission was created for or the mission
    //! is referencing and would be applied to), creatorID(the system for which created the actual mission), and lastly
    //! the missionID as the unique identifier associated with the actual mission.
    //!
    std::map<int,std::map<int,int>> mapMissionID;
    //methods
public:
    MissionItem::MissionKey appendAssociatedMissionMap(const MissionItem::MissionList &missionList);
    MissionItem::MissionKey appendAssociatedMissionMap(const int &newSystemID, const MissionItem::MissionList &missionList);
private:
    int getAvailableMissionID(const MissionItem::MissionKey &key);

    /*
    The following aids in handling mission reception to/from the core.
    */
    //variables
private:
    mutable std::mutex MUTEXMissions;
    std::map<MissionItem::MissionKey,MissionItem::MissionList> mapMissions;
    std::map<int,MissionItem::MissionKey> mapCurrentMission;
    //methods
public:
    /*
    The following methods aid in handling the reception of a new mission over the external link. The items handled
    in here will be partial lists and should not migrate into the main mission queue.
    */
    bool updateCurrentMissionItem(const MissionItem::MissionItemCurrent &current);

    /*
    The following methods aid getting the mission list from the mace data class. The following methods aid getting
    the current mission object and keys.
    */
    bool getMissionList(const int &systemID, const MissionItem::MISSIONTYPE &type, const MissionItem::MISSIONSTATE &state, MissionItem::MissionList &missionList) const;
    bool getMissionList(const MissionItem::MissionKey &missionKey, MissionItem::MissionList &missionList) const;
    bool getCurrentMissionKey(const int &systemID, MissionItem::MissionKey &key) const;
    bool getCurrentMission(const int &systemID, MissionItem::MissionList &cpyMission) const;
    bool getCurrentMissionValidity(const int &systemID) const;
    bool getMissionKeyValidity(const MissionItem::MissionKey &key) const;


    /*
    The following methods aid getting the mission list from the mace data class. The following methods aid getting
    the current mission object and keys.
    */
    std::vector<MissionItem::MissionKey> getOnboardMissionKeys(const int &systemID);
    void removeFromMissionMap(const MissionItem::MissionKey &missionKey);
    MissionItem::MissionKey receivedMissionACKKey(const MissionItem::MissionKey &key, const MissionItem::MISSIONSTATE &newState);

    void receivedNewMission(const MissionItem::MissionList &missionList);

    /*
    The following methods update the mission type state of the appropriate mission items.
    */
    void updateMissionExeState(const MissionItem::MissionKey &missionKey, const Data::MissionExecutionState &state);
    bool updateOnboardMission(const MissionItem::MissionKey &missionKey);
    bool checkForCurrentMission(const MissionItem::MissionKey &missionKey);

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// MACE BOUNDARY METHODS
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
      * Methods for working with the boundaries associated with MACE.
      * Boundaries defined in these functions are generic to include
      * operational and resource boundaries. The only accepted data type
      * at the current time is a cartesian boundary. This simplifies
      * math operations when global origin and other updates are required.
      */

public:
    void updateBoundary(const BoundaryItem::BoundaryList &boundary);

    bool getBoundary(BoundaryItem::BoundaryList* operationBoundary, const BoundaryItem::BoundaryKey &key) const;

    void getOperationalBoundary(BoundaryItem::BoundaryList* operationBoundary, const int &vehicleID = 0) const;

    void getResourceBoundary(BoundaryItem::BoundaryList* resourceBoundary, const int &vehicleID);

private:
    void updateBoundariesNewOrigin(const double &distance, const double &bearing);

private:
    mutable std::mutex m_EnvironmentalBoundaryMutex;
    std::map<BoundaryItem::BoundaryMapPair,BoundaryItem::BoundaryList> m_EnvironmentalBoundaryMap;

};

} //END MaceCore Namespace

#endif // I_MACE_DATA_H
