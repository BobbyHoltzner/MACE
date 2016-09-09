#ifndef I_MACE_DATA_H
#define I_MACE_DATA_H

#include <string>
#include <map>
#include <stdexcept>
#include <functional>
#include <mutex>

#include <Eigen/Dense>

#include "vehicle_data.h"

#include "observation_history.h"

#include "matrix_operations.h"

namespace MaceCore
{

class MaceCore;

class MaceData
{
friend class MaceCore;

    static const uint64_t DEFAULT_MS_RECORD_TO_KEEP = 1000;

public:

    MaceData() :
        m_MSTOKEEP(DEFAULT_MS_RECORD_TO_KEEP)
    {

    }

    MaceData(uint64_t historyToKeepInms) :
        m_MSTOKEEP(historyToKeepInms)
    {

    }

    /////////////////////////////////////////////////////////
    /// DATA FUSION METHODS
    /////////////////////////////////////////////////////////


    virtual VectorDynamics FuseDynamics(const TIME &time, const VectorDynamics &v0, const TIME &t0, const VectorDynamics &v1, const TIME &t1) const = 0;


    virtual VehicleLife FuseVehicleLife(const TIME &time, const VehicleLife &v0, const TIME &t0, const VehicleLife &v1, const TIME &t1) const = 0;


    /////////////////////////////////////////////////////////
    /// VEHICLE DATA
    /////////////////////////////////////////////////////////

private:

    void AddVehicle(const std::string &rn)
    {
        if(m_PositionDynamicsHistory.find(rn) != m_PositionDynamicsHistory.cend())
            throw std::runtime_error("resource name already exists");

        m_PositionDynamicsHistory.insert({rn, ObservationHistory<TIME, VectorDynamics>(m_MSTOKEEP)});
        m_AttitudeDynamicsHistory.insert({rn, ObservationHistory<TIME, VectorDynamics>(m_MSTOKEEP)});
        m_VehicleLifeHistory.insert({rn, ObservationHistory<TIME, VehicleLife>(m_MSTOKEEP)});
    }

    void RemoveVehicle(const std::string &rn)
    {
        if(m_PositionDynamicsHistory.find(rn) == m_PositionDynamicsHistory.cend())
            throw std::runtime_error("resource name does not exists");

        m_PositionDynamicsHistory.erase(rn);
        m_AttitudeDynamicsHistory.erase(rn);
        m_VehicleLifeHistory.erase(rn);
    }

    void AddPositionDynamics(const std::string rn, const TIME &time, const Eigen::Vector3d &pos, const Eigen::Vector3d &velocity)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics obj;
        obj.dx0 = pos;
        obj.dx1 = velocity;

        m_PositionDynamicsHistory.at(rn).InsertObservation(time, obj);
    }

    void AddAttitudeDynamics(const std::string rn, const TIME &time, const Eigen::Vector3d &att, const Eigen::Vector3d &att_rates)
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics obj;
        obj.dx0 = att;
        obj.dx1 = att_rates;

        m_AttitudeDynamicsHistory.at(rn).InsertObservation(time, obj);
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


    bool GetPositionDynamics(const std::string rn, const TIME &time, Eigen::Vector3d &pos, Eigen::Vector3d &velocity) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics vec;
        bool success = GetObservation<VectorDynamics>(m_PositionDynamicsHistory.at(rn), time, vec, [&](const TIME& t, const VectorDynamics& d0, const TIME& t0, const VectorDynamics& d1, const TIME& t1){ return FuseDynamics(t, d0, t0, d1, t1);});

        if(success == false)
            return false;

        pos = vec.dx0;
        velocity = vec.dx1;
        return true;
    }

    bool GetAttitudeDynamics(const std::string rn, const TIME &time, Eigen::Vector3d &att, Eigen::Vector3d &att_rates) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        VectorDynamics vec;
        bool success = GetObservation<VectorDynamics>(m_AttitudeDynamicsHistory.at(rn), time, vec, [&](const TIME& t, const VectorDynamics& d0, const TIME& t0, const VectorDynamics& d1, const TIME& t1){ return FuseDynamics(t, d0, t0, d1, t1);});

        if(success == false)
            return false;

        att = vec.dx0;
        att_rates = vec.dx1;
        return true;
    }

    bool GetVehicleLife(const std::string &rn, const TIME &time, VehicleLife &life) const
    {
        std::lock_guard<std::mutex> guard(m_VehicleDataMutex);

        return GetObservation<VehicleLife>(m_VehicleLifeHistory.at(rn), time, life, [this](const TIME& t, const VehicleLife& d0, const TIME& t0, const VehicleLife& d1, const TIME& t1){ return FuseVehicleLife(t, d0, t0, d1, t1);});
    }


    //!
    //! \brief get the list of targets that a specific vehicle is to move to
    //!
    //! The target is a macro-level list, of general positions a vehicle is to acheive.
    //! Targets may not express the actuall path a vehicle is to take, therefore a vehicle should not be flown based on targets.
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
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        m_OccupancyMap = newOccupancyMap;
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
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        ReplaceCellsInMatrix(m_OccupancyMap, cells);
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
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        func(m_OccupancyMap);
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




    //!
    //! \brief Retreive a copy of the occupancy map
    //!
    //! Thread safe
    //! \return Copy of occupancy map
    //!
    Eigen::MatrixXd OccupancyMap_GetCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        return m_OccupancyMap;
    }



    //!
    //! \brief Read specific cells from occupancy map
    //! \param cells Vector of cells to read
    //!
    void OccupancyMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        ReadCellsInMatrix(m_OccupancyMap, cells);
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
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        func(m_OccupancyMap);
    }




    //!
    //! \brief Retreive a copy of the Probibility map
    //!
    //! Thread safe
    //! \return Copy of Probibility map
    //!
    Eigen::MatrixXd ProbibilityMap_GetCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbibilityMap);

        return m_ProbibilityMap;
    }


    //!
    //! \brief Read specific cells from Probibility map
    //! \param cells Vector of cells to read
    //!
    void ProbibilityMap_ReadCells(std::vector<MatrixCellData<double>> &cells)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_ProbibilityMap);

        ReadCellsInMatrix(m_ProbibilityMap, cells);
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
        std::lock_guard<std::mutex> guard(m_Mutex_ProbibilityMap);

        func(m_ProbibilityMap);
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

    uint64_t m_MSTOKEEP;

    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_PositionDynamicsHistory;

    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_AttitudeDynamicsHistory;

    std::map<std::string, ObservationHistory<TIME, VehicleLife> > m_VehicleLifeHistory;


    std::map<std::string, std::vector<Eigen::Vector3d> > m_VehicleTargetPositionList;


    std::map<std::string, std::vector<FullVehicleDynamics> > m_VehicleCommandDynamicsList;


    Eigen::MatrixXd m_ResourceMap;
    Eigen::MatrixXd m_OccupancyMap;
    Eigen::MatrixXd m_ProbabilityMap;


    mutable std::mutex m_VehicleDataMutex;

    mutable std::mutex m_Mutex_ResourceMap;
    mutable std::mutex m_Mutex_OccupancyMap;
    mutable std::mutex m_Mutex_ProbabilityMap;


};

} //END MaceCore Namespace

#endif // I_MACE_DATA_H
