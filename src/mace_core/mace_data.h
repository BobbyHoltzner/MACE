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
    //! \brief get the Target list of a specific vehicle
    //!
    //! The target is a macro-level list, a vehicle should not be flown based on targets.
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
    //! Commands are micro-level list of attitudes the vehicle is to physically achieve to reach a target.
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
    //! \brief Entirely replaces the stored occupancy map with given matrix
    //! \param occupancy map to replace with
    //!
    void ReplaceOccupanyMap(const Eigen::MatrixXd &newOccupancyMap)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        m_OccupancyMap = newOccupancyMap;
    }


    //!
    //! \brief Call lambda function to modify components of Occupancy map
    //!
    //! Thread Safe
    //! May be faster than ReplaceOccupanyMap if operations are sparse
    //! \param func Lambda function to modify map
    //!
    void OperateOnOccupanyMap(std::function<void(Eigen::MatrixXd &)> &func)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        func(m_OccupancyMap);
    }

public:

    //!
    //! \brief Retreive a copy of the occupancy map
    //!
    //! Thread safe
    //! \return Copy of occupancy map
    //!
    Eigen::MatrixXd GetOccupancyMapCopy() const
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        return m_OccupancyMap;
    }


    //!
    //! \brief Call lambda to read from occupancy map
    //!
    //! Thread Safe
    //! May be faster than GetOccupancyMapCopy if matrix is large.
    //! \param func Lambda function to read from map
    //!
    void UseOccupancyMap(std::function<void(const Eigen::MatrixXd &)> &func)
    {
        std::lock_guard<std::mutex> guard(m_Mutex_OccupancyMap);

        func(m_OccupancyMap);
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



    Eigen::MatrixXd m_OccupancyMap;


    mutable std::mutex m_VehicleDataMutex;
    mutable std::mutex m_Mutex_OccupancyMap;


};

} //END MaceCore Namespace

#endif // I_MACE_DATA_H
