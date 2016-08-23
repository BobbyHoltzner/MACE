#ifndef I_MACE_DATA_H
#define I_MACE_DATA_H

#include <string>
#include <map>
#include <stdexcept>

#include "vehicle_data.h"

#include "observation_history.h"

namespace MaceCore
{



class MaceData
{

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
    /// VEHICLE DATA STORAGE
    /////////////////////////////////////////////////////////

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

    void AddPositionDynamics(const std::string rn, const TIME &time, const VECTOR3D &pos, const VECTOR3D &velocity)
    {
        VectorDynamics obj;
        obj.dx0 = pos;
        obj.dx1 = velocity;

        m_PositionDynamicsHistory.at(rn).InsertObservation(time, obj);
    }

    void AddAttitudeDynamics(const std::string rn, const TIME &time, const VECTOR3D &att, const VECTOR3D &att_rates)
    {
        VectorDynamics obj;
        obj.dx0 = att;
        obj.dx1 = att_rates;

        m_AttitudeDynamicsHistory.at(rn).InsertObservation(time, obj);
    }

    void AddVehicleLife(const std::string &rn, const TIME &time, const VehicleLife &life)
    {
        m_VehicleLifeHistory.at(rn).InsertObservation(time, life);
    }


    bool GetPositionDynamics(const std::string rn, const TIME &time, VECTOR3D &pos, VECTOR3D &velocity) const
    {
        std::vector<TIME> prevTimes;
        std::vector<VectorDynamics> prevDyn;
        m_PositionDynamicsHistory.at(rn).GetClosestObservation(time, BACKWARD_EXCLUSION, 1, prevDyn, prevTimes);

        std::vector<TIME> nextTimes;
        std::vector<VectorDynamics> nextDyn;
        m_PositionDynamicsHistory.at(rn).GetClosestObservation(time, FORWARD_INCLUSION, 1, nextDyn, nextTimes);

        //if neither vector has data then we cant get anything so return false.
        if(prevTimes.size() == 0 && nextTimes.size() == 0)
            return false;

        //if previous is empty then return what is in next
        if(prevTimes.size() == 0)
        {
            pos = nextDyn.at(0).dx0;
            velocity = nextDyn.at(0).dx1;
            return true;
        }

        //if next is empty then return what is in prev
        if(nextTimes.size() == 0)
        {
            pos = prevDyn.at(0).dx0;
            velocity = prevDyn.at(0).dx1;
            return true;
        }

        //neither is empty so call fusion method and return
        VectorDynamics vec = FuseDynamics(time, prevDyn.at(0), prevTimes.at(0), nextDyn.at(0), nextTimes.at(0));
        pos = vec.dx0;
        velocity = vec.dx1;
        return true;
    }

    bool GetAttitudeDynamics(const std::string rn, const TIME &time, VECTOR3D &att, VECTOR3D &att_rates) const
    {
        std::vector<TIME> prevTimes;
        std::vector<VectorDynamics> prevDyn;
        m_AttitudeDynamicsHistory.at(rn).GetClosestObservation(time, BACKWARD_EXCLUSION, 1, prevDyn, prevTimes);

        std::vector<TIME> nextTimes;
        std::vector<VectorDynamics> nextDyn;
        m_AttitudeDynamicsHistory.at(rn).GetClosestObservation(time, FORWARD_INCLUSION, 1, nextDyn, nextTimes);

        //if neither vector has data then we cant get anything so return false.
        if(prevTimes.size() == 0 && nextTimes.size() == 0)
            return false;

        //if previous is empty then return what is in next
        if(prevTimes.size() == 0)
        {
            att = nextDyn.at(0).dx0;
            att_rates = nextDyn.at(0).dx1;
            return true;
        }

        //if next is empty then return what is in prev
        if(nextTimes.size() == 0)
        {
            att = prevDyn.at(0).dx0;
            att_rates = prevDyn.at(0).dx1;
            return true;
        }

        //neither is empty so call fusion method and return
        VectorDynamics vec = FuseDynamics(time, prevDyn.at(0), prevTimes.at(0), nextDyn.at(0), nextTimes.at(0));
        att = vec.dx0;
        att_rates = vec.dx1;
        return true;
    }

    bool GetVehicleLife(const std::string &rn, const TIME &time, VehicleLife &life)
    {
        std::vector<TIME> prevTimes;
        std::vector<VehicleLife> prevLife;
        m_VehicleLifeHistory.at(rn).GetClosestObservation(time, BACKWARD_EXCLUSION, 1, prevLife, prevTimes);

        std::vector<TIME> nextTimes;
        std::vector<VehicleLife> nextLife;
        m_VehicleLifeHistory.at(rn).GetClosestObservation(time, FORWARD_INCLUSION, 1, nextLife, nextTimes);

        //if neither vector has data then we cant get anything so return false.
        if(prevTimes.size() == 0 && nextTimes.size() == 0)
            return false;

        //if previous is empty then return what is in next
        if(prevTimes.size() == 0)
        {
            life = nextLife.at(0);
            return true;
        }

        //if next is empty then return what is in prev
        if(nextTimes.size() == 0)
        {
            life = prevLife.at(0);
            return true;
        }

        //neither is empty so call fusion method and return
        life = FuseVehicleLife(time, prevLife.at(0), prevTimes.at(0), nextLife.at(0), nextTimes.at(0));
        return true;
    }


    /////////////////////////////////////////////////////////
    /// PATH PLANNING STORAGE
    /////////////////////////////////////////////////////////

    virtual void UpdateOccupanyMap(void* occupancyMap) = 0;

    virtual void* GetOccupancyMap() const = 0;


private:

    uint64_t m_MSTOKEEP;

    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_PositionDynamicsHistory;

    std::map<std::string, ObservationHistory<TIME, VectorDynamics> > m_AttitudeDynamicsHistory;

    std::map<std::string, ObservationHistory<TIME, VehicleLife> > m_VehicleLifeHistory;


};

} //END MaceCore Namespace

#endif // I_MACE_DATA_H
