#ifndef I_MACE_DATA_H
#define I_MACE_DATA_H

#include <string>
#include <map>

#include "vehicle_data.h"

#include "observation_history.h"

class MaceData
{

public:



    /////////////////////////////////////////////////////////
    /// VEHICLE DATA STORAGE
    /////////////////////////////////////////////////////////

    virtual void AddPositionDynamics(const std::string rn, const TIME &time, const VECTOR3D &pos, const VECTOR3D &velocity) = 0;

    virtual void AddAttitudeDynamics(const std::string rn, const TIME &time, const VECTOR3D &att, const VECTOR3D &att_rates) = 0;

    virtual void AddVehicleLife(const std::string &rn, const TIME &time, const VehicleLife &life) = 0;

    virtual void GetPositionDynamics(const std::string rn, const TIME &time, VECTOR3D &pos, VECTOR3D &velocity) const = 0;

    virtual void GetAttitudeDynamics(const std::string rn, const TIME &time, VECTOR3D &att, VECTOR3D &att_rates) const = 0;

    virtual VehicleLife GetVehicleLife(const std::string &rn, const TIME &time) = 0;


    /////////////////////////////////////////////////////////
    /// PATH PLANNING STORAGE
    /////////////////////////////////////////////////////////

    virtual void UpdateOccupanyMap(void* occupancyMap) = 0;

    virtual void* GetOccupancyMap() const = 0;


private:

    MaceCore::ObservationHistory<VectorDynamics> m_PositionDynamicsHistory;

    MaceCore::ObservationHistory<VectorDynamics> m_AttitudeDynamicsHistory;


};

#endif // I_MACE_DATA_H
