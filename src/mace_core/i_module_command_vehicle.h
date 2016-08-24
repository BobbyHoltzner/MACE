#ifndef I_VEHICLE_COMMS_H
#define I_VEHICLE_COMMS_H

#include "module_base.h"
#include "metadata_vehicle.h"

#include "i_module_events_vehicle.h"

#include <Eigen/Dense>


namespace MaceCore
{

class IModuleCommandVehicle : public ModuleBase<MetadataVehicle, IModuleEventsVehicle>
{
public:

    IModuleCommandVehicle(MetadataVehicle metadata):
        ModuleBase(metadata)
    {

    }

    virtual std::string ModuleName() const
    {
        return "Vehicle";
    }


public:

    //!
    //! \brief Issue a new target position for the vehicle
    //! \param position Position for the vehicle to achieve
    //!
    virtual void IssueTarget(const Eigen::Vector3d &position) = 0;

};


} //END MaceCore Namespace

#endif // I_VEHICLE_COMMS_H
