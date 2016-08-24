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
    //! \brief New commands have been updated that the vehicle is to follow immediatly
    //!
    virtual void FollowNewCommands() = 0;


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    virtual void FinishAndFollowNewCommands() = 0;


    //!
    //! \brief New commands have been appended to existing commands
    //!
    virtual void CommandsAppended() = 0;

};


} //END MaceCore Namespace

#endif // I_VEHICLE_COMMS_H
