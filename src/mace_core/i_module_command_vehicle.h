#ifndef I_VEHICLE_COMMS_H
#define I_VEHICLE_COMMS_H

#include "abstract_module_event_listeners.h"
#include "metadata_vehicle.h"

#include "i_module_events_vehicle.h"

#include <Eigen/Dense>


namespace MaceCore
{

class IModuleCommandVehicle : public AbstractModule_EventListeners<MetadataVehicle, IModuleEventsVehicle>
{
public:

    static Classes moduleClass;

    IModuleCommandVehicle():
        AbstractModule_EventListeners()
    {

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


public:


    //!
    //! \brief New commands have been updated that the vehicle is to follow immediatly
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void FollowNewCommands() = 0;


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void FinishAndFollowNewCommands() = 0;


    //!
    //! \brief New commands have been appended to existing commands
    //!
    //! Commands are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void CommandsAppended() = 0;

};


} //END MaceCore Namespace

#endif // I_VEHICLE_COMMS_H
