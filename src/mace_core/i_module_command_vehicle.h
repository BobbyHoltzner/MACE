#ifndef I_VEHICLE_COMMS_H
#define I_VEHICLE_COMMS_H

#include "data_generic_mission_item/mission_item_components.h"

#include "abstract_module_event_listeners.h"
#include "metadata_vehicle.h"

#include "i_module_topic_events.h"
#include "i_module_events_vehicle.h"

namespace MaceCore
{

enum class VehicleCommands
{
    CHANGE_VEHICLE_ARM,
    CHANGE_VEHICLE_MODE,
    CREATE_VEHICLE_OBJECT,
    REMOVE_VEHICLE_OBJECT,
    UPDATE_VEHICLE_OBJECT_LIST,
    FOLLOW_NEW_COMMANDS,
    FINISH_AND_FOLLOW_COMMANDS,
    COMMANDS_APPENDED
};


class MaceCore;

class MACE_CORESHARED_EXPORT IModuleCommandVehicle : public AbstractModule_EventListeners<MetadataVehicle, IModuleEventsVehicle, VehicleCommands>
{
friend class MaceCore;
public:

    static Classes moduleClass;

    IModuleCommandVehicle():
        AbstractModule_EventListeners()
    {
        //These are from MACE Core to modules
        AddCommandLogic<MissionItem::ActionArm>(VehicleCommands::CHANGE_VEHICLE_ARM, [this](const MissionItem::ActionArm &vehicleArm){
            ChangeVehicleArm(vehicleArm);
        });

        AddCommandLogic<MissionItem::ActionChangeMode>(VehicleCommands::CHANGE_VEHICLE_MODE, [this](const MissionItem::ActionChangeMode &vehicleMode){
            ChangeVehicleOperationalMode(vehicleMode);
        });

        AddCommandLogic<int>(VehicleCommands::CREATE_VEHICLE_OBJECT, [this](const int &vehicleID){
            CreateVehicleObject(vehicleID);
        });

        AddCommandLogic<int>(VehicleCommands::REMOVE_VEHICLE_OBJECT, [this](const int &vehicleID){
            RemoveVehicleObject(vehicleID);
        });

        AddCommandLogic<std::list<int>>(VehicleCommands::UPDATE_VEHICLE_OBJECT_LIST, [this](const std::list<int> &vehicleObjectList){
            UpdateVehicleObjectList(vehicleObjectList);
        });

        AddCommandLogic(VehicleCommands::FOLLOW_NEW_COMMANDS, [this](){
            FollowNewCommands();
        });

        AddCommandLogic(VehicleCommands::FINISH_AND_FOLLOW_COMMANDS, [this](){
            FinishAndFollowNewCommands();
        });

        AddCommandLogic(VehicleCommands::COMMANDS_APPENDED, [this](){
            CommandsAppended();
        });

    }

    virtual Classes ModuleClass() const
    {
        return moduleClass;
    }


public:

    virtual void ChangeVehicleArm(const MissionItem::ActionArm &vehicleArm) = 0;

    virtual void ChangeVehicleOperationalMode(const MissionItem::ActionChangeMode &vehicleMode) = 0;

    virtual void CreateVehicleObject(const int &vehicleID) = 0;

    virtual void RemoveVehicleObject(const int &vehicleID) = 0;

    virtual void UpdateVehicleObjectList(const std::list<int> &vehicleObjectList) = 0;

    //!
    //! \brief New commands have been updated that the vehicle is to follow immediatly
    //!
    //! Command Data can be retreived through the MaceData available through getDataObject()
    //!
    virtual void FollowNewCommands() = 0;


    //!
    //! \brief New commands have been issued to vehicle that are to be followed once current command is finished
    //!
    //! Command Data are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void FinishAndFollowNewCommands() = 0;


    //!
    //! \brief New commands have been appended to existing commands
    //!
    //! Command Data are to be retreived through the MaceData available through getDataObject()
    //!
    virtual void CommandsAppended() = 0;

};


} //END MaceCore Namespace

#endif // I_VEHICLE_COMMS_H
