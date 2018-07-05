#ifndef I_RTA_H
#define I_RTA_H

#include "abstract_module_event_listeners.h"
#include "metadata_rta.h"
#include "i_module_topic_events.h"
#include "i_module_events_rta.h"

#include "i_module_command_generic_boundaries.h"

namespace MaceCore
{

enum class RTACommands
{
    BASE_MODULE_LISTENER_ENUMS,
    GENERIC_MODULE_BOUNDARY_LISTENER_ENUMS,
    TEST_FUNCTION,
    NEWLY_UPDATED_GRID_SPACING
};

class MACE_CORESHARED_EXPORT IModuleCommandRTA :
        public AbstractModule_EventListeners<Metadata_RTA, IModuleEventsRTA, RTACommands>,
        public IModuleGenericBoundaries
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandRTA():
        AbstractModule_EventListeners()
    {
        IModuleGenericBoundaries::SetUp<Metadata_RTA, IModuleEventsRTA, RTACommands>(this);


        AddCommandLogic<int>(RTACommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableVehicle(vehicleID);
        });
        AddCommandLogic<int>(RTACommands::TEST_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            TestFunction(vehicleID);
        });

        AddCommandLogic<mace::pose::GeodeticPosition_3D>(RTACommands::NEWLY_UPDATED_GLOBAL_ORIGIN, [this](const mace::pose::GeodeticPosition_3D &position, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedGlobalOrigin(position);
        });

        AddCommandLogic<int>(RTACommands::NEWLY_UPDATED_GRID_SPACING, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdatedGridSpacing();
        });
    }

    virtual ModuleClasses ModuleClass() const
    {
        return moduleClass;
    }

public:
    virtual void NewlyAvailableVehicle(const int &vehicleID) = 0;

    virtual void TestFunction(const int &vehicleID) = 0;

    //!
    //! \brief NewlyUpdatedGlobalOrigin
    //!
    virtual void NewlyUpdatedGlobalOrigin(const mace::pose::GeodeticPosition_3D &position) = 0;


    //!
    //! \brief NewlyUpdatedGridSpacing
    //!
    virtual void NewlyUpdatedGridSpacing() = 0;
};

} //End MaceCore Namespace

#endif // I_RTA_H

