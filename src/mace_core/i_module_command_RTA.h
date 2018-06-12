#ifndef I_RTA_H
#define I_RTA_H

#include "abstract_module_event_listeners.h"
#include "metadata_rta.h"
#include "i_module_topic_events.h"
#include "i_module_events_rta.h"

namespace MaceCore
{

enum class RTACommands
{
    BASE_MODULE_LISTENER_ENUMS,
    TEST_FUNCTION,
    NEWLY_UPDATED_OPERATIONAL_FENCE,
    NEWLY_UPDATED_RESOURCE_FENCE,
    NEWLY_UPDATED_GRID_SPACING
};

class MACE_CORESHARED_EXPORT IModuleCommandRTA : public AbstractModule_EventListeners<Metadata_RTA, IModuleEventsRTA, RTACommands>
{
    friend class MaceCore;
public:

    static ModuleClasses moduleClass;

    IModuleCommandRTA():
        AbstractModule_EventListeners()
    {
        AddCommandLogic<int>(RTACommands::NEWLY_AVAILABLE_VEHICLE, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyAvailableVehicle(vehicleID);
        });
        AddCommandLogic<int>(RTACommands::TEST_FUNCTION, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            TestFunction(vehicleID);
        });

        AddCommandLogic<int>(RTACommands::NEWLY_UPDATED_GLOBAL_ORIGIN, [this](const int &vehicleID, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            UNUSED(vehicleID);
            NewlyUpdatedGlobalOrigin();
        });

        AddCommandLogic<BoundaryItem::BoundaryList>(RTACommands::NEWLY_UPDATED_OPERATIONAL_FENCE, [this](const BoundaryItem::BoundaryList &boundary, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedOperationalFence(boundary);
        });

        AddCommandLogic<BoundaryItem::BoundaryList>(RTACommands::NEWLY_UPDATED_RESOURCE_FENCE, [this](const BoundaryItem::BoundaryList &boundary, const OptionalParameter<ModuleCharacteristic> &sender){
            UNUSED(sender);
            NewlyUpdatedResourceFence(boundary);
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
    virtual void NewlyUpdatedGlobalOrigin() = 0;

    //!
    //! \brief NewlyUpdatedBoundaryVertices
    //!
    virtual void NewlyUpdatedOperationalFence(const BoundaryItem::BoundaryList &boundary) = 0;

    //!
    //! \brief NewlyUpdatedResourceFence
    //!
    virtual void NewlyUpdatedResourceFence(const BoundaryItem::BoundaryList &boundary) = 0;

    //!
    //! \brief NewlyUpdatedGridSpacing
    //!
    virtual void NewlyUpdatedGridSpacing() = 0;
};

} //End MaceCore Namespace

#endif // I_RTA_H

