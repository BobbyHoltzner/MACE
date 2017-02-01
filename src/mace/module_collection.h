#ifndef MODULE_COLLECTION_H
#define MODULE_COLLECTION_H


#include "module_external_link/module_external_link.h"
#include "module_ground_station/module_ground_station.h"

#include "module_path_planning_NASAPhase2/module_path_planning_nasaphase2.h"
#include "module_resource_task_allocation/module_rta.h"

#include "module_vehicle_sensors/module_vehicle_sensors.h"

#include "module_vehicle_ardupilot/module_vehicle_ardupilot.h"

#include "mace_core/module_factory.h"


class ModuleCollection : public MaceCore::ModuleFactory
{
public:

    static MaceCore::ModuleFactory* GenerateFactory()
    {
        MaceCore::ModuleFactory* factory = new MaceCore::ModuleFactory();

        Register<ModuleExternalLink>(factory, "NASAPhase2");
        Register<ModuleGroundStation>(factory, "NASAPhase2");
        Register<ModulePathPlanningNASAPhase2>(factory, "NASAPhase2");
        Register<ModuleRTA>(factory, "NASAPhase2");
        Register<ModuleVehicleSensors>(factory, "NASAPhase2");
        Register<ModuleVehicleArdupilot>(factory, "Ardupilot");

        return factory;
    }

private:

    template <typename T>
    static void Register(MaceCore::ModuleFactory* factory, const std::string &moduleName)
    {
        factory->RegisterFactory(T::moduleClass, moduleName, [](){return std::make_shared<T>();});
    }

    template <typename T>
    static void Register(MaceCore::ModuleFactory* factory)
    {
        factory->RegisterFactory(T::moduleClass, T::moduleName, [](){return std::make_shared<T>();});
    }
};

#endif // MODULE_COLLECTION_H
