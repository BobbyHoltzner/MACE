#ifndef MODULE_COLLECTION_H
#define MODULE_COLLECTION_H

#include "module_path_planning_NASAPhase2/module_path_planning_nasaphase2.h"


#include "module_RTA_NASAPhase2/module_rta_nasaphase2.h"


#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"


#include "module_ground_station/module_ground_station.h"

#include "mace_core/module_factory.h"


class ModuleCollection : public MaceCore::ModuleFactory
{
public:

    static MaceCore::ModuleFactory* GenerateFactory()
    {
        MaceCore::ModuleFactory* factory = new MaceCore::ModuleFactory();

        Register<ModulePathPlanningNASAPhase2>(factory, "NASAPhase2");
        Register<ModuleRTANASAPhase2>(factory, "NASAPhase2");
        Register<ModuleVehicleMAVLINK>(factory, "MAVLINK");
        Register<ModuleGroundStation>(factory, "NASAPhase2");

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

class ModuleRTANASAPhase2;

#endif // MODULE_COLLECTION_H
