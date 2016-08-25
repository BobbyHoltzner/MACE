#include <iostream>
#include <thread>

#include "mace_core/mace_core.h"

#include "module_RTA_NASAPhase2/module_rta_nasaphase2.h"
#include "module_vehicle_MAVLINK/module_vehicle_mavlink.h"
#include "module_path_planning_NASAPhase2/module_path_planning_nasaphase2.h"


#include "data_interpolation.h"


int main(int argc, char *argv[])
{

    //Initialize core and configure data object
    MaceCore::MaceCore core;
    std::shared_ptr<MaceCore::MaceData> data = std::make_shared<DataInterpolation>();
    core.AddDataFusion(data);


    // Add RTA Module
    MaceCore::Metadata_RTA RTAMetaData;
    std::shared_ptr<ModuleRTANASAPhase2> rtaModule = std::make_shared<ModuleRTANASAPhase2>(RTAMetaData);
    rtaModule->setDataObject(data);
    core.AddRTAModule(rtaModule);


    //Add Path Planning Module
    MaceCore::MetadataPathPlanning pathPlanningData;
    std::shared_ptr<ModulePathPlanningNASAPhase2> pathPlanningModule = std::make_shared<ModulePathPlanningNASAPhase2>(pathPlanningData);
    pathPlanningModule->setDataObject(data);
    core.AddPathPlanningModule(pathPlanningModule);


    //Add Vehicle(s)
    MaceCore::MetadataVehicle vehicle1MetaData;
    std::shared_ptr<ModuleVehicleMAVLINK> vehicle1Module = std::make_shared<ModuleVehicleMAVLINK>(vehicle1MetaData);
    vehicle1Module->setDataObject(data);
    core.AddVehicle("Vehicle1", vehicle1Module);


    //Start Threads
    std::thread rta_thread ([&](){rtaModule->start();});
    std::thread pathPlanning_thread ([&](){pathPlanningModule->start();});
    std::thread vehicle1_thread ([&](){vehicle1Module->start();});


    //Wait for thread to finish
    rta_thread.join();
    pathPlanning_thread.join();
    vehicle1_thread.join();


    return 0;
}
