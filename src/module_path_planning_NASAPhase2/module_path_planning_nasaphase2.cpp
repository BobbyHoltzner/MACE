#include "module_path_planning_nasaphase2.h"

#include "mace_core/module_factory.h"

#include <iostream>

ModulePathPlanningNASAPhase2::ModulePathPlanningNASAPhase2() :
    MaceCore::IModuleCommandPathPlanning()
{
}


//!
//! \brief Describes the strucure of the parameters for this module
//! \return Strucure
//!
std::shared_ptr<MaceCore::ModuleParameterStructure> ModulePathPlanningNASAPhase2::ModuleConfigurationStructure() const
{
    MaceCore::ModuleParameterStructure structure;
    return std::make_shared<MaceCore::ModuleParameterStructure>(structure);
}


//!
//! \brief Provides object contains parameters values to configure module with
//! \param params Parameters to configure
//!
void ModulePathPlanningNASAPhase2::ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params)
{

}


void ModulePathPlanningNASAPhase2::NewVehicle(const std::string &ID)
{

}


void ModulePathPlanningNASAPhase2::RemoveVehicle(const std::string &ID)
{

}


void ModulePathPlanningNASAPhase2::UpdatedPositionDynamics(const std::string &vehicleID)
{
    //std::cout<<"Path planning module | new positional information"<<std::endl;
    //MaceCore::TIME time;
    //get current time

    //Eigen::Vector3d pos;
    //Eigen::Vector3d vel;
    //data->GetPositionDynamics(vehicleID, time, pos, vel);

    //This is a sample of how to get data from the map containing vehicle information
    std::string flightMode = "";
    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();

    /*
    std::map<int, std::shared_ptr<VehicleObject>> vehicleDataMap;
    data->GetVehicleMap(vehicleDataMap);

    if(vehicleDataMap.find(1) == vehicleDataMap.cend())
    {
        std::cout<<"The vehicle with that ID is not there."<<std::endl;
    }else{
        vehicleDataMap.at(1)->getVehicleMode(flightMode);
        std::cout<<"The vehicle flight mode is currently: "<<flightMode<<std::endl;
    }
    */

//    //This is a sample of how to get data from the map containing vehicle information
//    std::shared_ptr<const MaceCore::MaceData> data = this->getDataObject();
//    std::map<int, std::shared_ptr<VehicleObject>> vehicleDataMap;
//    data->GetVehicleMap(vehicleDataMap);

//    std::cout << "In update vehicle map" << std::endl;

//    Eigen::Vector3d attitudeVector(10.0,10.0,10.0);
//    if(vehicleDataMap.find(1) == vehicleDataMap.cend())
//    {
//        std::cout << "The vehicle with that ID is not there." << std::endl;
//    }else{
//        vehicleDataMap.at(1)->getVehicleAttitude(attitudeVector);
//        std::cout << "The new vehicle roll attitude is: " << attitudeVector(0) << std::endl;
//    }

}


void ModulePathPlanningNASAPhase2::UpdateAttitudeDynamics(const std::string &vehicleID)
{

}


void ModulePathPlanningNASAPhase2::UpdatedVehicleLife(const std::string &vehicleID)
{

}


//!
//! \brief New targets have been assigned to the given vehicle
//! \param vehicleID ID of vehicle
//!
void ModulePathPlanningNASAPhase2::NewVehicleTarget(const std::string &vehicleID)
{

}


//!
//! \brief For one reason or another a recomputation of all vehicles' paths is requested
//!
void ModulePathPlanningNASAPhase2::RecomputePaths()
{

}
