#ifndef I_MODULE_EVENTS_PATH_PLANNING_H
#define I_MODULE_EVENTS_PATH_PLANNING_H

#include "i_module_events_general.h"

#include "maps/data_2d_grid.h"
#include "maps/octomap_wrapper.h"

#include "maps/octomap_sensor_definition.h"
#include "maps/octomap_2d_projection_definition.h"

namespace MaceCore
{

class IModuleEventsPathPlanning  : public IModuleEventsGeneral
{


public:

    virtual void EventPP_LoadOccupancyEnvironment(const ModuleBase* sender, const std::string &filePath) = 0;

    virtual void EventPP_LoadOctomapProperties(const ModuleBase* sender, const mace::maps::OctomapSensorDefinition &properties) = 0;

    virtual void EventPP_LoadMappingProjectionProperties(const ModuleBase* sender, const mace::maps::Octomap2DProjectionDefinition &properties) = 0;

    virtual void Event_SetOperationalBoundary(const ModuleBase* sender, const BoundaryItem::BoundaryList &boundary) = 0;

    virtual void EventPP_New2DOccupancyMap(const void* sender, const mace::maps::Data2DGrid<mace::maps::OccupiedResult> &map) = 0;

    virtual void EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path) = 0;

//    //!
//    //! \brief Event fired to indicate what planning horizon is being utilized by the path planning module
//    //! \param horizon ID of the horizon being utilized
//    //!
//    virtual void PlanningHorizon(const std::string &horizon) = 0;


//    virtual void ReplaceVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands) = 0;

//    virtual void ReplaceAfterCurrentVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands) = 0;

//    virtual void AppendVehicleCommands(const std::string &vehicleID, const std::vector<FullVehicleDynamics> &movementCommands) = 0;


//    //!
//    //! \brief Event fired when a new occupancy map to be invoked when PathPlanning module generates a new occupancy map.
//    //! \param occupancyMap New occupancy map
//    //!
//    virtual void NewOccupancyMap(const Eigen::MatrixXd &occupancyMap) = 0;


//    //!
//    //! \brief Event fired when the PathPlanning modules determines that a set of cells should be modified on the occupancy map.
//    //!
//    //! This event may be faster than NewOccupancyMap when the matrix is large and the modifcations are sparse
//    //! \param commands List of cells to modify
//    //!
//    virtual void ReplaceOccupancyMapCells(const std::vector<MatrixCellData<double>> &commands) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_PATH_PLANNING_H
