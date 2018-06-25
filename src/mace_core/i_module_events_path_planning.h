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

    virtual void EventPP_NewDynamicMissionQueue(const ModuleBase* sender, const TargetItem::DynamicMissionQueue &queue) = 0;

    virtual void EventPP_NewPathFound(const void* sender, const std::vector<mace::state_space::StatePtr> &path) = 0;

};

} //End MaceCore Namespace

#endif // I_MODULE_EVENTS_PATH_PLANNING_H
