#ifndef MODULE_PATH_PLANNING_NASAPHASE2_H
#define MODULE_PATH_PLANNING_NASAPHASE2_H

#include "module_path_planning_nasaphase2_global.h"

#include "mace_core/i_module_command_path_planning.h"


class MODULE_PATH_PLANNING_NASAPHASE2SHARED_EXPORT ModulePathPlanningNASAPhase2 : public MaceCore::IModuleCommandPathPlanning
{

public:
    ModulePathPlanningNASAPhase2();

public:

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr)
    {

    }


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    virtual void NewTopic(const std::string &topicName, int senderID, std::vector<std::string> &componentsUpdated);

public:


    virtual void NewVehicle(const std::string &ID);

    virtual void RemoveVehicle(const std::string &ID);

    virtual void UpdatedPositionDynamics(const std::string &vehicleID);

    virtual void UpdateAttitudeDynamics(const std::string &vehicleID);

    virtual void UpdatedVehicleLife(const std::string &vehicleID);


    //!
    //! \brief New targets have been assigned to the given vehicle
    //! \param vehicleID ID of vehicle
    //!
    virtual void NewVehicleTarget(const std::string &vehicleID);


    //!
    //! \brief For one reason or another a recomputation of all vehicles' paths is requested
    //!
    virtual void RecomputePaths();
};

#endif // MODULE_PATH_PLANNING_NASAPHASE2_H
