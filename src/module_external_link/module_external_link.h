#ifndef MODULE_EXTERNAL_LINK_H
#define MODULE_EXTERNAL_LINK_H

#include "module_external_link_global.h"

#include "mace_core/i_module_topic_events.h"

#include "data_vehicle_ardupilot/mavlink_parser_ardupilot.h"

#include "data_vehicle_generic/components.h"
#include "data_vehicle_MAVLINK/components.h"
#include "data_vehicle_ardupilot/components.h"

#include "mace_core/i_module_command_external_link.h"

class MODULE_EXTERNAL_LINKSHARED_EXPORT ModuleExternalLink : public MaceCore::IModuleCommandExternalLink
{

public:
    ModuleExternalLink();

    //!
    //! \brief This module as been attached as a module
    //! \param ptr pointer to object that attached this instance to itself
    //!
    virtual void AttachedAsModule(MaceCore::IModuleTopicEvents* ptr);

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

private:

    Data::TopicDataObjectCollection<DATA_VEHICLE_ARDUPILOT_TYPES, DATA_VEHICLE_MAVLINK_TYPES, DATA_VEHICLE_GENERIC_TYPES> m_VehicleDataTopic;
};

#endif // MODULE_EXTERNAL_LINK_H
