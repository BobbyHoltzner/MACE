#ifndef I_RTA_EVENTS_H
#define I_RTA_EVENTS_H

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace MaceCore
{

class IModuleEventsRTA
{
public:

    //!
    //! \brief Event fired when a new list of targets are produced for a specific vehicle
    //! \param vehicleID Vechile new targets are to be applied to
    //! \param target List of positional targets
    //!
    virtual void NewVehicleTargets(const std::string &vehicleID, const std::vector<Eigen::Vector3d> &target) = 0;
};

} //End MaceCore Namespace

#endif // I_RTA_EVENTS_H
