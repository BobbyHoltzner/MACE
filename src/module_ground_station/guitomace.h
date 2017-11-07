#ifndef GUITOMACE_H
#define GUITOMACE_H

#include "mace_core/i_module_command_ground_station.h"

class GUItoMACE {

public:
    GUItoMACE(const MaceCore::IModuleCommandGroundStation* ptrRef)
    {
        mParent = ptrRef;
    }

public:
    parseTCPData(QJsonData data) {
        if(data.id = "sync") {
            issueCommand(data);
        }
    }

    issueCommand(QJsonData data) {
        // Here is where I am missing how to use the interface. 'this' should refer to the ModuleGroundStation class,
        //      but I can't include that here because it would be a circular include.
        //      - Conceptually, I guess you could implement a callback for every type of listener to notify,
        //          but that doesn't sound like what you were recommending, and it doesn't really clean code up
        //          because you'd have to have that callback on the ModuleGroundStation side anyway, kind of defeating
        //          the purpose
        mParent->NotifyListeners([&](MaceCore::IModuleEventsGroundStation* ptr){
            ptr->Event_ForceVehicleDataSync(this, data.vehicleID);
        });
    }

private:
    const MaceCore::IModuleCommandGroundStation* mParent;

};

#endif // GUITOMACE_H
