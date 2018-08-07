#include "sensor_circular_camera.h"

namespace DataVehicleSensors
{

const char Circular_Camera_name[] = "sensor_circular_camera";
const MaceCore::TopicComponentStructure Circular_Camera_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("viewHalfAngle");

    return structure;
}();

MaceCore::TopicDatagram SensorCircularCamera::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<double>("viewHalfAngle", viewHalfAngle);

    return datagram;
}

void SensorCircularCamera::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    viewHalfAngle = datagram.GetTerminal<double>("viewHalfAngle");
}


} //end of namespace DataVehicleSensors
