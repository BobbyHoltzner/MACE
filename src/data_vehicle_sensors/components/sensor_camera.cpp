#include "sensor_camera.h"

namespace DataVehicleSensors
{

const char Camera_name[] = "sensor_camera";
const MaceCore::TopicComponentStructure Camera_structure = []{
    MaceCore::TopicComponentStructure structure;
    structure.AddTerminal<double>("FOV horizontal angle");
    structure.AddTerminal<double>("FOV vertical angle");
    structure.AddTerminal<double>("focal length");
    structure.AddTerminal<double>("image width");
    structure.AddTerminal<double>("image height");
    structure.AddTerminal<double>("sensor width");
    structure.AddTerminal<double>("sensor height");
    structure.AddTerminal<double>("image rate");

    return structure;
}();

MaceCore::TopicDatagram SensorCamera::GenerateDatagram() const {
    MaceCore::TopicDatagram datagram;

    datagram.AddTerminal<double>("FOV horizontal angle",m_HFOVA);
    datagram.AddTerminal<double>("FOV vertical angle",m_VFOVA);
    datagram.AddTerminal<double>("focal length",m_FocalLength);
    datagram.AddTerminal<double>("image width",m_Image_Width);
    datagram.AddTerminal<double>("image height",m_Image_Height);
    datagram.AddTerminal<double>("sensor width",m_Sensor_Width);
    datagram.AddTerminal<double>("sensor height",m_Sensor_Height);
    datagram.AddTerminal<double>("image rate",m_Image_Rate);

    return datagram;
}

void SensorCamera::CreateFromDatagram(const MaceCore::TopicDatagram &datagram) {
    m_HFOVA = datagram.GetTerminal<double>("FOV horizontal angle");
    m_VFOVA = datagram.GetTerminal<double>("FOV vertical angle");
    m_FocalLength = datagram.GetTerminal<double>("focal length");
    m_Image_Width = datagram.GetTerminal<double>("image width");
    m_Image_Height = datagram.GetTerminal<double>("image height");
    m_Sensor_Width = datagram.GetTerminal<double>("sensor width");
    m_Sensor_Height = datagram.GetTerminal<double>("sensor height");
    m_Image_Rate = datagram.GetTerminal<double>("image rate");
}

void SensorCamera::updateCameraProperties()
{
    m_HFOVA = atan(m_Sensor_Width/(2*m_FocalLength)) * 2;
    m_VFOVA = atan(m_Sensor_Height/(2*m_FocalLength)) * 2;
}


} //end of namespace DataVehicleSensors
