#include "data_container_ardupilot.h"
namespace DataArdupilot
{

DataContainer::DataContainer() :
    m_CurrentArduVehicleState(NULL), m_CurrentArduVehicleStatus(NULL), m_CurrentArduGlobalPosition(NULL),
    m_CurrentArduVehicleFuel(NULL), m_CurrentArduVehicleGPS(NULL), m_CurrentArduVehicleText(NULL),
    m_CurrentArduLocalPosition(NULL)
{

}


} // end of namespace DataArdupilot
