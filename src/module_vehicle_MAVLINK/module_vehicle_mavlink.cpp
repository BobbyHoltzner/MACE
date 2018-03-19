#include <iostream>

#include "module_vehicle_mavlink.h"

#include "mace_core/module_factory.h"

#include <QSerialPort>

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"


////////////////////////////////////////////////////////////////////////////////////////////////////////
///             CONFIGURE
////////////////////////////////////////////////////////////////////////////////////////////////////////

void FinishedMessage(const bool completed, const uint8_t finishCode)
{
    if(completed == false)
    {
        printf("Controller timed out sending message, gave up sending message\n");
    }
    else {
        printf("Controller Received Final ACK with code of %d\n", finishCode);
    }
}

template <typename T>
T* Helper_CreateAndSetUp(ModuleVehicleMAVLINK* obj, Controllers::MessageModuleTransmissionQueue<mace_message_t> *queue, uint8_t chan)
{
    T* newController = new T(obj, queue, chan);
    newController->setLambda_DataReceived([obj](const MaceCore::ModuleCharacteristic &sender, const std::shared_ptr<AbstractCommandItem> &command){obj->ReceivedCommand(sender, command);});
    newController->setLambda_Finished(FinishedMessage);
    return newController;
}

template <typename ...VehicleTopicAdditionalComponents>
ModuleVehicleMAVLINK::ModuleVehicleMAVLINK():
    ModuleVehicleGeneric<VehicleTopicAdditionalComponents..., DataMAVLINK::EmptyMAVLINK>(),
    airborneInstance(false)
{
    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKControllers::CommandLand<mavlink_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKControllers::CommandTakeoff<mavlink_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKControllers::CommandARM<mavlink_message_t>>(this, queue, m_LinkChan));
    m_Controllers.Add(Helper_CreateAndSetUp<MAVLINKControllers::CommandRTL<mavlink_message_t>>(this, queue, m_LinkChan));


    auto controller_SystemMode = new MAVLINKControllers::ControllerSystemMode<mavlink_message_t>(this, queue, m_LinkChan);
    controller_SystemMode->setLambda_Finished(FinishedMessage);
    m_Controllers.Add(controller_SystemMode);
}


