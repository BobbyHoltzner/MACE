#define MAVLINK_NEED_BYTE_SWAP
#include <iostream>

#include "module_vehicle_mavlink.h"

#include "mace_core/module_factory.h"

#include <QSerialPort>

#include "comms/serial_link.h"
#include "comms/udp_link.h"
#include "comms/protocol_mavlink.h"
#include "data_vehicle_MAVLINK/altitude_reference_frames.h"

#include "data_vehicle_generic/local_position.h"




