#include "mavlink_parser.h"

#include "data_vehicle_generic/local_position.h"
#include "data_vehicle_generic/local_velocity.h"
#include "data_vehicle_generic/global_position.h"
#include "data_vehicle_generic/global_velocity.h"

namespace DataVehicleMAVLINK
{


std::unordered_map<std::string, std::shared_ptr<Data::ITopicComponentDataObject>> MAVLINKParser::Parse(const mavlink_message_t* message) const{

    std::unordered_map<std::string, std::shared_ptr<Data::ITopicComponentDataObject>> components;

    int messageID = (int)message->msgid;

    if(messageID == MAVLINK_MSG_ID_LOCAL_POSITION_NED) {
        mavlink_local_position_ned_t decodedMSG;
        mavlink_msg_local_position_ned_decode(message, &decodedMSG);
        std::shared_ptr<DataVehicleGeneric::LocalPosition> pos_ptr = std::make_shared<DataVehicleGeneric::LocalPosition>();
        std::shared_ptr<DataVehicleGeneric::LocalVelocity> vel_ptr = std::make_shared<DataVehicleGeneric::LocalVelocity>();

        pos_ptr->x = decodedMSG.x;
        pos_ptr->y = decodedMSG.y;
        pos_ptr->z = decodedMSG.z;
        pos_ptr->frame = DataVehicleGeneric::CoordinateFrame::NED;

        vel_ptr->x = decodedMSG.vx;
        vel_ptr->y = decodedMSG.vy;
        vel_ptr->z = decodedMSG.vz;
        vel_ptr->frame = DataVehicleGeneric::CoordinateFrame::NED;

        components.insert({DataVehicleGeneric::LocalPosition::Name(), pos_ptr});
        components.insert({DataVehicleGeneric::LocalVelocity::Name(), vel_ptr});
    }

    if(messageID == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {

        mavlink_global_position_int_t decodedMSG;
        mavlink_msg_global_position_int_decode(message, &decodedMSG);

        std::shared_ptr<DataVehicleGeneric::GlobalPosition> pos_ptr = std::make_shared<DataVehicleGeneric::GlobalPosition>();
        std::shared_ptr<DataVehicleGeneric::GlobalVelocity> vel_ptr = std::make_shared<DataVehicleGeneric::GlobalVelocity>();

        //The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It is designed as scaled integer message since the resolution of float is not sufficient.
        pos_ptr->latitude = decodedMSG.lat / pow(10,7);
        pos_ptr->longitude = decodedMSG.lon / pow(10,7);
        pos_ptr->altitude[(int)DataVehicleMAVLINK::AltitudeReferenceFrames::ASL] = decodedMSG.alt  / 1000.0;
        pos_ptr->altitude[(int)DataVehicleMAVLINK::AltitudeReferenceFrames::AGL] = decodedMSG.relative_alt  / 1000.0;

        //Positive assuming a NED convention
        vel_ptr->x = decodedMSG.vx / 100.0;
        vel_ptr->y = decodedMSG.vy / 100.0;
        vel_ptr->z = decodedMSG.vz / 100.0;
        vel_ptr->heading = decodedMSG.hdg;
        vel_ptr->frame = DataVehicleGeneric::CoordinateFrame::NED;

        components.insert({DataVehicleGeneric::GlobalPosition::Name(), pos_ptr});
        components.insert({DataVehicleGeneric::GlobalVelocity::Name(), vel_ptr});
    }

    return components;
}

}
