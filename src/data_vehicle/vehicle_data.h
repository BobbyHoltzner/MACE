#ifndef VEHICLEDATA_H
#define VEHICLEDATA_H

#include <string>
#include <iostream>

namespace Data {

enum VehicleProtocol
{
    PROTOCOL_ARDUPILOT,
    PROTOCOL_DJI,
    PROTOCOL_MAVLINK
};


class VehicleData
{
public:
    /*!
     * \brief Destructor
     *  Deletes the VehicleData object if it is set.
     */
    VehicleData();

    /*!
     * \brief Destructor
     *  Deletes the VehicleData object if it is set.
     */
    virtual ~VehicleData();


    /**
     * @brief getVehicleProtocol
     * @return
     */
    virtual VehicleProtocol getProtocolDefinition() const = 0;

    // //////////////////////////////////////////////////////////////
    // ////////////// STATIC METHODS ////////////////////////////////
    // //////////////////////////////////////////////////////////////

    //! Convert a SensorType enum to a String
    //!
    //!  \param SensorType enum value
    //!  \return String value corresponding to the enumeration
    //!
    static std::string VehicleProtocolToString(const VehicleProtocol &protocolType);

    //!
    //! \brief static method to convert a SensorTypes enum to a string
    //! \param unitString String to convert
    //! \return typeEnum SensorType enum corresponding to unitString
    //!
    static VehicleProtocol StringToVehicleProtocolEnum(const std::string &protocolString);

private:

};

} //end of namespace Data
#endif // VEHICLEDATA_H
