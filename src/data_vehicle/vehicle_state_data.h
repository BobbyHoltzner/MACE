#ifndef VEHICLE_STATE_DATA_H
#define VEHICLE_STATE_DATA_H

#include <memory>

#include "data_vehicle_global.h"
#include "Arducopter/arducopter_collection.h"

namespace Data {

class DATA_VEHICLESHARED_EXPORT VehicleStateData
{

public:
    /*!
     * \brief Default constructor
     *  Allocates dynamic space
     */
    VehicleStateData();

    /*!
     * \brief Destructor
     *  Deletes the VehicleData object if it is set.
     */
    ~VehicleStateData();


    //!
    //! \brief Get a pointer to the vehicle data held by this state.
    //! \return Pointer to data, Null if no data provided.
    //!
    std::shared_ptr<VehicleData> getVehicleData() const;


    //!
    //! \brief set vehicle data held in this object
    //! \param sensorData Data to set to
    //!
    void setVehicleData(const std::shared_ptr<VehicleData> &vehicleData);


private:
    VehicleProtocol m_VehicleProtocol;
    std::shared_ptr<VehicleData> m_VehicleData;
};

} //end of namespace Data
#endif // VEHICLE_STATE_DATA_H
