#ifndef MODULE_GROUND_STATION_H
#define MODULE_GROUND_STATION_H

#include "module_ground_station_global.h"

#include <string>

#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>
#include <QThread>
#include <QJsonObject>
#include <QJsonDocument>

#include "../mace_core/i_module_command_ground_station.h"

#include "data/global_position.h"

using namespace std;

class MODULE_GROUND_STATIONSHARED_EXPORT ModuleGroundStation : public MaceCore::IModuleCommandGroundStation
{

public:
    ModuleGroundStation();

    ~ModuleGroundStation();


    //!
    //! \brief Describes the strucure of the parameters for this module
    //! \return Strucure
    //!
    virtual std::shared_ptr<MaceCore::ModuleParameterStructure> ModuleConfigurationStructure() const;


    //!
    //! \brief Provides object contains parameters values to configure module with
    //! \param params Parameters to configure
    //!
    virtual void ConfigureModule(const std::shared_ptr<MaceCore::ModuleParameterValue> &params);

    //!
    //! \brief Starts the TCP server for the GCS to send requests to
    //! \return
    //!
    virtual bool StartTCPServer();


    //!
    //! \brief Called when a new Vehicle has been introduced into MACE
    //!
    //! \param ID ID of the Vehicle
    //!
    virtual void NewVehicle(const std::string &ID) {}


    //!
    //! \brief Called when a vehicle has been removed from MACE
    //!
    //! \param ID ID of vehicle
    //!
    virtual void RemoveVehicle(const std::string &ID) {}


    //!
    //! \brief Signal indicating a vehicle's position dynamics has been updated
    //!
    //! The vehicle's position can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdatedPositionDynamics(const std::string &vehicleID);


    //!
    //! \brief Signal indicating a a vehicle's attitude dynamics have been updated
    //!
    //! The vehicle's attitude can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdateAttitudeDynamics(const std::string &vehicleID);


    //!
    //! \brief Singal to indicate a vehicle's life has been updated
    //!
    //! The vehicle's life can be retreived from MaceData object in getDataObject()
    //! \param vehicleID ID of vehicle
    //!
    virtual void UpdatedVehicleLife(const std::string &vehicleID);



private:
    void UpdatedVehicleMap(const std::string &vehicleID);

    void getVehiclePosition(const int &vehicleID, QByteArray &vehiclePosition);

    void getVehicleAttitude(const int &vehicleID, QByteArray &vehicleAttitude);

    void parseTCPRequest(QJsonObject jsonObj, QByteArray &returnData);

public slots:
    void on_newConnection();



private:

    QTcpServer *m_TcpServer;
    QThread *m_ListenThread;

    std::map<int, std::shared_ptr<VehicleObject>> m_VehicleMap;
	
};

#endif // MODULE_GROUND_STATION_H




