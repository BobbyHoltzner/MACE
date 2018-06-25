import * as deepcopy from 'deepcopy';
import { getRandomRGB } from '../misc/Colors';
import { Vehicle } from '../Vehicle/Vehicle';

export class VehicleDB {
    vehicles: {[id: string]: Vehicle};
    globalOrigin: PositionType;
    environmentBoundary?: PositionType[];
    messagePreferences?: MessagePreferencesType;

    constructor(){
        this.vehicles = {};
        this.globalOrigin =  {lat: 0, lng: 0, alt: 0};;
        this.messagePreferences = {
            emergency: true,
            alert: true,
            critical: true,
            error: true,
            warning: true,
            notice: true,
            info: true,
            debug: true
        };
        this.environmentBoundary = [];
    }


    parseJSONData = (jsonData: TCPReturnType) => {
        let stateCopy = deepcopy(this.vehicles);
        // Log message:
        // this.logger.info("[MACE Data: " + JSON.stringify(jsonData) + "]");

        if(jsonData.dataType === "ConnectedVehicles") {
            let jsonVehicles = jsonData as ConnectedVehiclesType;

            console.log("Connected vehicles return: " + jsonVehicles.connectedVehicles);

            // Check if vehicle is already in the map. If so, do nothing. If not, add it:
            for(let i = 0; i < jsonVehicles.connectedVehicles.length; i++){
                if(stateCopy[jsonVehicles.connectedVehicles[i].toString()] !== undefined){
                    // console.log("Vehicle found: " + jsonVehicles.connectedVehicles[i]);
                    continue;
                }
                else {
                    console.log("Vehicle NOT found: " + jsonVehicles.connectedVehicles[i]);
                    let newVehicle = new Vehicle(jsonVehicles.connectedVehicles[i]);
                    let rgb = getRandomRGB();

                    newVehicle.highlightColor = 'rgba('+ rgb.r +','+ rgb.g +','+ rgb.b +',1)';
                    newVehicle.opaqueHighlightColor = 'rgba('+ rgb.r +','+ rgb.g +','+ rgb.b +',.2)';
                    stateCopy[jsonVehicles.connectedVehicles[i].toString()] = newVehicle;
                }
            }

            // Check if we need to remove a vehicle from the state. If we find it, continue. Else, delete it:
            let idArrays: string[] = Object.keys(stateCopy);
            for(let i = 0; i < idArrays.length; i++){
                if(jsonVehicles.connectedVehicles.indexOf(parseInt(idArrays[i])) >= 0) {
                    continue;
                }
                else {
                    console.log("Delete vehicle: " + idArrays[i]);
                    delete stateCopy[idArrays[i]];
                }
            }

            this.vehicles = stateCopy;
        }
        else if(jsonData.dataType === 'GlobalOrigin') {
            let jsonOrigin = jsonData as TCPOriginType;
            let origin = {lat: jsonOrigin.lat, lng: jsonOrigin.lng, alt: jsonOrigin.alt};
            this.globalOrigin = origin;

            /* TODO-PAT: Update PolygonHelper class with this data: */
            // let settings = deepcopy(this.environmentSettings);
            // settings.gridSpacing = jsonOrigin.gridSpacing;
            // this.environmentSettings = settings;
        }
        else if(jsonData.dataType === 'EnvironmentBoundary') {
            let jsonBoundary = jsonData as TCPEnvironmentBoundaryType;
            console.log(jsonBoundary);
            if(jsonBoundary.vehicleID === 0) {
                this.environmentBoundary = jsonBoundary.environmentBoundary;
            }
            else {
                stateCopy[jsonBoundary.vehicleID].resourceBoundary = jsonBoundary.environmentBoundary;
                this.vehicles = stateCopy;
            }
        }
        // Vehicle specific data:
        else {
            // Only process if we have the vehicle in the map:
            if(stateCopy[jsonData.vehicleID]) {
                if(jsonData.dataType === "VehiclePosition"){
                    let vehiclePosition = jsonData as TCPPositionType;

                    stateCopy[vehiclePosition.vehicleID].position.lat = vehiclePosition.lat;
                    stateCopy[vehiclePosition.vehicleID].position.lng = vehiclePosition.lng;
                    stateCopy[vehiclePosition.vehicleID].position.alt = vehiclePosition.alt;
                    stateCopy[vehiclePosition.vehicleID].numSats = vehiclePosition.numSats;
                    stateCopy[vehiclePosition.vehicleID].positionFix = vehiclePosition.positionFix;

                    stateCopy[vehiclePosition.vehicleID].updateVehicleMarkerPosition(vehiclePosition);

                    if(stateCopy[vehiclePosition.vehicleID].isNew &&
                        (stateCopy[vehiclePosition.vehicleID].gps.gpsFix !== "NO GPS" || stateCopy[vehiclePosition.vehicleID].gps.gpsFix !== "GPS NO FIX") &&
                        // Object.keys(this.connectedVehicles).length === 1)
                        Object.keys(this.vehicles).length === 1){
                            stateCopy[vehiclePosition.vehicleID].isNew = false;
                            // this.setState({mapCenter: [stateCopy[vehiclePosition.vehicleID].position.lat, stateCopy[vehiclePosition.vehicleID].position.lon], mapZoom: 19});
                        }

                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === "VehicleAttitude"){
                    let vehicleAttitude = jsonData as TCPAttitudeType;

                    stateCopy[vehicleAttitude.vehicleID].attitude.roll = vehicleAttitude.roll;
                    stateCopy[vehicleAttitude.vehicleID].attitude.pitch = vehicleAttitude.pitch;
                    stateCopy[vehicleAttitude.vehicleID].attitude.yaw = vehicleAttitude.yaw;

                    stateCopy[vehicleAttitude.vehicleID].updateMarkerAttitude(vehicleAttitude);

                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === "VehicleAirspeed"){
                    let vehicleAirspeed = jsonData as TCPAirspeedType;

                    stateCopy[vehicleAirspeed.vehicleID].airspeed = vehicleAirspeed.airspeed;
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleMission') {
                    let vehicleMission = jsonData as TCPMissionType;
                    let stateCopy = deepcopy(this.vehicles);
                    stateCopy[vehicleMission.vehicleID].setVehicleMission(vehicleMission);
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleHome') {
                    let vehicleHome = jsonData as (TCPReturnType & MissionItemType);
                    // let stateCopy = deepcopy(this.connectedVehicles);
                    let stateCopy = deepcopy(this.vehicles);
                    let tmpHome = {
                        lat: vehicleHome.lat,
                        lon: vehicleHome.lng,
                        alt: vehicleHome.alt
                    }
                    stateCopy[vehicleHome.vehicleID].updateHomePosition(tmpHome);
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleFuel') {
                    let vehicleFuel = jsonData as TCPFuelType;

                    stateCopy[vehicleFuel.vehicleID].fuel.batteryRemaining = vehicleFuel.batteryRemaining;
                    stateCopy[vehicleFuel.vehicleID].fuel.batteryCurrent = vehicleFuel.batteryCurrent;
                    stateCopy[vehicleFuel.vehicleID].fuel.batteryVoltage = vehicleFuel.batteryVoltage;

                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleMode') {
                    let vehicleMode = jsonData as TCPModeType;
                    stateCopy[vehicleMode.vehicleID].vehicleMode = vehicleMode.vehicleMode;
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleText') {
                    let vehicleText = jsonData as TCPTextType;
                    let showMessage = false;
                    // let title = '';
                    // let level = 'info';
                    if(vehicleText.severity === "EMERGENCY") {
                        // title = 'EMERGENCY -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'error';
                        showMessage = this.messagePreferences.emergency;
                    }
                    if(vehicleText.severity === "ALERT") {
                        // title = 'Alert -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'warning';
                        showMessage = this.messagePreferences.alert;
                    }
                    if(vehicleText.severity === "CRITICAL") {
                        // title = 'CRITICAL -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'error';
                        showMessage = this.messagePreferences.critical;
                    }
                    if(vehicleText.severity === "ERROR") {
                        // title = 'ERROR -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'error';
                        showMessage = this.messagePreferences.error;
                    }
                    if(vehicleText.severity === "WARNING") {
                        // title = 'Warning -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'warning';
                        showMessage = this.messagePreferences.warning;
                    }
                    if(vehicleText.severity === "NOTICE") {
                        // title = 'Notice -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'success';
                        showMessage = this.messagePreferences.notice;
                    }
                    if(vehicleText.severity === "INFO") {
                        // title = 'Info -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'info';
                        showMessage = this.messagePreferences.info;
                    }
                    if(vehicleText.severity === "DEBUG") {
                        // title = 'Debug -- Vehicle ' + vehicleText.vehicleID;
                        // level = 'info';
                        showMessage = this.messagePreferences.debug;
                    }

                    if(showMessage) {
                        // this.showNotification(title, vehicleText.text, level, 'bl', 'Got it');
                        stateCopy[vehicleText.vehicleID].messages.unshift({severity: vehicleText.severity, text: vehicleText.text, timestamp: new Date()});
                        this.vehicles = stateCopy;
                    }
                }
                else if(jsonData.dataType === 'SensorFootprint') {
                    let jsonFootprint = jsonData as TCPSensorFootprintType;
                    stateCopy[jsonFootprint.vehicleID].sensorFootprint = jsonFootprint.sensorFootprint;
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleGPS') {
                    let jsonGPS = jsonData as TCPGPSType;
                    stateCopy[jsonGPS.vehicleID].gps.visibleSats = jsonGPS.visibleSats;
                    stateCopy[jsonGPS.vehicleID].gps.gpsFix = jsonGPS.gpsFix;
                    stateCopy[jsonGPS.vehicleID].gps.hdop = jsonGPS.hdop;
                    stateCopy[jsonGPS.vehicleID].gps.vdop = jsonGPS.vdop;
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'CurrentMissionItem') {
                    let jsonMissionItem = jsonData as TCPCurrentMissionItemType;
                    stateCopy[jsonMissionItem.vehicleID].updateCurrentMissionItem(jsonMissionItem.missionItemIndex, false);
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'MissionItemReached') {
                    let jsonMissionItem = jsonData as TCPMissionItemReachedType;
                    if(jsonMissionItem.itemIndex === stateCopy[jsonMissionItem.vehicleID].vehicleMission.icons.length - 1) {
                        stateCopy[jsonMissionItem.vehicleID].updateCurrentMissionItem(jsonMissionItem.itemIndex, true);
                    }
                }
                else if(jsonData.dataType === 'VehicleHeartbeat') {
                    let jsonHeartbeat = jsonData as TCPHeartbeatType;
                    stateCopy[jsonHeartbeat.vehicleID].general.autopilot = jsonHeartbeat.autopilot;
                    stateCopy[jsonHeartbeat.vehicleID].general.commsProtocol = jsonHeartbeat.commsProtocol;
                    stateCopy[jsonHeartbeat.vehicleID].general.aircraftType = jsonHeartbeat.aircraftType;
                    stateCopy[jsonHeartbeat.vehicleID].general.companion = jsonHeartbeat.companion;
                    stateCopy[jsonHeartbeat.vehicleID].general.lastHeard = new Date();
                    stateCopy[jsonHeartbeat.vehicleID].setAvailableVehicleModes();
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'VehicleArm') {
                    let jsonArm = jsonData as TCPVehicleArmType;
                    stateCopy[jsonArm.vehicleID].isArmed = jsonArm.armed;
                    this.vehicles = stateCopy;
                }
                else if(jsonData.dataType === 'CurrentVehicleTarget') {
                    let jsonVehicleTarget = jsonData as TCPVehicleTargetType;
                    stateCopy[jsonVehicleTarget.vehicleID].currentTarget.active = true;
                    stateCopy[jsonVehicleTarget.vehicleID].currentTarget.distanceToTarget = jsonVehicleTarget.distanceToTarget;
                    stateCopy[jsonVehicleTarget.vehicleID].currentTarget.targetPosition.lat = jsonVehicleTarget.lat;
                    stateCopy[jsonVehicleTarget.vehicleID].currentTarget.targetPosition.lng = jsonVehicleTarget.lng;
                    stateCopy[jsonVehicleTarget.vehicleID].currentTarget.targetPosition.alt = jsonVehicleTarget.alt;
                    this.vehicles = stateCopy;
                }
            }
        }
    }

}