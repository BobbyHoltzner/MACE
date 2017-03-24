import { backgroundColors, opaqueBackgroundColors } from './util/Colors';

export class Vehicle{

    vehicleId: number;
    isSelected: boolean;
    position: PositionType;
    attitude: AttitudeType;
    fuel: FuelType;
    numSats: number;
    positionFix: number;
    vehicleMode: VehicleModeType;
    positionInterval: number;
    attitudeInterval: number;
    vehicleMarker: MarkerType;
    vehicleMission: MissionLayerType;
    homePosition: MarkerType;
    vehicleType: VehicleTypeType;

    constructor(vehicleId: number, position?: PositionType, attitude?: AttitudeType){
        this.vehicleId = vehicleId;
        this.isSelected = false;
        this.numSats = 0;
        this.positionFix = 0;
        this.vehicleMode = 'UNKNOWN';
        this.vehicleType = 'Quad';
        this.fuel = {batteryCurrent: 0, batteryRemaining: 0, batteryVoltage: 0};
        if(position){
            this.position = position;
        }
        else {
            // this.position = {lat: 0, lon: 0, alt: 0};
            this.position = {lat: -35.363272, lon: 149.165249, alt: 0};
        }
        if(attitude){
            this.attitude = attitude;
        }
        else {
            this.attitude = {roll: 0, pitch: 0, yaw: 0};
        }

        let vehicleIconHTML = '<div style="background-color: ' + opaqueBackgroundColors[this.vehicleId] + '; color: white; width: 41px; text-align: center;">' + this.vehicleId + '</div><img src="./images/drone-icon.png" alt="Drone icon" style="width:41px; height:41px; -webkit-transform: rotate(' + this.attitude.yaw + 'deg); -moz-transform: rotate(' + this.attitude.yaw + 'deg); -o-transform: rotate(' + this.attitude.yaw + 'deg); -ms-transform: rotate(' + this.attitude.yaw + 'deg); transform: rotate(' + this.attitude.yaw + 'deg);">';

        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(this.position.lat, this.position.lon),
            altitude: this.position.alt,
            icon: new L.DivIcon({
                html: vehicleIconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
            })
        };

        // Set blank mission
        this.vehicleMission = {descriptions: [], latLons: [], itemTypes: [], icons: []};

        // Set default homePosition
        let homeIcon = new L.Icon({
            iconUrl: './images/Home-icon.png',
            iconSize: [25, 41], // size of the icon
            iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
            popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
        });
        this.homePosition = {
            vehicleId: this.vehicleId,
            // latLon: new L.LatLng(this.position.lat, this.position.lon),
            latLon: new L.LatLng(0, 0),
            altitude: this.position.alt,
            icon: homeIcon
        }
    }

    setPosition(position: PositionType) {
        this.position = position;
    }

    setAttitude(attitude: AttitudeType) {
        this.attitude = attitude;
    }

    setVehicleMode(vehicleMode: VehicleModeType) {
        this.vehicleMode = vehicleMode
    }

    setNumSats(numSats: number) {
        this.numSats = numSats;
    }

    setPositionFix(positionFix: number) {
        this.positionFix = positionFix;
    }

    setVehicleMission(mission: TCPMissionType) {
        for(let i = 0; i < mission.missionItems.length; i++) {
            this.vehicleMission.descriptions.push(mission.missionItems[i].description);
            this.vehicleMission.itemTypes.push(mission.missionItems[i].type);
            let tmpLatLng = new L.LatLng(mission.missionItems[i].lat, mission.missionItems[i].lon);
            this.vehicleMission.latLons.push(tmpLatLng);

            let tmpIcon = new L.Icon({
              iconUrl: './images/marker-icon.png',
              iconSize: [25, 41], // size of the icon
              iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
              popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
            });
            this.vehicleMission.icons.push(tmpIcon);
        }
    }

    setVehicleHome(home: (TCPReturnType & MissionItemType)) {
        console.log("In set vehicle home: " + home.lat + " / " + home.lon);
        let iconBackgroundColor = this.isSelected ? backgroundColors[this.vehicleId] : opaqueBackgroundColors[this.vehicleId]
        let iconHTML = '<div style="background-color: ' + iconBackgroundColor+ '; color: white; width: 41px; text-align: center;">' + this.vehicleId + '</div><img src="./images/Home-Icon.png" alt="Home icon" style="width:41px; height:41px; ">';

        this.homePosition = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(home.lat, home.lon),
            altitude: 0,
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
            })
        };
    }

    updateMarkerPosition(newPos?: PositionType) {
        let posUpdate = this.position;
        if(newPos){
            posUpdate = newPos;
        }

        let iconBackgroundColor = this.isSelected ? backgroundColors[this.vehicleId] : opaqueBackgroundColors[this.vehicleId];
        let iconHTML = '<div style="background-color: ' + iconBackgroundColor + '; color: white; width: 41px; text-align: center;">' + this.vehicleId + '</div><img src="./images/drone-icon.png" alt="Drone icon" style="width:41px; height:41px; -webkit-transform: rotate(' + this.attitude.yaw + 'deg); -moz-transform: rotate(' + this.attitude.yaw + 'deg); -o-transform: rotate(' + this.attitude.yaw + 'deg); -ms-transform: rotate(' + this.attitude.yaw + 'deg); transform: rotate(' + this.attitude.yaw + 'deg);">';

        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(posUpdate.lat, posUpdate.lon),
            altitude: 0,
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
            })
        };
    }

    updateMarkerAttitude(newAtt?: AttitudeType) {
        let attUpdate = this.attitude;
        if(newAtt){
            attUpdate = newAtt;
        }

        let iconBackgroundColor = this.isSelected ? backgroundColors[this.vehicleId] : opaqueBackgroundColors[this.vehicleId];
        let iconHTML = '<div style="background-color: ' + iconBackgroundColor + '; color: white; width: 41px; text-align: center;">' + this.vehicleId + '</div><img src="./images/drone-icon.png" alt="Drone icon" style="width:41px; height:41px; -webkit-transform: rotate(' + attUpdate.yaw + 'deg); -moz-transform: rotate(' + attUpdate.yaw + 'deg); -o-transform: rotate(' + attUpdate.yaw + 'deg); -ms-transform: rotate(' + attUpdate.yaw + 'deg); transform: rotate(' + attUpdate.yaw + 'deg);">';

        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            latLon: new L.LatLng(this.position.lat, this.position.lon),
            altitude: 0,
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
            })
        };
    }
}