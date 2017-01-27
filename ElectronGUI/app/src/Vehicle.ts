

// TODO: Figure out a better way to do background colors
const backgroundColors = ['rgba(255,255,255, 1)', 'rgba(255,0,0,0.2)', 'rgba(0,0,255,0.2)', 'rgba(0,0,0,0.2)', 'rgba(0,255,0,0.2)', 'rgba(255,255,0,0.2)', 'rgba(255,153,0,0.2)'];


export class Vehicle{

    vehicleId: number;
    position: PositionType;
    attitude: AttitudeType;
    numSats: number;
    positionFix: number;
    vehicleMode: VehicleModeType;
    positionInterval: number;
    attitudeInterval: number;
    vehicleMarker: MarkerType;

    constructor(vehicleId: number, position?: PositionType, attitude?: AttitudeType){
        this.vehicleId = vehicleId;
        this.numSats = 0;
        this.positionFix = 0;
        this.vehicleMode = 'UNKNOWN';
        if(position){
            this.position = position;
        }
        else {
            this.position = {lat: 0, lon: 0, alt: 0};
        }
        if(attitude){
            this.attitude = attitude;
        }
        else {
            this.attitude = {roll: 0, pitch: 0, yaw: 0};
        }


        let iconHTML = '<div style="background-color: ' + backgroundColors[this.vehicleId] + '; color: white; width: 41px; text-align: center;">' + this.vehicleId + '</div><img src="./images/drone-icon.png" alt="Drone icon" style="width:41px; height:41px; -webkit-transform: rotate(' + this.attitude.yaw + 'deg); -moz-transform: rotate(' + this.attitude.yaw + 'deg); -o-transform: rotate(' + this.attitude.yaw + 'deg); -ms-transform: rotate(' + this.attitude.yaw + 'deg); transform: rotate(' + this.attitude.yaw + 'deg);">';
        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            position: new L.LatLng(this.position.lat, this.position.lon),
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
            })
        };
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

    updateMarker(newPos?: PositionType, newAtt?: AttitudeType) {
        let posUpdate = this.position;
        if(newPos){
            posUpdate = newPos;
        }
        let attUpdate = this.attitude;
        if(newAtt){
            attUpdate = newAtt;
        }

        let iconHTML = '<div style="background-color: ' + backgroundColors[this.vehicleId] + '; color: white; width: 41px; text-align: center;">' + this.vehicleId + '</div><img src="./images/drone-icon.png" alt="Drone icon" style="width:41px; height:41px; -webkit-transform: rotate(' + attUpdate.yaw + 'deg); -moz-transform: rotate(' + attUpdate.yaw + 'deg); -o-transform: rotate(' + attUpdate.yaw + 'deg); -ms-transform: rotate(' + attUpdate.yaw + 'deg); transform: rotate(' + attUpdate.yaw + 'deg);">';
        this.vehicleMarker = {
            vehicleId: this.vehicleId,
            position: new L.LatLng(posUpdate.lat, posUpdate.lon),
            icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
            })
        };

    }


}