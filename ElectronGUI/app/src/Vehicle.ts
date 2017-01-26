

export class Vehicle{

    vehicleId: number;
    position: PositionType;
    attitude: AttitudeType;
    numSats: number;
    positionFix: number;
    vehicleMode: VehicleModeType;
    positionInterval: number;
    attitudeInterval: number;

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


}