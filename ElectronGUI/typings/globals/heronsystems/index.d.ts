type VehicleStateType = {
    position: {
        lat: number,
        lon: number,
        alt: number,
        numSats: number,
        positionFix: number
    },
    attitude: {
        roll: number,
        pitch: number,
        yaw: number 
    }
}

type VehicleMapType = {[id: string]: VehicleStateType};