type VehicleTypeType = 'Quad' | 'Fixed';

type PositionType = {
    lat: number,
    lon: number,
    alt: number
};

type AttitudeType = {
    roll: number,
    pitch: number,
    yaw: number
};

type FuelType = {
    batteryRemaining: number,
    batteryCurrent: number,
    batteryVoltage: number
}

type VehicleModeType = 'LOITER' | 'RTL' | 'LAND' | 'AUTO' | 'GUIDED' | 'UNKNOWN';

type VehicleStateType = {
    position: PositionType,
    attitude: AttitudeType,
    numSats: number,
    positionFix: number,
    vehicleMode: VehicleModeType
};

type VehicleMapType = {[id: string]: VehicleStateType};



type TCPDescriptorType = {
  dataType: string,
  vehicleID: number
}

type ConnectedVehiclesType = TCPDescriptorType & {
  connectedVehicles: number[]
}

type TCPPositionType = TCPDescriptorType & PositionType & {
  positionFix: number,
  numSats: number
};

type TCPAttitudeType = TCPDescriptorType & AttitudeType;

type TCPFuelType = TCPDescriptorType & FuelType;

type MissionItemType = PositionType & {
  description: string,
  type: string
};

type TCPMissionType = TCPDescriptorType & {
  missionItems: MissionItemType[]
};

type TCPReturnType = ConnectedVehiclesType | TCPPositionType | TCPAttitudeType | TCPFuelType | TCPMissionType;


type MarkerType = {
  latLon: L.LatLng,
  icon: L.Icon,
  altitude: number,
  vehicleId?: number
};

type LayerGroupType = {
  type: string,
  latLons: L.LatLng[]
};

type MissionLayerType = {
  descriptions: string[],
  latLons: L.LatLng[],
  itemTypes: string[],
  icons: L.Icon[]
};