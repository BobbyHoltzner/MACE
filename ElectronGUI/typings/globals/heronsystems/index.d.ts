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

type TCPPositionType = TCPDescriptorType & {
  lat: number,
  lon: number,
  alt: number,
  positionFix: number,
  numSats: number
}

type TCPAttitudeType = TCPDescriptorType & {
  roll: number,
  pitch: number,
  yaw: number
}

type MissionItemType = {
  description: string,
  lat: number,
  lon: number,
  alt: number,
  type: string
}

type TCPMissionType = TCPDescriptorType & {
  missionItems: MissionItemType[]
}

type TCPReturnType = ConnectedVehiclesType | TCPPositionType | TCPAttitudeType | TCPMissionType;


type MarkerType = {
  latLon: L.LatLng,
  icon: L.Icon,
  altitude: number,
  vehicleId?: number
}

type LayerGroupType = {
  type: string,
  latLons: L.LatLng[]
}

type MissionLayerType = {
  descriptions: string[],
  latLons: L.LatLng[],
  itemTypes: string[],
  icons: L.Icon[]
}