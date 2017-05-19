type VehicleTypeType = 'Copter' | 'Plane';

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

type ModeType = {
  vehicleMode: string,
  isArmed: boolean
}

type TextType = {
  severity: string,
  text: string
}

type GPSType = {
  visibleSats: number,
  gpsFix: string,
  hdop: number,
  vdop: number
}

type VehicleModeType = 'LOITER' | 'RTL' | 'LAND' | 'AUTO' | 'GUIDED' | 'UNKNOWN';

// type VehicleStateType = {
//     position: PositionType,
//     attitude: AttitudeType,
//     numSats: number,
//     positionFix: number,
//     vehicleMode: VehicleModeType
// };

// type VehicleMapType = {[id: string]: VehicleStateType};


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

type TCPModeType = TCPDescriptorType & ModeType;

type TCPTextType = TCPDescriptorType & TextType;

type TCPGPSType = TCPDescriptorType & GPSType;

type MissionItemType = PositionType & {
  description: string,
  type: string
};

type TCPMissionType = TCPDescriptorType & {
  missionItems: MissionItemType[]
};

type TCPSensorFootprintType = TCPDescriptorType & {
  sensorFootprint: PositionType[]
}

type TCPCurrentMissionItemType = TCPDescriptorType & {
  missionItemIndex: number
}

type TCPReturnType = ConnectedVehiclesType | TCPPositionType | TCPAttitudeType |
                     TCPFuelType | TCPMissionType | TCPModeType | TCPTextType |
                     TCPSensorFootprintType | TCPCurrentMissionItemType |
                     TCPGPSType;


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

type MessagePreferencesType = {
  emergency: boolean,
  alert: boolean,
  critical: boolean,
  error: boolean,
  warning: boolean,
  notice: boolean,
  info: boolean,
  debug: boolean
}