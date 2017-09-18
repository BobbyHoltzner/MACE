
declare var electronRequire: any;
declare module 'deepcopy';
declare module 'react-leaflet-heatmap-layer';

type MACEConfig = {
  MACEComms?: {
    ipAddress?: string,
    listenPortNumber?: number,
    sendPortNumber?: number
  },
  GUIInit?: {
    mapCenter?: PositionType,
    mapZoom?: number,
    maxZoom?: number
  },
  VehicleSettings?: {
    defaultTakeoffAlt?: number
  }
}


type PositionType = {
    lat: number,
    lng: number,
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

type TextType = {
  severity: string,
  text: string,
  timestamp: Date
}

type GPSType = {
  visibleSats: number,
  gpsFix: string,
  hdop: number,
  vdop: number
}

type VehicleModeType = 'LOITER' | 'RTL' | 'LAND' | 'AUTO' | 'GUIDED' | 'UNKNOWN';

type VehicleTypeType = 'GENERIC' | 'HELICOPTER' | 'GCS' | 'REPEATER' | 'GROUND_ROVER' |
                       'SURFACE_BOAT' | 'TRICOPTER' | 'QUADROTOR' | 'HEXAROTOR' |
                       'OCTOROTOR' | 'ONBOARD_CONTROLLER' | 'FIXED_WING';

type HeartbeatType = {
  autopilot: string,
  aircraftType: VehicleTypeType,
  companion: boolean,
  commsProtocol: string
}

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

type TCPModeType = TCPDescriptorType & {
  vehicleMode: string
};

type TCPTextType = TCPDescriptorType & TextType;

type TCPGPSType = TCPDescriptorType & GPSType;

type MissionItemType = PositionType & {
  description: string,
  type: string
};

type TCPMissionType = TCPDescriptorType & {
  missionItems: MissionItemType[],
  missionType?: string,
  creatorID?: number,
  missionID?: number,
  missionState?: string
};

type TCPSensorFootprintType = TCPDescriptorType & {
  sensorFootprint: PositionType[]
}

type TCPEnvironmentBoundaryType = TCPDescriptorType & {
  environmentBoundary: PositionType[]
}

type TCPCurrentMissionItemType = TCPDescriptorType & {
  missionItemIndex: number
}

type TCPVehicleTargetType = TCPDescriptorType & PositionType & {
  distanceToTarget: number
}

type TCPVehicleArmType = TCPDescriptorType & {
  armed: boolean
}

type TCPMissionItemReachedType = TCPDescriptorType & {
  itemIndex: number
}

type TCPAirspeedType = TCPDescriptorType & {
  airspeed: number
}

type TCPHeartbeatType = TCPDescriptorType & HeartbeatType;

type TCPOriginType = TCPDescriptorType & PositionType & {
  gridSpacing: number
}

type TCPReturnType = ConnectedVehiclesType | TCPPositionType | TCPAttitudeType |
                     TCPFuelType | TCPMissionType | TCPModeType | TCPTextType |
                     TCPSensorFootprintType | TCPCurrentMissionItemType |
                     TCPGPSType | TCPHeartbeatType | TCPMissionItemReachedType |
                     TCPVehicleArmType | TCPAirspeedType | TCPEnvironmentBoundaryType |
                     TCPVehicleTargetType | TCPOriginType;


type MarkerType = {
  latLon: L.LatLng,
  icon: L.DivIcon,
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
  icons: L.Icon[],
  missionType?: string,
  creatorID?: number,
  missionID?: number,
  missionState?: string
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

type EnvironmentSettingsType = {
  minSliderVal: number,
  maxSliderVal: number,
  showBoundingBox: boolean,
  gridSpacing: number
}

type HeatmapOptions = {
  size?: number,
  units?: 'm' | 'px',
  opacity?: number,
  gradientTexture?: string,
  alphaRange?: number
}