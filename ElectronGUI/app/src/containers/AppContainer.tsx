import * as React from 'react';

import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();

var NotificationSystem = require('react-notification-system');
import { ConnectedVehiclesContainer } from './ConnectedVehiclesContainer';
import { VehicleWarningsContainer, VehicleWarning } from './VehicleWarningsContainer';
import { VehicleCommandsContainer } from './VehicleCommandsContainer';
import { DrawButtonsContainer } from './DrawButtonsContainer';
import { AppDrawer } from './AppDrawer';
import AppBar from 'material-ui/AppBar';
import * as colors from 'material-ui/styles/colors';
// import IconMenu from 'material-ui/IconMenu';
// import MenuItem from 'material-ui/MenuItem';
// import IconButton from 'material-ui/IconButton';
// import MoreVertIcon from 'material-ui/svg-icons/navigation/more-vert';
import { Vehicle } from '../Vehicle';
import { VehicleHomeDialog } from '../components/VehicleHomeDialog';
import { GlobalOriginDialog } from '../components/GlobalOriginDialog';
import { MessagesDialog } from '../components/MessagesDialog';
import { TakeoffDialog } from '../components/TakeoffDialog';
import MACEMap from '../components/MACEMap';
import { getRandomRGB } from '../util/Colors';
// import FontIcon from 'material-ui/FontIcon';
import FlatButton from 'material-ui/FlatButton';

import * as deepcopy from 'deepcopy';


var injectTapEventPlugin = require("react-tap-event-plugin");
injectTapEventPlugin();
var net = electronRequire('net');

// // Performance testing:
// var Perf = require('react-addons-perf');
// // End performance testing

type Props = {
}

type State = {
  tcpSendHost?: string,
  tcpSendPort?: number,
  tcpListenHost?: string,
  tcpListenPort?: number,
  connectedVehicles?: {[id: string]: Vehicle}
  vehicleWarnings?: VehicleWarning[]
  selectedVehicleID?: string,
  openDrawer?: boolean,
  tcpSockets?: any[],
  tcpServer?: any,
  allowVehicleSelect?: boolean,
  showEditVehicleHomeDialog?: boolean,
  showEditGlobalHomeDialog?: boolean,
  showMessagesMenu?: boolean,
  messagePreferences?: MessagePreferencesType,
  takeoffAlt?: string,
  showTakeoffDialog?: boolean,
  showSaveTakeoff?: boolean,
  maxZoom?: number,
  mapZoom?: number,
  mapCenter?: PositionType
  globalOrigin?: PositionType
  useContext?: boolean,
  contextAnchor?: L.MouseEvent,
  MACEConnected?: boolean,
  environmentBoundary?: PositionType[],
  showDraw?: boolean,
  drawPolygonPts?: PositionType[]
}

export default class AppContainer extends React.Component<Props, State> {
  notificationSystem: any; // TODO: Figure out why I cant make this a NotificationSystem type...
  m_AttitudeInterval: number[];
  m_AttitudeTimeout: number;
  m_PositionInterval: number[];
  m_PositionTimeout: number;

  vehicleDB: {[id: string]: Vehicle};

  logger: any;

  constructor(props: Props) {
    super(props);

    this.vehicleDB = {};

    this.m_AttitudeInterval = [];
    this.m_PositionInterval = [];
    this.m_AttitudeTimeout = 1111;
    this.m_PositionTimeout = 1234;

    this.state = {
      tcpSendHost: '127.0.0.1',
      tcpSendPort: 5678,
      tcpListenHost: '127.0.0.1',
      tcpListenPort: 1234,
      maxZoom: 21,
      mapZoom: 20,
      mapCenter: {lat: 37.889231, lng: -76.810302, alt: 0}, // Bob's Farm
      // mapCenter: [-35.363272, 149.165249], // SITL Default
      // mapCenter: [45.283410, -111.400850], // Big Sky
      connectedVehicles: {},
      vehicleWarnings: [],
      openDrawer: false,
      tcpSockets: [],
      tcpServer: null,
      allowVehicleSelect: true,
      showEditVehicleHomeDialog: false,
      showEditGlobalHomeDialog: false,
      globalOrigin: {lat: 0, lng: 0, alt: 0},
      selectedVehicleID: "0",
      showMessagesMenu: false,
      messagePreferences: {
        emergency: true,
        alert: true,
        critical: true,
        error: true,
        warning: true,
        notice: true,
        info: true,
        debug: true
      },
      takeoffAlt: "5",
      showTakeoffDialog: false,
      showSaveTakeoff: false,
      MACEConnected: false,
      environmentBoundary: [],
      showDraw: false,
      drawPolygonPts: []
    }

  }

  componentDidMount(){
    // // Performance testing:
    // setTimeout(() => {
    //   console.log("Start performance testing...");
    //   Perf.start();
    //   setTimeout(() => {
    //     Perf.stop();
    //     const measurements = Perf.getLastMeasurements();
    //     Perf.printWasted(measurements);
    //   }, 30000);
    // }, 5000);
    // // End performance testing


    // Parse XML File:
    this.parseXMLConfig();


    this.notificationSystem = this.refs.notificationSystem;
    this.setupTCPServer();

    this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");

    setInterval(() => {
      this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
    }, 3000);
  }

  parseXMLConfig = () => {
    let jsonConfig: MACEConfig = require('../../../../GUIConfig.json');

    if(jsonConfig.MACEComms) {
      if(jsonConfig.MACEComms.listenAddress) {
        this.setState({tcpListenHost: jsonConfig.MACEComms.listenAddress});
      }
      if(jsonConfig.MACEComms.listenPortNumber) {
        this.setState({tcpListenPort: jsonConfig.MACEComms.listenPortNumber});
      }
    }
    if(jsonConfig.GUIInit) {
      if(jsonConfig.GUIInit.mapCenter) {
        let center = {lat: jsonConfig.GUIInit.mapCenter[0], lng: jsonConfig.GUIInit.mapCenter[1], alt: 0};
        this.setState({mapCenter: center});
      }
      if(jsonConfig.GUIInit.mapZoom) {
        this.setState({mapZoom: jsonConfig.GUIInit.mapZoom});
      }
      if(jsonConfig.GUIInit.maxZoom) {
        this.setState({maxZoom: jsonConfig.GUIInit.maxZoom});
      }
    }
    if(jsonConfig.VehicleSettings) {
      if(jsonConfig.VehicleSettings.defaultTakeoffAlt) {
        this.setState({takeoffAlt: jsonConfig.VehicleSettings.defaultTakeoffAlt.toString()});
      }
    }


    // TODO:
    //       3) MAKE SURE WE CAN STILL SET UP A SERVE COMMUNICATION TO MACE
    //       4) ADD CONFIG ELEMENTS TO MACE
    //       5) USE VALGRIND TO START PROFILING WITH VM

    //       6) DOWN THE ROAD, MAKE GUI ELEMENT TO SET CERTAIN STATE ITEMS (like comms stuff and other map related things)


  }

  setupTCPServer = () => {
    // Create a TCP socket listener
    let tcpServer = net.createServer(function (socket: any) {

        // Add the new client socket connection to the array of sockets
        this.state.tcpSockets.push(socket);

        // 'data' is an event that means that a message was just sent by the client application
        socket.on('data', function (msg_sent: any) {
          // console.log("Data from socket: " + msg_sent);
          let jsonData: TCPReturnType = JSON.parse(msg_sent);

          this.parseTCPClientData(jsonData);

        }.bind(this));
        // Use splice to get rid of the socket that is ending.
        // The 'end' event means tcp client has disconnected.
        socket.on('end', function () {
            // let sockets = deepcopy(this.state.tcpSockets);
            let i = this.state.tcpSockets.indexOf(socket);
            this.state.tcpSockets.splice(i, 1);
        }.bind(this));


    }.bind(this));

    this.setState({tcpServer: tcpServer}, () => {
      // TODO: Allow for user configuration of the port and probably address too
      try{
        this.state.tcpServer.listen(this.state.tcpListenPort, this.state.tcpListenHost);
        this.setState({MACEConnected: true});
      }
      catch(e) {
        console.log('Error: ' + e);
      }

      console.log('System listening at http://' + this.state.tcpListenHost + ':' + this.state.tcpListenPort);

      // Set interval to set state to DB:
      setInterval(() => {
        this.setState({connectedVehicles: this.vehicleDB});
      }, 1500);
    });

  }


  parseTCPClientData = (jsonData: TCPReturnType) => {
    let stateCopy = deepcopy(this.state.connectedVehicles);

    // Log message:
    // this.logger.info("[MACE Data: " + JSON.stringify(jsonData) + "]");

    if(jsonData.dataType === "ConnectedVehicles"){
      let jsonVehicles = jsonData as ConnectedVehiclesType;

      // console.log("Connected vehicles: " + jsonVehicles.connectedVehicles);

      // Check if vehicle is already in the map. If so, do nothing. If not, add it:
      for(let i = 0; i < jsonVehicles.connectedVehicles.length; i++){
        if (stateCopy[jsonVehicles.connectedVehicles[i].toString()] !== undefined){
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

      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === "VehiclePosition"){
      let vehiclePosition = jsonData as TCPPositionType;

      stateCopy[vehiclePosition.vehicleID].position.lat = vehiclePosition.lat;
      stateCopy[vehiclePosition.vehicleID].position.lng = vehiclePosition.lng;
      stateCopy[vehiclePosition.vehicleID].position.alt = vehiclePosition.alt;
      stateCopy[vehiclePosition.vehicleID].numSats = vehiclePosition.numSats;
      stateCopy[vehiclePosition.vehicleID].positionFix = vehiclePosition.positionFix;

      stateCopy[vehiclePosition.vehicleID].updateVehicleMarkerPosition(vehiclePosition);

      if(stateCopy[vehiclePosition.vehicleID].isNew &&
        (stateCopy[vehiclePosition.vehicleID].gps.gpsFix !== "NO GPS" || stateCopy[vehiclePosition.vehicleID].gps.gpsFix !== "GPS NO FIX") &&
        Object.keys(this.state.connectedVehicles).length === 1)
      {
        stateCopy[vehiclePosition.vehicleID].isNew = false;
        // this.setState({mapCenter: [stateCopy[vehiclePosition.vehicleID].position.lat, stateCopy[vehiclePosition.vehicleID].position.lon], mapZoom: 19});
      }

      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === "VehicleAttitude"){
      let vehicleAttitude = jsonData as TCPAttitudeType;

      stateCopy[vehicleAttitude.vehicleID].attitude.roll = vehicleAttitude.roll;
      stateCopy[vehicleAttitude.vehicleID].attitude.pitch = vehicleAttitude.pitch;
      stateCopy[vehicleAttitude.vehicleID].attitude.yaw = vehicleAttitude.yaw;

      stateCopy[vehicleAttitude.vehicleID].updateMarkerAttitude(vehicleAttitude);

      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === "VehicleAirspeed"){
      let vehicleAirspeed = jsonData as TCPAirspeedType;

      stateCopy[vehicleAirspeed.vehicleID].airspeed = vehicleAirspeed.airspeed;
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'VehicleMission') {
      let vehicleMission = jsonData as TCPMissionType;
      let stateCopy = deepcopy(this.state.connectedVehicles);
      console.log(Object.keys(stateCopy).length);
      stateCopy[vehicleMission.vehicleID].setVehicleMission(vehicleMission);
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'VehicleHome') {
      let vehicleHome = jsonData as (TCPReturnType & MissionItemType);
      let stateCopy = deepcopy(this.state.connectedVehicles);
      let tmpHome = {
        lat: vehicleHome.lat,
        lon: vehicleHome.lng,
        alt: vehicleHome.alt
      }
      stateCopy[vehicleHome.vehicleID].updateHomePosition(tmpHome);
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'VehicleFuel') {
      let vehicleFuel = jsonData as TCPFuelType;

      stateCopy[vehicleFuel.vehicleID].fuel.batteryRemaining = vehicleFuel.batteryRemaining;
      stateCopy[vehicleFuel.vehicleID].fuel.batteryCurrent = vehicleFuel.batteryCurrent;
      stateCopy[vehicleFuel.vehicleID].fuel.batteryVoltage = vehicleFuel.batteryVoltage;

      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'VehicleMode') {
      let vehicleMode = jsonData as TCPModeType;
      stateCopy[vehicleMode.vehicleID].vehicleMode = vehicleMode.vehicleMode;
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'VehicleText') {
      let vehicleText = jsonData as TCPTextType;
      let showMessage = false;
      let title = '';
      let level = 'info';
      if(vehicleText.severity === "EMERGENCY") {
        title = 'EMERGENCY -- Vehicle ' + vehicleText.vehicleID;
        level = 'error';
        showMessage = this.state.messagePreferences.emergency;
      }
      if(vehicleText.severity === "ALERT") {
        title = 'Alert -- Vehicle ' + vehicleText.vehicleID;
        level = 'warning';
        showMessage = this.state.messagePreferences.alert;
      }
      if(vehicleText.severity === "CRITICAL") {
        title = 'CRITICAL -- Vehicle ' + vehicleText.vehicleID;
        level = 'error';
        showMessage = this.state.messagePreferences.critical;
      }
      if(vehicleText.severity === "ERROR") {
        title = 'ERROR -- Vehicle ' + vehicleText.vehicleID;
        level = 'error';
        showMessage = this.state.messagePreferences.error;
      }
      if(vehicleText.severity === "WARNING") {
        title = 'Warning -- Vehicle ' + vehicleText.vehicleID;
        level = 'warning';
        showMessage = this.state.messagePreferences.warning;
      }
      if(vehicleText.severity === "NOTICE") {
        title = 'Notice -- Vehicle ' + vehicleText.vehicleID;
        level = 'success';
        showMessage = this.state.messagePreferences.notice;
      }
      if(vehicleText.severity === "INFO") {
        title = 'Info -- Vehicle ' + vehicleText.vehicleID;
        level = 'info';
        showMessage = this.state.messagePreferences.info;
      }
      if(vehicleText.severity === "DEBUG") {
        title = 'Debug -- Vehicle ' + vehicleText.vehicleID;
        level = 'info';
        showMessage = this.state.messagePreferences.debug;
      }

      if(showMessage) {
        this.showNotification(title, vehicleText.text, level, 'bl', 'Got it');
        stateCopy[vehicleText.vehicleID].messages.unshift({severity: vehicleText.severity, text: vehicleText.text, timestamp: new Date()});
        this.vehicleDB = stateCopy;
      }
    }
    else if(jsonData.dataType === 'GlobalOrigin') {
      let jsonOrigin = jsonData as TCPPositionType;
      let origin = {lat: jsonOrigin.lat, lng: jsonOrigin.lng, alt: jsonOrigin.alt};
      this.setState({globalOrigin: origin});
    }
    else if(jsonData.dataType === 'SensorFootprint') {
      let jsonFootprint = jsonData as TCPSensorFootprintType;
      stateCopy[jsonFootprint.vehicleID].sensorFootprint = jsonFootprint.sensorFootprint;
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'EnvironmentBoundary') {
      let jsonBoundary = jsonData as TCPEnvironmentBoundaryType;
      this.setState({environmentBoundary: jsonBoundary.environmentBoundary});
    }
    else if(jsonData.dataType === 'VehicleGPS') {
      let jsonGPS = jsonData as TCPGPSType;
      stateCopy[jsonGPS.vehicleID].gps.visibleSats = jsonGPS.visibleSats;
      stateCopy[jsonGPS.vehicleID].gps.gpsFix = jsonGPS.gpsFix;
      stateCopy[jsonGPS.vehicleID].gps.hdop = jsonGPS.hdop;
      stateCopy[jsonGPS.vehicleID].gps.vdop = jsonGPS.vdop;
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'CurrentMissionItem') {
      let jsonMissionItem = jsonData as TCPCurrentMissionItemType;
      stateCopy[jsonMissionItem.vehicleID].updateCurrentMissionItem(jsonMissionItem.missionItemIndex, false);
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'MissionItemReached') {
      let jsonMissionItem = jsonData as TCPMissionItemReachedType;
      if(jsonMissionItem.itemIndex === stateCopy[jsonMissionItem.vehicleID].vehicleMission.icons.length - 1) {
        stateCopy[jsonMissionItem.vehicleID].updateCurrentMissionItem(jsonMissionItem.itemIndex, true)
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
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'VehicleArm') {
      let jsonArm = jsonData as TCPVehicleArmType;
      stateCopy[jsonArm.vehicleID].isArmed = jsonArm.armed;
      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'CurrentVehicleTarget') {
      let jsonVehicleTarget = jsonData as TCPVehicleTargetType;
      stateCopy[jsonVehicleTarget.vehicleID].currentTarget.active = true;
      stateCopy[jsonVehicleTarget.vehicleID].currentTarget.distanceToTarget = jsonVehicleTarget.distanceToTarget;
      stateCopy[jsonVehicleTarget.vehicleID].currentTarget.targetPosition.lat = jsonVehicleTarget.lat;
      stateCopy[jsonVehicleTarget.vehicleID].currentTarget.targetPosition.lng = jsonVehicleTarget.lng;
      stateCopy[jsonVehicleTarget.vehicleID].currentTarget.targetPosition.alt = jsonVehicleTarget.alt;
      this.vehicleDB = stateCopy;
    }
  }


  makeTCPRequest = (vehicleID: number, tcpCommand: string, vehicleCommand: string) => {

    // Log message:
    // this.logger.info("{TCP Command: " + tcpCommand + "}  {Vehicle Command: " + vehicleCommand + "}");

    let socket = new net.Socket();
    this.setupTCPClient(socket);
    socket.connect(this.state.tcpSendPort, this.state.tcpSendHost, function() {
      // console.log('Connected to: ' + this.state.tcpHost + ':' + this.state.tcpPort);
      let tcpRequest = {
        tcpCommand: tcpCommand,
        vehicleID: vehicleID,
        vehicleCommand: vehicleCommand
      };
      socket.write(JSON.stringify(tcpRequest));
      socket.end();
    }.bind(this));
  }


  setupTCPClient = (socket: any) => {
    // Add a 'data' event handler for the client socket
    // data is what the server sent to this socket
    socket.on('data', function(data: any) {
        // console.log('DATA: ' + data);
        // let jsonData = JSON.parse(data);
        // this.parseTCPServerData(jsonData);

        // Close the client socket completely
        // socket.destroy();

        if(this.state.MACEConnected === false) {
          this.setState({MACEConnected: true});
        }
    }.bind(this));

    // Add a 'close' event handler for the client socket
    socket.on('close', function() {
        // console.log('Connection closed');
        socket.destroy();
    }.bind(this));

    // Add an 'error' event handler
    socket.on('error', function(err: any) {
        console.log('Error: ' + err);
        let str = err+"";
        if(str.indexOf("ECONNREFUSED") > 0){
          // this.handleClearGUI();
        }
        socket.destroy();
    }.bind(this));
  }

  showNotification = (title: string, message: string, level: string, position: string, label: string) => {
    let notification = {
      title: title,
      message: message,
      level: level,
      position: position,
      action: {
        label: label
      }
    }

    this.notificationSystem.addNotification(notification);
  }

  handleClearGUI = () => {
    this.vehicleDB = {};
    this.setState({connectedVehicles: {}, selectedVehicleID: "0", MACEConnected: false});
  }

  handleAircraftCommand = (id: string, tcpCommand: string, vehicleCommand: string) => {
    console.log(tcpCommand);
    this.makeTCPRequest(parseInt(id), tcpCommand, vehicleCommand);
  }

  handleDrawerAction = (action: string) => {
    if(action === "Messages"){
      this.setState({showMessagesMenu: true, showTakeoffDialog: false, showSaveTakeoff: false, openDrawer: false});
    }
    else if(action === "Takeoff"){
      this.setState({showMessagesMenu: false, showTakeoffDialog: true, showSaveTakeoff: true, openDrawer: false});
    }
    else if(action === "TestButton1") {
      this.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION1", "");
    }
    else if(action === "TestButton2") {
      this.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION2", "");
    }
    else if(action === "EditBoundary") {
      this.setState({showDraw: true, openDrawer: false});
    }
  }

  handleSaveVehicleHome = (vehicleID: string, vehicleHome: PositionType) => {
    this.handleAircraftCommand(vehicleID, "SET_VEHICLE_HOME", JSON.stringify(vehicleHome));
    // let tmpHome: any = {
    //   lat: vehicleHome.lat,
    //   lon: vehicleHome.lon,
    //   alt: vehicleHome.alt,
    // }
    // if(this.state.connectedVehicles[vehicleID]) {
    //   this.state.connectedVehicles[vehicleID].updateHomePosition(tmpHome);
    // }
    // else {
    //   console.log("No vehicle with ID: " + vehicleID);
    // }
  }

  handleSaveGlobalOrigin = (globalOrigin: PositionType) => {
    this.handleAircraftCommand("0", "SET_GLOBAL_ORIGIN", JSON.stringify(globalOrigin));
    this.setState({globalOrigin: globalOrigin});
  }

  contextSetHome = () => {
    this.setState({
      showEditVehicleHomeDialog: true,
      allowVehicleSelect: true,
      showEditGlobalHomeDialog: false,
      showTakeoffDialog: false,
      useContext: true
    });
  }

  contextSetGlobal = () => {
    this.setState({
      showEditGlobalHomeDialog: true,
      allowVehicleSelect: false,
      showEditVehicleHomeDialog: false,
      useContext: true
    });
  }

  contextSetTakeoff = () => {
    this.setState({
      showEditVehicleHomeDialog: false,
      allowVehicleSelect: false,
      showEditGlobalHomeDialog: false,
      showTakeoffDialog: true,
      useContext: true
    })
  }

  contextGoHere = () => {
    this.setState({
      showEditGlobalHomeDialog: false,
      allowVehicleSelect: false,
      showEditVehicleHomeDialog: false,
      useContext: true
    });
    let goHere = {
      lat: this.state.contextAnchor.latlng.lat,
      lon: this.state.contextAnchor.latlng.lng
    };
    this.handleAircraftCommand(this.state.selectedVehicleID, "SET_GO_HERE", JSON.stringify(goHere));
  }

  handleSelectedAircraftUpdate = (id: string) => {
    let stateCopy = deepcopy(this.state.connectedVehicles);
    let selectedID = "0";
    Object.keys(this.state.connectedVehicles).map((key: string) => {
      if(key === id){
        stateCopy[id].isSelected = !stateCopy[id].isSelected;
        selectedID = stateCopy[id].isSelected ? id : "0";

        if(stateCopy[id].isSelected === true && (stateCopy[id].position.lat !== 0 && stateCopy[id].position.lat !== 0)){
          // this.setState({mapCenter: [stateCopy[id].position.lat, stateCopy[id].position.lon]});
        }
      }
      else {
        stateCopy[key].isSelected = false;
      }
      stateCopy[key].updateHomePosition();
      stateCopy[key].updateVehicleMarkerPosition();
    });

    this.vehicleDB = stateCopy;
    this.setState({connectedVehicles: stateCopy, selectedVehicleID: selectedID});
  }

  handleSaveMessagingPreferences = (preferences: MessagePreferencesType) => {
    this.setState({messagePreferences: preferences});
  }

  handleTakeoff = (vehicleID: string, takeoffAlt: string, takeoffLat?: string, takeoffLon?: string) => {
    let takeoffPosition = {
      lat: takeoffLat ? parseFloat(takeoffLat) : 0,
      lon: takeoffLon ? parseFloat(takeoffLon) : 0,
      alt: parseFloat(takeoffAlt)
    }
    let latLonFlag = false;
    if(takeoffLat && takeoffLon) {
      latLonFlag = true;
    }
    this.makeTCPRequest(parseInt(vehicleID), "VEHICLE_TAKEOFF", JSON.stringify({takeoffPosition: takeoffPosition, latLonFlag: latLonFlag}));
  }

  updateMapCenter = (e: L.DragEndEvent) => {
    this.setState({mapCenter: {lat: e.target.getCenter().lat, lng: e.target.getCenter().lng, alt: 0}, mapZoom: e.target.getZoom()});
  }

  handleSyncAll = () => {
    this.makeTCPRequest(0, "GET_ENVIRONMENT_BOUNDARY", "");

    this.makeTCPRequest(0, "ISSUE_COMMAND", "FORCE_DATA_SYNC");
  }

  handleAddPolygonPt = (e: L.MouseEvent) => {
    if(this.state.showDraw) {
      let tmpPts = this.state.drawPolygonPts;
      tmpPts.push({lat: e.latlng.lat, lng: e.latlng.lng, alt: 0});
      this.setState({drawPolygonPts: tmpPts});
    }
  }

  handleDeleteLastPolygonPt = () => {
    let tmpPts = this.state.drawPolygonPts;
    tmpPts.pop();
    this.setState({drawPolygonPts: tmpPts});
  }

  handleDisableDraw = () => {
    this.setState({showDraw: false, drawPolygonPts: []});
  }

  handleSubmitBoundary = () => {
    // TODO:
    //  1) Send environment boundary to MACE

    // TODO: Send to MACE:

    if(this.state.drawPolygonPts.length > 2) {
      this.setState({showDraw: false, drawPolygonPts: []});
      this.makeTCPRequest(0, "SET_ENVIRONMENT_VERTICES", JSON.stringify({boundary: this.state.drawPolygonPts}));
    }
    else {
      let title = 'Draw boundary';
      let level = 'info';
      this.showNotification(title, 'Boundary must have 3 or more vertices to be valid', level, 'tc', 'Got it');
    }
  }

  handleClearPts = () => {
    this.setState({drawPolygonPts: []});
  }

  render() {

    const width = window.screen.width;
    const height = window.screen.height;
    const parentStyle = {height: height + 'px', width: width + 'px'};

    const ToolbarRight = () => (

      <FlatButton
        label={"Sync all"}
        labelPosition={"before"}
        onClick={this.handleSyncAll}
        icon={<i className="material-icons">cached</i>}
        style={{color: "white"}}
        />

    );

    return (
        <MuiThemeProvider muiTheme={lightMuiTheme}>
          <div style={parentStyle}>

            <AppBar
                title="MACE"
                style={{backgroundColor: colors.orange700, position: 'fixed'}}
                onLeftIconButtonTouchTap={() => this.setState({openDrawer: !this.state.openDrawer})}
                iconElementRight={<ToolbarRight />}
            />

            <AppDrawer
              openDrawer={this.state.openDrawer}
              onToggleDrawer={(open: boolean) => this.setState({openDrawer: open})}
              onDrawerAction={(action: string) => this.handleDrawerAction(action)}
              showMessagesMenu={this.state.showMessagesMenu}
             />

            <ConnectedVehiclesContainer
              connectedVehicles={this.state.connectedVehicles}
              onAircraftCommand={this.handleAircraftCommand}
              handleChangeSelectedVehicle={this.handleSelectedAircraftUpdate}
              selectedVehicleID={this.state.selectedVehicleID}
            />

            <VehicleCommandsContainer
              connectedVehicles={this.state.connectedVehicles}
              onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
              onAircraftCommand={this.handleAircraftCommand}
              selectedAircraftID={this.state.selectedVehicleID}
              handleTakeoff={() => this.setState({showTakeoffDialog: true, showSaveTakeoff: false, useContext: false})}
            />

            <VehicleWarningsContainer
              vehicleWarnings={this.state.vehicleWarnings}
            />

            {this.state.showEditVehicleHomeDialog &&
              <VehicleHomeDialog
                open={this.state.showEditVehicleHomeDialog}
                handleClose={() => this.setState({showEditVehicleHomeDialog: false, useContext: false})}
                vehicles={this.state.connectedVehicles}
                selectedVehicleID={this.state.selectedVehicleID}
                handleSave={this.handleSaveVehicleHome}
                contextAnchor={this.state.contextAnchor}
                useContext={this.state.useContext}
                allowVehicleSelect={this.state.allowVehicleSelect}
                onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                showNotification={this.showNotification}
              />
            }

            {this.state.showEditGlobalHomeDialog &&
              <GlobalOriginDialog
                open={this.state.showEditGlobalHomeDialog}
                handleClose={() => this.setState({showEditGlobalHomeDialog: false, useContext: false})}
                onGlobalHomeCommand={this.handleAircraftCommand}
                globalOrigin={this.state.globalOrigin}
                handleSave={this.handleSaveGlobalOrigin}
                contextAnchor={this.state.contextAnchor}
                useContext={this.state.useContext}
              />
            }

            {this.state.showMessagesMenu &&
              <MessagesDialog
                open={this.state.showMessagesMenu}
                handleClose={() => this.setState({showMessagesMenu: false})}
                handleSave={this.handleSaveMessagingPreferences}
                preferences={this.state.messagePreferences}
              />
            }

            {this.state.showTakeoffDialog &&
              <TakeoffDialog
                open={this.state.showTakeoffDialog}
                handleClose={() => this.setState({showTakeoffDialog: false, useContext: false})}
                vehicles={this.state.connectedVehicles}
                selectedVehicleID={this.state.selectedVehicleID}
                handleTakeoff={this.handleTakeoff}
                takeoffAlt={this.state.takeoffAlt}
                onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
                showSaveTakeoff={this.state.showSaveTakeoff}
                handleSaveTakeoff={(alt: string) => this.setState({takeoffAlt: alt})}
                contextAnchor={this.state.contextAnchor}
                useContext={this.state.useContext}
                showNotification={this.showNotification}
              />
            }


            {this.state.showDraw &&
              <DrawButtonsContainer
                onDeleteLastPolygonPt={this.handleDeleteLastPolygonPt}
                onDisableDraw={this.handleDisableDraw}
                onSubmitBoundary={this.handleSubmitBoundary}
                onClearAllPts={this.handleClearPts}
              />
            }


            <MACEMap
              handleSelectedAircraftUpdate={this.handleSelectedAircraftUpdate}
              setContextAnchor={(anchor: L.MouseEvent) => this.setState({contextAnchor: anchor})}
              connectedVehicles={this.state.connectedVehicles}
              selectedVehicleID={this.state.selectedVehicleID}
              mapCenter={this.state.mapCenter}
              maxZoom={this.state.maxZoom}
              mapZoom={this.state.mapZoom}
              globalOrigin={this.state.globalOrigin}
              updateMapCenter={this.updateMapCenter}
              contextAnchor={this.state.contextAnchor}
              contextSetGlobal={this.contextSetGlobal}
              contextSetHome={this.contextSetHome}
              contextSetTakeoff={this.contextSetTakeoff}
              contextGoHere={this.contextGoHere}
              MACEConnected={this.state.MACEConnected}
              environmentBoundary={this.state.environmentBoundary}
              drawPolygonPts={this.state.drawPolygonPts}
              onAddPolygonPt={this.handleAddPolygonPt}
             />


            <div>
              <NotificationSystem ref="notificationSystem" />
            </div>

          </div>
        </MuiThemeProvider>
    );
  }
}
