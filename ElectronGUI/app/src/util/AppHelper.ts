import * as deepcopy from 'deepcopy';
var fs = electronRequire('fs');
var net = electronRequire('net');
import { Vehicle } from '../Vehicle';
import { getRandomRGB } from './Colors';
var turf = require('@turf/turf');
var geometryHelper = require('leaflet-geometryutil');
import * as L from 'leaflet';


/*
  TODO:
    1) Move TCP/network stuff into its own class
    2) Figure out how to move notifications out of AppContainer.tsx?
    3) Move grid/voronoi/survey generator code into its own class (somewhat deprecated, but still want to support it)
*/



export class AppHelper {
    state: any; // TODO: Figure out how to get State type in here...
    vehicleDB: {[id: string]: Vehicle};
    MACEconfig: ConfigSettingsType;
    notificationSystem: any; // TODO: Figure out why I cant make this a NotificationSystem type...
    getVehiclesInterval: any;
    tcpSockets: any[];
    connectedVehicles: {[id: string]: Vehicle};
    tcpServer: any;
    globalOrigin: PositionType;
    drawPolygonPts: PositionType[];
    selectedVehicleID?: string;
    openDrawer?: boolean;
    allowVehicleSelect?: boolean;
    showEditVehicleHomeDialog?: boolean;
    showEditGlobalHomeDialog?: boolean;
    showMessagesMenu?: boolean;
    showConfigDialog?: boolean;
    messagePreferences?: MessagePreferencesType;
    showTakeoffDialog?: boolean;
    showSaveTakeoff?: boolean;
    useContext?: boolean;
    contextAnchor?: L.LeafletMouseEvent;
    MACEConnected?: boolean;
    environmentBoundary?: PositionType[];
    showDraw?: boolean;
    gridPts?: {inPoly: L.LatLng[], trimmedPts: L.LatLng[]};
    showEnvironmentSettings?: boolean;
    environmentSettings?: EnvironmentSettingsType;
    pauseMACEComms?: boolean;
    envBoundingBox?: PositionType[];
    getConnectedVehiclesTimeout?: number;

    constructor(appState: any){
        this.state = appState;
        this.vehicleDB = {};
        this.getVehiclesInterval = null;
        this.MACEconfig = {
            filename: '../GUIConfig.json',
            config: {
            MACEComms: {
                ipAddress: '127.0.0.1',
                listenPortNumber: 1234,
                sendPortNumber: 5678
            },
            GUIInit: {
                mapCenter: {lat: 37.889231, lng: -76.810302, alt: 0}, // Bob's Farm
                // mapCenter: [-35.363272, 149.165249], // SITL Default
                // mapCenter: [45.283410, -111.400850], // Big Sky
                mapZoom: 20,
                maxZoom: 21
            },
            VehicleSettings: {
                defaultTakeoffAlt: 5
            }
          }
        };
        this.tcpServer = null;
        this.tcpSockets = [];
        // this.connectedVehicles = {};
        this.tcpServer = null;
        this.globalOrigin =  {lat: 0, lng: 0, alt: 0};
        this.openDrawer = false;
        this.allowVehicleSelect = true;
        this.showEditVehicleHomeDialog = false;
        this.showEditGlobalHomeDialog = false;
        this.selectedVehicleID = "0";
        this.showMessagesMenu = false;
        this.showConfigDialog = false;
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
        this.showTakeoffDialog = false;
        this.showSaveTakeoff = false;
        this.MACEConnected = false;
        this.environmentBoundary = [];
        this.showDraw = false;
        this.drawPolygonPts = [];
        this.gridPts = {inPoly: [], trimmedPts: []};
        this.showEnvironmentSettings = false;
        this.environmentSettings = {minSliderVal: 25, maxSliderVal: 100, showBoundingBox: false, gridSpacing: -1};
        this.pauseMACEComms = false;
        this.envBoundingBox = [];
        this.getConnectedVehiclesTimeout = 3000;


        // Parse XML File:
        this.parseJSONConfig(this.MACEconfig.filename);

        this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");

        this.getVehiclesInterval = setInterval(() => {
          this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
        }, this.state.getConnectedVehiclesTimeout);
    }


  parseJSONConfig = (filename: string, restartServer: boolean = true) => {
    // let jsonConfig: MACEConfig = require(filename);
    // let jsonConfig: MACEConfig = require("C:/Code/MACE/GUIConfig.json");

    let jsonConfig = JSON.parse(fs.readFileSync(filename));
    let MACEconfig: ConfigSettingsType = this.MACEconfig;

    MACEconfig.filename = filename;
    if(jsonConfig.MACEComms) {
      if(jsonConfig.MACEComms.ipAddress) {
        MACEconfig.config.MACEComms.ipAddress = jsonConfig.MACEComms.ipAddress;
        if(restartServer) {
          this.setupTCPServer();
        }

        // Reset interval and start requests again:
        if(this.getVehiclesInterval) {
          clearInterval(this.getVehiclesInterval);
          this.getVehiclesInterval = setInterval(() => {
            this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
          }, this.state.getConnectedVehiclesTimeout);
        }
      }
      if(jsonConfig.MACEComms.listenPortNumber) {
        MACEconfig.config.MACEComms.listenPortNumber = jsonConfig.MACEComms.listenPortNumber;
        if(restartServer) {
          this.setupTCPServer();
        }
      }
      if(jsonConfig.MACEComms.sendPortNumber) {
        MACEconfig.config.MACEComms.sendPortNumber = jsonConfig.MACEComms.sendPortNumber;

        // Reset interval and start requests again:
        if(this.getVehiclesInterval) {
          clearInterval(this.getVehiclesInterval);
          this.getVehiclesInterval = setInterval(() => {
            this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
          }, this.state.getConnectedVehiclesTimeout);
        }
      }
    }
    if(jsonConfig.GUIInit) {
      if(jsonConfig.GUIInit.mapCenter) {
        let center = {lat: jsonConfig.GUIInit.mapCenter.lat, lng: jsonConfig.GUIInit.mapCenter.lng, alt: 0};
        MACEconfig.config.GUIInit.mapCenter = center;
      }
      if(jsonConfig.GUIInit.mapZoom) {
        MACEconfig.config.GUIInit.mapZoom = jsonConfig.GUIInit.mapZoom;
      }
      if(jsonConfig.GUIInit.maxZoom) {
        MACEconfig.config.GUIInit.maxZoom = jsonConfig.GUIInit.maxZoom;
      }
    }
    if(jsonConfig.VehicleSettings) {
      if(jsonConfig.VehicleSettings.defaultTakeoffAlt) {
        MACEconfig.config.VehicleSettings.defaultTakeoffAlt = jsonConfig.VehicleSettings.defaultTakeoffAlt;
      }
    }

    this.MACEconfig = MACEconfig;
  }

  setupTCPServer = () => {
    // Close server if already exists:
    if(this.tcpServer !== null) {
      this.tcpServer.close();
    }

    // Close all existing sockets:
    let tcpSockets = deepcopy(this.tcpSockets);
    for(let i = 0; i < tcpSockets.length; i++) {
      tcpSockets[i].destroy();
    }
    this.tcpSockets = [];

    // Create a TCP socket listener
    let tcpServer = net.createServer(function (socket: any) {
      let remoteAddr = socket.remoteAddress.replace(/^.*:/, '')
      if(remoteAddr === this.MACEconfig.config.MACEComms.ipAddress) {
        // Add the new client socket connection to the array of sockets
        this.tcpSockets.push(socket);

        // 'data' is an event that means that a message was just sent by the client application
        socket.on('data', function (msg_sent: any) {
          // console.log("Data from socket: " + msg_sent);
          let jsonData: TCPReturnType = JSON.parse(msg_sent);
          this.parseTCPClientData(jsonData);
        }.bind(this));
        // Use splice to get rid of the socket that is ending.
        // The 'end' event means tcp client has disconnected.
        socket.on('end', function () {
            // let sockets = deepcopy(this.tcpSockets);
            let i = this.tcpSockets.indexOf(socket);
            this.tcpSockets.splice(i, 1);
        }.bind(this));

      }
    }.bind(this));

    this.tcpServer = tcpServer;

    // TODO: Allow for user configuration of the port and probably address too
    try{
      this.tcpServer.listen(this.MACEconfig.config.MACEComms.listenPortNumber);
      this.MACEConnected = true;
    }
    catch(e) {
      console.log(e);
    }

    console.log('System listening at http://' + this.MACEconfig.config.MACEComms.ipAddress + ':' + this.MACEconfig.config.MACEComms.listenPortNumber);

    // Set interval to set state to DB:
    // setInterval(() => {
    //   if(!this.pauseMACEComms) {
    //       this.connectedVehicles = this.vehicleDB;
    //   }
    // }, 1500);

  }

  parseTCPClientData = (jsonData: TCPReturnType) => {
    // let stateCopy = deepcopy(this.connectedVehicles);
    let stateCopy = deepcopy(this.vehicleDB);
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

      this.vehicleDB = stateCopy;
    }
    else if(jsonData.dataType === 'GlobalOrigin') {
      let jsonOrigin = jsonData as TCPOriginType;
      let origin = {lat: jsonOrigin.lat, lng: jsonOrigin.lng, alt: jsonOrigin.alt};
      let settings = deepcopy(this.environmentSettings);
      settings.gridSpacing = jsonOrigin.gridSpacing;
      this.globalOrigin = origin;
      this.environmentSettings = settings;
    }
    else if(jsonData.dataType === 'EnvironmentBoundary') {
      let jsonBoundary = jsonData as TCPEnvironmentBoundaryType;
      this.environmentBoundary = jsonBoundary.environmentBoundary;
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
            Object.keys(this.vehicleDB).length === 1)
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
          // let stateCopy = deepcopy(this.connectedVehicles);
          let stateCopy = deepcopy(this.vehicleDB);
          // console.log(Object.keys(stateCopy).length);
          stateCopy[vehicleMission.vehicleID].setVehicleMission(vehicleMission);
          this.vehicleDB = stateCopy;
        }
        else if(jsonData.dataType === 'VehicleHome') {
          let vehicleHome = jsonData as (TCPReturnType & MissionItemType);
          // let stateCopy = deepcopy(this.connectedVehicles);
          let stateCopy = deepcopy(this.vehicleDB);
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
            this.vehicleDB = stateCopy;
          }
        }
        else if(jsonData.dataType === 'SensorFootprint') {
          let jsonFootprint = jsonData as TCPSensorFootprintType;
          stateCopy[jsonFootprint.vehicleID].sensorFootprint = jsonFootprint.sensorFootprint;
          this.vehicleDB = stateCopy;
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
    }
  }

  makeTCPRequest = (vehicleID: number, tcpCommand: string, vehicleCommand: string) => {
    // Log message:
    // this.logger.info("{TCP Command: " + tcpCommand + "}  {Vehicle Command: " + vehicleCommand + "}");

    let socket = new net.Socket();
    this.setupTCPClient(socket);
    socket.connect(this.MACEconfig.config.MACEComms.sendPortNumber, this.MACEconfig.config.MACEComms.ipAddress, function() {
      // console.log('Connected to: ' + this.state.tcpHost + ':' + this.state.tcpPort);

      // If vehicle ID == 0, loop over every vehicle ID and make a request:
      if(vehicleID !== 0 || tcpCommand === "GET_CONNECTED_VEHICLES") {
        let tcpRequest = {
          tcpCommand: tcpCommand,
          vehicleID: vehicleID,
          vehicleCommand: vehicleCommand
        };
        socket.write(JSON.stringify(tcpRequest));
        socket.end();
      }
      else if(vehicleID === 0) {
        // for(let vehicle in this.connectedVehicles) {
        for(let vehicle in this.vehicleDB) {
          let tcpRequest = {
            tcpCommand: tcpCommand,
            vehicleID: vehicle,
            vehicleCommand: vehicleCommand
          };
          socket.write(JSON.stringify(tcpRequest));
          socket.end();
        }
      }

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

        if(this.MACEConnected === false) {
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
        console.log('Socket ' + err);
        let str = err+"";
        if(str.indexOf("ECONNREFUSED") > 0){
          // this.handleClearGUI();
          // let title = '';
          // let level = 'error'
          // this.showNotification(title, 'Lost connection to MACE. ', level, 'tc', 'Got it');
        }
        socket.destroy();
    }.bind(this));
  }

  handleClearGUI = () => {
    this.vehicleDB = {};
    // this.connectedVehicles = {};
    this.state.selectedVehicleID = "0";
    this.MACEConnected = false;
  }

  handleAircraftCommand = (id: string, tcpCommand: string, vehicleCommand: string) => {
    console.log(tcpCommand);
    this.makeTCPRequest(parseInt(id), tcpCommand, vehicleCommand);
  }

  handleDrawerAction = (action: string) => {
    if(action === "MACEConfig"){
      this.state.showMessagesMenu = false;
      this.state.showConfigDialog = true;
      this.state.showTakeoffDialog = false;
      this.state.showSaveTakeoff = false;
      this.state.openDrawer = false;
      this.pauseMACEComms = true;
    }
    else if(action === "Messages"){
      this.state.showMessagesMenu = true;
      this.state.showConfigDialog = false;
      this.state.showTakeoffDialog = false;
      this.state.showSaveTakeoff = false;
      this.state.openDrawer = false;
      this.pauseMACEComms = false;
    }
    else if(action === "Takeoff"){
      this.state.showMessagesMenu = false;
      this.state.showConfigDialog = false;
      this.state.showTakeoffDialog = true;
      this.state.showSaveTakeoff = true;
      this.state.openDrawer = false;
      this.pauseMACEComms = true;
    }
    else if(action === "TestButton1") {
      this.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION1", "");
    }
    else if(action === "TestButton2") {
      this.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION2", "");
    }
    else if(action === "EditEnvironment") {
      // Ask for global origin:
      this.makeTCPRequest(0, "GET_GLOBAL_ORIGIN", "");

      this.state.showDraw = true;
      this.state.openDrawer = false;
    }
  }

  handleSaveVehicleHome = (vehicleID: string, vehicleHome: PositionType) => {
    this.handleAircraftCommand(vehicleID, "SET_VEHICLE_HOME", JSON.stringify(vehicleHome));
  }

  handleSaveGlobalOrigin = (globalOrigin: PositionType) => {
    this.handleAircraftCommand("0", "SET_GLOBAL_ORIGIN", JSON.stringify(globalOrigin));
    this.globalOrigin = globalOrigin;
  }

  contextSetHome = () => {
    this.state.showEditVehicleHomeDialog = true;
    this.state.allowVehicleSelect = true;
    this.state.showEditGlobalHomeDialog = false;
    this.state.showTakeoffDialog = false;
    this.state.useContext = true;
    this.pauseMACEComms = true;
  }

  contextSetGlobal = () => {
    this.state.showEditGlobalHomeDialog = true;
    this.state.allowVehicleSelect = false;
    this.state.showEditVehicleHomeDialog = false;
    this.state.useContext = true;
    this.pauseMACEComms = true;
  }

  contextSetTakeoff = () => {
    this.state.showEditVehicleHomeDialog = false;
    this.state.allowVehicleSelect = false;
    this.state.showEditGlobalHomeDialog = false;
    this.state.showTakeoffDialog = true;
    this.state.useContext = true;
    this.pauseMACEComms = true;
  }

  contextGoHere = () => {
    this.state.showEditGlobalHomeDialog = false;
    this.state.allowVehicleSelect = false;
    this.state.showEditVehicleHomeDialog = false;
    this.state.useContext = true;
    let goHere = {
      lat: this.state.contextAnchor.latlng.lat,
      lon: this.state.contextAnchor.latlng.lng
    };
    this.handleAircraftCommand(this.state.selectedVehicleID, "SET_GO_HERE", JSON.stringify(goHere));
  }

  handleSelectedAircraftUpdate = (id: string) => {
    // let stateCopy = deepcopy(this.connectedVehicles);
    let stateCopy = deepcopy(this.vehicleDB);
    let selectedID = "0";
    // Object.keys(this.connectedVehicles).map((key: string) => {
    Object.keys(this.vehicleDB).map((key: string) => {
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
    // this.connectedVehicles = stateCopy;
    this.state.selectedVehicleID = selectedID;
  }

  handleSaveMessagingPreferences = (preferences: MessagePreferencesType) => {
    this.messagePreferences = preferences;
  }

  handleSaveMACEConfig = (config: ConfigSettingsType, reload: boolean = false) => {
    this.MACEconfig = config;
    if(reload) {
      this.setupTCPServer();
    }
    // Reset interval and start requests again:
    if(this.getVehiclesInterval) {
      clearInterval(this.getVehiclesInterval);
      this.getVehiclesInterval = setInterval(() => {
        this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
      }, this.state.getConnectedVehiclesTimeout);
    }
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
    let MACEconfig = this.MACEconfig;
    MACEconfig.config.GUIInit.mapCenter = {lat: e.target.getCenter().lat, lng: e.target.getCenter().lng, alt: 0};
    MACEconfig.config.GUIInit.mapZoom = e.target.getZoom();
    this.MACEconfig = MACEconfig;
  }

  handleSyncAll = () => {
    this.makeTCPRequest(0, "GET_ENVIRONMENT_BOUNDARY", "");

    this.makeTCPRequest(0, "ISSUE_COMMAND", "FORCE_DATA_SYNC");
  }

  handleAddPolygonPt = (e: L.LeafletMouseEvent) => {
    if(this.showDraw) {
      let tmpPts = this.drawPolygonPts;

      // Make sure the new point is not causing an intersection with the existing polygon:
      let intersection = false;
      if(this.drawPolygonPts.length > 2) {
        let prevPt = this.drawPolygonPts[this.drawPolygonPts.length-1];
        let newLine = turf.lineString([[prevPt.lat, prevPt.lng], [e.latlng.lat, e.latlng.lng]]);
        for(let i = 1; i < this.drawPolygonPts.length-1; i++) {
          let pt1 = [this.drawPolygonPts[i-1].lat, this.drawPolygonPts[i-1].lng];
          let pt2 = [this.drawPolygonPts[i].lat, this.drawPolygonPts[i].lng];
          let tmpLine = turf.lineString([pt1, pt2]);
          let intersects = turf.lineIntersect(tmpLine, newLine);

          if(intersects.features[0]) {
            intersection = true;
          }
        }
      }

      if(!intersection) {
        tmpPts.push({lat: e.latlng.lat, lng: e.latlng.lng, alt: 0});
        this.drawPolygonPts = tmpPts;
        this.updateGrid();
      }
      else {
        // let title = 'Draw boundary';
        // let level = 'warning';
        // this.showNotification(title, 'Segment cannot intersect boundary.', level, 'tc', 'Got it');
      }
    }
  }

  handleDeleteLastPolygonPt = () => {
    let tmpPts = this.drawPolygonPts;
    tmpPts.pop();
    this.drawPolygonPts = tmpPts;
    this.updateGrid();
  }

  handleDisableDraw = () => {
    this.showDraw = false;
    this.drawPolygonPts = [];
    this.gridPts = {inPoly: [], trimmedPts: []};
    this.pauseMACEComms = false;
  }

  handleSubmitBoundary = () => {
    // TODO:
    //  1) Send environment boundary to MACE

    // TODO: Send to MACE:

    if(this.drawPolygonPts.length > 2) {
      this.showDraw = false;
      this.drawPolygonPts = [];
      this.makeTCPRequest(0, "SET_ENVIRONMENT_VERTICES", JSON.stringify({boundary: this.drawPolygonPts}));
    }
    else {
      // let title = 'Draw boundary';
      // let level = 'info';
      // this.showNotification(title, 'Boundary must have 3 or more vertices to be valid', level, 'tc', 'Got it');
    }
  }

  handleClearPts = () => {
    this.drawPolygonPts = [];
    this.gridPts = {inPoly: [], trimmedPts: []};
  }

  handleChangeGridSpacing = (val: number) => {
    let settings = deepcopy(this.environmentSettings);
    settings.gridSpacing = val;
    this.environmentSettings = settings;
    this.updateGrid();
  }

  updateGrid = () => {
    let coordinatesArr: any = [];
    this.drawPolygonPts.forEach(function(coord: PositionType) {
      coordinatesArr.push([coord.lat, coord.lng]);
    });

    // let geoJsonData: GeoJSON.GeoJsonObject = {"type": "Feature", "properties": {},
    //   "geometry": {
    //     "type": "Polygon",
    //     "coordinates": [
    //       coordinatesArr
    //     ]
    //   }
    // };


    let geoJsonData: GeoJSON.GeoJsonObject = {
      type: "Feature",
      bbox: coordinatesArr
    };

    let geoJsonLayer = L.geoJSON(geoJsonData);
    let bounds: L.LatLngBounds = geoJsonLayer.getBounds();
    let boundingBox: PositionType[] = [];
    boundingBox.push({lat: bounds.getSouthWest().lng, lng: bounds.getSouthWest().lat, alt: 0}); // Bottom Left
    boundingBox.push({lat: bounds.getSouthWest().lng, lng: bounds.getNorthEast().lat, alt: 0}); // Bottom Right
    boundingBox.push({lat: bounds.getNorthEast().lng, lng: bounds.getNorthEast().lat, alt: 0}); // Top Right
    boundingBox.push({lat: bounds.getNorthEast().lng, lng: bounds.getSouthWest().lat, alt: 0}); // Top Left
    this.envBoundingBox = boundingBox;

    // Calculate grid lines based on global origin:
    this.calculateGridPts(boundingBox);
  }

  calculateGridPts = (boundingBox: PositionType[]) => {
    // Only if lat/lng are not at the origin and the grid spacing is greater than 0
    if(this.globalOrigin.lat !== 0 && this.globalOrigin.lng !== 0) {
      if(this.environmentSettings.gridSpacing > 0) {
        let bottomLeft = new L.LatLng(boundingBox[0].lat, boundingBox[0].lng);
        let bottomRight = new L.LatLng(boundingBox[1].lat, boundingBox[1].lng);
        // let topRight = new L.LatLng(boundingBox[2].lat, boundingBox[2].lng);
        let topLeft = new L.LatLng(boundingBox[3].lat, boundingBox[3].lng);
        let horizDistance = geometryHelper.length([bottomLeft, bottomRight]); // distance between bottom two points
        let vertDistance = geometryHelper.length([bottomLeft, topLeft]); // distance between two left points

        let distanceToNextPt = this.environmentSettings.gridSpacing;
        let prevPt = bottomLeft;
        let tmpGridPts: L.LatLng[] = [];
        let tmpTrimmedPts: L.LatLng[] = [];
        let numXPts = Math.round(horizDistance/distanceToNextPt);
        let numYPts = Math.round(vertDistance/distanceToNextPt);
        for(let i = 0; i <= numYPts; i++) {
          // Add previous point to the array:
          if(this.isPtInPoly(prevPt, this.drawPolygonPts)) {
            tmpGridPts.push(prevPt);
          }
          else {
            tmpTrimmedPts.push(prevPt);
          }
          let tmpNewPt = prevPt;
          for(let j = 0; j <= numXPts; j++) {
            // Move East to the next point and add to the map:
            tmpNewPt = geometryHelper.destination(tmpNewPt, 90, distanceToNextPt);;

            // Check if in the polygon or not:
            if(this.isPtInPoly(tmpNewPt, this.drawPolygonPts)) {
              tmpGridPts.push(tmpNewPt);
            }
            else {
              tmpTrimmedPts.push(tmpNewPt);
            }
          }

          // Move North to the next point:
          prevPt = geometryHelper.destination(prevPt, 0, distanceToNextPt);;
        }

        // Set the grid points to display:
        let pts = {inPoly: tmpGridPts, trimmedPts: tmpTrimmedPts};
        this.gridPts = pts;
      }
    }
  }


  isPtInPoly = (marker: L.LatLng, boundary: PositionType[]): boolean => {
    let polyPoints: L.LatLng[] = [];
    for(let i = 0; i < boundary.length; i++) {
      polyPoints.push(new L.LatLng(boundary[i].lat, boundary[i].lng));
    }
    var x = marker.lat;
    let y = marker.lng;

    var inside = false;
    for (var i = 0, j = polyPoints.length - 1; i < polyPoints.length; j = i++) {
      var xi = polyPoints[i].lat, yi = polyPoints[i].lng;
      var xj = polyPoints[j].lat, yj = polyPoints[j].lng;

      var intersect = ((yi > y) != (yj > y))
          && (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
      if (intersect) inside = !inside;
    }

    return inside;
  };

  saveEnvironmentSettings = (settings: EnvironmentSettingsType) => {
    if(settings.gridSpacing >= settings.minSliderVal && settings.gridSpacing <= settings.maxSliderVal) {
      console.log("Settings: " + JSON.stringify(settings));
      this.environmentSettings = settings;
      this.updateGrid();
    }
    else {
      // let title = 'Environment settings';
      // let level = 'info';
      // this.showNotification(title, 'Grid spacing must be within the minimum and maximum values.', level, 'tc', 'Got it');
    }
  }

  handleSaveTakeoff = (takeoffAlt: string) => {
    let MACEconfig = this.MACEconfig;
    MACEconfig.config.VehicleSettings.defaultTakeoffAlt = parseFloat(takeoffAlt);
    this.MACEconfig = MACEconfig;
  }
}