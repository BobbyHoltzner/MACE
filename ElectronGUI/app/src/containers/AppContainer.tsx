import * as React from 'react';

import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();

var NotificationSystem = require('react-notification-system');
import { Map, TileLayer, LayerGroup, Marker, Polyline, Polygon } from 'react-leaflet';
import { ConnectedVehiclesContainer } from './ConnectedVehiclesContainer';
import { VehicleWarningsContainer, VehicleWarning } from './VehicleWarningsContainer';
import { VehicleCommandsContainer } from './VehicleCommandsContainer';
import { AppDrawer } from './AppDrawer';
import AppBar from 'material-ui/AppBar';
import * as colors from 'material-ui/styles/colors';
import IconMenu from 'material-ui/IconMenu';
import MenuItem from 'material-ui/MenuItem';
import IconButton from 'material-ui/IconButton';
import MoreVertIcon from 'material-ui/svg-icons/navigation/more-vert';
import { Vehicle } from '../Vehicle';
import { backgroundColors, opaqueBackgroundColors } from '../util/Colors';
import { VehicleHomeDialog } from '../components/VehicleHomeDialog';
import { GlobalOriginDialog } from '../components/GlobalOriginDialog';
import { MessagesDialog } from '../components/MessagesDialog';
import { ContextMenu } from '../components/ContextMenu';

import * as deepcopy from 'deepcopy';

var injectTapEventPlugin = require("react-tap-event-plugin");
injectTapEventPlugin();
var net = require('net');


type Props = {
}

type State = {
  tcpClient?: any,
  tcpHost?: string,
  tcpPort?: number,
  maxZoom?: number,
  initialZoom?: number,
  mapCenter?: number[],
  connectedVehicles?: {[id: string]: Vehicle}
  vehicleWarnings?: VehicleWarning[]
  selectedVehicleID?: string,
  openDrawer?: boolean,
  tcpSockets?: any[],
  tcpServer?: any,
  allowVehicleSelect?: boolean,
  showEditVehicleHomeDialog?: boolean,
  showEditGlobalHomeDialog?: boolean,
  globalOrigin?: PositionType,
  showContextMenu?: boolean,
  contextAnchor?: L.LeafletMouseEvent,
  useContext?: boolean,
  showMessagesMenu?: boolean,
  messagePreferences?: MessagePreferencesType
}

export default class AppContainer extends React.Component<Props, State> {
  leafletMap: L.Map;
  notificationSystem: NotificationSystem;
  m_AttitudeInterval: number[];
  m_AttitudeTimeout: number;
  m_PositionInterval: number[];
  m_PositionTimeout: number;
  constructor(props: Props) {
    super(props);

    this.m_AttitudeInterval = [];
    this.m_PositionInterval = [];
    this.m_AttitudeTimeout = 1111;
    this.m_PositionTimeout = 1234;

    this.state = {
      tcpClient: new net.Socket(),
      tcpHost: '127.0.0.1',
      tcpPort: 5678,
      maxZoom: 20,
      initialZoom: 18,
      // mapCenter: [37.889231, -76.810302], // Bob's Farm
      mapCenter: [-35.363272, 149.165249], // SITL Default
      // mapCenter: [45.283410, -111.400850], // Big Sky
      connectedVehicles: {},
      vehicleWarnings: [],
      openDrawer: false,
      tcpSockets: [],
      tcpServer: null,
      allowVehicleSelect: true,
      showEditVehicleHomeDialog: false,
      showEditGlobalHomeDialog: false,
      globalOrigin: {lat: 0, lon: 0, alt: 0},
      selectedVehicleID: "0",
      showContextMenu: false,
      contextAnchor: null,
      useContext: false,
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
      }
    }
  }

  componentDidMount(){
    this.leafletMap = this.refs.map;
    this.notificationSystem = this.refs.notificationSystem;
    this.setupTCPServer();

    setInterval(() => {
      this.makeTCPRequest(0, "GET_CONNECTED_VEHICLES", "");
    }, 2000);
  }

  setupTCPServer = () => {
    // Create a TCP socket listener
    this.state.tcpServer = net.Server(function (socket: any) {

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


    // TODO: Allow for user configuration of the port and probably address too
    try{
      this.state.tcpServer.listen(1234);
    }
    catch(e) {
      console.log('Error: ' + e);
    }

    console.log('System listening at http://localhost:1234');
  }


  parseTCPClientData = (jsonData: TCPReturnType) => {
    let stateCopy = deepcopy(this.state.connectedVehicles);

    if(jsonData.dataType === "ConnectedVehicles"){
      let jsonVehicles = jsonData as ConnectedVehiclesType;

      // console.log("Connected vehicles: " + jsonVehicles.connectedVehicles);

      // Check if vehicle is already in the map. If so, do nothing. If not, add it:
      for(let i = 0; i < jsonVehicles.connectedVehicles.length; i++){
        if (stateCopy[jsonVehicles.connectedVehicles[i].toString()] !== undefined){
          return;
        }
        else {
          stateCopy[jsonVehicles.connectedVehicles[i].toString()] = new Vehicle(jsonVehicles.connectedVehicles[i]);
        }
      }

      // Check if we need to remove a vehicle from the state. If we find it, continue. Else, delete it:
      let idArrays: string[] = Object.keys(stateCopy);
      for(let i = 0; i < idArrays.length; i++){
        if(jsonVehicles.connectedVehicles.indexOf(parseInt(idArrays[i])) >= 0) {
          continue;
        }
        else {
          delete stateCopy[idArrays[i]];
        }
      }

      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === "VehiclePosition"){
      let vehiclePosition = jsonData as TCPPositionType;

      stateCopy[vehiclePosition.vehicleID].position.lat = vehiclePosition.lat;
      stateCopy[vehiclePosition.vehicleID].position.lon = vehiclePosition.lon;
      stateCopy[vehiclePosition.vehicleID].position.alt = vehiclePosition.alt;
      stateCopy[vehiclePosition.vehicleID].numSats = vehiclePosition.numSats;
      stateCopy[vehiclePosition.vehicleID].positionFix = vehiclePosition.positionFix;

      stateCopy[vehiclePosition.vehicleID].updateMarkerPosition(vehiclePosition);

      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === "VehicleAttitude"){
      let vehicleAttitude = jsonData as TCPAttitudeType;

      stateCopy[vehicleAttitude.vehicleID].attitude.roll = vehicleAttitude.roll;
      stateCopy[vehicleAttitude.vehicleID].attitude.pitch = vehicleAttitude.pitch;
      stateCopy[vehicleAttitude.vehicleID].attitude.yaw = vehicleAttitude.yaw;

      stateCopy[vehicleAttitude.vehicleID].updateMarkerAttitude(vehicleAttitude);

      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === 'VehicleMission') {
      let vehicleMission = jsonData as TCPMissionType;
      let stateCopy = deepcopy(this.state.connectedVehicles);
      stateCopy[vehicleMission.vehicleID].setVehicleMission(vehicleMission);
      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === 'VehicleHome') {
      let vehicleHome = jsonData as (TCPReturnType & MissionItemType);
      let stateCopy = deepcopy(this.state.connectedVehicles);
      let tmpHome = {
        lat: vehicleHome.lat,
        lon: vehicleHome.lon,
        alt: vehicleHome.alt
      }
      stateCopy[vehicleHome.vehicleID].updateHomePosition(tmpHome);
      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === 'VehicleFuel') {
      let vehicleFuel = jsonData as TCPFuelType;

      stateCopy[vehicleFuel.vehicleID].fuel.batteryRemaining = vehicleFuel.batteryRemaining;
      stateCopy[vehicleFuel.vehicleID].fuel.batteryCurrent = vehicleFuel.batteryCurrent;
      stateCopy[vehicleFuel.vehicleID].fuel.batteryVoltage = vehicleFuel.batteryVoltage;

      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === 'VehicleMode') {
      let vehicleMode = jsonData as TCPModeType;

      stateCopy[vehicleMode.vehicleID].isArmed = vehicleMode.isArmed;
      stateCopy[vehicleMode.vehicleID].vehicleMode = vehicleMode.vehicleMode;
      // TODO: vehicle type (i.e. quad, fixed, etc.)

      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === 'VehicleText') {
      let vehicleText = jsonData as TCPTextType;
      let showMessage = false;
      let title = '';
      let level = 'info';
      if(vehicleText.severity === "STATUS_EMERGENCY") {
        title = 'EMERGENCY -- Vehicle ' + vehicleText.vehicleID;
        level = 'error';
        showMessage = this.state.messagePreferences.emergency;
      }
      if(vehicleText.severity === "STATUS_ALERT") {
        title = 'Alert -- Vehicle ' + vehicleText.vehicleID;
        level = 'warning';
        showMessage = this.state.messagePreferences.alert;
      }
      if(vehicleText.severity === "STATUS_CRITICAL") {
        title = 'CRITICAL -- Vehicle ' + vehicleText.vehicleID;
        level = 'error';
        showMessage = this.state.messagePreferences.critical;
      }
      if(vehicleText.severity === "STATUS_ERROR") {
        title = 'ERROR -- Vehicle ' + vehicleText.vehicleID;
        level = 'error';
        showMessage = this.state.messagePreferences.error;
      }
      if(vehicleText.severity === "STATUS_WARNING") {
        title = 'Warning -- Vehicle ' + vehicleText.vehicleID;
        level = 'warning';
        showMessage = this.state.messagePreferences.warning;
      }
      if(vehicleText.severity === "STATUS_NOTICE") {
        title = 'Notice -- Vehicle ' + vehicleText.vehicleID;
        level = 'success';
        showMessage = this.state.messagePreferences.notice;
      }
      if(vehicleText.severity === "STATUS_INFO") {
        title = 'Info -- Vehicle ' + vehicleText.vehicleID;
        level = 'info';
        showMessage = this.state.messagePreferences.info;
      }
      if(vehicleText.severity === "STATUS_DEBUG") {
        title = 'Debug -- Vehicle ' + vehicleText.vehicleID;
        level = 'info';
        showMessage = this.state.messagePreferences.debug;
      }

      if(showMessage) {
        this.showNotification(title, vehicleText.text, level, 'bl', 'Got it');
      }
    }
    else if(jsonData.dataType === 'GlobalOrigin') {
      let jsonOrigin = jsonData as TCPPositionType;
      let origin = {lat: jsonOrigin.lat, lon: jsonOrigin.lon, alt: jsonOrigin.alt};
      this.setState({globalOrigin: origin})
    }
    else if(jsonData.dataType === 'SensorFootprint') {
      let jsonFootprint = jsonData as TCPSensorFootprintType;
      stateCopy[jsonFootprint.vehicleID].sensorFootprint = jsonFootprint.sensorFootprint;
      this.setState({connectedVehicles: stateCopy});
    }
    else if(jsonData.dataType === 'CurrentMissionItem') {
      let jsonMissionItem = jsonData as TCPCurrentMissionItemType;

      console.log("Current mission item: " + jsonMissionItem.missionItemIndex);
      stateCopy[jsonMissionItem.vehicleID].updateCurrentMissionItem(jsonMissionItem.missionItemIndex);
      this.setState({connectedVehicles: stateCopy});
    }
  }


  makeTCPRequest = (vehicleID: number, tcpCommand: string, vehicleCommand: string) => {
    let socket = new net.Socket();
    this.setupTCPClient(socket);
    // this.state.tcpClient.connect(this.state.tcpPort, this.state.tcpHost, function() {
    socket.connect(this.state.tcpPort, this.state.tcpHost, function() {
      // console.log('Connected to: ' + this.state.tcpHost + ':' + this.state.tcpPort);
      let tcpRequest = {
        tcpCommand: tcpCommand,
        vehicleID: vehicleID,
        vehicleCommand: vehicleCommand
      };
      socket.write(JSON.stringify(tcpRequest));
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
        socket.destroy();
    }.bind(this));

    // Add a 'close' event handler for the client socket
    socket.on('close', function() {
        // console.log('Connection closed');
        socket.destroy();
    }.bind(this));

    // Add an 'error' event handler
    socket.on('error', function(err: any) {
        console.log('Error: ' + err);
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

  handleAircraftCommand = (id: string, tcpCommand: string, vehicleCommand: string) => {
    console.log(tcpCommand);
    this.makeTCPRequest(parseInt(id), tcpCommand, vehicleCommand);
  }

  handleDrawerAction = (action: string) => {
    if(action === "Settings"){
      this.setState({showMessagesMenu: true, openDrawer: false});
    }
    else if(action === "TestButton1") {
      this.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION1", "");
    }
    else if(action === "TestButton2") {
      this.makeTCPRequest(parseInt(this.state.selectedVehicleID), "TEST_FUNCTION2", "");
    }
  }

  onOpenVehicleEdit = (vehicleID: string) => {
    // If we are passing in a vehicle ID, don't allow the dropdown to be selectable on the edit window as we are editing a specific vehicle:
    this.handleSelectedAircraftUpdate(vehicleID);
    this.setState({allowVehicleSelect: vehicleID ? false : true, showEditVehicleHomeDialog: true});
  }

  handleSaveVehicleHome = (vehicleID: string, vehicleHome: PositionType) => {
    console.log("Vehicle ID: " + vehicleID);
    this.handleAircraftCommand(vehicleID, "SET_VEHICLE_HOME", JSON.stringify(vehicleHome));
    let tmpHome: any = {
      lat: vehicleHome.lat,
      lon: vehicleHome.lon,
      alt: vehicleHome.alt,
    }
    if(this.state.connectedVehicles[vehicleID]) {
      this.state.connectedVehicles[vehicleID].updateHomePosition(tmpHome);
    }
    else {
      console.log("No vehicle with ID: " + vehicleID);
    }
  }

  handleSaveGlobalOrigin = (globalOrigin: PositionType) => {
    this.handleAircraftCommand("0", "SET_GLOBAL_ORIGIN", JSON.stringify(globalOrigin));
    this.setState({globalOrigin: globalOrigin});
  }

  triggerContextMenu = (event: L.LeafletMouseEvent) => {
    this.setState({contextAnchor: event, showContextMenu: !this.state.showContextMenu});
  }

  contextSetHome = () => {
    this.setState({
      showContextMenu: false,
      showEditVehicleHomeDialog: true,
      allowVehicleSelect: true,
      showEditGlobalHomeDialog: false,
      useContext: true
    });
  }

  contextSetGlobal = () => {
    this.setState({
      showContextMenu: false,
      showEditGlobalHomeDialog: true,
      allowVehicleSelect: false,
      showEditVehicleHomeDialog: false,
      useContext: true
    });
  }

  contextGoHere = () => {
    this.setState({
      showContextMenu: false,
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
      }
      else {
        stateCopy[key].isSelected = false;
      }
      stateCopy[key].updateMarkerPosition();
      stateCopy[key].updateHomePosition();
    });

    this.setState({connectedVehicles: stateCopy, selectedVehicleID: selectedID});
  }

  handleMapClick = (e: L.LeafletMouseEvent) => {
    this.setState({showContextMenu: false});
  }

  handleMarkerClick = (e: L.LeafletMouseEvent, vehicleId: string, type: string) => {
    this.handleSelectedAircraftUpdate(vehicleId);
    this.setState({showContextMenu: false});
  }

  handleSaveMessagingPreferences = (preferences: MessagePreferencesType) => {
    this.setState({messagePreferences: preferences});
  }

  render() {

    const width = window.screen.width;
    const height = window.screen.height;
    const parentStyle = {height: height + 'px', width: width + 'px'};
    const mapStyle = { top: 0, left: 0, height: height + 'px', width: width + 'px' };

    const MoreVertMenu = () => (
      <IconMenu
        iconButtonElement={<IconButton><MoreVertIcon color="white" /></IconButton>}
        anchorOrigin={{horizontal: 'right', vertical: 'top'}}
        targetOrigin={{horizontal: 'right', vertical: 'top'}}
      >
        <MenuItem onClick={() => console.log("RTA")} primaryText="RTA Parameters" />
        <MenuItem onClick={() => console.log("Path Planning")} primaryText="Path Planning Parameters" />
      </IconMenu>
    );

    const globalOriginMarker = {
      position: new L.LatLng(this.state.globalOrigin.lat, this.state.globalOrigin.lon),
      icon: new L.Icon({
          iconUrl: './images/userlocation_icon.png',
          iconSize: [41, 41], // size of the icon
          iconAnchor: [20, 20], // point of the icon which will correspond to marker's location
          popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
      })
    };

    return (
        <MuiThemeProvider muiTheme={lightMuiTheme}>
          <div style={parentStyle}>


            <AppBar
                title="MACE"
                style={{backgroundColor: colors.orange700}}
                onLeftIconButtonTouchTap={() => this.setState({openDrawer: !this.state.openDrawer})}
                iconElementRight={<MoreVertMenu />}
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
              handleOpenVehicleEdit={this.onOpenVehicleEdit}
              selectedVehicleID={this.state.selectedVehicleID}
            />

            <VehicleCommandsContainer
              connectedVehicles={this.state.connectedVehicles}
              onSelectedAircraftChange={this.handleSelectedAircraftUpdate}
              onAircraftCommand={this.handleAircraftCommand}
              selectedAircraftID={this.state.selectedVehicleID}
            />

            <VehicleWarningsContainer
              vehicleWarnings={this.state.vehicleWarnings}
            />

            <VehicleHomeDialog
              open={this.state.showEditVehicleHomeDialog}
              handleClose={() => this.setState({showEditVehicleHomeDialog: false})}
              vehicles={this.state.connectedVehicles}
              selectedVehicleID={this.state.selectedVehicleID}
              handleSave={this.handleSaveVehicleHome}
              contextAnchor={this.state.contextAnchor}
              useContext={this.state.useContext}
              allowVehicleSelect={this.state.allowVehicleSelect}
            />

            <GlobalOriginDialog
              open={this.state.showEditGlobalHomeDialog}
              handleClose={() => this.setState({showEditGlobalHomeDialog: false})}
              onGlobalHomeCommand={this.handleAircraftCommand}
              globalOrigin={this.state.globalOrigin}
              handleSave={this.handleSaveGlobalOrigin}
              contextAnchor={this.state.contextAnchor}
              useContext={this.state.useContext}
            />

            <MessagesDialog
              open={this.state.showMessagesMenu}
              handleClose={() => this.setState({showMessagesMenu: false})}
              handleSave={this.handleSaveMessagingPreferences}
              preferences={this.state.messagePreferences}
            />

            {this.state.showContextMenu &&
              <ContextMenu
                menuAnchor={this.state.contextAnchor}
                handleClose={() => this.setState({showContextMenu: false})}
                handleSetHome={this.contextSetHome}
                handleSetGlobal={this.contextSetGlobal}
                handleGoHere={this.contextGoHere}
              />
            }

            <Map ref="map" center={this.state.mapCenter} zoom={this.state.initialZoom} style={mapStyle} zoomControl={false} onContextmenu={this.triggerContextMenu} onDrag={() => this.setState({showContextMenu: false})} >
                {/* <TileLayer url='http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' />  */}
                <TileLayer url='http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}' maxZoom={this.state.maxZoom} subdomains={['mt0','mt1','mt2','mt3']} />

                <LayerGroup>

                  {/* Aircraft Icons */}
                  {Object.keys(this.state.connectedVehicles).map((key: string) => {
                    return (
                      <Marker onclick={(e: L.LeafletMouseEvent) => this.handleMarkerClick(e, key, "vehicle")} key={key} position={this.state.connectedVehicles[key].vehicleMarker.latLon} icon={this.state.connectedVehicles[key].vehicleMarker.icon} title={key}>
                      {/*
                        <Popup open={true}>
                        </Popup>
                      */}
                      </Marker>
                    );
                  })}

                  {/* Home Icons */}
                  {Object.keys(this.state.connectedVehicles).map((key: string) => {
                    return (
                      <Marker onclick={(e: L.LeafletMouseEvent) => this.handleMarkerClick(e, key, "home")} key={key} position={this.state.connectedVehicles[key].homePosition.latLon} icon={this.state.connectedVehicles[key].homePosition.icon} title={key}>
                      {/*
                        <Popup open={true}>
                          <span>Selected</span>
                        </Popup>
                      */}
                      </Marker>
                    );
                  })}

                  {/* Global Origin */}
                  <Marker position={globalOriginMarker.position} icon={globalOriginMarker.icon}>
                  {/*
                    <Popup open={true}>
                      <span>Selected</span>
                    </Popup>
                  */}
                  </Marker>

                  {/* Mission Paths */}
                  {Object.keys(this.state.connectedVehicles).map((key: string) => {
                    return (
                      <Polyline key={key} positions={this.state.connectedVehicles[key].vehicleMission.latLons} color={this.state.selectedVehicleID === key ? backgroundColors[parseInt(key)] : opaqueBackgroundColors[parseInt(key)]} />
                    );
                  })}

                  {/* Mission Markers */}
                  {Object.keys(this.state.connectedVehicles).map((key: string) => {
                    let markers: JSX.Element[] = [];
                    for(let i = 0; i < this.state.connectedVehicles[key].vehicleMission.latLons.length; i++){
                      markers.push(<Marker key={i} position={this.state.connectedVehicles[key].vehicleMission.latLons[i]} icon={this.state.connectedVehicles[key].vehicleMission.icons[i]} title={key} />);
                    }
                    return (
                      markers
                    );
                  })}

                  {/* Sensor Footprint */}
                  {Object.keys(this.state.connectedVehicles).map((key: string) => {
                    return (
                      <Polygon key={key} positions={this.state.connectedVehicles[key].sensorFootprint} color={this.state.selectedVehicleID === key ? backgroundColors[parseInt(key)] : opaqueBackgroundColors[parseInt(key)]} fillColor={colors.amber500} />
                    );
                  })}


                </LayerGroup>
            </Map>

            <div>
              <NotificationSystem ref="notificationSystem" />
            </div>

          </div>
        </MuiThemeProvider>
    );
  }
}
