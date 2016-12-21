import * as React from 'react';

var NotificationSystem = require('react-notification-system');
import { Map, TileLayer } from 'react-leaflet';
import { ConnectedVehiclesContainer } from './ConnectedVehiclesContainer';
import { VehicleWarningsContainer, VehicleWarning } from './VehicleWarningsContainer';
import { VehicleMapType, generateNewVehicle } from '../components/VehicleHUD'
import { VehicleCommandsContainer } from './VehicleCommandsContainer';

import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();
import * as deepcopy from 'deepcopy';

var injectTapEventPlugin = require("react-tap-event-plugin");
injectTapEventPlugin();
var net = require('net');

type TCPDescriptorType = {
  dataType: string,
  vehicleID: number
}

type ConnectedVehiclesType = TCPDescriptorType & {
  connectedVehicles: number[]
}

type VehiclePositionType = TCPDescriptorType & {
  lat: number,
  lon: number,
  alt: number,
  positionFix: number,
  numSats: number
}

type VehicleAttitudeType = TCPDescriptorType & {
  roll: number,
  pitch: number,
  yaw: number
}

type TCPReturnType = ConnectedVehiclesType | VehiclePositionType | VehicleAttitudeType;


type Props = {
}

type State = {
  tcpClient?: any,
  tcpHost?: string,
  tcpPort?: number,
  maxZoom?: number,
  initialZoom?: number,
  mapCenter?: number[],
  connectedVehicles?: VehicleMapType,
  vehicleWarnings?: VehicleWarning[]
}

export default class AppContainer extends React.Component<Props, State> {    
  leafletMap: L.Map;
  notificationSystem: NotificationSystem;
  m_AttitudeInterval: number[];
  m_AttitudeTimeout: number;
  m_PositionInterval: number[];
  m_PositionTimeout: number;
  constructor() {
    super();  

    this.m_AttitudeInterval = [];
    this.m_PositionInterval = [];
    this.m_AttitudeTimeout = 1111;
    this.m_PositionTimeout = 1234;

    this.state = {
      tcpClient: new net.Socket(),
      tcpHost: '127.0.0.1',
      tcpPort: 1234,
      maxZoom: 20,
      initialZoom: 18,
      mapCenter: [37.889231, -76.810302],
      connectedVehicles: {},
      vehicleWarnings: []
    }
  }

  componentDidMount(){
    this.leafletMap = this.refs.map;
    this.notificationSystem = this.refs.notificationSystem;

    this.setupTCPClient();

    setInterval(() => {
      this.makeTCPRequest("GET_CONNECTED_VEHICLES", 1);
    }, 3000);
    // this.testTCPRequest();
  }

  testTCPRequest = () => {
    this.makeTCPRequest("GET_ATTITUDE", 1);
  }

  makeTCPRequest = (command: string, vehicleID: number) => {
    this.state.tcpClient.connect(this.state.tcpPort, this.state.tcpHost, function() {
      // console.log('Connected to: ' + this.state.tcpHost + ':' + this.state.tcpPort);
      let attitudeRequest = {
        command: command,
        vehicleID: vehicleID
      };
      this.state.tcpClient.write(JSON.stringify(attitudeRequest));
    }.bind(this));
  }
  
  setupTCPClient = () => {
      // Add a 'data' event handler for the client socket
      // data is what the server sent to this socket
      this.state.tcpClient.on('data', function(data: any) {        
          // console.log('DATA: ' + data);
          let jsonData = JSON.parse(data);
          this.parseTCPResponse(jsonData);
          // Close the client socket completely
          this.state.tcpClient.destroy();        
      }.bind(this));

      // Add a 'close' event handler for the client socket
      this.state.tcpClient.on('close', function() {
          // console.log('Connection closed');
      }.bind(this));      

      // Add an 'error' event handler
      this.state.tcpClient.on('error', function(err: any) {
          console.log('Error: ' + err);
          this.state.tcpClient.destroy();
      }.bind(this));
  }

  parseTCPResponse = (jsonData: TCPReturnType) => {
    let stateCopy = deepcopy(this.state.connectedVehicles);
    
    if(jsonData.dataType === "ConnectedVehicles"){
      let jsonVehicles = jsonData as ConnectedVehiclesType;      

      console.log("Connected vehicles: " + jsonVehicles.connectedVehicles);

      // Check if vehicle is already in the map. If so, do nothing. If not, add it:
      for(let i = 0; i < jsonVehicles.connectedVehicles.length; i++){
        if (stateCopy[jsonVehicles.connectedVehicles[i].toString()] !== undefined){          
          return;
        }
        else {
          stateCopy[jsonVehicles.connectedVehicles[i].toString()] = generateNewVehicle();
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

      // Set intervals:
      if (Object.keys(stateCopy).length > 0){
        idArrays = Object.keys(stateCopy);
        console.log("ID ARRAY: " + idArrays);
        for(let i = 0; i < idArrays.length; i++){
          if(this.m_AttitudeInterval[i] === undefined){
            this.m_AttitudeInterval[i] = setInterval(() => {
              this.makeTCPRequest("GET_ATTITUDE", parseInt(idArrays[i]));
            }, this.m_AttitudeTimeout + 20 + i);
          }
        }
      }
      if (Object.keys(stateCopy).length > 0){
        idArrays = Object.keys(stateCopy);
        for(let i = 0; i < idArrays.length; i++){
          if(this.m_PositionInterval[i] === undefined){
            this.m_PositionInterval[i] = setInterval(() => {
              this.makeTCPRequest("GET_POSITION", parseInt(idArrays[i]));
            }, this.m_PositionTimeout + 23 + i);
          }
        }
      }

      this.setState({connectedVehicles: stateCopy});
    }

    else if(jsonData.dataType === "VehiclePosition"){
      let vehiclePosition = jsonData as VehiclePositionType;

      let idArrays: string[] = Object.keys(stateCopy);
      for(let i = 0; i < idArrays.length; i++){
        if(parseInt(idArrays[i]) === vehiclePosition.vehicleID) {
          stateCopy[idArrays[i]].position.lat = vehiclePosition.lat;
          stateCopy[idArrays[i]].position.lon = vehiclePosition.lon;
          stateCopy[idArrays[i]].position.alt = vehiclePosition.alt;
          stateCopy[idArrays[i]].position.numSats = vehiclePosition.numSats;
          stateCopy[idArrays[i]].position.positionFix = vehiclePosition.positionFix;
        }
      }

      this.setState({connectedVehicles: stateCopy});
    }

    else if(jsonData.dataType === "VehicleAttitude"){
      let vehicleAttitude = jsonData as VehicleAttitudeType;

      stateCopy[vehicleAttitude.vehicleID].attitude.roll = vehicleAttitude.roll;
      stateCopy[vehicleAttitude.vehicleID].attitude.pitch = vehicleAttitude.pitch;
      stateCopy[vehicleAttitude.vehicleID].attitude.yaw = vehicleAttitude.yaw;

      this.setState({connectedVehicles: stateCopy});
    }
  }


  showNotification = (title: string, message: string, level: string, position: string, label: string) => {
    let notification =     {
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


  render() {

    const width = window.screen.width;
    const height = window.screen.height;
    const parentStyle = {height: height + 'px', width: width + 'px'};
    const mapStyle = { top: 0, left: 0, height: height + 'px', width: width + 'px' };

    // const centerButtonStyle = { width: 100 + '%' };

    return (
        <div style={parentStyle}>

          <ConnectedVehiclesContainer
            connectedVehicles={this.state.connectedVehicles}
           />

          <VehicleCommandsContainer
           />

          <VehicleWarningsContainer
            vehicleWarnings={this.state.vehicleWarnings}
           />

           {/* 
          <MuiThemeProvider muiTheme={lightMuiTheme}>
              <FlatButton style={centerButtonStyle} label="Test Button" onClick={this.testTCPRequest}/>
          </MuiThemeProvider>
          */}
           

          <Map ref="map" center={this.state.mapCenter} zoom={this.state.initialZoom} style={mapStyle} zoomControl={false} >
              {/* <TileLayer url='http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' />  */}
              <TileLayer url='http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}' maxZoom={this.state.maxZoom} subdomains={['mt0','mt1','mt2','mt3']} />
          </Map>

          <div>
            <NotificationSystem ref="notificationSystem" />
          </div>

        </div>
    );
  }
}
