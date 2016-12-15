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
  alt: number
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
  constructor() {
    super();  

    this.state = {
      tcpClient: new net.Socket(),
      tcpHost: '127.0.0.1',
      tcpPort: 1234,
      maxZoom: 20,
      initialZoom: 18,
      mapCenter: [37.889231, -76.810302],
      connectedVehicles: {
        "1": {
          position: {
              lat: 1,
              lon: 1,
              alt: 1
          },
          attitude: {
              roll: 1,
              pitch: 1,
              yaw: 1
          }
        },
        "2": {
          position: {
              lat: 1,
              lon: 1,
              alt: 1
          },
          attitude: {
              roll: 1,
              pitch: 1,
              yaw: 1
          }
        }
      },
      vehicleWarnings: []
    }
  }

  componentDidMount(){
    this.leafletMap = this.refs.map;
    this.notificationSystem = this.refs.notificationSystem;

    this.setupTCPClient();
    this.makeTCPRequest("GET_CONNECTED_VEHICLES", 1);

    // this.startPythonServerAjax();
  }

  testTCPRequest = () => {
    this.makeTCPRequest("GET_ATTITUDE", 1);
  }

  makeTCPRequest = (command: string, vehicleID: number) => {
    this.state.tcpClient.connect(this.state.tcpPort, this.state.tcpHost, function() {
      console.log('Connected to: ' + this.state.tcpHost + ':' + this.state.tcpPort);
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
          console.log('DATA: ' + data);
          let jsonData = JSON.parse(data);
          this.parseTCPResponse(jsonData);
          // Close the client socket completely
          // this.state.tcpClient.destroy();        
      }.bind(this));

      // Add a 'close' event handler for the client socket
      this.state.tcpClient.on('close', function() {
          console.log('Connection closed');
      }.bind(this));

      // Add an 'error' event handler
      this.state.tcpClient.on('error', function(err: any) {
          console.log('Error: ' + err);
      }.bind(this));
  }

  parseTCPResponse = (jsonData: TCPReturnType) => {
    if(jsonData.dataType === "ConnectedVehicles"){
      let stateCopy = this.state.connectedVehicles;
      let jsonVehicles = jsonData as ConnectedVehiclesType;
      console.log("In Connected Vehicles response");
      console.log("Test 1: " + this.state.connectedVehicles["1"]);

      // Check if vehicle is already in the map. If so, do nothing. If not, add it:
      for(let i = 0; i < jsonVehicles.connectedVehicles.length; i++){
        if (this.state.connectedVehicles[i.toString()] !== undefined){
          return;
        }
        else {
          stateCopy[jsonVehicles.connectedVehicles[i]] = generateNewVehicle();
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
    if(jsonData.dataType === "VehiclePosition"){
      let vehiclePosition = jsonData as VehiclePositionType;
    }
    if(jsonData.dataType === "VehicleAttitude"){
      let vehicleAttitude = jsonData as VehicleAttitudeType;
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
