import * as React from 'react';

var NotificationSystem = require('react-notification-system');
import { Map, TileLayer  } from 'react-leaflet';
import { ConnectedVehiclesContainer } from './ConnectedVehiclesContainer';
import { VehicleWarningsContainer, VehicleWarning } from './VehicleWarningsContainer';
import { VehicleType } from '../components/VehicleHUD'
import { VehicleCommandsContainer } from './VehicleCommandsContainer';
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
  connectedVehicles?: VehicleType[],
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
      connectedVehicles: [],
      vehicleWarnings: []
    }
  }

  componentDidMount(){
    this.leafletMap = this.refs.map;
    this.notificationSystem = this.refs.notificationSystem;

    this.setupTCPClient();
    this.makeTCPRequest("GET_ATTITUDE", 1);
    // this.startPythonServerAjax();
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
          console.log(jsonData["roll"]);
          // Close the client socket completely
          this.state.tcpClient.destroy();        
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
