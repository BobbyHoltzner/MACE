// Material Design Default Themes:
import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
const lightMuiTheme = getMuiTheme();
const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
import { Map, Marker, Popup, TileLayer, FeatureGroup, Polyline, LayerGroup  } from 'react-leaflet';
import { EditControl } from "react-leaflet-draw";
import RaisedButton from 'material-ui/RaisedButton';
import {Card, CardTitle, CardText} from 'material-ui/Card';
import Checkbox from 'material-ui/Checkbox';
import {RadioButton, RadioButtonGroup} from 'material-ui/RadioButton';
import Slider from 'material-ui/Slider';
import Toggle from 'material-ui/Toggle';
var PythonShell = require('python-shell');
var UtmConverter = require('utm-converter');
import MercatorConversion from '../helpers/MercatorConversion';
var NotificationSystem = require('react-notification-system');
import DropDownMenu from 'material-ui/DropDownMenu';
import MenuItem from 'material-ui/MenuItem';
import * as $ from 'jquery';
var injectTapEventPlugin = require("react-tap-event-plugin");
injectTapEventPlugin();
// var $ = require('jquery');
// var XMLHttpRequest = require('xmlhttprequest').XMLHttpRequest;

type MarkerType = {
  position: L.LatLng,
  icon: L.Icon,
  comm_port?: string
}

type LayerGroupType = {
  type: string,
  latLons: L.LatLng[]
}

type PathType = {
  waypoints: L.LatLng[]
}

type UTMPropertiesType = {
  utmZone: number,
  isSouthern: boolean
}

type AircraftType = {
  comm_port: string,
  isConnected: boolean
}

type Props = {
}

type State = {
  markers?: MarkerType[],
  boundaryVerts?: LayerGroupType,
  pointsOfInterest?: LayerGroupType[],
  aircraftPaths?: PathType[],
  boundaryPaths?: PathType[],
  utmProperties?: UTMPropertiesType,
  boundaryLayers?: any[],
  hotSpotLayers?: any[],
  screenWidth?: number,
  screenHeight?: number,
  pSliderVal?: number,
  gridSliderVal?: number,
  disableRegenerate?: boolean,
  pathDirection?: string,
  aircraftPorts?: AircraftType[],
  openPorts?: string[],
  showAircraftCommands?: boolean,
  selectedAircraftPort?: number,
  aircraftMarkers?: MarkerType[],
  sendToAllAircraft?: boolean,
  useTestPoints?: boolean,
  getAircraftLocationsStarted?: boolean
}

export default class AppContainer extends React.Component<Props, State> {
  utmConverter: UtmConverter;
  mercatorConverter: MercatorConversion;
  leafletMap: L.Map;
  notificationSystem: NotificationSystem;
  backgroundColors: string[];
  constructor() {
    super();

    this.backgroundColors = ['rgba(0,0,255,0.4)', 'rgba(255,0,0,0.4)', 'rgba(0,0,0,0.4)', 'rgba(0,255,0,0.4)', 'rgba(255,255,0,0.4)', 'rgba(255,153,0,0.4)'];

    this.utmConverter = new UtmConverter();

    this.mercatorConverter = new MercatorConversion();

    this.state = {
      markers: [],
      aircraftMarkers: [],
      boundaryVerts: null,
      aircraftPaths: [],
      boundaryPaths: [],
      utmProperties: {
        utmZone: 55, // Default to bobs farm values
        isSouthern: true // Default to bobs farm values
      },
      pointsOfInterest: [],
      boundaryLayers: [],
      hotSpotLayers: [],
      screenWidth: 500,
      screenHeight: 500,
      pSliderVal: 0.05,
      gridSliderVal: 1.5,
      disableRegenerate: true,
      pathDirection: 'EastWest',
      aircraftPorts: [],
      openPorts: [],
      showAircraftCommands: false,
      selectedAircraftPort: 0,
      sendToAllAircraft: false,
      useTestPoints: true,
      getAircraftLocationsStarted: false
    }
  }

  componentDidMount(){
    this.leafletMap = this.refs.map;
    this.notificationSystem = this.refs.notificationSystem;

    window.addEventListener("resize", (e: any) => {
      let width = e.width;
      let height = e.height;
      this.setState({
        screenWidth: width,
        screenHeight: height
      });
    });

    this.startPythonServerAjax();
  }


  _onEditPath = (e: L.LeafletEvent) => {
    // console.log('Path edited !');
    // console.log(e);
  }

  _onCreate = (e: any) => {
    let layerType = e.layerType;
    let latLons: L.LatLng[] = [];
    let type: string;
    if(layerType === 'rectangle' || layerType === 'polygon'){
      for(let i = 0; i < e.layer._latlngs.length; i++){
        latLons.push(e.layer._latlngs[i]);
      }
      type = 'boundary';

      let tmpLayerGroup = {
        type: type,
        latLons: latLons
      }
      let tmpLayers = this.state.boundaryLayers;
      tmpLayers.push(e.layer);
      this.setState({
        boundaryVerts: tmpLayerGroup,
        boundaryLayers: tmpLayers
      });

      console.log("BOUNDARY POINTS:");
      console.log(tmpLayerGroup.latLons);
    }
    else if (layerType === 'marker'){
      latLons.push(e.layer._latlng);
      type = 'poi';

      let tmpPOIs: LayerGroupType[] = this.state.pointsOfInterest;
      let tmpLayerGroup = {
        type: type,
        latLons: latLons
      }
      tmpPOIs.push(tmpLayerGroup);

      let tmpLayers = this.state.hotSpotLayers;
      tmpLayers.push(e.layer);
      this.setState({
        pointsOfInterest: tmpPOIs,
        hotSpotLayers: tmpLayers
      });

      console.log("HOT SPOTS:");
      console.log(tmpLayerGroup.latLons);
    }
  }

  _onDeleted = (e: L.LeafletEvent) => {
    // console.log('Path deleted !');
  }

  _mounted = (drawControl: L.LeafletEvent) => {
    // console.log('Component mounted !');
  }


  startPythonServerAjax = () => {
    console.log("Python server start sent");

    let options = {
      mode: "text",
      pythonOptions: ["-u"],
      scriptPath: "./python/"
    };

    PythonShell.run('server.py', options, function(err: any, results: any) {
      if(err){
        throw err;
      }
    }.bind(this));

    this.ajaxAction('detectSerialPorts', {}, this.setCommPorts);
  }

  setCommPorts = (commList: string[]) => {
    console.log('COMM PORTS:' + commList);

    let tmpPorts = this.state.openPorts;
    for(let i = 0; i < commList.length; i++){
        tmpPorts.push(commList[i]);
    }

    this.setState({openPorts: tmpPorts})
  }

  connectToAircraftAjax = () => {
    console.log("Connect to aircraft pressed");
    console.log(this.state.openPorts[this.state.selectedAircraftPort]);

    let connectData = {
      comm_port: this.state.openPorts[this.state.selectedAircraftPort],
      // any other data to send?
    }

    this.ajaxAction('connect', connectData, this.connectToAircraftCallback);
  }

  connectToAircraftCallback = (result: any) => {
    // console.log("Connect to aircraft callback: " + JSON.parse(result));

    let tmpAircraftPorts: AircraftType[] = this.state.aircraftPorts;
    tmpAircraftPorts.push({
      comm_port: this.state.openPorts[this.state.selectedAircraftPort],
      isConnected: result
    })
    this.setState({showAircraftCommands: true, aircraftPorts: tmpAircraftPorts});

    this.getAircraftHomeLocations();
  }


  getAircraftLocations = () => {
    // console.log("Get aircraft locations");
      for(let i = 0; i < this.state.aircraftPorts.length; i++){
        let homeData = {
          comm_port: this.state.aircraftPorts[i].comm_port
        }

        // console.log('TEST COMM PORTS SEND: ' + this.state.aircraftPorts[i].comm_port);
        if(this.state.aircraftPorts[i].isConnected){
          console.log("In get location if statement");
          console.log("COMM PORT: " + this.state.aircraftPorts[i]);
          this.ajaxAction('getLocation', homeData, this.handleUpdateVehicleLocations);
        }
      }
  }

  getAircraftHomeLocations = () => {
    for(let i = 0; i < this.state.aircraftPorts.length; i++){
      let homeData = {
        comm_port: this.state.aircraftPorts[i].comm_port
      }
      if(this.state.aircraftPorts[i].isConnected){
        this.ajaxAction('getHomeLocation', homeData, this.handleUpdateHomeLocations);
      }
    }
  }

  handleUpdateVehicleLocations = (results: any) => {
    // console.log("UPDATE VEHICLE LOCATIONS: " +  + results.latitude + " / " + results.longitude + " / " + results.altitude + " / " + results.heading + " / " + results.comm_port);

    for(let key in this.leafletMap.leafletElement._layers){
      if(this.leafletMap.leafletElement._layers[key]._latlng){
        let tmpIcon: string = this.leafletMap.leafletElement._layers[key].options.icon.options.iconUrl;
        let tmpIconTitle: string = this.leafletMap.leafletElement._layers[key].options.title;
        for(let i = 0; i < this.state.aircraftPorts.length; i++){
          if(this.state.aircraftPorts[i].comm_port === results.comm_port){
            // let tmpMarker = this.leafletMap.leafletElement._layers[key];
            // this.leafletMap.leafletElement.removeLayer(this.leafletMap.leafletElement._layers[key]);

            let tmpAircraftMarkers = this.state.aircraftMarkers;
            // let iconHTML = '<img src="./images/drone-icon.png" alt="Drone icon" style="background-color: transparent; width:41px; height:41px; -webkit-transform: rotate(' + results.heading + 'deg); -moz-transform: rotate(' + results.heading + 'deg); -o-transform: rotate(' + results.heading + 'deg); -ms-transform: rotate(' + results.heading + 'deg); transform: rotate(' + results.heading + 'deg);">';
            let iconHTML = '<div style="background-color: ' + this.backgroundColors[i] + '; color: white; width: 41px; text-align: center;">' + this.state.aircraftPorts[i].comm_port + '</div><img src="./images/drone-icon.png" alt="Drone icon" style="width:41px; height:41px; -webkit-transform: rotate(' + results.heading + 'deg); -moz-transform: rotate(' + results.heading + 'deg); -o-transform: rotate(' + results.heading + 'deg); -ms-transform: rotate(' + results.heading + 'deg); transform: rotate(' + results.heading + 'deg);">';
            let updatedMarker: MarkerType = {
              comm_port: this.state.aircraftPorts[i].comm_port,
              position: new L.LatLng(results.latitude, results.longitude),
              icon: new L.DivIcon({
                html: iconHTML,
                iconAnchor: [20, 38], // point of the icon which will correspond to marker's location
                popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
              })
              // icon: new L.Icon({
              //     iconUrl: './images/drone-icon.png',
              //     iconSize: [41, 41], // size of the icon
              //     iconAnchor: [20, 20], // point of the icon which will correspond to marker's location
              //     popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
              //   })
            }
            tmpAircraftMarkers[i] = updatedMarker;
            console.log("IN UPDATE VEHICLE LOCATIONS: " + tmpAircraftMarkers[i].comm_port)
            this.setState({aircraftMarkers: tmpAircraftMarkers});
          }
        }
      }
    }
  }

  handleUpdateHomeLocations = (results: any) => {
    // console.log("HOME LOCATIONS: " + results.latitude + " / " + results.longitude + " / " + results.altitude);

    let iconHTML = '<img src="./images/drone-icon.png" alt="Drone icon" style="width:41px;height:41px;">'
    let tmpMarker: MarkerType = {
      position: new L.LatLng(results.latitude, results.longitude),
      icon: new L.DivIcon({
        html: iconHTML,
        iconAnchor: [20, 20], // point of the icon which will correspond to marker's location
        popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
      }),
      // icon: new L.Icon({
      //     iconUrl: './images/drone-icon.png',
      //     iconSize: [41, 41], // size of the icon
      //     iconAnchor: [20, 20], // point of the icon which will correspond to marker's location
      //     popupAnchor: [0, -18] // point from which the popup should open relative to the iconAnchor
      //   }),
      comm_port: this.state.openPorts[this.state.selectedAircraftPort]
    };

    let tmpAircraftMarkers = this.state.aircraftMarkers;
    tmpAircraftMarkers.push(tmpMarker);
    this.setState({aircraftMarkers: tmpAircraftMarkers});

    // // Get locations of all connected aircraft:
    if(this.state.getAircraftLocationsStarted){
      setInterval(function() {
          this.getAircraftLocations();
      }.bind(this), 1000);

      this.setState({getAircraftLocationsStarted: true});
    }

  }

  disconnectFromAircraftAjax = () => {
    console.log("disconnect from to aircraft pressed");

    let connectData = {
      comm_port: this.state.openPorts[this.state.selectedAircraftPort],
      // any other data to send?
    }

    this.ajaxAction('disconnect', connectData, this.connectToAircraftCallback);
  }

  generateWaypointsAjax = () => {
    console.log("Generate waypoints pressed");
    let boundaryVerts: any = [];
    let hotSpots: any = [];

    // Change to false to have the test points used:
    // Default parameters:
    let pSliderVal = 0.01;
    let gridSliderVal = 0.5;
    let pathDirection = "NorthSouth";
    if(this.state.useTestPoints === true){
      // Boundary Verticies:
      boundaryVerts.push([37.88866901916639, -76.81350946426392]);
      boundaryVerts.push([37.88981208998668, -76.81350946426392]);
      boundaryVerts.push([37.88981208998668, -76.8103176355362]);
      boundaryVerts.push([37.88866901916639, -76.8103176355362]);

      // Hot spots:
      hotSpots.push([37.88938873249277, -76.81289792060852]);
      hotSpots.push([37.88901194227583, -76.81118667125702]);

      this.plotPoints(hotSpots);
    }
    else {
      if(this.state.boundaryVerts === null){
        // console.log('NO BOUNDARY');
        this.showNotification('Error!', 'You have to define a boundary before generating waypoints.', 'error', 'tr', 'Got it')
        return
      }
      if(this.state.pointsOfInterest.length <= 0){
        this.showNotification('Error!', 'You have to define at least one one point of interest before generating waypoints.', 'error', 'tr', 'Got it')
        return
      }

      for(let i = 0; i < this.state.boundaryVerts.latLons.length; i++){
        boundaryVerts.push([this.state.boundaryVerts.latLons[i].lat, this.state.boundaryVerts.latLons[i].lng]);
      }
      for(let i = 0; i < this.state.pointsOfInterest.length; i++){
        hotSpots.push([this.state.pointsOfInterest[i].latLons[0].lat, this.state.pointsOfInterest[i].latLons[0].lng]);
      }

      pSliderVal = this.state.pSliderVal;
      gridSliderVal = this.state.gridSliderVal;
      pathDirection = this.state.pathDirection;
    }


    let waypointData = {
      boundaryVerts: boundaryVerts,
      hotSpots: hotSpots,
      pSliderVal: pSliderVal,
      gridSliderVal: gridSliderVal,
      pathDirection: pathDirection
    }

    this.ajaxAction('generateWaypoints', waypointData, this.generateWaypointsCallback);
  }

  generateWaypointsCallback = (results: any) => {
    console.log("Update waypoitns on map");

    // let parsedBoundaryResults = JSON.parse(results.planningRegions);
    this.updatePythonBoundaries(results.planningRegions);

    // let parsedPathResults = JSON.parse(results.vehiclePaths);
    this.updateAircraftPaths(results.vehiclePaths);

    this.setState({disableRegenerate: true});
  }

  sendWPsToACAjax = () => {
    if(this.state.sendToAllAircraft){
      for(let i = 0; i < this.state.openPorts.length; i++){
        let aircraftWithComm = this.state.aircraftPorts.findIndex((item: AircraftType) => {
          return item.comm_port === this.state.openPorts[i];
        });

        if(aircraftWithComm < 0){
          continue;
        }
        else {

          console.log("IN SEND WPS TO AC AJAX");
          console.log("Mission length: " + this.state.aircraftPaths[aircraftWithComm].waypoints.length)

          let waypointData = {
            comm_port: this.state.openPorts[i],
            waypoints: this.state.aircraftPaths[aircraftWithComm].waypoints.map((item) => {
              return [item.lat, item.lng];
            })
          }

          if(this.state.aircraftPorts[aircraftWithComm].isConnected){
            this.ajaxAction('sendWaypoints', waypointData, this.sendWPsToACCallback);
          }
        }
      }
    }
    else {
      let aircraftWithComm = this.state.aircraftPorts.findIndex((item: AircraftType) => {
        return item.comm_port === this.state.openPorts[this.state.selectedAircraftPort];
      });


      console.log("IN SEND WPS TO AC AJAX");
      console.log("Mission length: " + this.state.aircraftPaths[aircraftWithComm].waypoints.length)

      let waypointData = {
        comm_port: this.state.openPorts[this.state.selectedAircraftPort],
        // waypoints: this.state.aircraftPaths[1].waypoints.map((item) => {
        //   return [item.lat, item.lng];
        // })
        waypoints: this.state.aircraftPaths[aircraftWithComm].waypoints.map((item) => {
          return [item.lat, item.lng];
        })
      }

      // PAT -- IF THIS DOESNT WORK, JUST COMMENT OUT IF STATEMENT AND LEAVE AJAX
      console.log("BEFORE SEND WP IF STATEMENT.....")
      if(this.state.aircraftPorts[aircraftWithComm].isConnected){
        console.log("IN SEND WP IF STATEMENT for COMM: " + this.state.aircraftPorts[aircraftWithComm].comm_port);
        this.ajaxAction('sendWaypoints', waypointData, this.sendWPsToACCallback);
      }
    }

  }

  sendWPsToACCallback = (results: any) => {
    let msg = 'Waypoints received';
    console.log(msg);
    this.showNotification('Success!', msg, 'success', 'bc', 'Got it');
  }


  // TODO: What to do with these?
  takeoffCallback = () => {
  }
  landCallback = () => {
  }
  rtlCallback = () => {
  }
  guidedCallback = () => {
  }
  loiterCallback = () => {
  }
  missionCallback = () => {
  }

  aircraftCommand = (command: string) => {
    // connect, takeoff, land, guided, rtl, disconnect, sendWaypoints, genWaypoints
    console.log("IN AIRCRAFT COMMAND: " + command);
    console.log("IN AIRCRAFT COMMAND: " + this.state.sendToAllAircraft);
    let commPorts: string[] = [];
    if(this.state.sendToAllAircraft){
      for(let i = 0; i < this.state.aircraftPorts.length; i++){
        if(this.state.aircraftPorts[i].isConnected){
          commPorts.push(this.state.aircraftPorts[i].comm_port);
        }
      }
    }
    else {
      commPorts.push(this.state.openPorts[this.state.selectedAircraftPort]);
    }

    for(let j = 0; j < commPorts.length; j++){
      if(command === 'launch') {
        this.ajaxAction('launch', {comm_port: commPorts[j]}, this.takeoffCallback);
      }
      if(command === 'land') {
        this.ajaxAction('land', {comm_port: commPorts[j]}, this.landCallback);
      }
      if(command === 'rtl') {
        this.ajaxAction('rtl', {comm_port: commPorts[j]}, this.rtlCallback);
      }
      if(command === 'guided') {
        this.ajaxAction('guided', {comm_port: commPorts[j]}, this.guidedCallback);
      }

      if(command === 'loiter') {
        this.ajaxAction('loiter', {comm_port: commPorts[j]}, this.loiterCallback);
      }
      if(command === 'mission') {
        this.ajaxAction('mission', {comm_port: commPorts[j]}, this.missionCallback);
      }
    }

  }

  ajaxAction = (postAction: string, data: any, successFn: any) => {
    let postUrl: string = "http://localhost:10081/" + postAction;
    // console.log("AJAX ACTION")
    // console.log(postAction);
    // console.log(data);
    $.ajax({
        type: "POST",
        url: postUrl,
        data: JSON.stringify(data),
        cache: false,
        success: function (result: any) {
            successFn(result);
        },
        error: function (jqXHR: any, exception: any) {
            let msg: string;
            if (jqXHR.status === 0) {
              msg = 'Not connected.\n Verify Network.';
            } else if (jqXHR.status == 404) {
              msg = 'Requested page not found [404]';
            } else if (jqXHR.status == 500) {
              msg = 'Internal Server Error [500]';
            } else if (exception === 'parsererror') {
              msg = 'Requested JSON parse failed';
            } else if (exception === 'timeout') {
              msg = 'Time out error';
            } else if (exception === 'abort') {
              msg = 'Ajax request aborted';
            } else {
              msg = 'Uncaught Error.\n' + jqXHR.responseText;
            }

            console.log('Error!: ' + msg);
            // this.showNotification('Error!', msg, 'error', 'tr', 'Got it');
        }.bind(this)
    });
  }


  updatePythonBoundaries = (results: any) => {
    let tmpBoundaryPaths: PathType[] = [];
    for(let i = 0; i < results.length; i++) {
      let waypoints = results[i];
      let wpLatLng: L.LatLng[] = [];
      for(let j = 0; j < waypoints.length; j++) {
        let tmpLatLng = new L.LatLng(waypoints[j][0], waypoints[j][1]);
        wpLatLng.push(tmpLatLng);
      }
      wpLatLng.push(wpLatLng[0]);

      tmpBoundaryPaths.push({waypoints: wpLatLng});
    }
    this.setState({boundaryPaths: tmpBoundaryPaths});
  }

  updateAircraftPaths = (results: any) => {
    let tmpPythonPaths: PathType[] = [];
    for(let i = 0; i < results.length; i++) {
      let waypoints = results[i];
      let wpLatLng: L.LatLng[] = [];
      for(let j = 0; j < waypoints.length; j++) {
        let tmpLatLng = new L.LatLng(waypoints[j][0], waypoints[j][1]);
        wpLatLng.push(tmpLatLng);
      }

      tmpPythonPaths.push({waypoints: wpLatLng});
      // // Uncomment to plot actual waypoints
      // this.plotPoints(wpLatLng);
    }

    this.setState({aircraftPaths: tmpPythonPaths});
  }

  plotPoints = (results: any[]) => {
    let tmpMarkers = this.state.markers;
    for(let i = 0; i < results.length; i++){
        let tmpMarker: MarkerType = {
          position: results[i],
          icon: new L.Icon({
              iconUrl: './images/marker-icon.png',
              iconSize: [25, 41], // size of the icon
              iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
              popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
            })
        };

        tmpMarkers.push(tmpMarker);
    }

    this.setState({markers: tmpMarkers});
  }

  clearWaypoints = () => {
    this.setState({
      aircraftPaths: [],
      boundaryPaths: [],
      markers: []
    })
  }

  clearPointsofInterest = () => {
    for(let key in this.leafletMap.leafletElement._layers){
      if(this.leafletMap.leafletElement._layers[key]._latlng){
        this.leafletMap.leafletElement.removeLayer(this.leafletMap.leafletElement._layers[key]);
      }
    }

    this.clearWaypoints();

    this.setState({
      pointsOfInterest: [],
      hotSpotLayers: []
    })
  }

  clearBoundary = () => {
    for(let key in this.leafletMap.leafletElement._layers){
      if(this.leafletMap.leafletElement._layers[key]._latlngs){
        this.leafletMap.leafletElement.removeLayer(this.leafletMap.leafletElement._layers[key]);
      }
    }

    this.clearPointsofInterest();

    this.setState({
      boundaryVerts: null,
      boundaryLayers: []
    })
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

  handleSliderChange = (e: any, value: number, slider: string) => {
    if(slider === 'pVal'){
      this.setState({pSliderVal: value});
    }
    else if (slider === 'grid'){
      this.setState({gridSliderVal: value});
    }

    if(this.state.pointsOfInterest.length > 0 && this.state.boundaryVerts !== null){
      this.setState({disableRegenerate: false});
    }
  }

  updatePathDirection = (direction: string) => {
    this.setState({pathDirection: direction});

    if(this.state.pointsOfInterest.length > 0 && this.state.boundaryVerts !== null){
      this.setState({disableRegenerate: false});
    }
  }

  handleDropdownChange = (event: any, index: number, value: number) => {
    this.setState({selectedAircraftPort: value});
  }

  render() {

    const position = [37.889231, -76.810302];
    var width = window.screen.width;
    var height = window.screen.height;
    const wpColors = ['blue', 'red', 'black', 'green', 'yellow', 'orange'];
    const boundaryColors = ['blue', 'red', 'black', 'green', 'yellow', 'orange'];
    const backgroundColors = ['rgba(255,0,0,0.2)', 'rgba(0,0,255,0.2)', 'rgba(0,0,0,0.2)', 'rgba(0,255,0,0.2)', 'rgba(255,255,0,0.2)', 'rgba(255,153,0,0.2)'];
    const parentStyle = {height: height + 'px', width: width + 'px'};
    const mapStyle = { top: 0, left: 0, height: height + 'px', width: width + 'px' };
    const buttonContainer = { position: 'absolute', top: 15, right: 15, zIndex: 999};
    const useTestPointsContainer = { position: 'absolute', top: 60, right: 0, width: 200, zIndex: 999};
    const aircraftCommsContainer = { position: 'absolute', bottom: 20, left: 15, zIndex: 999};
    const sliderContainer = { position: 'absolute', bottom: 15, right: 15, zIndex: 999, width: 500};
    const radioContainer = {marginBottom: 16};
    const radioButton = {display: 'inline-block', width: '150px'}
    const buttonStyle = { marginLeft: 10 };
    const centerButtonStyle = { width: 100 + '%' };
    const aircraftCommand = { width: 100 + '%', marginBottom: 16 };
    const initialZoom = 18;
    const maxZoom = 20;
    const toggleStyle = { marginBottom: 16 };
    const toggleContainer = { position: 'relative', width: 200, top: 16, right: -275 };
    const checkbox = { marginBottom: 16 }
    let connectOrDisconnectButton: any = null;

    // if (this.state.aircraftPorts[this.state.selectedAircraftPort] && this.state.aircraftPorts[this.state.selectedAircraftPort].isConnected){
    //   connectOrDisconnectButton = (<MuiThemeProvider muiTheme={lightMuiTheme}>
    //     <RaisedButton backgroundColor={backgroundColors[this.state.selectedAircraftPort]} style={buttonStyle} label="Disconnect from aircraft" onClick={this.disconnectFromAircraftAjax}/>
    //   </MuiThemeProvider>);
    // }
    // else{
      connectOrDisconnectButton = (<MuiThemeProvider muiTheme={lightMuiTheme}>
        <RaisedButton backgroundColor={backgroundColors[this.state.selectedAircraftPort]} style={buttonStyle} label="Connect to aircraft" onClick={this.connectToAircraftAjax}/>
      </MuiThemeProvider>);
    // }

    return (

        <div style={parentStyle}>
          <div style={buttonContainer}>
            {this.state.aircraftPaths.length > 0 &&
              <MuiThemeProvider muiTheme={lightMuiTheme}>
                <RaisedButton style={buttonStyle} label="Clear Waypoints" onClick={this.clearWaypoints}/>
              </MuiThemeProvider>
            }
            {this.state.pointsOfInterest.length > 0 &&
              <MuiThemeProvider muiTheme={lightMuiTheme}>
                <RaisedButton style={buttonStyle} label="Clear Hot Spots" onClick={this.clearPointsofInterest}/>
              </MuiThemeProvider>
            }
            {this.state.boundaryVerts !== null &&
              <MuiThemeProvider muiTheme={lightMuiTheme}>
                <RaisedButton style={buttonStyle} label="Clear All" onClick={this.clearBoundary}/>
              </MuiThemeProvider>
            }

            <MuiThemeProvider muiTheme={lightMuiTheme}>
              <RaisedButton style={buttonStyle} label="Generate Waypoints" onClick={this.generateWaypointsAjax}/>
            </MuiThemeProvider>

          </div>

          <div style={useTestPointsContainer}>
            <MuiThemeProvider muiTheme={darkMuiTheme}>
              <Checkbox
                label="Use test points"
                style={checkbox}
                checked={this.state.useTestPoints}
                onCheck={() => this.setState({useTestPoints: !this.state.useTestPoints})}
              />
            </MuiThemeProvider>
          </div>

          {this.state.openPorts.length > 0 &&
            <div style={aircraftCommsContainer}>
              <MuiThemeProvider muiTheme={lightMuiTheme}>
                <DropDownMenu style={{width: 150, backgroundColor: lightMuiTheme.palette.canvasColor}} value={this.state.selectedAircraftPort} onChange={this.handleDropdownChange}>
                  {this.state.openPorts.map((item: string, i: number) => {
                    return(
                      <MenuItem key={i} value={i} primaryText={this.state.openPorts[i]} label={this.state.openPorts[i]} />
                    );
                  })}
                </DropDownMenu>
              </MuiThemeProvider>
              {connectOrDisconnectButton}

              <MuiThemeProvider muiTheme={lightMuiTheme}>
                <RaisedButton backgroundColor={backgroundColors[this.state.selectedAircraftPort]} style={buttonStyle} label="Send waypoints to aircraft" onClick={this.sendWPsToACAjax}/>
              </MuiThemeProvider>
            </div>
          }


          <div style={sliderContainer}>
            <MuiThemeProvider muiTheme={lightMuiTheme}>
              <Card containerStyle={{backgroundColor: backgroundColors[this.state.selectedAircraftPort]}}>
                  <div style={toggleContainer}>
                    <Toggle
                      label="Show Path/Show Aircraft Commands"
                      style={toggleStyle}
                      onToggle={() => this.setState({showAircraftCommands: !this.state.showAircraftCommands})}
                      />
                  </div>

                  {!this.state.showAircraftCommands &&
                    <div>
                      <CardTitle title="Adjust path parameters" />
                      <CardText>
                        <div style={{marginBottom: 8}}>
                          Path direction:
                        </div>
                        <RadioButtonGroup style={radioContainer} name="pathDirection" defaultSelected="EastWest" onChange={(e: any, selected: string) => this.updatePathDirection(selected)}>
                          <RadioButton
                            value="EastWest"
                            label="East-West"
                            style={radioButton}
                          />
                          <RadioButton
                            value="NorthSouth"
                            label="North-South"
                            style={radioButton}
                          />
                        </RadioButtonGroup>

                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                          <Slider
                            description={'Current value for "p": ' + this.state.pSliderVal}
                            min={0.01}
                            max={1}
                            value={this.state.pSliderVal}
                            onChange={(e: any, value: number) => this.handleSliderChange(e, value, 'pVal')}
                          />
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                          <Slider
                            description={'Current value for grid density: ' + this.state.gridSliderVal}
                            min={0.25}
                            max={2}
                            value={this.state.gridSliderVal}
                            onChange={(e: any, value: number) => this.handleSliderChange(e, value, 'grid')}
                          />
                        </MuiThemeProvider>
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                          <RaisedButton style={centerButtonStyle} disabled={this.state.disableRegenerate} label="Regenerate Waypoints" onClick={this.generateWaypointsAjax}/>
                        </MuiThemeProvider>
                      </CardText>
                    </div>
              }
              {this.state.showAircraftCommands &&
                <div>
                    <CardTitle title="Aircraft commands" />
                    <CardText>

                      <Checkbox
                        label="Send to all aircraft"
                        style={checkbox}
                        onCheck={() => this.setState({sendToAllAircraft: !this.state.sendToAllAircraft})}
                      />

                      <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Launch" onClick={() => this.aircraftCommand('launch')}/>
                      </MuiThemeProvider>
                      {/*
                        <MuiThemeProvider muiTheme={lightMuiTheme}>
                          <RaisedButton style={aircraftCommand} label="Loiter" onClick={() => this.aircraftCommand('loiter')}/>
                        </MuiThemeProvider>
                      */}
                      <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Return to Launch" onClick={() => this.aircraftCommand('rtl')}/>
                      </MuiThemeProvider>
                      <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Land" onClick={() => this.aircraftCommand('land')}/>
                      </MuiThemeProvider>
                      {/*
                      <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={aircraftCommand} label="Start Mission" onClick={() => this.aircraftCommand('mission')}/>
                      </MuiThemeProvider>
                      */}
                     {!this.state.useTestPoints &&
                       <MuiThemeProvider muiTheme={lightMuiTheme}>
                         <RaisedButton style={aircraftCommand} label="Switch to Guided" onClick={() => this.aircraftCommand('guided')}/>
                       </MuiThemeProvider>
                     }

                    </CardText>
                  </div>
              }

              </Card>
            </MuiThemeProvider>
          </div>


          <Map ref="map" center={position} zoom={initialZoom} style={mapStyle} >
              {/* <TileLayer url='http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' />  */}
              <TileLayer url='http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}' maxZoom={maxZoom} subdomains={['mt0','mt1','mt2','mt3']} />

              <FeatureGroup>
                <EditControl
                  position='topleft'
                  onEdited={this._onEditPath}
                  onCreated={this._onCreate}
                  onDeleted={this._onDeleted}
                  draw={{
                      polyline: false,
                      polygon: {
                          allowIntersection: true, // Restricts shapes to simple polygons
                          drawError: {
                              color: '#e1e100', // Color the shape will turn when intersects
                              message: '<strong>NOTE:<strong> You must select points in a CCW fashion' // Message that will show when intersect
                          },
                          shapeOptions: {
                              color: '#bada55'
                          }
                      },
                      circle: false, // Turns off this drawing tool
                      rectangle: {
                          shapeOptions: {
                              clickable: true
                          }
                      },
                      marker: {
                          icon: new L.Icon({
                              iconUrl: './images/marker-icon.png',
                              iconSize: [25, 41], // size of the icon
                              iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
                              popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
                            }),
                          }
                    }}
                />
            </FeatureGroup>

            <LayerGroup>
              {this.state.markers.map((item: MarkerType, i: number) => {
                return (
                  <Marker key={i} position={item.position} icon={item.icon} title={"TEST"}>
                    <Popup>
                      <span>A pretty CSS3 popup.<br/>Easily customizable.</span>
                    </Popup>
                  </Marker>
                );
              })}

              {this.state.aircraftMarkers.map((item: MarkerType, i: number) => {
                return (
                  <Marker key={i} position={item.position} icon={item.icon} title={item.comm_port}>
                  {/*
                    <Popup open={true}>
                      <span>{item.comm_port}</span>
                    </Popup>
                    */}
                  </Marker>
                );
              })}

              {this.state.aircraftPaths.map((item: PathType, i: number) => {
                return (
                  <Polyline key={i} positions={item.waypoints} color={wpColors[i]} />
                );
              })}

              {this.state.boundaryPaths.map((item: PathType, i: number) => {
                return (
                  <Polyline key={i} positions={item.waypoints} color={boundaryColors[i]} />
                );
              })}
            </LayerGroup>


          </Map>

          <div>
            <NotificationSystem ref="notificationSystem" />
          </div>

        </div>
    );
  }
}
