import * as React from 'react';

import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();

import { Map, TileLayer, LayerGroup, Marker, Polyline, Polygon } from 'react-leaflet';
// import { EditControl } from "react-leaflet-draw"
import * as colors from 'material-ui/styles/colors';
import { Vehicle } from '../Vehicle';
import { ContextMenu } from '../components/ContextMenu';

type Props = {
    handleSelectedAircraftUpdate: (id: string) => void,
    connectedVehicles: {[id: string]: Vehicle},
    selectedVehicleID: string,
    maxZoom: number,
    mapZoom: number,
    mapCenter: PositionType,
    globalOrigin: PositionType,
    updateMapCenter: (e: L.DragEndEvent) => void,
    contextSetHome: () => void,
    contextSetGlobal: () => void,
    contextGoHere: () => void,
    contextSetTakeoff: () => void,
    setContextAnchor: (e: L.MouseEvent) => void
    contextAnchor: L.MouseEvent,
    MACEConnected: boolean,
    environmentBoundary: PositionType[],
    drawPolygonPts?: PositionType[],
    onAddPolygonPt: (e: L.MouseEvent) => void,
    environmentSettings: EnvironmentSettingsType,
    gridPts?: {inPoly: L.LatLng[], trimmedPts: L.LatLng[]},
    envBoundingBox?: PositionType[]
}

type State = {
    showContextMenu?: boolean,
    dragging?: boolean,
}

export default class MACEMap extends React.Component<Props, State> {
  leafletMap: L.Map;
  constructor(props: Props) {
    super(props);

    this.state = {
      showContextMenu: false,
      dragging: false
    }
  }

  // shouldComponentUpdate(nextProps: Props, nextState: State) {
  //   if(nextState.showContextMenu === this.state.showContextMenu) {
  //     return false;
  //   }

  //   return true;
  // }

  componentDidMount(){
    this.leafletMap = this.refs.map as any;
  }


  handleMarkerClick = (e: L.MouseEvent, vehicleId: string, type: string) => {
    this.props.handleSelectedAircraftUpdate(vehicleId);
  }

  triggerContextMenu = (event: L.MouseEvent) => {
    this.props.setContextAnchor(event);
    this.setState({showContextMenu: !this.state.showContextMenu});
  }

  // _onDrawEditPath = (e: any) => {
  //   console.log('Path edited !');
  // }

  // _onDrawCreate = (e: any) => {
  //   // polyline = e.layer;
  //   // To edit this polyline call : polyline.handler.enable()
  //   console.log('Path created !');
  // }

  // _onDrawDeleted = (e: any) => {
  //   console.log('Path deleted !');
  // }

  // _mountedDraw = (drawControl: any) => {
  //   console.log('Component mounted !');
  // }

  // _onDrawEditStart = () => {
  //   console.log('Edit is starting !');
  // }

  // _onDrawEditStop = () => {
  //   console.log('Edit is stopping !');
  // }

  // _onDrawDeleteStart = () => {
  //   console.log('Delete is starting ! ... Probably want to make this an "undo" style button. Undo the previous layer');
  // }

  // _onDrawDeleteStop = () => {
  //   console.log('Delete is stopping ! ... Probably want to just delete');
  // }

  handleMapClick = (event: L.MouseEvent) => {
    if(!this.state.dragging) {
      this.props.onAddPolygonPt(event);
    }
  }

  handleDragStart = (e: L.MouseEvent) => {
    console.log("Drag start");
    // this.setState({dragging: true});
  }

  handleDragEnd = (e: L.MouseEvent) => {
    console.log("Drag end");
  }

  handleDrag = (e: L.MouseEvent) => {
    console.log("Dragging...");
  }



  render() {

    const width = window.screen.width;
    const height = window.screen.height;
    const parentStyle = {height: height + 'px', width: width + 'px'};
    const mapStyle = { top: 64, left: 0, height: height + 'px', width: width + 'px' };

    const globalOriginMarker = {
      position: new L.LatLng(this.props.globalOrigin.lat, this.props.globalOrigin.lng),
      icon: new L.Icon({
          iconUrl: './images/userlocation_icon.png',
          iconSize: [41, 41], // size of the icon
          iconAnchor: [20, 20], // point of the icon which will correspond to marker's location
          popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
      })
    };

    const drawingMarkers = [];
    let icon = new L.Icon({
          iconUrl: './images/marker-icon-orange.png',
          iconSize: [25, 41], // size of the icon
          iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
          popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
      });
    for(let i = 0; i < this.props.drawPolygonPts.length; i++) {
      drawingMarkers.push(<Marker key={i} position={this.props.drawPolygonPts[i]} title={i.toString()} icon={icon} draggable={false} />);
    }

    let tmpPts = [];
    for(let i = 0; i < this.props.drawPolygonPts.length; i++) {
      tmpPts.push(this.props.drawPolygonPts[i]);
    }

    let tmpBBoxPts = [];
    if(this.props.drawPolygonPts.length > 2) {
      for(let i = 0; i < this.props.envBoundingBox.length; i++) {
        tmpBBoxPts.push(this.props.envBoundingBox[i]);
      }
    }


    let tmpGridPts = [];;
    if(this.props.drawPolygonPts.length > 2) {
      let gridIcon = new L.Icon({
            iconUrl: './images/ic_add_white_24dp_1x.png',
            iconSize: [25, 25], // size of the icon
            iconAnchor: [12, 12], // point of the icon which will correspond to marker's location
            popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
        });
      for(let i = 0; i < this.props.gridPts.inPoly.length; i++) {
        tmpGridPts.push(<Marker key={i} position={this.props.gridPts.inPoly[i]} title={i.toString()} icon={gridIcon} draggable={false} />);
      }
    }

    let tmpTrimmedGridPts = [];;
    if(this.props.drawPolygonPts.length > 2) {
      let trimmedIcon = new L.Icon({
            iconUrl: './images/ic_add_grey600_48dp.png',
            iconSize: [25, 25], // size of the icon
            iconAnchor: [12, 12], // point of the icon which will correspond to marker's location
            popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
        });
      for(let i = 0; i < this.props.gridPts.trimmedPts.length; i++) {
        tmpTrimmedGridPts.push(<Marker key={i} position={this.props.gridPts.trimmedPts[i]} title={i.toString()} icon={trimmedIcon} draggable={false} />);
      }
    }

    return (
        <MuiThemeProvider muiTheme={lightMuiTheme}>
          <div style={parentStyle}>

            {this.state.showContextMenu &&
              <ContextMenu
                menuAnchor={this.props.contextAnchor}
                handleClose={() => this.setState({showContextMenu: false})}
                handleSetHome={() => {this.setState({showContextMenu: false}); this.props.contextSetHome()}}
                handleSetGlobal={() => {this.setState({showContextMenu: false}); this.props.contextSetGlobal()}}
                handleGoHere={() => {this.setState({showContextMenu: false}); this.props.contextGoHere()}}
                handleSetTakeoff={() => {this.setState({showContextMenu: false}); this.props.contextSetTakeoff()}}
              />
            }

            <Map ref="map" ondragend={this.props.updateMapCenter} useFlyTo={true} animate={true} center={this.props.mapCenter} zoom={this.props.mapZoom} style={mapStyle} zoomControl={false} maxZoom={this.props.maxZoom} oncontextmenu={this.triggerContextMenu} onclick={this.handleMapClick} >
                {/* <TileLayer url='http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' />  */}
                <TileLayer url='http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}' maxZoom={this.props.maxZoom} subdomains={['mt0','mt1','mt2','mt3']} />

                {this.props.MACEConnected &&
                  <LayerGroup>

                    {/* Aircraft Icons */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      return (
                        <Marker zIndexOffset={1000} onclick={(e: L.MouseEvent) => this.handleMarkerClick(e, key, "vehicle")} key={key} position={this.props.connectedVehicles[key].vehicleMarker.latLon} icon={this.props.connectedVehicles[key].vehicleMarker.icon} title={key}>
                        {/*
                          <Popup open={true}>
                          </Popup>
                        */}
                        </Marker>
                      );
                    })}

                    {/* Home Icons */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      return (
                        <Marker onclick={(e: L.MouseEvent) => this.handleMarkerClick(e, key, "home")} key={key} position={this.props.connectedVehicles[key].homePosition.latLon} icon={this.props.connectedVehicles[key].homePosition.icon} title={key}>
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
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      return (
                        <Polyline key={key} positions={this.props.connectedVehicles[key].vehicleMission.latLons} color={this.props.selectedVehicleID === key ? this.props.connectedVehicles[key].highlightColor : this.props.connectedVehicles[key].opaqueHighlightColor} />
                      );
                    })}

                    {/* Mission Markers */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      let markers: JSX.Element[] = [];
                      for(let i = 0; i < this.props.connectedVehicles[key].vehicleMission.latLons.length; i++){
                        markers.push(<Marker key={i} position={this.props.connectedVehicles[key].vehicleMission.latLons[i]} icon={this.props.connectedVehicles[key].vehicleMission.icons[i]} title={key} />);
                      }
                      return (
                        markers
                      );
                    })}

                    {/* Sensor Footprint */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      return (
                        <Polygon key={key} positions={this.props.connectedVehicles[key].sensorFootprint} color={this.props.selectedVehicleID === key ? this.props.connectedVehicles[key].highlightColor : this.props.connectedVehicles[key].opaqueHighlightColor} fillColor={colors.amber500} />
                      );
                    })}

                    {/* EnvironmentBoundary */}
                    <Polygon positions={this.props.environmentBoundary} color={colors.white} fillColor={colors.green300} />

                    {/* Guided target */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      return (
                        <Marker key={key} position={this.props.connectedVehicles[key].currentTarget.targetPosition} icon={this.props.connectedVehicles[key].currentTarget.icon} title={key}>
                        </Marker>
                      );
                    })}
                    {/* Guided target Paths */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      if(this.props.connectedVehicles[key].currentTarget.active) {
                        return (
                          <Polyline key={key} positions={[this.props.connectedVehicles[key].vehicleMarker.latLon, this.props.connectedVehicles[key].currentTarget.targetPosition]} color={this.props.selectedVehicleID === key ? this.props.connectedVehicles[key].highlightColor : this.props.connectedVehicles[key].opaqueHighlightColor} />
                        );
                      }
                    })}

                    {/* Bounding box */}
                    {this.props.environmentSettings.showBoundingBox &&
                      <Polygon positions={tmpBBoxPts} color={colors.blue100} fillColor={colors.blue100} />
                    }
                    {/* Grid points */}
                    {tmpGridPts}
                    {/* Trimmed points */}
                    {this.props.environmentSettings.showBoundingBox &&
                      tmpTrimmedGridPts
                    }



                    {/* Drawing polygon */}
                    <Polygon positions={tmpPts} color={colors.white} fillColor={colors.purple400} />
                    {drawingMarkers}


                  </LayerGroup>
                }

                {/* {this.props.showDraw && */}
                    {/* <FeatureGroup>
                      <EditControl
                        position='topleft'
                        onEdited={this._onDrawEditPath}
                        onCreated={this._onDrawCreate}
                        onDeleted={this._onDrawDeleted}
                        onMounted={this._mountedDraw}
                        onEditStart={this._onDrawEditStart}
                        onEditStop={this._onDrawEditStop}
                        onDeleteStart={this._onDrawDeleteStart}
                        onDeleteStop={this._onDrawDeleteStop}
                        draw={
                          {
                            marker: false,
                            circle: false,
                            polyline: false,
                            delete: false
                          }
                        }
                        style={{size: 100}}
                      />
                  </FeatureGroup> */}
                 {/* } */}

            </Map>
          </div>
        </MuiThemeProvider>
    );
  }
}
