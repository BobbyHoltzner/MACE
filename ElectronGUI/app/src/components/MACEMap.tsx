import * as React from 'react';

import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
const lightMuiTheme = getMuiTheme();

import { Map, TileLayer, LayerGroup, Marker, Polyline, Polygon } from 'react-leaflet';
import * as colors from 'material-ui/styles/colors';
import { Vehicle } from '../Vehicle';
import { ContextMenu } from '../components/ContextMenu';

type Props = {
    handleSelectedAircraftUpdate: (id: string) => void,
    connectedVehicles: {[id: string]: Vehicle},
    selectedVehicleID: string,
    maxZoom: number,
    mapZoom: number,
    mapCenter: number[],
    globalOrigin: PositionType,
    updateMapCenter: (e: L.LeafletMouseEvent) => void,
    contextSetHome: () => void,
    contextSetGlobal: () => void,
    contextGoHere: () => void,
    contextSetTakeoff: () => void,
    setContextAnchor: (e: L.LeafletMouseEvent) => void
    contextAnchor: L.LeafletMouseEvent,
    MACEConnected: boolean,
    environmentBoundary: PositionType[]
}

type State = {
    showContextMenu?: boolean
}

export default class MACEMap extends React.Component<Props, State> {
  leafletMap: L.Map;
  constructor(props: Props) {
    super(props);

    this.state = {
      showContextMenu: false,
    }
  }

  // shouldComponentUpdate(nextProps: Props, nextState: State) {
  //   if(nextState.showContextMenu === this.state.showContextMenu) {
  //     return false;
  //   }

  //   return true;
  // }

  componentDidMount(){
    this.leafletMap = this.refs.map;
  }


  handleMarkerClick = (e: L.LeafletMouseEvent, vehicleId: string, type: string) => {
    this.props.handleSelectedAircraftUpdate(vehicleId);
  }

  triggerContextMenu = (event: L.LeafletMouseEvent) => {
    this.props.setContextAnchor(event);
    this.setState({showContextMenu: !this.state.showContextMenu});
  }


  render() {

    const width = window.screen.width;
    const height = window.screen.height;
    const parentStyle = {height: height + 'px', width: width + 'px'};
    const mapStyle = { top: 0, left: 0, height: height + 'px', width: width + 'px' };

    const globalOriginMarker = {
      position: new L.LatLng(this.props.globalOrigin.lat, this.props.globalOrigin.lon),
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

            <Map ref="map" onDragend={this.props.updateMapCenter} useFlyTo={true} animate={true} center={this.props.mapCenter} zoom={this.props.mapZoom} style={mapStyle} zoomControl={false} maxZoom={this.props.maxZoom} onContextmenu={this.triggerContextMenu} >
                {/* <TileLayer url='http://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}' />  */}
                <TileLayer url='http://{s}.google.com/vt/lyrs=s&x={x}&y={y}&z={z}' maxZoom={this.props.maxZoom} subdomains={['mt0','mt1','mt2','mt3']} />

                {this.props.MACEConnected &&
                  <LayerGroup>

                    {/* Aircraft Icons */}
                    {Object.keys(this.props.connectedVehicles).map((key: string) => {
                      return (
                        <Marker zIndexOffset={1000} onclick={(e: L.LeafletMouseEvent) => this.handleMarkerClick(e, key, "vehicle")} key={key} position={this.props.connectedVehicles[key].vehicleMarker.latLon} icon={this.props.connectedVehicles[key].vehicleMarker.icon} title={key}>
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
                        <Marker onclick={(e: L.LeafletMouseEvent) => this.handleMarkerClick(e, key, "home")} key={key} position={this.props.connectedVehicles[key].homePosition.latLon} icon={this.props.connectedVehicles[key].homePosition.icon} title={key}>
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

                  </LayerGroup>
                }

            </Map>
          </div>
        </MuiThemeProvider>
    );
  }
}
