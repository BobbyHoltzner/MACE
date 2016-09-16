// Material Design Default Themes:
// import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
// import getMuiTheme from 'material-ui/styles/getMuiTheme';
// import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
// const lightMuiTheme = getMuiTheme();
// const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
// import { Map, Marker, Popup, TileLayer, FeatureGroup  } from 'react-leaflet';
// import { EditControl } from "react-leaflet-draw";
// import RaisedButton from 'material-ui/RaisedButton';
//import PythonShell from 'python-shell';

type MarkerType = {
  position: L.LatLng,
  icon: L.Icon
}

type LayerGroupType = {
  type: string,
  latLons: L.LatLng[]
}

type Props = {
}

type State = {
  markers?: MarkerType[],
  layerGroups?: LayerGroupType[]
}

export class AppContainer extends React.Component<Props, State> {

  constructor() {
    super();

    this.state = {
      markers: [],
      layerGroups: []
    }
  }

  // appendMarker = (event: L.LeafletMouseEvent) => {
  //   let tmpMarker: MarkerType = {
  //     position: event.latlng,
  //     icon: new L.Icon({
  //         iconUrl: './app/images/marker-icon.png',
  //         iconSize: [25, 41], // size of the icon
  //         iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
  //         popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
  //       })
  //   };
  //
  //   let tmpMarkers = this.state.markers;
  //   tmpMarkers.push(tmpMarker);
  //   this.setState({markers: tmpMarkers});
  // }
  //
  // _onEditPath = (e: L.LeafletEvent) => {
  //   console.log('Path edited !');
  //   console.log(e);
  // }
  //
  // _onCreate = (e: any) => {
  //   // let polyline = e.layer;
  //   // To edit this polyline call : polyline.handler.enable()
  //   console.log('Path created !');
  //   console.log(e);
  //   console.log(e.layerType);
  //   console.log(e.layer._latlng);
  //   let layerType = e.layerType;
  //   let latLons: L.LatLng[] = [];
  //   let type: string;
  //   if(layerType === 'rectangle'){
  //     for(let i = 0; i < e.layer._latlngs.length; i++){
  //       latLons.push(e.layer._latlngs[i]);
  //     }
  //     type = 'boundary';
  //   }
  //   else if (layerType === 'marker'){
  //     latLons.push(e.layer._latlng);
  //     type = 'poi';
  //   }
  //
  //   let layerGroups = this.state.layerGroups;
  //   let tmpLayerGroup = {
  //     type: type,
  //     latLons: latLons
  //   }
  //   layerGroups.push(tmpLayerGroup);
  // }
  //
  // _onDeleted = (e: L.LeafletEvent) => {
  //   console.log('Path deleted !');
  // }
  //
  // _mounted = (drawControl: L.LeafletEvent) => {
  //   console.log('Component mounted !');
  // }
  //
  // executePython = () => {
  //   console.log('executePython');
  //
  //
  //   // let boundaryVerts = [[1,1], [2.5,3], [4,4]];
  //   // let hotSpots = [(0, 0), (0, 5), (5, 5), (5, 0), (0, 0)];
  //
  //   // PythonShell.run('C:/Code/NASA_GUI/app/containers/test.py', function (err: any) {
  //   //   if (err) throw err;
  //   //   console.log('finished');
  //   // });
  //
  //
  //   // let options = {
  //   //   mode: 'text',
  //   //   pythonPath: 'C:/Anaconda2/',
  //   //   pythonOptions: ['-u'],
  //   //   scriptPath: 'C:/Code/NASA_GUI/app/containers/',
  //   //   args: [boundaryVerts, hotSpots]
  //   // };
  //   //
  //   // PythonShell.run('test.py', options, function(err: any, results: any) {
  //   //   if(err){
  //   //     throw err;
  //   //   }
  //   //
  //   //   console.log('results: %j', results);
  //   // })
  //
  // }

  render() {

    // const position = [37.889231, -76.810302];
    // var width = window.screen.availWidth;
    // var height = window.screen.availHeight;
    // const style = { top: 0, left: 0, height: height + 'px', width: width + 'px' };
    // const zoom = 13;


    return (

        <div>
        A little bit of text
        {/*
          <MuiThemeProvider muiTheme={darkMuiTheme}>
            <RaisedButton label="Execute Python" onClick={this.executePython}/>
          </MuiThemeProvider>

          <Map ref="map1" center={position} zoom={zoom} style={style} >
              <TileLayer url='http://{s}.tile.osm.org/{z}/{x}/{y}.png' />

              <FeatureGroup>
                <EditControl
                  position='topleft'
                  onEdited={this._onEditPath}
                  onCreated={this._onCreate}
                  onDeleted={this._onDeleted}
                  draw={{
                      polyline: {
                          shapeOptions: {
                              color: '#f357a1',
                              weight: 10
                          }
                      },
                      polygon: {
                          allowIntersection: false, // Restricts shapes to simple polygons
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
                              iconUrl: './app/images/marker-icon.png',
                              iconSize: [25, 41], // size of the icon
                              iconAnchor: [12, 41], // point of the icon which will correspond to marker's location
                              popupAnchor: [0, -38] // point from which the popup should open relative to the iconAnchor
                            }),
                          }
                    }}
                />
            </FeatureGroup>
          </Map>
        */}
        </div>
    );
  }
}
