import MuiThemeProvider from 'material-ui/styles/MuiThemeProvider';
import getMuiTheme from 'material-ui/styles/getMuiTheme';
import darkBaseTheme from 'material-ui/styles/baseThemes/darkBaseTheme';
const lightMuiTheme = getMuiTheme();
const darkMuiTheme = getMuiTheme(darkBaseTheme);
import * as React from 'react';
import RaisedButton from 'material-ui/RaisedButton';
import Checkbox from 'material-ui/Checkbox';

export type LayerGroupType = {
  type: string,
  latLons: L.LatLng[]
}

export type PathType = {
  waypoints: L.LatLng[]
}

type Props = {
    boundaryVerts: LayerGroupType,
    pointsOfInterest: LayerGroupType[],
    aircraftPaths: PathType[],
    clearWaypoints: () => void,
    clearPointsofInterest: () => void,
    clearBoundary: () => void,
    generateWaypointsAjax: () => void
}

type State = {
    useTestPoints?: boolean
}

export class MapControlButtons extends React.Component<Props, State> {

    constructor() {
        super();

        this.state = {
            useTestPoints: false
        }
    }


    render() {
        const buttonContainer = { position: 'absolute', top: 15, right: 15, zIndex: 999};
        const useTestPointsContainer = { position: 'absolute', top: 60, right: 0, width: 200, zIndex: 999};
        const buttonStyle = { marginLeft: 10 };
        const checkbox = { marginBottom: 16 }


        return(
            <div>
                <div style={buttonContainer}>
                    {this.props.aircraftPaths.length > 0 &&
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={buttonStyle} label="Clear Waypoints" onClick={this.props.clearWaypoints}/>
                    </MuiThemeProvider>
                    }
                    {this.props.pointsOfInterest.length > 0 &&
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={buttonStyle} label="Clear Hot Spots" onClick={this.props.clearPointsofInterest}/>
                    </MuiThemeProvider>
                    }
                    {this.props.boundaryVerts !== null &&
                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={buttonStyle} label="Clear All" onClick={this.props.clearBoundary}/>
                    </MuiThemeProvider>
                    }

                    <MuiThemeProvider muiTheme={lightMuiTheme}>
                        <RaisedButton style={buttonStyle} label="Generate Waypoints" onClick={this.props.generateWaypointsAjax}/>
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
          </div>
        )
    }
}


